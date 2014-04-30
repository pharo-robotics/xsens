#include <xsens_node/xsens.h>

class XSensNode {

	/*******************************
	 * Fields
	 *******************************/
private:
	const static unsigned int MEASURES_TRACK_SIZE = 2;

	std::queue<tf::Vector3> measures;
	NodeConfiguration configuration;
	ros::NodeHandle node;
	ros::NodeHandle private_node;

	XSens driver;

	struct {
		ros::Publisher imu;
		ros::Publisher calibrationStatus;
		ros::Publisher magnetic;
	} publishing;

	struct {
		ros::ServiceServer calibration;
	} services;

	struct {
		tf::Vector3 orientation;
		tf::Vector3 drift;
	} compensation;

	/*******************************
	 * Private methods
	 *******************************/

private:

	/************************************************************
	 * Load given parameters, load default for the rest.
	 ************************************************************/
	void loadParametersFrom(ros::NodeHandle& aNode) {
		aNode.param("port", configuration.location.serialPort, std::string("/dev/ttyUSB0"));

		aNode.param("frame_id", configuration.broadcasting.frameId, std::string("XSens-Mt0"));
		aNode.param("imu_topic_name", configuration.broadcasting.imu_topic_name, std::string("/imu/data"));
		aNode.param("mangetic_topic_name", configuration.broadcasting.magnetic_topic_name, std::string("/magnetic"));
		aNode.param("calibrated_topic_name", configuration.broadcasting.calibrated_topic_name, std::string("/imu/is_calibrated"));

		aNode.param("autocalibrate", configuration.calibration.autocalibrate, true);
		aNode.param("max_drift_rate", configuration.calibration.maxDriftRate, 0.001);

		aNode.param("drift_calibration_steps", configuration.calibration.driftCalibrationSteps, 550);
		aNode.param("orientation_calibration_steps", configuration.calibration.orientationCalibrationSteps, 110);

		aNode.param("orientation_standard_deviation", configuration.standardDeviation.orientation, 0.035);
		aNode.param("angular_velocity_standard_deviation", configuration.standardDeviation.angularVelocity, 0.012);
		aNode.param("linear_acceleration_standard_deviation", configuration.standardDeviation.linearAcceleration, 0.098);

		aNode.param("imu_topic_name", configuration.standardDeviation.linearAcceleration, 0.098);

		configuration.calibration.driftIsCalibrated = true;

	}

	/************************************************************
	 * Open XSens. Explodes if there is a failure.
	 ************************************************************/
	void initializeXSensDriver() {
		ROS_INFO("Initializing driver");
		driver = new XSensDriver();

		ROS_DEBUG(driver->strConfiguration().c_str());
		ROS_INFO("Beginning calculations");
		ROS_WARN("All the broadcasted information is based on the device [0]");
	}

	/************************************************************
	 * Configure topics and services
	 ************************************************************/
	void initializeTopicsAndServicesThrough(ros::NodeHandle & aNode) {
		publishing.magnetic = aNode.advertise < geometry_msgs::Vector3Stamped > (configuration.broadcasting.magnetic_topic_name, 1);
		// /xsens/magnetic

		publishing.calibrationStatus = aNode.advertise < std_msgs::Bool > (configuration.broadcasting.calibrated_topic_name, 1);
		publishing.imu = aNode.advertise < sensor_msgs::Imu > (configuration.broadcasting.imu_topic_name, 1);
		// /xsens/imu/data
		services.calibration = aNode.advertiseService("calibrate", &XSensNode::calibrate, this);
	}

	/************************************************************
	 * Calculate the error compensation for orientation
	 ************************************************************/

	tf::Vector3 calibrateOrientation(int samples) {
		tf::Vector3 newCompensation;

		ROS_INFO("Running orientation calibration...");

		for (int i = 0; i < samples && ros::ok(); i++) {
			newCompensation += peekLastDataFromImu();
			ros::spinOnce();
			ros::Rate(110).sleep();
		}

		newCompensation /= (double) (-samples);
		compensation.orientation = newCompensation;
		return newCompensation;
	}

	/************************************************************
	 * Calculate the error compensation for drift
	 ************************************************************/
	tf::Vector3 calibrateDrift(int samples) {
		ROS_INFO("Running drift calibration...");

		tf::Vector3 newCompensation = peekLastDataFromImu();
		tf::Vector3 data;
		for (int i = 0; i < samples && ros::ok(); i++) {
			data = peekLastDataFromImu();
			ros::spinOnce();
			ros::Rate(110).sleep();
		}
		newCompensation -= data;
		newCompensation /= (double) (samples);

		compensation.drift = newCompensation;
		return newCompensation;
	}

	/************************************************************
	 * Push for update, give the last orientation value, or the
	 * previous one if there is a problem to access data
	 ************************************************************/
	tf::Vector3 peekLastDataFromImu() {
		updateOrientation();
		return measures.front();
	}

	/************************************************************
	 * Push for IMU update and register the last orientation measure
	 ************************************************************/

	void updateOrientation() {
		try {
			pushIMUForUpdate();
			tf::Vector3 transformedIMUData;
			driver->orientation[1] >> transformedIMUData;

			while (measures.size() >= MEASURES_TRACK_SIZE) {
				measures.pop();
			}

			measures.push(transformedIMUData);

		} catch (XSensDriverException & exception ) {
			ROS_WARN("Failed to update data during this cycle...");
		}

	}

	/************************************************************
	 * Push for IMU update
	 ************************************************************/

	void pushIMUForUpdate() {
		driver->fetchData();
	}

	/************************************************************
	 * Broadcast IMU information.
	 ************************************************************/

	void broadcastIMU() {
		sensor_msgs::Imu data;



		data.header.frame_id = configuration.broadcasting.frameId;

		driver->acceleration[0] >> data.linear_acceleration;
		driver->gyroscope[0] >> data.angular_velocity;
		driver->orientation[0] >> data.orientation;

		data.angular_velocity_covariance[0] = data.angular_velocity_covariance[4] = data.angular_velocity_covariance[8] = pow(
				configuration.standardDeviation.angularVelocity, 2);
		data.linear_acceleration_covariance[0] = data.linear_acceleration_covariance[4] = data.linear_acceleration_covariance[8] = pow(
				configuration.standardDeviation.linearAcceleration, 2);
		data.orientation_covariance[0] = data.orientation_covariance[4] = data.orientation_covariance[8] = pow(configuration.standardDeviation.orientation, 2);

		publishing.imu.publish(data);

	}


	void broadcastMagneticField () {
		geometry_msgs::Vector3Stamped magneticData;
		driver->magneticField[0] >> magneticData;
		magneticData.header.frame_id = configuration.broadcasting.frameId;

		publishing.magnetic.publish(magneticData);
	}

	/************************************************************
	 * Check for calibration status. Update the general knowledge
	 * about calibration
	 ************************************************************/

	void checkCalibration() {
		ROS_DEBUG("Checking calibration...");
		double drift_rate = sqrt(pow(compensation.drift.x(), 2) + pow(compensation.drift.y(), 2) + pow(compensation.drift.z(), 2));
		configuration.calibration.driftIsCalibrated = drift_rate <= configuration.calibration.maxDriftRate;
		ROS_DEBUG("Drift rate: %f Max drift rate: %f", drift_rate, configuration.calibration.maxDriftRate);
	}


	/************************************************************
	 * Broadcast the last calibration status
	 ************************************************************/

	void broadcastCalibration() {
		std_msgs::Bool message;
		message.data = configuration.calibration.driftIsCalibrated;
		publishing.calibrationStatus.publish(message);
	}


	/************************************************************
	 * Manage the calibration refresh for each loop.
	 ************************************************************/

	void manageCalibration() {
		checkCalibration();
		broadcastCalibration();

		if (configuration.calibration.autocalibrate && !configuration.calibration.driftIsCalibrated) {
			calibrateDrift(configuration.calibration.driftCalibrationSteps);
		}
	}

	/*******************************
	 * Public Methods
	 *******************************/
public:

	/************************************************************
	 * Basic initialization - Constructor
	 ************************************************************/

	XSensNode(): private_node("~") {
		this->loadParametersFrom(private_node);
		this->initializeXSensDriver();
		this->initializeTopicsAndServicesThrough(node);
	}


	/************************************************************
	 * At the end we should push driver tear down.
	 ************************************************************/

	virtual ~XSensNode() {
		delete this->driver;
	}

	/*************************************************************************************
	 * Calibrate callback (for calibrate service). The Service message
	 * includes the sample size to calculate compensation and the kind of calibration.
	 *************************************************************************************/

	bool calibrate(xsens::Calibrate::Request &request, xsens::Calibrate::Response &response) {
		tf::Vector3 calibrateResult;

		if (request.kind == 1) {
			calibrateResult = this->calibrateOrientation(request.samples);
		} else if (request.kind == 2) {
			calibrateResult = this->calibrateDrift(request.samples);
		} else { // inform incorrect kind.
			return false;
		}

		calibrateResult >> response.compensation;

		return true;
	}


	/************************************************************
	 * Manage the refreshing of each iteration.
	 ************************************************************/

	void spinOnce() {
		try {
			pushIMUForUpdate();
			manageCalibration();
			broadcastIMU();
			broadcastMagneticField();
		} catch (XSensDriverException & exception) {
			if (exception.isResumable()) {
				ROS_WARN("%s", exception.what());
			} else {
				ROS_FATAL("%s", exception.what());
				ROS_FATAL("(remember to check that all the devices are switched ON and the ports are available for reading/writing (sudo chmod 666 /dev/ttyUSB0)");
				ROS_FATAL("%s", "exiting");
				throw exception;
			}
		}
	}

};

