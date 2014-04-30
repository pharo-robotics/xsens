#include <queue>
#include <ros/ros.h>
#include <tf/tf.h>
#include <xsens_driver/xsens_driver.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <exception>

//msgs
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include<geometry_msgs/Vector3Stamped.h>

//srv

#include <xsens/Calibrate.h>





/************************************************************
 * Streaming operators
 ************************************************************/

void operator +=(XSensDriver::Vector3 * v1, tf::Vector3 & v2) {
	v1->x += v2.getX();
	v1->y += v2.getY();
	v1->z += v2.getZ();
}

void operator >>(XSensDriver::Vector3 * v1, geometry_msgs::Vector3 & v2) {
	v2.x = v1->x;
	v2.y = v1->y;
	v2.z = v1->z;
}


void operator >>(XSensDriver::Vector3 * v1, geometry_msgs::Vector3Stamped & v2) {
	v1 >> v2.vector;
}



void operator >>(XSensDriver::Vector3 * v1, tf::Vector3 & v2) {
	v2.setX(v1->x);
	v2.setY(v1->y);
	v2.setZ(v1->z);
}


void operator >>(XSensDriver::Vector3 * v1, geometry_msgs::Quaternion & q) {
	tf::Quaternion quaternion;
	quaternion.setRPY(v1->x, v1->y, v1->z);

	q.x = quaternion.x();
	q.y = quaternion.y() ;
	q.z = quaternion.z();
	q.w = quaternion.w();
}


void operator >>(tf::Vector3 & v1, geometry_msgs::Vector3 & v2) {
	v2.x = v1.getX();
	v2.y = v1.getY();
	v2.z = v1.getZ();
}




/************************************************************
 * Hidding pointers
 ************************************************************/

typedef XSensDriver * XSens;




/************************************************************
 * Configuration holder
 ************************************************************/

struct NodeConfiguration {
	struct {
		std::string serialPort;
	} location;
	struct {
		std::string frameId;
		std::string imu_topic_name;
		std::string magnetic_topic_name;
		std::string calibrated_topic_name;
	}broadcasting;

	struct {
		bool autocalibrate;
		bool assumeAsCalibrated;
		bool driftIsCalibrated;
		int driftCalibrationSteps;
		int orientationCalibrationSteps;
		double maxDriftRate;
	} calibration;
	struct {
		double orientation;
		double angularVelocity;
		double linearAcceleration;
	} standardDeviation;
};

