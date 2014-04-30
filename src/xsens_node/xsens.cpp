

#include "xsens_node.cpp"

/************************************************************
 * Initialize ROS, manage errors and iteration rate.
 ************************************************************/


int main(int argc, char** argv) {

		ros::init(argc, argv, "xsens");
		XSensNode node;
		ros::Rate rate(60);

		while (ros::ok()) {
			node.spinOnce();
			rate.sleep();
		}

}
