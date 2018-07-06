#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <sync/config.h>

// Self defined type.
#include <bfsdriver_1_1/ImageStamp.h>

const int NUM_CAMERAS = 3;
 

int main(int argc, char** argv) {

	av_register_all(); // Call this first.

	ros::init(argc, argv, "image_subscriber");
	ros::NodeHandle nh;
	std::vector<Listener*> subObjs;

	/* Separate pushing, 1) to keep mem reference (ros), 2) avoid STL reassign memory */
	for (int i = 0; i < NUM_CAMERAS; ++i) { subObjs.push_back( new Listener(nh, i)); }
	ros::spin(); // Would make objects created on stack goes out of scope.
	return 0;
}

