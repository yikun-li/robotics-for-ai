/*
 * main.cc
 *
 *  Created on: Apr 3, 2015
 *      Author: rik
 */

#include <ros/ros.h>
#include <alice_controller/alicecontroller.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "alice_controller");
	ros::NodeHandle node_handle;

	AliceController aliceController(node_handle);

	ros::spin();
}



