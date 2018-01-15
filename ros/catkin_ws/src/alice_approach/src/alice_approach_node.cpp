/*
 * alice_approach_node.cpp
 *
 *  Created on: Apr 6, 2015
 *      Author: rik
 */

#include <ros/ros.h>
#include <alice_approach/alice_approach.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "alice_approach");
	ros::NodeHandle node_handle;

	AliceApproach aliceApproach(node_handle);

	ros::spin();
}



