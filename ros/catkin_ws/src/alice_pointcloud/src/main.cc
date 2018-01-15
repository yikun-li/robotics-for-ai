#include <iostream>
#include <ros/ros.h>

#include <pointcloudhandler.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "alice_pointcloud");
	ros::NodeHandle node_handle;

	PointCloudHandler pointCloudHandler(node_handle);

	ros::spin();
}
