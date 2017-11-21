#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "borg_pointcloud/TransformPoint.h"
#include <geometry_msgs/PointStamped.h>

typedef geometry_msgs::PointStamped Point;

// If you declare a listener in a callback function, then it loses the history of the tf data.
// It therefore starts nagging about "Can't find frame_id 'base_link'" and stuff. So a global listener is required.
// The listener can't be initialized here, as ros::init has to be called first.
tf::TransformListener* listener;

bool transform(borg_pointcloud::TransformPoint::Request &req, borg_pointcloud::TransformPoint::Response &res)
{
    ROS_INFO("Request transforming point: x=%f, y=%f, z=%f; from frame_id=%s to frame_id=%s", req.x, req.y, req.z, req.frame_id.c_str(), req.target_frame_id.c_str());
    
    // Wait for transform to become available
    ros::Time now = ros::Time::now();
    listener->waitForTransform(req.frame_id, req.target_frame_id, now, ros::Duration(1.0));
    
    // Check if transform is available
    if (!listener->canTransform(req.frame_id, req.target_frame_id, now))
    {
        // Not available
        ROS_INFO("Transform not available");
        return false;
    }
    
    // Populate input point
    Point input;
    input.header.stamp = now;
    input.header.frame_id = req.frame_id;
    input.point.x = req.x;
    input.point.y = req.y;
    input.point.z = req.z;
    
    // Tranformed point
    Point output;
    
    // Do the actual transformation
    listener->transformPoint(req.target_frame_id, input, output);
    
    // Save transformed point in response
    res.t_x = output.point.x;
    res.t_y = output.point.y;
    res.t_z = output.point.z;
    
    ROS_INFO("Returning transformed point: x=%f, y=%f, z=%f", res.t_x, res.t_y, res.t_z);
    
    return true;
}

int main(int argc, char** argv) 
{
    // ROS initialization stuff
	ros::init(argc, argv, "pointcloud_tf_point_service");
	ros::NodeHandle n;
	
	// Create listener and set global pointer
	tf::TransformListener temp_listener;
	listener = &temp_listener;
    
    ros::ServiceServer service = n.advertiseService("transform_point", transform);
    ROS_INFO("Ready to transform points.");
      
    ros::spin();
    return 0;
}	
