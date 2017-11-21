/*
 * transform.h
 *
 *  Created on: Jan 21, 2015
 *      Author: borg
 */

#ifndef SOURCE_DIRECTORY__BORG_POINTCLOUD_INCLUDE_TRANSFORM_H_
#define SOURCE_DIRECTORY__BORG_POINTCLOUD_INCLUDE_TRANSFORM_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
//For cropping hull out of point cloud
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

//Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <borg_pointcloud/parametersConfig.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PCLPointCloud;
typedef pcl::PointCloud<Point>::Ptr PCLPointCloudPtr;


// For publishing transformed PointCloud data. We don't want to create a new publisher every time
ros::Publisher transformedPointCloud_pub;
ros::Publisher transformedPointCloud_pub2;

// If you declare a listener in the callback function, then it loses the history of the tf data.
// It therefore starts nagging about "Can't find frame_id 'base_link'" and stuff. So a global listener is required.
// The listener can't be initialized here, as ros::init has to be called first.
tf::TransformListener* listener;

// Global parameter values (I know, don't use global variables, but screw you)
double x_min;
double leaf_size;
bool remove_noise;
std::string input_topic;
std::string output_topic;
bool input_topic_changed;
bool output_topic_changed;

std::vector<pcl::Vertices> polygons;
pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);

float wheel_width = 0.1;
float constwheel_diameter = 0.27;
float robot_length = 0.56;
float robot_width = 0.50;
float robot_height_from_base_link = 0.9;


float length_front_left, length_front_right, length_back_left, length_back_right;
float width_front_left, width_front_right, width_back_left, width_back_right;
float height_top_left, height_top_right, height_bottom_left, height_bottom_right;

void build_convex_hull(); //Builds convex hull for point cloud extraction




#endif /* SOURCE_DIRECTORY__BORG_POINTCLOUD_INCLUDE_TRANSFORM_H_ */
