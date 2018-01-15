#ifndef _H_UTILS
#define _H_UTILS

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread/thread.hpp>

#include <std_msgs/Float64.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <alice_msgs/pointcloudfunctionAction.h>
#include <alice_msgs/pointcloudfunctionResult.h>
#include <alice_msgs/alicecontrollerfunctionAction.h>
#include <alice_msgs/Location.h>
#include <alice_msgs/PcPlane.h>

#include <tf/transform_listener.h>


using namespace std;
using namespace pcl;
using namespace ros;

using boost::property_tree::ptree;

typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

#endif
