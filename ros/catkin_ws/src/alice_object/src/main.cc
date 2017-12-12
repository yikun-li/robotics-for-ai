#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

// PointCloud includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/feature.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>

// boost includes

#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <alice_msgs/ObjectROIAction.h>
#include <alice_msgs/ROI.h>

// namespaces
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

ros::Publisher publisher;
ros::Subscriber depth_sub;

bool failed;
bool success;
bool processing;

int width, height;

std::vector<alice_msgs::ROI> vROI;

tf::TransformListener *tfTransformer;

void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	ROS_INFO_STREAM("Received cloud");

	ros::Time now = ros::Time::now();

	PCLPointCloudPtr receivedCloud(new PCLPointCloud);
	PCLPointCloudPtr unmodCloud(new PCLPointCloud);

	fromROSMsg(*cloudMsg, *receivedCloud); // convertion from ROS to PCL formats
	fromROSMsg(*cloudMsg, *unmodCloud); // convertion from ROS to PCL formats


	ROS_INFO_STREAM(unmodCloud->width << ", " << unmodCloud->height);
	width = unmodCloud->width;
	height = unmodCloud->height;

	// Remove planar surface
	PCLPointCloudPtr surflessCloud(new PCLPointCloud);
	surflessCloud->header.frame_id = "front_xtion_link";

	pcl::VoxelGrid<Point> vg;
  vg.setInputCloud (receivedCloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
    //vg.filter (*receivedCloud);
    
	PassThrough<Point> passthrough_filter;
	passthrough_filter.setInputCloud(receivedCloud);
	passthrough_filter.setFilterFieldName("z");
	passthrough_filter.setFilterLimits(0.3, 1.0);
	passthrough_filter.filter(*surflessCloud);

	PCLPointCloudPtr segmentedCloud (new PCLPointCloud);

	ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	SACSegmentation<Point> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.015);  // 1.5cm

	seg.setInputCloud(surflessCloud);
	seg.segment(*inliers, *coefficients);

	ExtractIndices<Point> extract;

	extract.setInputCloud(surflessCloud);
	extract.setIndices(inliers);
	extract.setNegative(true); // true means remove planes
	extract.filter(*segmentedCloud);

	segmentedCloud->header.frame_id = "camera_link";
	publisher.publish(*segmentedCloud);

	if (segmentedCloud->points.size() < 1)
		return;

	std::vector<pcl::PointIndices> clusters;     // Cluster information is stored here

	pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);

	tree->setInputCloud(segmentedCloud);
	EuclideanClusterExtraction<Point> ec;

	ec.setClusterTolerance(0.05);       // in meters
	ec.setMinClusterSize(50);         // Minimal points that must belong to a cluster
	ec.setMaxClusterSize(50000);      // Maximal points that must belong to a cluster
	ec.setSearchMethod(tree);
	ec.setInputCloud(segmentedCloud);
	ec.extract(clusters);

	if (clusters.size() == 0)
	{
		ROS_INFO_STREAM("No objects found");
		depth_sub.shutdown();
		processing = false;
		return;
	}
	
	struct ROI
	{
		int left = 9999;
		int right = 0;
		int top = 9999;
		int bottom = 0;
	};

	ROS_INFO_STREAM("Objects found: " << clusters.size());
	
	pcl::search::KdTree<Point>::Ptr ktree (new pcl::search::KdTree<Point>);
	ktree->setInputCloud(unmodCloud);
	
	std::vector<int> kIndex(1);
	std::vector<float> kDist(1); // Needed, not used
	
	std::vector<alice_msgs::ROI> vRois;

	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
	{
		ROI roi;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			Point point = segmentedCloud->points[*pit];
			ktree->nearestKSearch(point, 1, kIndex, kDist);
			int index = kIndex.at(0);

			int x = index % width;
			int y = floor(index / width);

			if (x < roi.left)
				roi.left = x;
			if (x > roi.right)
				roi.right = x;
			if (y < roi.top)
				roi.top = y;
			if (y > roi.bottom)
				roi.bottom = y;
		}

		alice_msgs::ROI troi;
		troi.left = roi.left;
		troi.right = roi.right;
		troi.bottom = roi.bottom;
		troi.top = roi.top;

		vROI.push_back(troi);


	}

	success = true;
	depth_sub.shutdown();
	processing = false;

}

void ExecuteCB(const alice_msgs::ObjectROIGoalConstPtr &goal, actionlib::SimpleActionServer<alice_msgs::ObjectROIAction> *as, ros::NodeHandle &n)
{
	failed = false;
	success = false;
	processing = false;
	vROI.clear();
	ROS_INFO_STREAM("Received goal");

	if (goal->action == "findObjects")
	{
		depth_sub = n.subscribe<sensor_msgs::PointCloud2>("/front_xtion/depth_registered/points", 1, pcCallBack);

		processing = true;

		ros::Rate rate(10);

		while (processing) // dirty sleep hack
		{
			rate.sleep();
		}

		if (failed)
			as->setAborted();
		else if (success)
		{
			alice_msgs::ObjectROIResult ObjectROIs;
			ObjectROIs.roi = vROI;
			as->setSucceeded(ObjectROIs);
		}
	}
	else
	{
		ROS_INFO_STREAM(goal->action << ", is not defined, not doing anything...");
		as->setAborted();
	}
}

int main(int argc, char **argv)
{
	// ROS initialization stuff
	ros::init(argc, argv, "object2d");
	ros::NodeHandle n;

	tf::TransformListener temp_listener;
	tfTransformer = &temp_listener;
	actionlib::SimpleActionServer<alice_msgs::ObjectROIAction> as(n, "/ObjectROI", boost::bind(&ExecuteCB, _1, &as, n), false);
	as.start();
	//Subscribe to point cloud data

	publisher = n.advertise<PCLPointCloud>("simpleFeature/output", 1);

	ros::spin();
}
