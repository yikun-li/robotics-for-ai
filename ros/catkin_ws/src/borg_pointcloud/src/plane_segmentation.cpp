#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>



// Plane segmentation
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PCLPointCloud;
typedef pcl::PointCloud<Point>::Ptr PCLPointCloudPtr;

// For publishing PointCloud data. We don't want to create a new publisher every time
ros::Publisher tablePointCloud_pub;

tf::TransformListener *tfTransformer;

// This function is called whenever new point cloud data arrives
void depthCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // =======================================================
    // PointCloud => Extract biggest plane surface
    // =======================================================


    
    // Convert PointCloud2 to PCL::PointCloud
    PCLPointCloudPtr pclCloud (new PCLPointCloud);
  //  pcl::fromROSMsg(*cloudMsg, *pclCloud);
    
    tfTransformer->waitForTransform("/base_link", "/front_xtion_link", ros::Time::now(), ros::Duration(1.0));

	if (!tfTransformer->canTransform("/base_link", cloudMsg->header.frame_id, cloudMsg->header.stamp))
	{
		ROS_INFO_STREAM("No transform found");
		return;
	}

	PCLPointCloudPtr pointcloud(new PCLPointCloud);
	sensor_msgs::PointCloud2 cloudMsg_transformed;
	pcl_ros::transformPointCloud("/base_link", *cloudMsg, cloudMsg_transformed, *tfTransformer);

	fromROSMsg(cloudMsg_transformed, *pclCloud);

    // If for any reason segmentation fails, create an empty PointCloud for publishing
    PCLPointCloudPtr pclCloud_empty (new PCLPointCloud);
    pclCloud_empty->header = pclCloud->header;
    
    // Disregard floor using a passthrough filter
    PCLPointCloudPtr pclCloud_filtered (new PCLPointCloud);
    pcl::PassThrough<Point> passthrough_filter;
    passthrough_filter.setInputCloud(pclCloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(0.3, 2);     // Remove everything with a height < 0.3 m
    passthrough_filter.filter(*pclCloud_filtered);

    passthrough_filter.setInputCloud(pclCloud_filtered);
	passthrough_filter.setFilterFieldName("x");
	passthrough_filter.setFilterLimits(0.1, 5);
	passthrough_filter.filter(*pclCloud_filtered);

    // Check if there are any point left in the image
    if (pclCloud_filtered->points.size() < 100)
    {
        ROS_WARN("There are not enough points in the image after filtering");
        tablePointCloud_pub.publish(*pclCloud_empty);   // Publish empty message
        return;
    }
    
    // Plane segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);   // Plane information is stored here
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                  // Plane information is stored here
    pcl::SACSegmentation<Point> seg;            // Do the actual segmentation
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);      // Plane model
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);             // The higher this number, the more curvature is allowed
    seg.setInputCloud((*pclCloud_filtered).makeShared());
    seg.segment(*inliers, *coefficients);

    // Check if there are points belonging to the planes in the image, if not publish empty PointCloud
    if (inliers->indices.size() == 0)
    {
        ROS_WARN("Could not estimate a planar model in the current image.");
        tablePointCloud_pub.publish(*pclCloud_empty);   // Publish empty message
        return;
    }
    
    // Extract the planes from the PointCloud
    PCLPointCloudPtr pclCloud_filtered_planes (new PCLPointCloud);
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(pclCloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);     // We want to extract the planes, not remove them
    extract.filter(*pclCloud_filtered_planes);
    
    // Check if there are any point left in the image
    if (pclCloud_filtered_planes->points.size() < 1000)
    {
        ROS_WARN("There are not enough points in the image after filtering");
        tablePointCloud_pub.publish(*pclCloud_empty);   // Publish empty message
        return;
    }
    
    // We want to only select the biggest plane (planes are treated as clusters here)
    // First, extract clusters
    std::vector<pcl::PointIndices> cluster_indices;     // Cluster information is stored here
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);  // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud (pclCloud_filtered_planes);
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(0.02);       // 2 cm
    ec.setMinClusterSize(100);          // Minimal points that must belong to a cluster
    ec.setMaxClusterSize(2500000);      // Maximal points that must belong to a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(pclCloud_filtered_planes);
    ec.extract(cluster_indices);
    
    // Check if there are any clusters that are big enough
    if (cluster_indices.size() < 0)
    {
        ROS_WARN("There are no clusters big enough to be a table.");
        tablePointCloud_pub.publish(*pclCloud_empty);   // Publish empty message
        return;
    }
    
    // Determine biggest cluster
    size_t biggest_cluster = 0;
    size_t biggest_cluster_size = cluster_indices.front().indices.size();
    if (cluster_indices.size() > 1)
    {
        size_t current_cluster = 1;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin() + 1; it != cluster_indices.end(); ++it)
        {
            size_t cluster_size = it->indices.size();
            if (cluster_size > biggest_cluster_size)
            {
                biggest_cluster_size = cluster_size;
                biggest_cluster = current_cluster;
            }
            ++current_cluster;
        }
    }
    
    // Populate new PointCloud with points from the biggest cluster
    PCLPointCloudPtr pclCloud_table (new PCLPointCloud);
    pclCloud_table->header = pclCloud_filtered_planes->header;
    pcl::PointIndices point_indices = cluster_indices.at(biggest_cluster);
    for (std::vector<int>::const_iterator pit = point_indices.indices.begin(); pit != point_indices.indices.end(); ++pit)
    {
        pclCloud_table->points.push_back(pclCloud_filtered_planes->points[*pit]);
    }
    pclCloud_table->width = pclCloud_table->points.size();
    pclCloud_table->height = 1;
    pclCloud_table->is_dense = 1;
    
    // Publish PointCloud
    tablePointCloud_pub.publish(*pclCloud_table);
}

int main(int argc, char** argv) 
{
    // ROS initialization stuff
	ros::init(argc, argv, "pointcloud_planeSeg");
	ros::NodeHandle n;
	ros::Rate r(20);

	// Create listener and set global pointer
	tf::TransformListener temp_listener;
	tfTransformer = &temp_listener;
    	
    //Subscribe to kinect point cloud data
	ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>("front_xtion/depth_registered/points", 1, depthCallback);
	
	// Create a ROS publisher for the output PointClouds
    tablePointCloud_pub = n.advertise<PCLPointCloud>("plane_cloud/output", 1);
    
    ros::spin();
    
    return 0;
}
