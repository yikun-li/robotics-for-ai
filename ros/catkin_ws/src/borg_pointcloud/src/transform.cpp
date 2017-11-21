#include <transform.h>

void callback(borg_pointcloud::parametersConfig &config, uint32_t level) {
  ROS_DEBUG_STREAM("Reconfigure Request: " << " " <<
            config.x_min << " " << 
            config.input_topic.c_str() << " " <<
	        config.output_topic.c_str() <<  " " <<
            (config.remove_noise?"True":"False") << " " << 
            config.robot_length << " " <<
            config.robot_width << " " <<
            config.robot_height);
    x_min = config.x_min;
    leaf_size = config.leaf_size;
    remove_noise = config.remove_noise;
    robot_length = config.robot_length;
    robot_width = config.robot_width;
    robot_height_from_base_link = config.robot_height;
    if (input_topic != config.input_topic)
    {
        input_topic = config.input_topic;
        input_topic_changed = true;
    } 
    if (output_topic != config.output_topic)
    {
        output_topic = config.output_topic;
        output_topic_changed = true;
    } 
    build_convex_hull();

  
}

// This function is called whenever new point cloud data arrives
void depthCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{    
    // =======================================================
    // PointCloud frame_id transform: camera_link => base_link
    // =======================================================
    
    if (transformedPointCloud_pub.getNumSubscribers() == 0)
    {
        ROS_WARN_ONCE("No subscribers");
	    return;
    }
    else
	    ROS_WARN_ONCE("Subscriber active, resuming calculation");

    // Wait for transform to become available

    try
    {
        ros::Time now = ros::Time::now();
        listener->waitForTransform("/base_link_dummy", cloudMsg->header.frame_id,
                                          now, ros::Duration(0.1));
    }
    catch (...)
    {
        ROS_WARN("Transform not available");
    }
    if (!listener->canTransform("/base_link_dummy", cloudMsg->header.frame_id, cloudMsg->header.stamp))
    {
        ROS_WARN("Cannot do transform to base_link_dummy");
        return;
    }

    // Transform PointCloud
    // After transform: X = distance from robot (0 = center of robot), Y = horizontal distance (0 = center of robot), Z = height (0 = ground)
    sensor_msgs::PointCloud2 cloudMsg_transformed;
    pcl_ros::transformPointCloud("/base_link_dummy", *cloudMsg, cloudMsg_transformed, *listener);
    
    // Convert PointCloud2 to PCL::PointCloud
    PCLPointCloudPtr pclCloud (new PCLPointCloud);
    pcl::fromROSMsg(cloudMsg_transformed, *pclCloud);
    
    // Discard Nao body and change leaf size using a voxelgrid filter
    PCLPointCloudPtr pclCloud_filtered_x (new PCLPointCloud);
    pcl::VoxelGrid<Point> voxelgrid_filter;
    voxelgrid_filter.setInputCloud(pclCloud);
    voxelgrid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);  // Leaf size: 1 cm
    voxelgrid_filter.setFilterFieldName("x");
    voxelgrid_filter.setFilterLimits(x_min, 5);
    voxelgrid_filter.filter(*pclCloud_filtered_x);
    
    // Remove noise using radius outlier removal
    if (remove_noise)
    {
        PCLPointCloudPtr pclCloud_cleaned (new PCLPointCloud);
        pcl::RadiusOutlierRemoval<Point> radius_outlier_removal;
        radius_outlier_removal.setInputCloud(pclCloud_filtered_x);
        radius_outlier_removal.setRadiusSearch(0.15);
        radius_outlier_removal.setMinNeighborsInRadius(20);
        radius_outlier_removal.filter(*pclCloud_filtered_x);
        
        // Publish transformed and filtered PointCloud
        //transformedPointCloud_pub.publish(*pclCloud_cleaned);
        //return;
    }

    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>());
    //inputCloud->push_back(pcl::PointXYZ(0, 0, 0));
    //inputCloud->push_back(pcl::PointXYZ(0.5, 0.5, 0.3));

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
    
	pcl::CropHull<pcl::PointXYZ> cropHull;
	cropHull.setDim(3); 
	cropHull.setInputCloud(pclCloud_filtered_x);
	//cropHull.setInputCloud(inputCloud);
	cropHull.setHullIndices(polygons);
	//cropHull.setHullCloud(boundingbox_ptr);
	cropHull.setHullCloud(surface_hull);
	cropHull.setCropOutside(true);

	std::vector<int> indices;
	cropHull.filter(indices);
	cropHull.filter(*objects);

	// Publish transformed PointCloud
	
	pcl::PointIndices::Ptr fInliers (new pcl::PointIndices); 
	fInliers->indices = indices;
    //populate fInliers with indices of part_of_full_cloud i.e. 
    //fInliers := indices of part_of_full_cloud; 

    // Extract fInliers from the input cloud 
    pcl::ExtractIndices<pcl::PointXYZ> extract ; 
    extract.setInputCloud (pclCloud_filtered_x); 
    extract.setIndices (fInliers); 
    //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud 
    extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest 
    extract.filter (*objects); 
	transformedPointCloud_pub.publish(*objects);
	//transformedPointCloud_pub2.publish(*objects);
	
    //std::cout << "input cloud size" << pclCloud_filtered_x->size() << std::endl;
	//std::cout << "output cloud size" << objects->size() << std::endl;
	//std::cout << "I/O point difference" << pclCloud_filtered_x->size() - objects->size() << std::endl;
}

void build_convex_hull()
{
    polygons.clear();
    boundingbox_ptr->clear();
    surface_hull->clear();
    // Building the Convex Hull, in this case a box centered at base link.
    ROS_DEBUG_STREAM(robot_length << " " << robot_width << " " << robot_height_from_base_link );
    length_front_left = length_front_right = robot_length / 2 + 0.02;
    length_back_left = length_back_right = - robot_length / 2;
    width_front_left = width_back_left = - robot_width / 2;
    width_front_right = width_back_right = robot_width / 2;
    height_top_left = height_top_right = robot_height_from_base_link;
    height_bottom_left = height_bottom_right = -.30;
    
    float left_side_offset = -0.02;
    //Main Body
    boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left, width_front_left +left_side_offset, height_top_left));
    boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right, width_front_right, height_top_right));
    boundingbox_ptr->push_back(pcl::PointXYZ(length_back_right, width_back_right, height_top_right));
    boundingbox_ptr->push_back(pcl::PointXYZ(length_back_left, width_back_left + left_side_offset, height_top_left));

    boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left, width_front_left, height_bottom_left));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right, width_front_right, height_bottom_right));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_back_right, width_back_right, height_bottom_right));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_back_left, width_back_left, height_bottom_left));

	//Front Wheels
	float front_l_offset = 0.1;
	float back_l_offset = 0.17;
	float front_w_offset = 0.06;
	float  back_w_offset = 0.06;
	//Front Left Wheel
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left + front_l_offset, width_front_left - front_w_offset, 0.06));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left + front_l_offset, width_front_left + front_w_offset, 0.06));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left + front_l_offset, width_front_left - front_w_offset, -0.25));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left + front_l_offset, width_front_left + front_w_offset, -0.25));

	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left - back_l_offset, width_front_left - front_w_offset, 0.06));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left - back_l_offset, width_front_left + front_w_offset, 0.06));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left - back_l_offset, width_front_left - front_w_offset, -0.25));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_left - back_l_offset, width_front_left + front_w_offset, -0.25));

	//Front Right Wheel
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right + front_l_offset, width_front_right - front_w_offset, 0.05));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right + front_l_offset, width_front_right + front_w_offset, 0.05));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right + front_l_offset, width_front_right - front_w_offset, -0.25));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right + front_l_offset, width_front_right + front_w_offset, -0.25));

	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right - back_l_offset, width_front_right - front_w_offset, 0.05));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right - back_l_offset, width_front_right + front_w_offset, 0.05));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right - back_l_offset, width_front_right - front_w_offset, -0.25));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_front_right - back_l_offset, width_front_right + front_w_offset, -0.25));
	//Rear castor wheel box
	float castor_l = 0.13;
	boundingbox_ptr->push_back(pcl::PointXYZ(length_back_right - castor_l, width_back_right, 0.06));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_back_left - castor_l, width_back_left, 0.06));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_back_right - castor_l, width_back_right, height_bottom_right));
	boundingbox_ptr->push_back(pcl::PointXYZ(length_back_left - castor_l, width_back_left, height_bottom_left));

    //Back xtion pole removal
    boundingbox_ptr->push_back(pcl::PointXYZ(-0.35, 0.0, 0.8));
    //boundingbox_ptr->push_back(pcl::PointXYZ(0.38, -0.30, 0.15)); //dup row 1

    //boundingbox_ptr->push_back(pcl::PointXYZ(0.38, -0.30, 0.15)); //dup row 6

	
	//Creating convex hull class, and setting input values to 8 points for the bounding box
	pcl::ConvexHull<pcl::PointXYZ> hull;
    //hull.setAlpha(0.1);
    hull.setInputCloud(boundingbox_ptr);
    hull.setDimension(3);
    //std::vector<pcl::Vertices> polygons;
    
    //Now, the convex hull will reconstruct a point cloud surface with polygons using those 8 points which results in the box
    hull.reconstruct(*surface_hull, polygons);
    //for(int i = 0; i < polygons.size(); i++)
	//std::cout << polygons[i] << std::endl << "-----\n";
}

int main(int argc, char** argv)
{

    input_topic_changed = false;
    output_topic_changed = false;

    // ROS initialization stuff
	ros::init(argc, argv, "pointcloud_tf");
	ros::NodeHandle n;
	ros::Rate r(30);

	// Create listener and set global pointer
	tf::TransformListener temp_listener;
	listener = &temp_listener;

    dynamic_reconfigure::Server<borg_pointcloud::parametersConfig> server;
    dynamic_reconfigure::Server<borg_pointcloud::parametersConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    // Parameters
    ros::param::param<double>("~x_min", x_min, 0.27);
    ros::param::param<bool>("~remove_noise", remove_noise, true);
    ros::param::param<double>("~leaf_size", leaf_size, 0.03);
    //std::string input_topic;
    ros::param::param<std::string>("~input_topic", input_topic, "camera/depth/points");
    //std::string output_topic;
    ros::param::param<std::string>("~output_topic", output_topic, "voxel_grid/output");

    //Subscribe to kinect point cloud data
    ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, depthCallback);
    ROS_INFO_STREAM("Depth_sub initial value: " << static_cast<bool>(depth_sub));
    // Create a ROS publisher for the output PointClouds
    transformedPointCloud_pub = n.advertise<PCLPointCloud>(output_topic, 1);
    transformedPointCloud_pub2 = n.advertise<PCLPointCloud>("voxel_test", 1);

    build_convex_hull();
	
    
    while (ros::ok())
    {   

        if (input_topic_changed)
        {
            ROS_INFO("Changing input topic");
            depth_sub.shutdown();
            depth_sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, depthCallback);
            input_topic_changed = false;
        }
        if (output_topic_changed)
        {
            ROS_INFO("Changing output topic");
            output_topic_changed = false;
            transformedPointCloud_pub = n.advertise<PCLPointCloud>(output_topic, 1);
        }
    ros::spinOnce();
	r.sleep();
    }
    
    return 0;
}	
