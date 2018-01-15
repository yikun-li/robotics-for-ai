/*
 * aliceapproach.cc
 *
 *  Created on: Apr 6, 2015
 *      Author: rik
 */


#include <alice_approach/alice_approach.h>

AliceApproach::AliceApproach(NodeHandle &n) :
	alicePointcloudClient("pointcloudfunction", true),
	aliceControllerClient("alicecontroller", true),
	as(n, "aliceapproach", boost::bind(&AliceApproach::execute, this, _1), false)
{
	d_nh = n;

	as.start();
	tiltPosition.data = 0.8;
	panPosition.data = 0.0;

	ROS_INFO_STREAM("Waiting for alice pointcloud server....");
	alicePointcloudClient.waitForServer();
	ROS_INFO_STREAM("Alice pointcloud action server connected");
	ROS_INFO_STREAM("Waiting for alice controller server....");
	aliceControllerClient.waitForServer();
	ROS_INFO_STREAM("Alice controller action server connected");

	d_tilt = d_nh.advertise<std_msgs::Float64>("tilt_controller/command", 1);
	d_pan = d_nh.advertise<std_msgs::Float64>("pan_controller/command", 1);

	d_panIsMoving = false;
	d_tiltIsMoving = false;
}

void AliceApproach::moveTilt(std_msgs::Float64 position)
{
	d_tilt.publish(position);

	ros::Rate rate(20);
	rate.sleep();

	while (d_tiltIsMoving)
		rate.sleep();
}

void AliceApproach::movePan(std_msgs::Float64 position)
{
	d_pan.publish(position);

	ros::Rate rate(20);
	rate.sleep();

	while (d_panIsMoving)
		rate.sleep();

}

void AliceApproach::panCallback(const dynamixel_msgs::JointStateConstPtr &state)
{
	if (state->is_moving)
		d_panIsMoving = true;
	else
		d_panIsMoving = false;
}

void AliceApproach::tiltCallback(const dynamixel_msgs::JointStateConstPtr &state)
{
	if (state->is_moving)
		d_tiltIsMoving = true;
	else
		d_tiltIsMoving = false;
}

void AliceApproach::execute(const alice_msgs::aliceapproachGoalConstPtr &goal)
{
	ROS_INFO_STREAM("Approaching");

	if (goal->action == "MoveToP1R")
	{
		MoveToP1R();
		as.setSucceeded();
	}

	else if (goal->action == "MoveToP2R")
	{
		MoveToP2R();
		as.setSucceeded();
	}

	else if (goal->action == "MoveToP3R")
	{
		MoveToP3R();
		as.setSucceeded();
	}

	else if (goal->action == "MoveForward")
	{
		ros::Rate rate(1);

		while (!AlignWithTableSidewaysBack())
				rate.sleep();

		move(0.05, 0.05);
		as.setSucceeded();
	}

	else if (goal->plane == true)
	{
		ROS_INFO_STREAM("Approaching Side");
		//approachPlane();
		approachSide();
	}
	else
	{
		ROS_INFO_STREAM("Approaching objects");
		approachObjects();
	}

	d_subDynamixelTilt.shutdown();
	d_subDynamixelPan.shutdown();
}

void AliceApproach::align()
{
	d_receivedPlane = false;
	subscribe();

	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud (d_plane);

	int K = 5;
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);

	Point searchPoint;
	searchPoint.x = 0.0f;
	searchPoint.y = 0.30f;
	searchPoint.z = 0.0f;

	float xOne = 0.0f;
	float yOne = 0.0f;

	if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		for (size_t idb = 0; idb < 5; ++idb)
		{
			xOne += d_plane->points[pointIdxNKNSearch[idb]].x;
			yOne += d_plane->points[pointIdxNKNSearch[idb]].y;
		}

		xOne /= 5;
		yOne /= 5;
	}

	searchPoint.x = 0.0f;
	searchPoint.y = -0.30f;
	searchPoint.z = 0.0f;

	float xTwo = 0.0f;
	float yTwo = 0.0f;

	if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		for (size_t idb = 0; idb < 5; ++idb)
		{
			xTwo += d_plane->points[pointIdxNKNSearch[idb]].x;
			yTwo += d_plane->points[pointIdxNKNSearch[idb]].y;
		}

		xTwo /= 5;
		yTwo /= 5;
	}

	ROS_INFO_STREAM(xOne <<"," << yOne << ", " << xTwo <<", " << yTwo << ", " << xTwo - xOne);

	float xG = xTwo - xOne;
	float yG = yTwo - yOne;

	float baseX = 1.0f; // vector 2 meters in front
	float baseY = 0.0f;

	ROS_INFO_STREAM(xOne << ", " << yOne);
	ROS_INFO_STREAM(xG << ", " <<yG);
	float angle = getAngle(baseX, baseY, xG, yG);
	ROS_INFO_STREAM("Angle: " << angle);

	if (angle <= 95.0f and angle >= 85.0f) // no rotation needed
	{
		ROS_INFO_STREAM("DONE");
		return; // no need to align
	}

	if (angle > 95.0f) // 5 degree error
	{
		angle -= 90.0f;
		angle *= -1;
	}
	else if (angle < 85.0f) // 5 degree error
	{
		angle = fabs(angle - 90.0f);
	}

	ROS_INFO_STREAM("ROTATION ANGLE: " << angle);

	turn(angle);
}

void AliceApproach::subscribe()
{
	d_receivedPlane = false;

	d_sub_planes = d_nh.subscribe<alice_msgs::PcPlane>("alice_pointcloud/vPlanes", 1, &AliceApproach::planesCallback, this);

	//d_subDynamixelTilt = d_nh.subscribe<dynamixel_msgs::JointState>("/tilt_controller/state", 1, &AliceApproach::tiltCallback, this);
	//d_subDynamixelPan = d_nh.subscribe<dynamixel_msgs::JointState>("/pan_controller/state", 1, &AliceApproach::panCallback, this);

	ros::Rate rate(10);

	// start alice_pointcloud action server
	alice_msgs::pointcloudfunctionGoal goal;
	goal.function = "detectMultiPlane";
	alicePointcloudClient.sendGoal(goal);
	alicePointcloudClient.waitForResult(ros::Duration(10.0));

	if (alicePointcloudClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO_STREAM("Got planes");
	}
	rate.sleep();
	while (d_receivedPlane == false) // waiting for vectors with plane in it is received
		rate.sleep();
}

float AliceApproach::getDistance(float x, float y)
{
	float distance = sqrt(pow(x,2) + pow(y,2));
	return distance;
}

void AliceApproach::turn(float angle)
{
	alice_msgs::alicecontrollerfunctionGoal goal;
	goal.function = "turn";
	goal.angle = angle;

	aliceControllerClient.sendGoal(goal);

	aliceControllerClient.waitForResult(ros::Duration(10.0));

	if (aliceControllerClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		return;
	}
	else
		ROS_INFO_STREAM("Something went wrong with turning");

}
void AliceApproach::move(float meter, float speed)
{
	alice_msgs::alicecontrollerfunctionGoal goal;
	goal.function = "move";
	goal.meter = meter;
	goal.speed = speed;

	aliceControllerClient.sendGoal(goal);

	aliceControllerClient.waitForResult(ros::Duration(10.0));

	if (aliceControllerClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		return;
	}
	else
		ROS_INFO_STREAM("Something went wrong with turning");
}


bool AliceApproach::calc2DHull()
{
	/*
	ProjectInliers<Point> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	//proj.setIndices (d_plane);
	proj.setInputCloud (d_plane);

	//proj.setModelCoefficients (coefficients);
	proj.filter (*d_cloud_projected);
	std::cerr << "PointCloud after projection has: "
			<< d_cloud_projected->points.size () << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::Vertices> polygonVertices;
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud (d_cloud_projected);
	chull.reconstruct (*cloud_hull, polygonVertices);
    pcl::Vertices polygon = polygonVertices.pop_back();

    pcl::PointXY minX, minY, maxX, maxY;

    minX.x = minY.y = 1000;
    maxX.x = maxY.y = -1000;

	for ( int i = 0 ; i < polygon.vertices.size(); ++i)
	{
		if (cloud_hull->at(polygon.vertices.at(i)).x < minX.x)
		{
			minX.x = cloud_hull->at(polygon.vertices.at(i)).x;
			minX.y = cloud_hull->at(polygon.vertices.at(i)).y;
		}
		if (cloud_hull->at(polygon.vertices.at(i)).y < minY.y)
		{
			minY.x = cloud_hull->at(polygon.vertices.at(i)).x;
			minY.y = cloud_hull->at(polygon.vertices.at(i)).y;
		}
		if (cloud_hull->at(polygon.vertices.at(i)).x > maxX.x)
		{
			maxX.x = cloud_hull->at(polygon.vertices.at(i)).x;
			maxX.y = cloud_hull->at(polygon.vertices.at(i)).y;
		}
		if (cloud_hull->at(polygon.vertices.at(i)).y > maxY.y)
		{
			maxY.x = cloud_hull->at(polygon.vertices.at(i)).x;
			maxY.y = cloud_hull->at(polygon.vertices.at(i)).y;
		}
	}

	*/
}






bool AliceApproach::approachPlane()
{
	movePan(panPosition);
	moveTilt(tiltPosition);

	align(); // first align before calculating approach point

	movePan(panPosition);
	moveTilt(tiltPosition);

	d_receivedPlane = false;
	subscribe();

	// calculate point
	float moveDistance = 0.50f;

	// vector right in front of base_link
	float baseX = 1.0f;
	float baseY = 0.0f;

	Point minPt, maxPt;
	getMinMax3D(*d_plane, minPt, maxPt);

	float x = minPt.x - moveDistance;
	float y = (maxPt.y + minPt.y) / 2;

	ROS_INFO_STREAM("Y: " << maxPt.y << ", " << minPt.y);
	ROS_INFO_STREAM("goto point: " << x << ", " << y);
	float angle = getAngle(baseX, baseY, x, y);
	float distance = getDistance(x,y);

	float xObj = minPt.x;
	float yObj = 0.0f;
	float baseXr = -x;
	float baseYr = -y;

	float returnAngle = 180.0f - getAngle(baseXr, baseYr, xObj, yObj);

	if (y <= 0.0f) // point is on the right so rotate right
		angle *= -1;
	else
		returnAngle *= -1; // return rotation needs to be opposite rotation of first rotation

	turn(angle);
	move(distance);
	turn(returnAngle);

	movePan(panPosition);
	moveTilt(tiltPosition);

	align(); // first align before calculating approach point
	as.setSucceeded();
}

float AliceApproach::getAngle(float baseX, float baseY, float x, float y)
{
	float dot = baseX * x + baseY * y;
	float magA = sqrt(pow(baseX, 2) + pow(baseY, 2));
	float magB = sqrt(pow(x, 2) + pow(y, 2));

	float angle = acos(dot / (magA * magB)) * (180.0 / M_PI);
	return angle;
}

bool AliceApproach::approachObjects()
{
	movePan(panPosition);
	moveTilt(tiltPosition);

	ROS_INFO_STREAM("Aligning..");
	align(); // first align before calculating approach point
	ROS_INFO_STREAM("Aligned");

	movePan(panPosition);
	moveTilt(tiltPosition);

	d_receivedPlane = false;

	d_sub_pc = d_nh.subscribe<sensor_msgs::PointCloud2>("front_xtion/depth_registered/points", 1, &AliceApproach::cbCloud, this);

	ros::Rate r(10);

	while (d_receivedPlane = false)
		r.sleep();

	as.setSucceeded();
}

bool AliceApproach::approachPoint(float x, float y)
{
}

void AliceApproach::cbCloud(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	ROS_INFO_STREAM("Received cloud");
	tfTransformer.waitForTransform("/base_link", "/front_xtion_link", ros::Time::now(), ros::Duration(0.05));

	if (!tfTransformer.canTransform("/base_link", cloudMsg->header.frame_id, cloudMsg->header.stamp))
	{
		ROS_INFO_STREAM("No transform found");
		return;
	}

	ROS_INFO_STREAM("Transformed");

	PCLPointCloudPtr pointcloud(new PCLPointCloud);
	sensor_msgs::PointCloud2 cloudMsg_transformed;
	pcl_ros::transformPointCloud("/base_link", *cloudMsg, cloudMsg_transformed, tfTransformer);

	fromROSMsg(cloudMsg_transformed, *pointcloud);

	vector<int> ind;
	removeNaNFromPointCloud(*pointcloud, *pointcloud, ind);

	PassThrough<Point> passthrough_filter;
	passthrough_filter.setInputCloud(pointcloud);
	passthrough_filter.setFilterFieldName("z");
	passthrough_filter.setFilterLimits(0.50, 0.9);
	passthrough_filter.filter(*pointcloud);

	passthrough_filter.setInputCloud(pointcloud);
	passthrough_filter.setFilterFieldName("x");
	passthrough_filter.setFilterLimits(0.0, 1.2);
	passthrough_filter.filter(*pointcloud);

	ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	SACSegmentation<Point> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);  // 0.015

	seg.setInputCloud(pointcloud);
	seg.segment(*inliers, *coefficients);

	ExtractIndices<Point> extract;

	extract.setInputCloud(pointcloud);
	extract.setIndices(inliers);

	PCLPointCloudPtr plane(new PCLPointCloud);
	extract.setNegative(false);  // keep the planes
	extract.filter(*plane);

	extract.setNegative(true); // true means remove planes
	extract.filter(*pointcloud);

	vector<pcl::PointIndices> cluster;
	pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);

	tree->setInputCloud(pointcloud);
	EuclideanClusterExtraction<Point> ec;

	ec.setClusterTolerance(0.05);       // in meters
	ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
	ec.setMaxClusterSize(40000);      // Maximal points that must belong to a cluster
	ec.setSearchMethod(tree);
	ec.setInputCloud(pointcloud);
	ec.extract(cluster);

	if (cluster.size() == 0)
	{
		ROS_INFO_STREAM("Cannot find any clusters");
		return;
	}

	if (cluster.size() != 0)
	{
		ROS_INFO_STREAM("Clusters: " << cluster.size());
		float avgY = 0.0f;
		float avgX = 0.0f;
		size_t correctObjCount = 0;

		for (size_t idx = 0; idx < cluster.size(); ++idx)
		{
			PCLPointCloudPtr objectCloud(new PCLPointCloud);
			pcl::PointIndices point_indices = cluster.at(idx);

			foreach (int index, point_indices.indices)
			{
				Point p = pointcloud->points[index];
				objectCloud->points.push_back(p);
			}

			Point minPt, maxPt;
			Eigen::Matrix<float, 4, 1> cenPoint;
			getMinMax3D(*objectCloud, minPt, maxPt);

			if ( fabs(maxPt.z - minPt.z) <= 0.20f && fabs(maxPt.z - minPt.z) > 0.04) // objects are not higher then 20cm
			{
				compute3DCentroid(*objectCloud, cenPoint);
				ROS_INFO_STREAM("Point: " << cenPoint[0] << ", " << cenPoint[1]);
				ROS_INFO_STREAM("Min/max: " << minPt.x << ", " << maxPt.x << ", " << minPt.y <<", " << maxPt.y);

				avgX += cenPoint[0];
				avgY += cenPoint[1];
				++correctObjCount;
			}
		}

		avgX /= correctObjCount; // the middle X position
		avgY /= correctObjCount; // the middle Y position

		ROS_INFO_STREAM("avgX: " << avgX <<", avgY: " << avgY);

		float moveDistance = 0.55f;

		// vector right in front of base_link
		float baseX = 1.0f;
		float baseY = 0.0f;

		float x = avgX - moveDistance;
		float y = avgY;
		ROS_INFO_STREAM("goto point: " << x << ", " << y);
		float angle = getAngle(baseX, baseY, x, y);
		float distance = getDistance(x,y);

		float xObj = avgX;
		float yObj = 0.0f;
		float baseXr = -x;
		float baseYr = -y;

		float returnAngle = 180.0f - getAngle(baseXr, baseYr, xObj, yObj);

	//	process = false;

		if (y <= 0.0f) // point is on the right so rotate right
			angle *= -1;
		else
			returnAngle *= -1; // return rotation needs to be opposite rotation of first rotation

		turn(angle);
		move(distance);
		turn(returnAngle);

		d_sub_pc.shutdown(); // kill this subscriber
		d_receivedPlane = true;
	}

}


void AliceApproach::planesCallback(const alice_msgs::PcPlaneConstPtr &vPlanes)
{
	vector <PCLPointCloudPtr> vPlane;

	ROS_INFO_STREAM("Size of vector: " << vPlanes->vector.size());

	if (vPlanes->vector.size() == 0)
		return;

	for (size_t idx = 0; idx < vPlanes->vector.size(); ++idx)
	{
		PCLPointCloudPtr cloud(new PCLPointCloud);
		fromROSMsg(vPlanes->vector.at(idx), *cloud);
		vPlane.push_back(cloud);
	}

	if (vPlane.size() > 1)
	{
		size_t hSize = 0;
		size_t index = 0;

		// find largest plane
		for (size_t idx = 0; idx < vPlane.size(); ++idx)
		{
			if (vPlane.at(idx)->points.size() > hSize)
			{
				hSize = vPlane.at(idx)->points.size();
				index = idx;
			}
		}

		d_plane = vPlane.at(index);
	}
	else
		d_plane = vPlane.at(0);

	d_receivedPlane = true;
	d_sub_planes.shutdown();
}


