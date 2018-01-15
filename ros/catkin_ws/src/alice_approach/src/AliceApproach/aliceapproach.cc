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
	tiltPosition.data = 0.7;

	ROS_INFO_STREAM("Waiting for action servers");
	alicePointcloudClient.waitForServer();
	ROS_INFO_STREAM("Alice pointcloud action server connected");
	aliceControllerClient.waitForServer();
	ROS_INFO_STREAM("Alice controller action server connected");

	d_subDynamixelTilt = d_nh.subscribe<dynamixel_msgs::JointState>("/tilt_controller/state", 1, &AliceApproach::tiltCallback, this);
	d_tilt = d_nh.advertise<std_msgs::Float64>("tilt_controller/command", 1);
}

void AliceApproach::moveTilt(std_msgs::Float64 position)
{
	d_tilt.publish(position);
	usleep(50000); // might need a larger sleep?

	ros::Rate rate(20);

	while (d_headIsMoving)
		rate.sleep();

}

void AliceApproach::tiltCallback(const dynamixel_msgs::JointStateConstPtr &state)
{
	if (state->is_moving)
		d_headIsMoving = true;
	else
		d_headIsMoving = false;
}

void AliceApproach::execute(const alice_msgs::aliceapproachGoalConstPtr &goal)
{
	ROS_INFO_STREAM("This is it!");

	if (goal->plane == true)
		approachPlane();
	else
	{
		approachPoint(goal->x, goal->y);
	}
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
void AliceApproach::move(float meter)
{
	alice_msgs::alicecontrollerfunctionGoal goal;
	goal.function = "move";
	goal.meter = meter;

	aliceControllerClient.sendGoal(goal);

	aliceControllerClient.waitForResult(ros::Duration(10.0));

	if (aliceControllerClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		return;
	}
	else
		ROS_INFO_STREAM("Something went wrong with turning");
}

bool AliceApproach::approachPlane()
{
	moveTilt(tiltPosition);
	align(); // first align before calculating approach point

	moveTilt(tiltPosition);
	d_receivedPlane = false;
	subscribe();

	// calculate point
	float moveDistance = 0.55f;

	// vector right in front of base_link
	float baseX = 1.0f;
	float baseY = 0.0f;

	Point minPt, maxPt;
	getMinMax3D(*d_plane, minPt, maxPt);

	float x = minPt.x - moveDistance;
	float y = (maxPt.y + minPt.y) / 2;

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

bool AliceApproach::approachPoint(float x, float y)
{
	ROS_INFO_STREAM("THIS FUNCTION DOES NOT DO ANYTHING AT THE MOMENT");
}


void AliceApproach::planesCallback(const alice_msgs::PcPlaneConstPtr &vPlanes)
{
	vector <PCLPointCloudPtr> vPlane;

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


