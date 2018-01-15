#include <alice_approach/alice_approach.h>


float Degrees(float x)
{
	return (x * 180.0f / M_PI);
}

float Radian(float x)
{
	return (x * M_PI / 180.0f);
}
bool AliceApproach::approachSide()
{
	std_msgs::Float64 lookDownPosition;
	lookDownPosition.data = Radian(45);
	std_msgs::Float64 lookRightPosition;
	lookRightPosition.data = Radian(-125);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);

	AlignWithTableSidewaysBack();

	/*
	ros::Rate rate(1);
	// Assumption: We are looking in the general direction of P1R (Point 1 Robot)
	// P1R is not a grasping point, but the first point for alignment with the table
	// P1R is 0.34m away from P1 of table (Corner Point), in extensions of the side of the table P2

	// Find P1R
	// Use Camera to locate P1 (closest point to the robot)
	//Align towards P1R
	AlignP1R(panPosition, tiltPosition);

	// Drive towards P1R
	DriveToP1R();

	// Align with table
	AlignWithTable();

	rate.sleep();
	while (!AlignWithTableSideways())
		rate.sleep();
	// Drive towards P2R (First Sweep Point)

	rate.sleep();
	// Driving to the first location  (2 squares away, 40cm)

	DriveToP2R();
	rate.sleep();
	*/
}

void AliceApproach::MoveToP1R()
{
	AlignP1R(panPosition, tiltPosition);
	DriveToP1R();
	AlignWithTable();
}

void AliceApproach::MoveToP2R()
{
	ros::Rate rate(1);
	std_msgs::Float64 lookDownPosition;
	lookDownPosition.data = Radian(25);
	std_msgs::Float64 lookRightPosition;
	lookRightPosition.data = Radian(-45);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	subscribe();

	rate.sleep();

	while (!AlignWithTableSideways()) // first we do our alignment to make sure we are aligned properly
		rate.sleep();

	move(0.2, 0.05);

	subscribe();

	rate.sleep();
	while (!AlignWithTableSideways()) // first we do our alignment to make sure we are aligned properly
		rate.sleep();

	lookRightPosition.data = Radian(-120);
	lookDownPosition.data = Radian(45);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	Point projection1;
	projection1.x = -0.5;
	projection1.y = -0.5;

	subscribe();
	rate.sleep();
	Point pAvg;
	int count = 0;
	for (float idy = -0.5; idy > -0.8; idy -= 0.05)
	{
		++count;
		projection1.y = idy;
		Point p = FindClosestPoint(projection1, 50);
		pAvg.x += p.x;
	}

	pAvg.x /= count;

	ROS_INFO_STREAM("Table offset: " << pAvg.x);

	// Point x if 0.405m away from the table edge

	float distance = 0.37 + pAvg.x;

	ROS_INFO_STREAM("Drive distance: " << distance);
	move(distance , 0.05);

	lookRightPosition.data = Radian(-45);
	lookDownPosition.data = Radian(25);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	while (!AlignWithTableSideways())
		rate.sleep();

	lookRightPosition.data = Radian(-120);
	lookDownPosition.data = Radian(45);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	projection1.x = -0.5;
	projection1.y = -0.5;

	subscribe();
	rate.sleep();
	pAvg.x = 0;
	count = 0;
	for (float idy = -0.5; idy > -0.8; idy -= 0.05)
	{
		++count;
		projection1.y = idy;
		Point p = FindClosestPoint(projection1, 50);
		pAvg.x += p.x;
	}

	pAvg.x /= count;

	ROS_INFO_STREAM("Table offset: " << pAvg.x);

	// Point x if 0.405m away from the table edge

	distance = 0.37 + pAvg.x;

	ROS_INFO_STREAM("Drive distance: " << distance);
	move(distance , 0.05);

	lookRightPosition.data = Radian(-45);
	lookDownPosition.data = Radian(25);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	while (!AlignWithTableSideways())
		rate.sleep();



//	while (!AlignWithTableSideways()) // first we do our alignment to make sure we are aligned properly
//			rate.sleep();
//	rate.sleep();
//	DriveToP2R();

}

void AliceApproach::MoveToP3R()
{

	ros::Rate rate(1);
	std_msgs::Float64 lookDownPosition;
	lookDownPosition.data = Radian(25);
	std_msgs::Float64 lookRightPosition;
	lookRightPosition.data = Radian(-45);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	subscribe();
	rate.sleep();

	for (size_t idx = 0 ; idx < 5; ++idx)
	{
		while (!AlignWithTableSideways())
			rate.sleep();
		rate.sleep();
		move(0.1, 0.05);
		rate.sleep();
	}

	while (!AlignWithTableSideways())
		rate.sleep();

	// check how far we need to drive
	lookDownPosition.data = Radian(45);
	lookRightPosition.data = Radian(-55);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();
	subscribe();
	rate.sleep();


	Point projection1;
	projection1.x = 0.8;
	projection1.y = -0.5;

	Point pAvg;
	int count = 0;
	for (float idy = -0.5; idy > -0.8; idy -= 0.05)
	{
		++count;
		projection1.y = idy;
		Point p = FindClosestPoint(projection1, 50);
		pAvg.x += p.x;
	}

	pAvg.x /= count;

	ROS_INFO_STREAM("Table offset: " << pAvg.x);

	// Point x if 0.405m away from the table edge

	float prefDistance = 0.43f;
	float distance = pAvg.x - prefDistance;

	ROS_INFO_STREAM("Drive distance: " << distance);

	move(distance, 0.05);

	lookDownPosition.data = Radian(45);
	lookRightPosition.data = Radian(-125);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	while (!AlignWithTableSidewaysBack())
			rate.sleep();

	rate.sleep();
	// check how far we need to drive
	lookDownPosition.data = Radian(35);
	lookRightPosition.data = Radian(-35);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();
	rate.sleep();
	subscribe();
	rate.sleep();


	projection1.x = 0.8;
	projection1.y = -0.5;

	pAvg.x = 0;
	count = 0;
	for (float idy = -0.5; idy > -0.8; idy -= 0.05)
	{
		++count;
		projection1.y = idy;
		Point p = FindClosestPoint(projection1, 50);
		pAvg.x += p.x;
	}

	pAvg.x /= count;

	ROS_INFO_STREAM("Table offset: " << pAvg.x);

	// Point x if 0.405m away from the table edge

	distance = pAvg.x - prefDistance;

	ROS_INFO_STREAM("Drive distance: " << distance);

	move(distance, 0.05);

	lookDownPosition.data = Radian(45);
	lookRightPosition.data = Radian(-125);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();

	while (!AlignWithTableSidewaysBack())
			rate.sleep();

}


void AliceApproach::DriveToP2R()
{
	ros::Rate rate(1);
	move(0.1, 0.05);
	rate.sleep();
	while (!AlignWithTableSideways())
			rate.sleep();

	move(0.1, 0.05);
	rate.sleep();
	while (!AlignWithTableSideways())
			rate.sleep();
	move(0.1, 0.05);
	rate.sleep();
	while (!AlignWithTableSideways())
			rate.sleep();
	move(0.1, 0.05);
	rate.sleep();
	while (!AlignWithTableSideways())
		rate.sleep();
}

bool  AliceApproach::AlignWithTableSidewaysBack()
{
	subscribe(); // get new point cloud
	Point p1;
	p1.x = -0.2;

	Point p1t = FindClosestPoint(p1, 50);

	Point p2;
	p2.x = -0.5;

	Point p2t = FindClosestPoint(p2, 50);

	ROS_INFO_STREAM("P1: " << p1t.x << ", " << p1t.y);
	ROS_INFO_STREAM("P4: " << p2t.x << ", " << p2t.y);

	Point directionTable;
	directionTable.x = p2t.x + p1t.x;
	directionTable.y = p2t.y - p1t.y;

	Point forwardPoint;
	forwardPoint.x = -0.1f;
	forwardPoint.y = 0.0f;

	float lengthDR = sqrt((directionTable.x * directionTable.x) + (directionTable.y * directionTable.y)); // directionR.y = 0 but just in case it ever changes
	float lengthFP = sqrt((forwardPoint.x * forwardPoint.x) + (forwardPoint.y * forwardPoint.y));

	directionTable.x /= lengthDR;
	directionTable.y /= lengthDR;

	forwardPoint.x /= lengthFP;
	forwardPoint.y /= lengthFP;

	// vectors are now normalized, so calculate the angle between the two vectors
	float angle = Degrees(atan2(forwardPoint.x, forwardPoint.y)) - Degrees(atan2(directionTable.x, directionTable.y));
	//float angle = acos(directionR.x * p1r.x + directionR.y * p1r.y);
	ROS_INFO_STREAM("Angle: " << angle);

	if (fabs(angle) > 5.0f)
	{
		if (angle < 0)
			turn(-1.0);
		else
			turn(1.0);
		ROS_INFO_STREAM("ERROR ANGLE TO BIG: " << angle);
		return false;
	}

	turn(angle);
	return true;
}

bool  AliceApproach::AlignWithTableSideways()
{

	subscribe(); // get new point cloud
	Point p1;
	p1.x = 0.2;

	Point p1t = FindClosestPoint(p1);

	Point p2;
	p2.x = 0.5;

	Point p2t = FindClosestPoint(p2);


	ROS_INFO_STREAM("P1: " << p1t.x << ", " << p1t.y);
	ROS_INFO_STREAM("P4: " << p2t.x << ", " << p2t.y);

	Point directionTable;
	directionTable.x = p2t.x - p1t.x;
	directionTable.y = p2t.y - p1t.y;

	Point forwardPoint;
	forwardPoint.x = 0.1f;
	forwardPoint.y = 0.0f;

	float lengthDR = sqrt((directionTable.x * directionTable.x) + (directionTable.y * directionTable.y)); // directionR.y = 0 but just in case it ever changes
	float lengthFP = sqrt((forwardPoint.x * forwardPoint.x) + (forwardPoint.y * forwardPoint.y));

	directionTable.x /= lengthDR;
	directionTable.y /= lengthDR;

	forwardPoint.x /= lengthFP;
	forwardPoint.y /= lengthFP;

	// vectors are now normalized, so calculate the angle between the two vectors
	float angle = Degrees(atan2(forwardPoint.x, forwardPoint.y)) - Degrees(atan2(directionTable.x, directionTable.y));
	//float angle = acos(directionR.x * p1r.x + directionR.y * p1r.y);
	ROS_INFO_STREAM("Angle: " << angle);

	if (fabs(angle) > 5.0f)
	{
		if (angle < 0)
			turn(-1.0);
		else
			turn(1.0);
		ROS_INFO_STREAM("ERROR ANGLE TO BIG: " << angle);
		return false;
	}

	turn(angle);
	return true;
}

Point AliceApproach::FindClosestPoint(Point p, int k)
{
	int K = k;
	vector<int> knnSearch(K);
	vector<float> squaredDistance(K);
	Point r;
	float xP4 = 0.0f, yP4 = 0.0f, zP4 = 0.0f;

	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(d_plane);

	// take the average x,y values of the found points
	int foundPoints = kdtree.nearestKSearch(p, K, knnSearch, squaredDistance);

	if (foundPoints > 0)
	{
		for (size_t idx = 0; idx < foundPoints; ++idx)
		{
			xP4 += d_plane->points[knnSearch.at(idx)].x;
			yP4 += d_plane->points[knnSearch.at(idx)].y;
			zP4 += d_plane->points[knnSearch.at(idx)].z;
		}

		// the average x,y position of P1 (corner of table
		xP4 /= foundPoints;
		yP4 /= foundPoints;
		zP4 /= foundPoints;
	}

	r.x = xP4;
	r.y = yP4;
	r.z = zP4;

	return r;
}


void AliceApproach::AlignWithTable()
{
	ros::Rate rate(2);
	std_msgs::Float64 lookDownPosition;
	lookDownPosition.data = Radian(30);
	std_msgs::Float64 lookRightPosition;
	lookRightPosition.data = Radian(-20);
	movePan(lookRightPosition);
	moveTilt(lookDownPosition);
	rate.sleep();
	Point p1 = FindP1();
	Point p3 = FindP3(p1);

	ROS_INFO_STREAM("P1: " << p1.x << ", " << p1.y);
	ROS_INFO_STREAM("P3: " << p3.x << ", " << p3.y);

	Point directionTable;
	directionTable.x = p3.x - p1.x;
	directionTable.y = p3.y - p1.y;

	Point forwardPoint;
	forwardPoint.x = 0.1f;
	forwardPoint.y = 0.0f;

	float lengthDR = sqrt((directionTable.x * directionTable.x) + (directionTable.y * directionTable.y)); // directionR.y = 0 but just in case it ever changes
	float lengthFP = sqrt((forwardPoint.x * forwardPoint.x) + (forwardPoint.y * forwardPoint.y));

	directionTable.x /= lengthDR;
	directionTable.y /= lengthDR;

	forwardPoint.x /= lengthFP;
	forwardPoint.y /= lengthFP;

	// vectors are now normalized, so calculate the angle between the two vectors
	float angle = Degrees(atan2(forwardPoint.x, forwardPoint.y)) - Degrees(atan2(directionTable.x, directionTable.y));
	//float angle = acos(directionR.x * p1r.x + directionR.y * p1r.y);
	ROS_INFO_STREAM("Angle: " << angle);

	turn(angle);
}

void AliceApproach::DriveToP1R()
{
	// first reset the camera cause table might not be in good view right now
	std_msgs::Float64 lookRightPosition;
	lookRightPosition.data = Radian(-20);
	std_msgs::Float64 lookUpPosition;
	lookUpPosition.data = Radian(35);

	movePan(lookRightPosition);
	moveTilt(tiltPosition);

	Point p1r = FindP1R();
	ROS_INFO_STREAM("P1R: " << p1r.x << ", " << p1r.y);

	//movePan(panPosition);
	//moveTilt(tiltPosition);

	move(p1r.x / 2); // only move halfway

	// align again to check we are still on target
	AlignP1R(lookRightPosition, lookUpPosition);

	p1r = FindP1R();
	ROS_INFO_STREAM("P1R: " << p1r.x << ", " << p1r.y);

	move(p1r.x);

}

Point AliceApproach::FindP1()
{
	// Get the plane (table)
	subscribe();

	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(d_plane);

	// find closest point
	int K = 5;
	vector<int> knnSearch(K);
	vector<float> squaredDistance(K);

	Point basePoint;
	basePoint.x = 0.0f;
	basePoint.x = 0.0f;
	basePoint.z = 0.0f;

	float xP1 = 0.0f, yP1 = 0.0f, zP1 = 0.0f;

	// take the average x,y values of the found points
	int foundPoints = kdtree.nearestKSearch(basePoint, K, knnSearch, squaredDistance);

	if (foundPoints > 0)
	{
		for (size_t idx = 0; idx < foundPoints; ++idx)
		{
			xP1 += d_plane->points[knnSearch.at(idx)].x;
			yP1 += d_plane->points[knnSearch.at(idx)].y;
			zP1 += d_plane->points[knnSearch.at(idx)].z;
		}

		// the average x,y position of P1 (corner of table
		xP1 /= foundPoints;
		yP1 /= foundPoints;
		zP1 /= foundPoints;
	}

	Point p1;
	p1.x = xP1;
	p1.y = yP1;
	p1.z = zP1;

	return p1;
}



Point AliceApproach::FindP2(Point p1)
{
	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(d_plane);

	// Find P2
	// use a radius search to find all points within range
	float radius = 0.32f; // distance of P1R of table

	vector<int> radiusSearch;
	vector<float> squaredDistanceRadius;

	Point p2;

	if (kdtree.radiusSearch(p1, radius, radiusSearch, squaredDistanceRadius) > 0)
	{
		// find the point with lowest y value
		for (size_t idx = 0; idx < radiusSearch.size(); ++idx)
		{
			if (idx == 0)
			{
				p2.x = d_plane->points[radiusSearch.at(idx)].x;
				p2.y = d_plane->points[radiusSearch.at(idx)].y;
			}
			else
			{
				Point pTemp;
				pTemp.x = d_plane->points[radiusSearch.at(idx)].x;
				pTemp.y = d_plane->points[radiusSearch.at(idx)].y;

				if (pTemp.y <= p2.y) // smallest y
				{
					p2.x = pTemp.x;
					p2.y = pTemp.y;
				}
			}
		}
	}

	return p2;
}

Point AliceApproach::FindP3(Point p1)
{
	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(d_plane);

	// Find P3
	// use a radius search to find all points within range
	float radius = 0.1f; // This distance is not that important, set at 10cm to make radius a bit smaller

	vector<int> radiusSearch;
	vector<float> squaredDistanceRadius;

	Point p3;

	if (kdtree.radiusSearch(p1, radius, radiusSearch, squaredDistanceRadius) > 0)
	{
		// find the point with highest y value
		for (size_t idx = 0; idx < radiusSearch.size(); ++idx)
		{
			if (idx == 0)
			{
				p3.x = d_plane->points[radiusSearch.at(idx)].x;
				p3.y = d_plane->points[radiusSearch.at(idx)].y;
			}
			else
			{
				Point pTemp;
				pTemp.x = d_plane->points[radiusSearch.at(idx)].x;
				pTemp.y = d_plane->points[radiusSearch.at(idx)].y;

				if (pTemp.y >= p3.y) // highest y
				{
					p3.x = pTemp.x;
					p3.y = pTemp.y;
				}
			}
		}
	}

	return p3;
}

Point AliceApproach::FindP4(Point p1)
{
	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(d_plane);

	// Find P4
	// use a radius search to find all points within range
	float radius = 0.1f; // This distance is not that important, set at 10cm to make radius a bit smaller

	vector<int> radiusSearch;
	vector<float> squaredDistanceRadius;

	Point p4;

	if (kdtree.radiusSearch(p1, radius, radiusSearch, squaredDistanceRadius) > 0)
	{
		// find the point with highest y value
		for (size_t idx = 0; idx < radiusSearch.size(); ++idx)
		{
			if (idx == 0)
			{
				p4.x = d_plane->points[radiusSearch.at(idx)].x;
				p4.y = d_plane->points[radiusSearch.at(idx)].y;
			}
			else
			{
				Point pTemp;
				pTemp.x = d_plane->points[radiusSearch.at(idx)].x;
				pTemp.y = d_plane->points[radiusSearch.at(idx)].y;

				if (pTemp.x >= p4.x) // highest y
				{
					p4.x = pTemp.x;
					p4.y = pTemp.y;
				}
			}
		}
	}

	return p4;
}

Point AliceApproach::FindP1R()
{
	Point p1 = FindP1();
	ROS_INFO_STREAM("P1: " << p1.x << ", " << p1.y);
	Point p2 = FindP2(p1);
	ROS_INFO_STREAM("P2: " << p2.x << ", " << p2.y);
	// P2 should be the most right point on the table now
	// Get the first position for the robot
	Point p1r; // the first point where the robot should go to
	p1r.x = p1.x - (p2.x - p1.x);
	p1r.y = p1.y - (p2.y - p1.y);

	return p1r;
}

void AliceApproach::AlignP1R(std_msgs::Float64 pan, std_msgs::Float64 tilt)
{
	ros::Rate rate(2);
	movePan(pan);
	moveTilt(tilt);
	rate.sleep();
	// Find P1R
	Point p1r = FindP1R();

	ROS_INFO_STREAM("Point P1R Found: " << p1r.x << ", " << p1r.y);
	// Now we should be able to align with the point.

	// Get a vector in front of the robot
	Point directionR;
	directionR.x = 0.3f;
	directionR.y = 0.0f;

	float lengthDR = sqrt((directionR.x * directionR.x) + (directionR.y * directionR.y)); // directionR.y = 0 but just in case it ever changes
	float lengthP1R = sqrt((p1r.x * p1r.x) + (p1r.y * p1r.y));

	directionR.x /= lengthDR;
	directionR.y /= lengthDR;

	p1r.x /= lengthP1R;
	p1r.y /= lengthP1R;

	// vectors are now normalized, so calculate the angle between the two vectors
	float angle = Degrees(atan2(directionR.x, directionR.y) - atan2(p1r.x, p1r.y));
	//float angle = acos(directionR.x * p1r.x + directionR.y * p1r.y);
	ROS_INFO_STREAM("Angle: " << angle);
	// align to the point
	turn(angle);


}
