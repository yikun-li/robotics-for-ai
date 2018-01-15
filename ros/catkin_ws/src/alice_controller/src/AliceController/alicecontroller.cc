/*
 * alicecontroller.cc
 *
 *  Created on: Apr 3, 2015
 *      Author: rik
 */

#include<alice_controller/alicecontroller.h>

AliceController::AliceController(NodeHandle &n) :
	as(n, "alicecontroller", boost::bind(&AliceController::execute, this, _1), false)
{
	d_nh = n;
	as.start();
	odom = d_nh.subscribe<nav_msgs::Odometry>("odom", 1, &AliceController::odomCB, this);
	cmd_vel = d_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void AliceController::execute(const alice_msgs::alicecontrollerfunctionGoalConstPtr &goal)
{
	if (goal->function == "move")
	{
		float meter = goal->meter;

		float speed = goal->speed;

		if (speed == 0)
			speed = 0.1;

		if (move(meter, speed))
			as.setSucceeded();
	}

	if (goal->function == "turn")
	{
		if (turn(goal->angle))
			as.setSucceeded();
	}
}

void AliceController::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentX = msg->pose.pose.position.x; // x position
    currentY = msg->pose.pose.position.y; // y position
    double w = msg->pose.pose.orientation.w; // for the rotation

    currentAngle = 2 * acos(w) * (180.0f / M_PI); // the current rotation angle in degrees
//    ROS_INFO_STREAM("current angle: " << currentAngle * (180.0f / M_PI));
}


bool AliceController::move(float meter, float speed)
{
	float startX = currentX;
	float startY = currentY;

	geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.angular.z = 0; // init at 0 to be sure

	if (meter < 0)
		speed = -1 * speed;

	base_cmd.linear.x = speed;
	cmd_vel.publish(base_cmd);

	float distance = 0.0f;

	ros::Rate rate(500);

	if (meter < 0)
		meter *= -1;

	while (distance <= meter) // keep driving until distance is enough meters
	{
		distance = fabs(sqrt(pow(currentX - startX, 2) + pow(currentY - startY,2)));
		cmd_vel.publish(base_cmd);
		ros::spinOnce();
		rate.sleep();
	}

	// done stop driving
	base_cmd.linear.x = 0.0f;
	cmd_vel.publish(base_cmd);

	return true; // alway return true for now
}

bool AliceController::turn(float angle, float speed)
{
	geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.angular.z = 0; // init at 0 to be sure

	float turnSpeed = speed; // turn speed

	if (angle < 0) // turn right, else just turn left
		turnSpeed *= -1.0f;

	float beginAngle = currentAngle;
	base_cmd.angular.z = turnSpeed;

	ros::Rate rate(500);
	float distance = 0.0f;
	float prevAngle = 0.0f;
	bool first = true;

	while (distance < fabs(angle))
	{
		float angleStep = 0.0f;

		if (first)
			angleStep = 0.0f;
		else
			angleStep = fabs(currentAngle - prevAngle);
		first = false;
		prevAngle = currentAngle;
		distance += angleStep;

	    ROS_INFO_STREAM("TURNING");
		cmd_vel.publish(base_cmd);

		ros::spinOnce();
		rate.sleep();
	}

	// stop turning
	base_cmd.angular.z = 0.0f;
	cmd_vel.publish(base_cmd);
	ROS_INFO_STREAM(" ");

	return true;  // alway return true for now
}





