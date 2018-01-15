/*
 * alicecontroller.h
 *
 *  Created on: Apr 3, 2015
 *      Author: rik
 */

#ifndef ALICECONTROLLER_H_
#define ALICECONTROLLER_H_

#include<utils.h>


class AliceController
{
	Publisher cmd_vel;
	Subscriber odom;

	NodeHandle d_nh;

	float currentX;
	float currentY;
	float currentAngle;

	actionlib::SimpleActionServer<alice_msgs::alicecontrollerfunctionAction> as;

	public:
		AliceController(ros::NodeHandle &n);

	private:
		void execute(const alice_msgs::alicecontrollerfunctionGoalConstPtr &goal);
		bool turn(float angle, float speed = 0.1); // turn angle degrees
		bool move(float meters, float speed = 0.1); // move meters either forward or back, with speed
		void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

};

#endif /* ALICECONTROLLER_H_ */
