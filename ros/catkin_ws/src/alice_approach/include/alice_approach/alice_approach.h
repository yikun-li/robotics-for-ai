/*
 * alice_approach.h
 *
 *  Created on: Apr 6, 2015
 *      Author: rik
 */

#ifndef ALICE_APPROACH_H_
#define ALICE_APPROACH_H_

#include <utils.h>

class AliceApproach
{
	NodeHandle d_nh;

	actionlib::SimpleActionServer<alice_msgs::aliceapproachAction> as;

	// client
	actionlib::SimpleActionClient<alice_msgs::pointcloudfunctionAction> alicePointcloudClient;
	actionlib::SimpleActionClient<alice_msgs::alicecontrollerfunctionAction> aliceControllerClient;

	Subscriber d_sub_planes;

	Subscriber d_subDynamixelTilt;

	PCLPointCloudPtr d_plane;

	bool d_headIsMoving;
	bool d_receivedPlane;

	Publisher d_tilt;

	std_msgs::Float64 tiltPosition;

	public:
		AliceApproach(ros::NodeHandle &n);
		void execute(const alice_msgs::aliceapproachGoalConstPtr &goal);

	private:
		bool approachPlane();
		bool approachPoint(float x, float y);
		void planesCallback(const alice_msgs::PcPlaneConstPtr &vPlanes);
		void calculatePoint(PCLPointCloudPtr &cloud, float &x, float &y);
		void align();
		float getAngle(float baseX, float baseY, float x, float y);
		void subscribe();
		float getDistance(float x, float y);
		void turn(float angle);
		void move(float meter);
		void tiltCallback(const dynamixel_msgs::JointStateConstPtr &state);
		void moveTilt(std_msgs::Float64 position);

};



#endif /* ALICE_APPROACH_H_ */
