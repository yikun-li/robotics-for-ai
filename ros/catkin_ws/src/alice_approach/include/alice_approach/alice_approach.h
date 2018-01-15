/*
 * alice_approach.h
 *
 *  Created on: Apr 6, 2015
 *      Author: rik
 */

#ifndef ALICE_APPROACH_H_
#define ALICE_APPROACH_H_

#include <utils.h>
#include <cmath>

class AliceApproach
{
	NodeHandle d_nh;

	actionlib::SimpleActionServer<alice_msgs::aliceapproachAction> as;

	// client
	actionlib::SimpleActionClient<alice_msgs::pointcloudfunctionAction> alicePointcloudClient;
	actionlib::SimpleActionClient<alice_msgs::alicecontrollerfunctionAction> aliceControllerClient;

	Subscriber d_sub_planes;
	Subscriber d_sub_pc;

	Subscriber d_subDynamixelTilt;
	Subscriber d_subDynamixelPan;

	PCLPointCloudPtr d_plane;

	bool d_panIsMoving;
	bool d_tiltIsMoving;
	bool d_receivedPlane;

	Publisher d_tilt;
	Publisher d_pan;

	std_msgs::Float64 tiltPosition;
	std_msgs::Float64 panPosition;

	tf::TransformListener tfTransformer;

	public:
		AliceApproach(ros::NodeHandle &n);
		void execute(const alice_msgs::aliceapproachGoalConstPtr &goal);

	private:
		bool approachPlane();
		bool approachSide();
		bool calc2DHull();
		bool approachPoint(float x, float y);
		bool approachObjects();
		void planesCallback(const alice_msgs::PcPlaneConstPtr &vPlanes);
		void cbCloud(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
		void calculatePoint(PCLPointCloudPtr &cloud, float &x, float &y);
		void align();
		float getAngle(float baseX, float baseY, float x, float y);
		void subscribe();
		float getDistance(float x, float y);
		void turn(float angle);
		void move(float meter, float speed = 0);
		void tiltCallback(const dynamixel_msgs::JointStateConstPtr &state);
		void panCallback(const dynamixel_msgs::JointStateConstPtr &state);
		void moveTilt(std_msgs::Float64 position);
		void movePan(std_msgs::Float64 position);
		void AlignP1R(std_msgs::Float64 pan, std_msgs::Float64 tilt);
		void AlignWithTableSide();
		Point FindP1R();
		Point FindP1();
		Point FindP2(Point p1);
		Point FindP3(Point p1);
		Point FindP4(Point p1);
		Point FindClosestPoint(Point p, int k = 10);
		void DriveToP1R();
		void AlignWithTable();
		void DriveToP2R();
		bool AlignWithTableSideways();
		bool AlignWithTableSidewaysBack();
		void MoveToP1R();
		void MoveToP2R();
		void MoveToP3R();

};



#endif /* ALICE_APPROACH_H_ */
