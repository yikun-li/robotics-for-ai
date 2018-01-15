#ifndef _H_POINTCLOUDHANDLER
#define _H_POINTCLOUDHANDLER

#include <utils.h>

class PointCloudHandler
{
	actionlib::SimpleActionServer<alice_msgs::pointcloudfunctionAction> as;
	alice_msgs::pointcloudfunctionResult d_result;

	bool d_detectPlane;
	bool d_detectMultiPlane;
	bool d_detectEmptyShelf;
	bool d_detectObjects;

	NodeHandle d_nh;
	SACSegmentation<Point> seg;
	ExtractIndices<Point> extract;

	Subscriber d_pc_sub; // point cloud subscriber

	Publisher plane_pub;
	Publisher object_pub;
	Publisher v_plane_pub;

	bool d_done;

	tf::TransformListener tfTransformer;

	public:
		PointCloudHandler(ros::NodeHandle &n);

	private:
		void execute(const alice_msgs::pointcloudfunctionGoalConstPtr &goal);
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
		void shutdownSubcriber();
		bool getPlane(PCLPointCloudPtr &pointcloud, PCLPointCloudPtr &planeCloud);
		void reset();
		void transformPC(PCLPointCloudPtr &cloud, string transformTo);
};

#endif
