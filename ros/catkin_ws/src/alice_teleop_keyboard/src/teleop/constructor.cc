#include "teleop.ih"
#include <geometry_msgs/Twist.h>
#include <string>

using namespace std;

namespace BORG
{
    TeleOp::TeleOp(ros::NodeHandle &nh, string const &topic)
    :
        d_handler(nh),
        d_terminalstate(0),
		d_pubStartTakingPictures(nh.advertise<std_msgs::Bool>("/headSpinner/start", 1)),
		aliceControllerClient("alicecontroller", true)
    {
        d_cmd_vel_pub = d_handler.advertise<geometry_msgs::Twist>(topic, 1);

   //     aliceControllerClient.waitForServer();
   //	ROS_INFO_STREAM("Alice controller action server connected");
        setupTerminal();
    }
}
