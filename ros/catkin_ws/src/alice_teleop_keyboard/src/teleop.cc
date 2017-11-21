#include <iostream>
#include <teleop_keyboard/teleop.h>
#include <ros/ros.h>

using namespace std;
using namespace BORG;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_keyboard"); 
    ros::NodeHandle nh; 

    //string topic = "/RosAria/cmd_vel";
    string topic = "/cmd_vel";

    ROS_INFO("Publishing commands to topic %s", topic.c_str());

    ROS_INFO("Topic message type: geometry_msgs/Twist");

    TeleOp teleop(nh, topic);
    teleop.run();
}
