#ifndef __INCLUDED_ABOT_TELEOP_H_
#define __INCLUDED_ABOT_TELEOP_H_

#include <ros/ros.h>
#include <string>
#include <termios.h>
#include <std_msgs/Bool.h>
#include <alice_msgs/alicecontrollerfunctionAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

namespace BORG
{
    class TeleOp
    {
        private:
            ros::NodeHandle d_handler;
            ros::Publisher d_cmd_vel_pub;
            ros::Publisher d_pubStartTakingPictures;
            termios d_start_tcflags;
            int d_start_fcflags;
            int d_terminalstate;
            bool d_turtlebot;

            actionlib::SimpleActionClient<alice_msgs::alicecontrollerfunctionAction> aliceControllerClient;

        public:
            TeleOp(ros::NodeHandle &nh, std::string const &topic);
            ~TeleOp();

            bool run();

        private:
            bool setupTerminal();
            bool restoreTerminal();
            void moveForward(float meter = 0.2f);
            void moveBackward(float meter = -0.2f);
    };
}
#endif
