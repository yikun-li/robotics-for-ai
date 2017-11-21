#include "teleop.ih"
#include <geometry_msgs/Twist.h>
#include <turtle_actionlib/Velocity.h>
#include <iostream>

using namespace std;

namespace BORG
{
	void TeleOp::moveForward(float meter)
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
			ROS_INFO_STREAM("Something went wrong with moving");
	}

	void TeleOp::moveBackward(float meter)
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
			ROS_INFO_STREAM("Something went wrong with moving");
	}

    bool TeleOp::run()
    {
        if (!d_cmd_vel_pub)
        {
            ROS_ERROR("ROS Topic was not initialized properly. Make sure "
                      "it is of the correct type.");
            return false;
        }

        cout << "Control the robot by pressing WASD\n"
        		//"Press 'f' to start taking images\n"
                //"Press 'r' to move forward\n" 
                "Press 'q' to exit\n"
                "Press ' ' to stop the robot\n"
                "When capslock is on, the speed will not decrease automatically\n";

        geometry_msgs::Twist base_cmd;

        double speed = 0.0;
        double angle = 0.0;

        double speed_step = 0.02;
        double angle_step = 0.05;
        double max_angle = 0.5;
        double max_speed = 0.3;


        bool speed_locked = false;
        ros::Rate ticker(10);
        bool quit = false;
        std_msgs::Bool t;

        while (d_handler.ok() and not quit)
        {
            int cmd = getchar();

            // Empty buffer
            while (getchar() != -1)
                ;

            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
            switch (cmd)
            {
                case 'w':
                    speed_locked = false;
                    ROS_DEBUG("Increasing speed");
                    speed += speed_step;
                    break;
                case 'W':
                    speed_locked = true;
                    ROS_DEBUG("Increasing speed");
                    if (speed >= speed_step)
                        speed *= 1.1;
                    else if (speed <= -speed_step)
                        speed = 0;
                    else
                        speed = speed_step;
                    break;
                case 's':
                    speed_locked = false;
                    ROS_DEBUG("Decreasing speed");
                    speed -= speed_step;
                    break;
                case 'S':
                    speed_locked = true;
                    ROS_DEBUG("Decreasing speed");
                    if (speed <= -speed_step)
                        speed *= 1.1;
                    else if (speed >= speed_step)
                        speed = 0;
                    else
                        speed = -speed_step;
                    break;
                case 'a':
                case 'A':
                    ROS_DEBUG("Turning left");
                    angle += angle_step;
                    break;
                case 'd':
                case 'D':
                    ROS_DEBUG("Turning right");
                    angle -= angle_step;
                    break;
                case ' ':
                    ROS_INFO("Stopping");
                    speed = 0;
                    angle = 0;
                    break;
                case 'q':
                case 'Q':
                    speed = angle = 0;
                    quit = true;
                    ROS_INFO("Quitting");
                    break;
                case 'f':
                    t.data = true;
                	d_pubStartTakingPictures.publish(t); // start taking the images
                	break;
                case 'r':
                	moveForward();
                	break;
                case 'v':
                	moveBackward();
                	break;
                default:

                    // Decay the speed and angle to 0
                    if (not speed_locked)
                        speed *= 0.9;
                    angle *= 0.75;
        
                    // Make the 0 when they are really close
                    if (abs(speed) < 0.001)
                        speed = 0.0;
                    if (abs(angle) < 0.001)
                        angle = 0.0;
                    break;

                    ;
            }

     
            if (speed > max_speed)
                speed = max_speed;
            else if (speed < -max_speed)
                speed = -max_speed;
            else if (angle > max_angle)
                angle = max_angle;
            else if (angle < -max_angle)
                angle = -max_angle;
           

            base_cmd.linear.x = speed;
            base_cmd.angular.z = angle;
 
            d_cmd_vel_pub.publish(base_cmd);

            ticker.sleep();
        }
        return true;
    }
}
