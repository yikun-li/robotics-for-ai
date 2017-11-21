#include <teleop_keyboard/teleop.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>

namespace BORG
{
    bool TeleOp::setupTerminal()
    {
        // Obtain current terminal flags
        int const fd = fileno(stdin);
        termios tcflags;
        if (tcgetattr(fd, &tcflags) < 0)
        {
            ROS_WARN("Unable to obtain current terminal settings");
            return false;
        }

        // Store start terminal flags
        d_start_tcflags = tcflags;

        // Disable line buffering
        tcflags.c_lflag &= ~ICANON; 
        // Disable input echo
        tcflags.c_lflag &= ~ECHO; 
        if (tcsetattr(fd, TCSANOW, &tcflags) < 0) 
        {
            ROS_WARN("Unable to set new terminal settings");
            return false;
        }
        ROS_DEBUG("Disabled line buffering and echo on terminal");
        d_terminalstate |= 1;

        // Obtain current flags
        int const fcflags = fcntl(fd, F_GETFL);
        if (fcflags < 0)
        {
            ROS_WARN("Unable to obtain current FD settings");
            return false;
        }

        // Store start FD flags
        d_start_fcflags = fcflags;

        // Update flags to set FD to non-blocking mode
        if (fcntl(fd, F_SETFL, fcflags | O_NONBLOCK) < 0) 
        {
            ROS_WARN("Cannot set FD to non-blocking");
            return false;
        }
        ROS_DEBUG("Set stdin to non-blocking mode");
        d_terminalstate |= 2;

        return true;
    }
}
