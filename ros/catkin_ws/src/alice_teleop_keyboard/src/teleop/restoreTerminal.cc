#include <teleop_keyboard/teleop.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>

using namespace std;

namespace BORG
{
    bool TeleOp::restoreTerminal()
    {
        int const fd = fileno(stdin);

        // Restore terminal
        if (d_terminalstate & 1)
        {
            if (tcsetattr(fd, TCSANOW, &d_start_tcflags) < 0) 
                ROS_WARN("Unable to restore terminal settings");
            else
            {
                ROS_DEBUG("Restored terminal settings");
                d_terminalstate ^= 1;
            }
        }

        // Restore FD state
        if (d_terminalstate & 2)
        {
            if (fcntl(fd, F_SETFL, d_start_fcflags) < 0) 
                ROS_WARN("Unable to restore stdin FD settings");
            else
            {
                ROS_DEBUG("Restored stdin FD settings");
                d_terminalstate ^= 2;
            }
        }

        return true;
    }
}
