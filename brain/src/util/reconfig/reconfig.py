import os
import sys
import rospy
import dynamic_reconfigure.client

import configurations

__all__ = [
    'set_config', 'current_config'
]


_initialized = False
_clients = {
    'move_base': None,
    'move_base/DWAPlannerROS': None,
    'move_base/global_costmap/inflation_layer': None,
    'move_base/local_costmap': None,
    'move_base/local_costmap/inflation_layer': None
}

current_config = configurations.DEFAULT

def set_config(configuration):
    global current_config
    if current_config == configuration:
        return
    for client in _clients:
        if client in configuration:
            if _clients[client] is None:
                _clients[client] = dynamic_reconfigure.client.Client(client)
            _clients[client].update_configuration(configuration[client])
    current_config = configuration
    os.system('rosservice call /move_base/clear_costmaps')


def main(args):
    config = configurations.DEFAULT
    if args[1:]:
        config = eval('configurations.{0}'.format(args[1]))
    set_config(config)
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
