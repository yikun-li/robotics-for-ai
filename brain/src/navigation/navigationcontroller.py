import util.receivesocket
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.NavigationController').addHandler(util.nullhandler.NullHandler())

class NavigationController(object):
    """Manage navigation system(s)"""

    def __init__(self, param_dict):
        """initialize navigation systems and the controller"""
        self.logger = logging.getLogger('Borg.Brain.NavigationController')
        ### PARAMETERS: ###
        self.receive_sockets = []
        navigation_params = param_dict.get_section('navigation_controller')
        modules_string = param_dict.get_option('navigation_controller','modules')

        ### INITIALIZERS: ###
        self.connect_modules(modules_string, navigation_params)

    def connect_modules(self, modules_string, navigation_params):
        """Start all modules in the modulestring"""
        if modules_string != None:
            modules = modules_string.split()
            for module in modules:
                try:
                    port = int( navigation_params[module] )
                    self.receive_sockets.append( util.receivesocket.ReceiveSocket(port) )
                except Exception as e:
                    print "exception: ", e
                    print "It could be no socket was found for module `" + module + "'. Is it in the config file?"
                    raise e

    def update(self):
        """Get update from all navigation modules"""
        detections = []
        for socket in self.receive_sockets:
#            socket.set_blocking(0)
#            detections += socket.read()
            try:
                socket.set_blocking(0)
                detections += socket.read()
                #print "Nav Controller: ", detections
            except Exception as e:
                #print "exception: ", e, " args: ", e.args
                pass
        return detections


if __name__ == "__main__":
    pass
