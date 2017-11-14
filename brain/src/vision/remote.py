import sys, os
import time
import logging
import termios
import threading

### Import local dependencies
import configparse
import util.nullhandler
import util.loggingextra as loggingextra
from util.ticker import Ticker
from abstractvisionmodule import AbstractVisionModule as avm

logging.getLogger('Borg.Brain.Vision.Remote').addHandler(util.nullhandler.NullHandler())

TERMIOS = termios
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 0
    new[6][TERMIOS.VTIME] = 1

    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c
     

class Remote(avm):
    """
    Remote control the pioneer / scanner
    """
    def __init__(self,
                 controller_ip,
                 controller_port,
                 update_frequency=5,
                 location_file=False,
                 use_pcd=False):
        """
        Constructor
        """
        self.logger = logging.getLogger("Borg.Brain.Vision.Remote")
        super(Remote, self).__init__(controller_ip, controller_port)

        # Store configuration
        self.__ticker = Ticker(frequency=update_frequency)
        self.last_odo = None
        if not location_file:
            self.location_file = os.environ['BORG'] + "/Brain/data/locations2/arena_istanbul.dat"
        else:
            self.location_file = location_file

        self.file_lock = threading.Lock()
        self.process = None
        self.__last_print = 0 

    def __del__(self):
        self.stop()

    def stop(self):
        """
        When stopping the module, make sure the PCD Writer and
        SLAM6D handlers also stop
        """
        self.disconnect()

    def train(self):
        pass

    def run(self):
        """start loop which gets a new image, then processes it"""
        while self.is_connected():
            self.__ticker.tick() # Tick (sleep)

            if self.process and self.process.is_alive():
                self.update()
                continue

            c = getkey() 
            if c:
                if c == 'w':
                    print "Moving forward"
                    self.add_property("name", "pioneer_command")
                    self.add_property("pioneer_command", "forward")
                elif c == 'a':
                    print "Turning left"
                    self.add_property("name", "pioneer_command")
                    self.add_property("pioneer_command", "left")
                elif c == 'd':
                    print "Turning right"
                    self.add_property("name", "pioneer_command")
                    self.add_property("pioneer_command", "right")
                elif c == 's':
                    print "Moving backward"
                    self.add_property("name", "pioneer_command")
                    self.add_property("pioneer_command", "backward")
                elif c == 'q':
                    self.add_property("name", "pioneer_command")
                    self.add_property("pioneer_command", "stop")
                elif c == 'r':
                    self.add_property("name", "pioneer_command")
                    self.add_property("pioneer_command", "terminate")
                elif c == 't':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "make_scan")
                elif c == 'm':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "save_map")
                elif c == 'l':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "load_map")
                elif c == 'c':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "match_map")
                elif c == 'x':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "make_map")
                elif c == 'z':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "discard_map")
                elif c == 'p':
                    self.save_pose()
                elif c == 'g':
                    self.add_property("name", "map_command")
                    self.add_property("map_command", "toggle_autoscan")
                elif c == 'h':
                    print "[w] = forward    [a] = left         [s] = backward           [d] = right"
                    print "[q] = stop       [t] = take scan    [c] = matching mode      [x] = mapping mode"
                    print "[m] = save map   [l] = load map     [p] = save current pose  [g] = Toggle autoscanning"
            
            ############################
            # Send data
            self.update()

    def handle_custom_commands(self, entry):
        if entry['command'] == "odometry":
            odo = entry['params'][1]
            if odo != self.last_odo:
                if self.__last_print < time.time() - 2:
                    print "Current position: %s" % repr(odo)
                    self.__last_print = time.time()
                self.last_odo = odo

    def save_pose(self):
        if self.process:
            if self.process.is_alive():
                return
            else:
                self.process = None
        odo = self.last_odo
        self.process = threading.Thread(target=do_save_pose, args=(odo, self.location_file, self.file_lock))
        self.process.start()

def do_save_pose(odo, location_file, file_lock):
    print "Enter name of location"
    name = ""
    while True:
        c = getkey() 
        if not c:
            continue
        if ord(c) == 127:
            name = name[:-1]
        if c == "\n":
            break
        name += c
    print "Saving location %s with name %s to file %s" % (repr(odo), name, location_file)
    with file_lock:
        if not os.path.exists(location_file):
            fp = open(location_file, "w")
            fp.write("name,x,y,angle\n")
        else:
            fp = open(location_file, "a")
        fp.write("%s,%f,%f,%f\n" % (name, odo['x'], odo['y'], odo['angle']))
        fp.close()
    print "File saved"

if __name__ == "__main__":
    logging.getLogger('Borg.Brain').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)

    #PARSE COMMAND LINE ARGUMENTS
    section = "remote" # in config_dict
    arguments = sys.argv[1:]
    config_dict = configparse.parse_args_to_param_dict(arguments, section)

    #READ PARAMETERS:
    controller_ip    = config_dict.get_option(section,'host')
    controller_port  = config_dict.get_option(section,'port')

    #START:
    remote = Remote(controller_ip, controller_port)
    remote.connect()
    remote.train()
    try:
        remote.run()
    finally:
        remote.stop()
