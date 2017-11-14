import time
import logging
import termios, sys, os
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
    Give typed-in text input for your system in a separate window.
    """
    def __init__(self,
                 controller_ip,
                 controller_port,
                 update_frequency=5):
        """
        Constructor
        """
        self.logger = logging.getLogger("Borg.Brain.Vision.Remote")
        super(Remote, self).__init__(controller_ip, controller_port)

        # Store configuration
        self.__ticker = Ticker(frequency=update_frequency)
        self.__last_print = 0 
        
        self.given_input = ""
        print "Type in your command in this window. \n Take care to not press two keys in the same fifth of a second, as the press might be missed."


    def __del__(self):
        self.stop()

    def stop(self):
        self.disconnect()

    def train(self):
        pass

    def run(self):
        """start checking for typed characters, then sends them."""
        while self.is_connected():
            self.__ticker.tick() # Tick (sleep)

            c = getkey() 
            if c:
                if c == '\n':
                    self.add_property("name", "text_command")
                    self.add_property("text_command", self.given_input)
                    print ""
                    print ("Command read: " + self.given_input + "\n")
                    self.given_input = ""
                else:
                    self.given_input += c
            
            ############################
            # Send data
            self.update()


if __name__ == "__main__":
    logging.getLogger('Borg.Brain').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)

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
