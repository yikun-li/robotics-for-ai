# brain.py is the main file where the initializations reside

# General imports
import time
import sys
import getopt
import logging
import os
from string import upper

# Brain imports
import sensorintegrator
import basebehavior.behaviorcontroller
import configparse
import body.bodycontroller
import util.nullhandler
import util.loggingextra
from util.ticker import Ticker

logging.getLogger('Borg.Brain.Brain').addHandler(util.nullhandler.NullHandler())

class Brain(object):
    """
    Main class that starts the sensors and the behaviors and connects them
    togethers
    """

    def __init__(self, param_dict):
        """
        Initialize brain class: create sensorintegrator, behaviorcontroller and bodycontroller
        """
        speed = int(param_dict.get_option('general', 'brain_speed', 10))
        self.logger = logging.getLogger('Borg.Brain.Brain')
        self.ticker = Ticker(frequency=speed, name="Brain", ignore_delay=True)
        starting_behavior = param_dict.get_option('general', 'starting_behavior')
        self.sensorintegrator = sensorintegrator.SensorIntegrator(param_dict)
        self.logger.info("Sensor Integrator initialiazed")
        self.behaviorcontroller = basebehavior.behaviorcontroller.BehaviorController(starting_behavior)
        self.logger.info("Behavior Controller initialized")
        self.set_brain_speed(speed)
        self.bodycontroller = body.bodycontroller.BodyController()
        self.bodycontroller.set_config(param_dict)
        self.logger.info("Body controller initialized")
        self.running = False

    def set_brain_speed(self, Hertz = 10):
        """
        The speed of the brain is measured in Hertz
        """
        self.ticker.set_frequency(Hertz)
        self.brain_speed = Hertz

    def get_brain_speed(self):
        """
        Return the speed of the brain measured in Hertz
        """
        return self.brain_speed

    def start(self):
        """
        This starts the robot brain.
        This method needs to be threaded otherwise we cannot stop the brain
        """
        self.running = True
        self.logger.info("Running brain")

        sleep_func = True

        if self.bodycontroller.get_nr_pioneers():
            if body.bodycontroller.pioneer.ROS_ENABLED:
                self.logger.info("ROS enabled in body controller. " \
                                 "Using ros.sleep instead of time.sleep")
                sleep_func = body.bodycontroller.pioneer.rospy.sleep

        while self.running == True:
            self.ticker.tick(sleep_func)
            memory = self.sensorintegrator.update()
            try:
                self.behaviorcontroller.update()
            except basebehavior.behaviorcontroller.BehaviorFinished:
                self.logger.warn("All behaviors have finished or stopped. Stopping brain")
                self.stop()
            self.bodycontroller.update()

    def stop(self, signum=None, frame=None):
        """
        the stop function only works if start is threaded
        """
        if self.running:
            self.running = False
            self.sensorintegrator.stop()
            self.bodycontroller.stop()

def parse_args(sysargs):
    """
    Parse command line arguments
    """
    optlist, args = getopt.getopt(sysargs[1:], 'hb:', 
        ['nao_ip=','no_nao','pioneer_ip=','no_pioneer','nao_port=','pioneer_port=',
        'kinect_ip=','communicator_port=','speech_port=','starting_behavior=','brain_speed=',
        'log=','log-level=','log-file=','log-format=','help','profile'])
    print optlist, args
    option_dict = configparse.ParameterDict()
    logger = None
    logger_format = None
    logger_filename = None
    logger_level = None
    for opt, value in optlist:
        if opt in ("-h", "--help"):
            print_usage()
            sys.exit()
        elif opt in ("--nao_ip"):
            option_dict.add_option('body', 'number_of_naos', 1)
            option_dict.add_option('body', 'nao_ip_0', value)
        elif opt in ("--no_nao"):
            option_dict.add_option('body', 'number_of_naos', 0)
        elif opt in ("--pioneer_ip"):
            option_dict.add_option('body', 'number_of_pioneers', 1)
            option_dict.add_option('body', 'pioneer_ip_0', value)
        elif opt in ("--no_pioneer"):
            option_dict.add_option('body', 'number_of_pioneers', 0)
        elif opt in ("--nao_port"):
            option_dict.add_option('body', 'nao_port_0', value)
        elif opt in ("--pioneer_port"):
            option_dict.add_option('body', 'pioneer_port_0', value)
        elif opt in ("--kinect_ip"):
            option_dict.add_option('body', 'kinect_ip', value)
        elif opt in ("--communicator_port"):
            option_dict.add_option('vision_controller', 'communicator_port', value)
        elif opt in ("--speech_port"):
            option_dict.add_option('speech_controller', 'speech_port', value)
        elif opt in ("-b", "--starting_behavior"):
            option_dict.add_option('general', 'starting_behavior', value)
        elif opt in ("--brain_speed"):
            option_dict.add_option('general', 'brain_speed', value)
        elif opt in ("--log"):
            logger = logging.getLogger(value)
        elif opt in ("--log-level"):
            logger_level = value
        elif opt in ("--log-file"):
            logger_filename = value
        elif opt in ("--log-format"):
            logger_format = value
        elif opt in ("--profile"):
            global profile
            profile = True
        else:
            print_usage("unhandled option: " + opt + " " + value)


    setup_logging(logger, logger_format, logger_filename, logger_level)
    return option_dict, args

def setup_logging(logger, logFormat, filename, level):
    if not logger:
        return

    if filename:
        logger.addHandler(util.loggingextra.FileOutput(filename, format=logFormat))
    else:
        logger.addHandler(util.loggingextra.ScreenOutput(logFormat))
    
    if not level:   
        level = "WARNING"

    level = upper(level)
    if level in ("DEBUG", "DBG", "5"):
        logger.setLevel(logging.DEBUG)
    elif level in ("INFO", "INF", "4"):
        logger.setLevel(logging.INFO)
    elif level in ("WARNING", "WARN", "3"):
        logger.setLevel(logging.WARN)
    elif level in ("ERROR", "ERR", "2"):
        logger.setLevel(logging.ERROR)
    elif level in ("CRITICAL", "CRIT", "1"):
        logger.setLevel(logging.CRITICAL)
    else:
        logger.setLevel(logging.INFO)

def check_options(option_dict):
    """
    Check whether required options are set (either on commandline or
    in configuration file).
    Throws an exception if one or more configuration options is invalid.
    """
    #validate nao configuration:
    number_of_naos = option_dict.get_option('body', 'number_of_naos')
    if number_of_naos == None:
        raise Exception("The number_of_naos is not specified!")
    number_of_naos = int(number_of_naos)
    if (number_of_naos < 0) or (number_of_naos > 10):
        raise Exception("Invalid number_of_naos specified!")
    for i in range(number_of_naos):
        if option_dict.get_option('body', 'nao_ip_%d' % i) == None:
            raise Exception("Option nao_ip_%d is not specified!" % d)
        if option_dict.get_option('body', 'nao_port_%d' % i) == None:
            raise Exception("Option nao_port_%d is not specified!" % d)

    #validate pioneer configuration:
    number_of_pioneers = option_dict.get_option('body', 'number_of_pioneers')
    if number_of_pioneers == None:
        raise Exception("The number_of_pioneers is not specified!")
    number_of_pioneers = int(number_of_pioneers)
    if (number_of_pioneers < 0) or (number_of_pioneers > 10):
        raise Exception("Invalid number_of_pioneers specified!")
    for i in range(number_of_pioneers):
        if option_dict.get_option('body', 'pioneer_ip_%d' % i) == None:
            raise Exception("Option pioneer_ip_%d is not specified!" % d)
        if option_dict.get_option('body', 'pioneer_port_%d' % i) == None:
            raise Exception("Option pioneer_port_%d is not specified!" % d)

def replace_placeholders(option_dict):
    """
    This method replaces occurences of <nao_ip>, <nao_port>, <pioneer_ip> and
    <pioneer_port> with the correct values, obtained by looking in the body
    section of the option dictionary.
    """
    pioneer_ip = option_dict.get_option("body", "pioneer_ip_0") 
    pioneer_port = option_dict.get_option("body", "pioneer_port_0") 
    nao_ip = option_dict.get_option("body", "nao_ip_0") 
    nao_port = option_dict.get_option("body", "nao_port_0")
    kinect_ip = option_dict.get_option("body", "kinect_ip")

    opts = option_dict.option_dict
    for section, settings in opts.iteritems():
        for setting, value in settings.iteritems():
            if type(value) == type(""):
                if pioneer_port:
                    value = value.replace("<pioneer_port>", pioneer_port)
                if pioneer_ip:
                    value = value.replace("<pioneer_ip>", pioneer_ip)
                if nao_port:
                    value = value.replace("<nao_port>", nao_port)
                if nao_ip:
                    value = value.replace("<nao_ip>", nao_ip)
                if kinect_ip:
                    value = value.replace("<kinect_ip>", kinect_ip)
            opts[section][setting] = value

def load_config(sysargs):
    """
    Load configuration file and combine this with command line arguments
    """
    if len(sysargs) < 2:
        print_usage("Specification of robot IP and starting behavior is mandatory!")
        sys.exit()
    option_dict, args = parse_args(sysargs)
    if len(args) >= 1:
        config_file = args[0]
        configparse.parse_config(config_file, option_dict) #does not overwrite existing arguments in option_dict
    try:
        check_options(option_dict)
    except Exception as e:
        print_usage("Failed to load valid configuration!")
        print e
        sys.exit()

    replace_placeholders(option_dict)
    return option_dict

def print_usage(message=None):
    """
    Print usage information
    """
    if message:
        print message
        print ""

    print "Usage: ", sys.argv[0], " [options] [conf_file]"
    print ""
    print "It is mandatory to specify starting_behavior, either"
    print "in the configuration file conf_file or as command-line options"
    print ""
    print "Options:"
    print "-h, --help                       Display this message then exit"
    print "--nao_ip=IP                      Set ip of the nao to be used to IP"
    print "                                 (and set number of nao's to 1)"
    print "--nao_port=PORT                  Set port of the nao to be used to PORT"
    print "--pioneer_ip=IP                  Set the ip of the pioneer to be used to IP"
    print "                                 (and set number of pioneers to 1)"
    print "--pioneer_port=PORT              Set the port of the pioneer to be used to PORT"
    print "--kinect_ip=IP                   Set the Obstacle Avoidance Kinect to be connected to IP"
    print "                                 (used to replace <kinect_ip> placeholder in config file)"
    print "--communicator_port=PORT         Set the port to be used to connect to communicators to PORT"
    print "--speech_port=PORT               Set the port to be used to connect to speech.jar to PORT"
    print "-b BEH, --starting_behavior=BEH  Set starting behavior to BEH"
    print "--brain_speed=SPEED              Set brain speed in Herz (default=10)"
    print "--log=NAME                       Log the given module (eg Borg.Brain)"
    print "--log-level=LEVEL                DEBUG, INFO, WARN, ERROR or CRITICAL"
    print "--log-file=FILENAME              The file to log to"
    print "--log-format=FORMAT              The format of the logging output"
    print "--profile                        Enable profiling of the brain"

if __name__ == "__main__":
    """
    Start the brain with command line arguments and options from the
    configuration file
    """
    print "arguments: ", sys.argv
    # Set this to True to enable profiling output of the brain, or use
    # --profile command line argument
    profile = False

    parameter_dict = load_config(sys.argv)
    brain = Brain(parameter_dict)
    import signal
    signal.signal(signal.SIGTERM, brain.stop)
    signal.signal(signal.SIGINT, brain.stop)

    try:
        if profile:
            import cProfile, pstats
            cProfile.run('brain.start()', 'brain.prof')
        else:
            brain.start()
    finally:
        brain.stop()
        if profile:
            p = pstats.Stats('brain.prof')
            p.strip_dirs().sort_stats('time').print_stats(30)
            os.unlink('brain.prof')

