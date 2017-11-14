#!/usr/bin/env python
## @package Communicator
#  Documentation for the communicator.
#
#This document describes how the communicator works. The communicator
#runs on the laptops which are not the Brain. Communicator on each laptop waits for connections and request from vision controller. The vision controller will ask
#the communicator to start/stop/kill a module with a set of parameters. For starting a process, the communicator will search through the folders to find the 
#specified module. If it finds it, it will start the module with the parameters it got from the vision controller and will tell the module to connect to the port
#which vision controller specified

import subprocess
import os
import sys
import logging
import signal
import threading
import time
import cv
import multiprocessing

import util.threadedsocket
import configparse
import util.nullhandler
import util.loggingextra
from util.ticker import Ticker
import util.iopoll as iopoll
import videomanager

#Used to compare to ros image topics:
import re
ros_pattern = re.compile("ros_")

logging.getLogger('Borg.Brain.Communicator').addHandler(util.nullhandler.NullHandler())
logging.getLogger('Borg.Brain.Communicator.Listener').addHandler(util.nullhandler.NullHandler())
logging.getLogger('Borg.Brain.Communicator.ConnectionHandler').addHandler(util.nullhandler.NullHandler())

# Set this to True to enable profiling output for the communicator,
# or use the command line --profile option
profile = False

class Listener(util.threadedsocket.ThreadedSocket):
    """This class listens on a specific port to handle connections"""

    def __init__(self, port, communicator):
        """Start up a listening socket on a specific port and register it with the controller"""
        self.logger = logging.getLogger("Borg.Brain.Communicator.Listener")
        super(Listener, self).__init__('', port, giveup=-1, retry_timeout=1, server=True, logger=self.logger)

        self.communicator = communicator
        self.port = port
        self.running = True

    def handle_accept(self, sock, address):
        if sock:
            handler = ConnectionHandler(sock, address, self.communicator)
            handler.start()

class ConnectionHandler(util.threadedsocket.ThreadedSocket):
    """This class handles an incoming connection"""

    def __init__(self, socket, address, communicator, timeout=10):
        """Initialize the vision module connection handler"""
        if address:
            self.host, self.port = address
            self.address = address
        else:
            self.host = "localhost"
            self.port = "communicator"
            self.address = ("localhost", "communicator")
        super(ConnectionHandler, self).__init__(self.host, self.port, 0, socket, server=False, handler=True)

        self.logger = logging.getLogger("Borg.Brain.Communicator.ConnectionHandler")
        self.communicator = communicator

        self.conn_id = self.communicator.register_connection(self)
        self._send_sources_time = 0

    def handle_shutdown(self):
        """
        This method makes sure the handler gets unregistered in the communicator
        when the module shuts down.
        """
        self.communicator.unregister_connection(self)

    def handle_receive(self, data):
        """Handle the data that are received from vision controller"""
        for command in data:
            if command == "diag":
                self.logger.debug("Sending diag message")
                active_sources = self.communicator.video_sources()
                if not active_sources:
                    active_sources = {'None': 0}
                diag = {"status": "Communicator is running ok",
                        "connections": self.communicator.num_connections(),
                        "video_sources": active_sources}
                self.send(diag)
                continue
            elif command == "available_sources":
                self.logger.debug("Sending available video sources")
                available_sources = self.communicator.check_sources()
                if not available_sources:
                    available_sources = ['None']
                diag = {"available_sources": available_sources}
                self.send(diag)
                continue
            
            self.communicator.add_command(command, self.conn_id)

    def send_sources(self):
        self._send_sources_time = 0

    def handle_loop(self):
        """
        This method is called in the main loop of the socket. It will sent the
        list of active sources to the vision controller
        """
        if self._send_sources_time < time.time() - 2:
            self._send_sources_time = time.time()
            active_sources = self.communicator.video_sources()
            source_list = active_sources.keys() 
            self.send({"command": "active_sources", "active_sources": source_list})
    

## Communicator Class Documentation.
class Communicator(object):
    def __init__(self, port=49152, visualize=None, camera = -1, logpath=None, only_local=False):
        """
        Sets up the communicator
        """

        global profile
        # Set up logging
        self.logger = logging.getLogger('Borg.Brain.Communicator')

        # Listener that accepts connections
        if only_local:
            self.__listener  = None
            self.__UD_listener = Listener("communicator", self)
        else:
            self.__listener = Listener(port, self)
            self.__UD_listener = Listener("communicator", self)

        # Registered connections
        self.__connection_list = []
        self.__next_connection_id = 0

        # The list for potential process names
        self.__processList = {}
        self.__connection_lock = threading.Lock()
        
        # The commands received from Vision Controller
        self.__commands = []
        self.__visualize = visualize
        self.__camera = camera
        self.__nao_camera = None

        # Video sources / vidmemwriter
        self.__vidmemwriter = None
        self.__vid_mem_lock = threading.Lock()
        self.__video_sources = {}
        self.__video_source_settings = {}
        self.__available_sources = []

        self.__pipe, self.__remote_pipe = multiprocessing.Pipe()
        self.__video_manager = videomanager.VideoManager(self.__remote_pipe, profile)
        self.__video_manager.start()
        self.__video_heartbeat = time.time()
        self.__video_poll = iopoll.IOPoll()
        self.__video_poll.register(self.__pipe)

        if self.__visualize:
            cv.NamedWindow(self.__visualize, cv.CV_WINDOW_AUTOSIZE)

        if logpath is None:
            self.__logpath = os.environ['BORG'] + '/brain/src/vision/logs'
        else:
            self.__logpath = logpath

        if self.__logpath: # If set to False, don't log
            self.logger.info("Logging output of vision modules to %s" % self.__logpath)
            os.system('mkdir -p %s' % self.__logpath)
            os.system('rm -fv %s/*.log' % self.__logpath)

    def wait_listen(self, timeout=2):
        result = True
        start = time.time()
        if self.__listener:
            result = result or self.__listener.wait_listen(timeout)
            timeout = timeout - (time.time() - start)
        if self.__UD_listener:
            cur_result = self.__UD_listener.wait_listen(timeout)
            result = result or cur_result

        return result

    def register_connection(self, connection):
        with self.__connection_lock:
            cur_id = self.__next_connection_id
            self.__next_connection_id += 1
            self.__connection_list.append((connection, cur_id))
            self.__processList[str(cur_id)] = []

            return cur_id

    def unregister_connection(self, connection=None, conn_id=None):
        if connection is None and conn_id is None:
            self.logger.error("No connection and no connection ID specified. " \
                              "Cannot unregister connection")
            return False

        with self.__connection_lock:
            if connection is None or conn_id is None:
                for cur_conn, cur_conn_id in self.__connection_list:
                    if connection != None and connection == cur_conn:
                        conn_id = cur_conn_id 
                    elif conn_id != None and conn_id == cur_conn_id:
                        connection = cur_conn

            if connection is None or conn_id is None:
                self.logger.error("No matching connection found. Cannot "   \
                                  "unregister connection")
    
            descriptor = (connection, conn_id)
            self.logger.info("Host %s disconnected." % repr(connection.address))
            if self.__processList[str(conn_id)]:
                self.logger.info("Killing all processes belonging to " \
                                 "connection ID %d" % conn_id)
            self.kill_all(conn_id)
            self.__connection_list.remove(descriptor)
            return True

    def check_video_manager(self, cmd=None, timeout=0):
        start_time = time.time()
        while True:
            events = self.__video_poll.poll(timeout)
            for fd, event in events:
                if event & iopoll.IO_READ:
                    try:
                        data = self.__pipe.recv()
                    except Exception, e:
                        self.logger.warning("Could not read from pipe. Error: %s" % str(e))
                        continue
                    self.logger.debug("Received from Video Manager: %s" % repr(data))
                    try:
                        command = data['command']
                        if cmd == command:
                            return data

                        if command == "heartbeat":
                            self.__video_heartbeat = time.time()
                    except:
                        pass
            if start_time + (timeout / 1000.0) < time.time():
                break

        # If we were waiting for a specific command, do not restart
        if not cmd is None:
            return None

        sources = self.__video_sources.keys()
        for source in sources:
            if self.__video_sources[source] <= 0:
                self.stop_video_source(source)
                del self.__video_sources[source]

        # Restart video manager if necessary
        if self.__video_heartbeat < time.time() - 10:
            self.__pipe.send({"command": "quit"})
            for i in range(20):
                if self.__video_manager.is_alive():
                    break
                time.sleep(0.01)
            for i in range(20):
                if self.__video_manager.is_alive():
                    break
                self.__video_manager.terminate()
                time.sleep(0.01)
            if self.__video_manager.is_alive():
                os.kill(self.__video_manager.pid, signal.SIGKILL)
            
            self.__video_poll.unregister(self.__pipe)
            del self.__video_manager 
            
            self.__pipe, self.__remote_pipe = multiprocessing.Pipe()
            self.__video_poll.register(self.__pipe)
            self.__video_manager = videomanager.VideoManager(self.__remote_pipe)
            self.__video_manager.start()
            self.__video_heartbeat = time.time()

            for source in self.__video_sources:
                settings = {}
                ip = None
                inputres = "640x480"
                outputres = "640x480"
                camera = -1
                if source in self.__video_source_settings:
                    settings = self.__video_source_settings[source]
                    ip = settings[0]
                    inputres = settings[1]
                    outputres = settings[2]
                    camera = settings[3]

                self.__pipe.send({"command": "start", "source": source,
                                  "ip": ip, "inputres": inputres,
                                  "outputres": outputres, "camera": camera})
        
    ##Destructor.
    # When communicator is stopped, all the processes should be killed immediately.
    def __del__(self):
        self.logger.info("Communicator ended. Stopping connections")
        self.stop_connections()

    def close(self):
        self.stop_connections()
        self.__pipe.send({"command": "quit"})
        for i in range(20):
            if not self.__video_manager.is_alive():
                break
            time.sleep(0.01)
        if self.__video_manager.is_alive():
            os.kill(self.__video_manager.pid, signal.SIGKILL)

    def stop_connections(self):
        if self.__listener:
            self.__listener.close()
        if self.__UD_listener:
            self.__UD_listener.close()
        if self.__connection_list:
            for connection, conn_id in self.__connection_list:
                connection.close()
    
    def listening(self):
        if self.__listener:
            if not self.__listener.connected():
                return False

        if self.__UD_listener:
            if not self.__UD_listener.connected():
                return False

        if not self.__listener and not self.__UD_listener:
            return False

        return True

    def run_communicator(self):
        self.run_ticker = Ticker(frequency=10)

        self.wait_listen()

        while True:
            self.run_ticker.tick()
            self.check_video_manager()
            cmd, conn_id = self.get_command()
            if cmd:
                self.process_command(cmd, conn_id)

    def add_command(self, command, conn_id):
        self.__commands.append((command, conn_id))

    def get_command(self):
        if self.__commands:
            return self.__commands.pop(0)
        return None, None

    def num_connections(self):
        with self.__connection_lock:
            return len(self.__connection_list)

    def video_sources(self):
        return self.__video_sources

    def send_sources(self):
        with self.__connection_lock:
            for conn, conn_id in self.__connection_list:
                conn.send_sources()
            
    def start_video_source(self, camType, ip=None, inputres="640x480", outputres="640x480"):
        if not camType or camType == "None":
            return True

        self.__video_source_settings[camType] = (ip, inputres, outputres, self.__camera)
        command = {"command": "start", "source": camType, "ip": ip, "inputres": inputres, "outputres": outputres, "camera": self.__camera}
        self.__pipe.send(command)
        
        cmd = self.check_video_manager("source_started", 5000)
        
        try:
            if cmd['result']:
                self.send_sources()
            return cmd['result']
        except Exception, e:
            self.logger.warning("Failed to start %s: %s" % (camType, repr(e)))
            return False

    def stop_video_source(self, video_source):
        if not video_source or video_source == "None":
            return True

        self.logger.info("I'm stopping %s" % video_source)
        command = {"command": "stop", "source": video_source}
        if video_source in self.__video_source_settings:
            del self.__video_source_settings[video_source]
        self.__pipe.send(command)

        cmd = self.check_video_manager("source_stopped", 3000)
        
        try:
            if cmd['result']:
                self.send_sources()
            return cmd['result']
        except:
            return False

    def check_sources(self):
        avail_sources = []
        self.__pipe.send({"command": "check_sources"})

        cmd = self.check_video_manager("available_sources", 6000)
        try:
            return cmd['available_sources']
        except:
            return []

    def kill_all(self, conn_id):
        if not str(conn_id) in self.__processList:
            self.logger.error("Kill All Processes request received from unknown connection ID %d" % conn_id)
            return False

        plist = self.__processList[str(conn_id)][:]
        for proc in plist:
            self.process_kill(proc[1], conn_id)

    def start_process(self, args, conn_id, bash=None, logfile=None, video_source="None"):
        """It starts the selected module in a new xterm window. -hold will keep the xterm window open even after crash for debug purpose. 
        -e let us send arguments to the python program which we run in xterm."""
        if not str(conn_id) in self.__processList:
            self.logger.error("Start Process request received from unknown connection ID %d" % conn_id)
            return False

        processList = self.__processList[str(conn_id)]

        ending = "'"
        if not bash:
            if logfile and self.__logpath:
                logfilename = self.__logpath + "/" + logfile
                format_time = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
                try:
                    _logfile = open(logfilename, 'a')
                    _logfile.write('\n---- PROCESS STARTING ON: %s ----\n\n' % format_time)
                    _logfile.close()
                except:
                    self.logger.warning("Cannot open logfile %s" % logfilename)
                self.logger.info("Logging output of module to file %s" % logfilename)
                #bash = "xterm -l -lf %s -hold -e 'source ~/.bashrc ; " % logfilename
                bash = "xterm -hold -e 'source ~/.bashrc ; " 
                ending = " 2>&1 | tee -a %s'" % logfilename
            else:
                bash = "xterm -hold -e 'source ~/.bashrc ; "
        command = bash + args + ending
        self.logger.info("Starting process " + command)

        processList.append((subprocess.Popen(command, shell=True, close_fds = True, preexec_fn=os.setsid), args, video_source))

        if video_source in self.__video_sources:
            self.__video_sources[video_source] += 1
        else:
            self.__video_sources[video_source] = 1

        return processList[-1][0]
    
    def process_is_alive(self, process):
        """Checks whether the process is still alive"""
        alive = process.poll()
        return alive
    
    def process_kill(self, processname, conn_id, stop_source=True):
        """This function searches through the running processes, if it matched the given processname, then it will initiate a
        kill group process by sending first a TERM signal and then a KILL signal."""
        if not str(conn_id) in self.__processList:
            self.logger.error("Kill Process request received from unknown connection ID %d" % conn_id)
            return False

        processList = self.__processList[str(conn_id)]

        video_source = "None"
        self.logger.info("Killing process " + processname)
        x = range(len(processList))
        indices = []
        for i in x:
            if processList[i][1] == processname:
                print "FOUND PROCESS TO KILL FOR %s: %s" % (processname, repr(processList[i]))
                pid = processList[i][0].pid
                video_source = processList[i][2]
                os.killpg(pid, signal.SIGTERM)
                time.sleep(1)
                os.killpg(pid, signal.SIGKILL)
                indices.append(i)
        indices.sort()
        for i in reversed(indices):
            del processList[i]

        if video_source in self.__video_sources:
            self.__video_sources[video_source] -= 1

        return -1
    
    def process_restart(self, processname, conn_id, logfile=None, video_source="None"):
        """Restarts a process"""
        self.logger.error("KILLING PROCESS")
        killresult = self.process_kill(processname, conn_id, stop_source=False)
        self.logger.error("KILLED PROCESS")
        startresult = self.start_process(processname, conn_id, logfile=logfile, video_source=video_source)
        self.logger.error("STARTED PROCESS")
        return killresult, startresult
        
    def copyModules2SHM(self):
        copyFrom = os.environ['BORG'] + '/brain/src/vision'
        os.system('rm -fr /dev/shm/visionmodules')
        os.system('mkdir -p /dev/shm/visionmodules')
        copyTo = '/dev/shm/visionmodules'
        os.system('cp -rf ' + copyFrom + '/* ' + copyTo)
        
    def process_command(self, cmd, conn_id):
        """This function Processes the incomming string command from the vision controller. It will check whether the command is start/kill or
        restart.
        It checks for the required vision module, port, and parameters
        and it finally makes a string command which can be run in an xterm window.
        Specifiying a vision module is mandatory. The module will return error if not specified"""
        
        try:
            command = cmd["command"]
        except:
            return -1, "Wrong Communication. It should be a dictionary."    

        if "command" in cmd and cmd['command'] == "set_nao_camera":
            cam = 0
            self.logger.info("Request received to set Nao Camera to %d" % cam)
            if "camera" in cmd:
                cam = cmd['camera']
            if cam != 0 and cam != 1:
                cam = 0
            self.__pipe.send({"command": "set_nao_camera", "camera": cam})
            return 0

        try:
            port = cmd["port"]
            module = cmd["module"]
        except:
            return -1, "Wrong Communication. No port or module specified."    

        address = ""
        splitted = module.split();
        arguments = configparse.parse_args(splitted)

        start_what = ""

        try:
            source = arguments["video_source"]
        except KeyError:
            source = None

        if "camera" in arguments:
            self.__camera = int(arguments['camera'])

        if "update_frequency" in arguments:
            self.logger.info("Setting update frequency to %d" % int(arguments['update_frequency']))
            freq = int(arguments['update_frequency'])
            self.run_ticker.set_frequency(freq)
            self.__pipe.send({"command": "set_speed", "frequency": freq})

        ip = None
        inputres = "640x480"
        outputres = "640x480"

        if source and ros_pattern.match(source):
            self.logger.info("I'm starting %s" % source)
            start_what = source
        elif source == "webcam":
            self.logger.info("I'm starting webcam")
            start_what = "webcam"
        elif source == "kinect":
            self.logger.info("I'm starting kinect")
            start_what = "kinect"
        elif source == "kinect_depth":
            self.logger.info("I'm starting kinect depth")
            start_what = "kinect_depth"
        elif source == "kinect_rgb":
            self.logger.info("I'm starting kinect rgb")
            start_what = "kinect_rgb"
        elif source == "naovideo":
            self.logger.info("I'm starting nao camera")
            if "nao" in arguments:
                ip = arguments['nao']
            if "inputres" in arguments:
                inputres = arguments['inputres']
            if "outputres" in arguments:
                outputres = arguments['outputres']
            start_what = "naovideo"
        elif source == "None" or source is None:
            self.logger.info("I'm not starting anything")
            start_what = "None"
            
        if not self.start_video_source(start_what, ip, inputres, outputres):
            self.logger.error("Video source not available. Not starting module")
            return -1, "Video source not available. Not starting module"
                
        path = os.environ['BORG'] + "/brain/src/util/" + splitted[0] 
        if os.path.exists(path):
            self.logger.info("Starting external executable file:" + path)
            alternateargs = path
        else:
            alternateargs = None
        
        path = os.environ['BORG'] + "/brain/src/vision/" + splitted[0] + ".py"
        if os.path.exists(path):
            address = path
        else:
            self.logger.error("The Following Path does not exist!\n" + path)
            return -1

        rest = splitted[1:len(splitted)]
        arguments = " " 
        for i in range(len(rest)):
            arguments = arguments + rest[i] + " "

        host_address = (None, None)
        with self.__connection_lock:
            for conn, cur_conn_id in self.__connection_list:
                if cur_conn_id == conn_id:
                    host_address = conn.address
                    break 

        host, not_the_port = host_address
        if not host: # Incoming addresses are not correct for Unix Domain Sockets
            host = "localhost"
        arguments = arguments + "port=" + str(port) + " host=" + str(host)
        if (alternateargs == None):
            args = "python " + address  + arguments
        else:
            args = path
        #print args
        
        if command == "start_module":
            self.logger.info("Received start module command from VisionController for module %s" % splitted[0])
            self.start_process(args, conn_id, logfile="%s.log" % os.path.basename(splitted[0]), video_source=start_what)
        elif command == "restart_module":
            self.logger.info("Received restart module command from VisionController for module %s" % splitted[0])
            self.process_restart(args, conn_id, logfile="%s.log" % os.path.basename(splitted[0]), video_source=start_what)
        elif command == "stop_module":
            self.logger.info("Received stop module command from VisionController for module %s" % splitted[0])
            self.process_kill(args, conn_id)
            
        with self.__vid_mem_lock:
            if self.__vidmemwriter:
                return 0
            else:
                return -1

original_handler = {}

def close_comm(signum=None, frame=None):
    global comm
    global original_handler

    comm.logger.warning("Caught signal, stopping")
    comm.close()
    if signum in original_handler and original_handler[signum]:
        original_handler[signum](signum, frame)
            
if __name__ == '__main__':
    print "PID: %d" % os.getpid()
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)
    port = 49152
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            port = 49152

    if "--profile" in sys.argv:
        profile = True

    only_local = "--local" in sys.argv

    comm = Communicator(port = port, only_local=only_local)

    import signal
    original_handler[signal.SIGINT] = signal.signal(signal.SIGINT, close_comm)
    original_handler[signal.SIGTERM] = signal.signal(signal.SIGTERM, close_comm)

    try:
        if profile:
            import cProfile, pstats
            cProfile.run('comm.run_communicator()', 'communicator.prof')
        else:
            comm.run_communicator()
    except:
        raise
    finally:
        comm.close()
        if profile:
            print "---- Profiling output for communicator process ----"
            p = pstats.Stats('communicator.prof')
            p.strip_dirs().sort_stats('cumulative').print_stats(20)
            os.unlink('communicator.prof')

            print "---- Profiling output for video manager process ----"
            p = pstats.Stats('videomanager.prof')
            p.strip_dirs().sort_stats('cumulative').print_stats(20)
            os.unlink('videomanager.prof')
