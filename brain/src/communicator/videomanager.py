import os
import multiprocessing
import threading
from util.binarysocket import BinarySocket
import time
import logging
import signal
import Queue
import traceback

from util.ticker import Ticker
import util.iopoll as iopoll
import util.vidmemwriter as vidmemwriter
import util.takeimages as takeimages
import util.iomanager as iomanager

#Used to compare to ros image topics:
import re
ros_pattern = re.compile("ros_")
import util.rosimage as rosimage

vminstance = None
class VideoSourceServer(BinarySocket):
    """
    This class is server for the VideoManager. It stores a reference to the
    VideoManager and wraps incoming connection in a VideoSourceHandler request.
    All other functionality is provided by the BinarySocket base class.
    """
    def __init__(self, manager, port = "VideoManager", compress=False, compress_level=6, bufsize="auto"):
        # Pass the host, port and compression settings on to the BinarySocket
        super(VideoSourceServer, self).__init__("",
                                                port,
                                                server=True,
                                                compress=compress, 
                                                compress_level=compress_level)
        self.__compression = (compress, compress_level)
        self.__bufsize = bufsize
        self.video_manager = manager
    
    def handle_accept(self, sock, address):
        if sock:
            handler = VideoSourceHandler(sock,
                                         self.video_manager,
                                         compress=self.__compression[0],
                                         compress_level=self.__compression[1],
                                         bufsize=self.__bufsize)

class VideoSourceHandler(BinarySocket):
    """
    This class is a subclass of BinarySocket that handles incoming connections
    for the VideoManager. It listens for requests for video on the socket and,
    in response, requests a new image to be sent to the client as soon as
    possible.
    """
    def __init__(self, sock, video_manager, compress=False, compress_level=6, bufsize="auto"):
        # Pass the host, port and compression settings on to the BinarySocket
        super(VideoSourceHandler, self).__init__("",
                                                 "VideoManager",
                                                 server=False,
                                                 handler=True,
                                                 use_socket=sock,
                                                 compress=compress,
                                                 compress_level=compress_level,
                                                 bufsize=bufsize)
        self.video_manager = video_manager

    def handle_receive(self, data):
        """
        Handle incoming data: check if it is a request for a new image.
        """
        super(VideoSourceHandler, self).handle_receive(data)
        with self._ThreadedSocket__send_lock:
            data = self._ThreadedSocket__receive_buffer
            self._ThreadedSocket__receive_buffer = []

        for command, binary in data:
            if type(command) is type({}):
                if "get_image" in command:
                    source = command['get_image']
                    self.video_manager.request_image(source, self)
                    self.video_manager._VideoManager__logger.debug(            \
                        "Image request for source %s received" % source)
                else:
                    self.video_manager._VideoManager__logger.warning(          \
                        "Invalid command received: %s" % repr(command))
            else:
                self.video_manager._VideoManager__logger.warning(              \
                    "Invalid command received: %s" % repr(command))

    def sendall(self, meta, binary):
        if self.connected():
            self.video_manager._VideoManager__logger.debug(                    \
                "[BinarySocket] Sending image to client")
            result = super(BinarySocket, self).sendall(meta, binary)
            self.video_manager._VideoManager__logger.debug(                    \
                "[BinarySocket] Sent image to client")
            return result
        self.video_manager._VideoManager__logger.info(                         \
            "[BinarySocket] Not sending image to disconnected client")
        return False

class VideoManager(multiprocessing.Process):
    """
    The VideoManager is a part of the Communicator. It handles the opening of
    video devices when requested and serves the images to the vision modules.

    The VideoManager is separated from the Communicator to avoid crashing the
    communicator when a video module fails. The communicator will automatically
    restart the video manager when it crashes.

    There are two modes available currently, which can be changed by setting the
    __server_mode boolean to True or False. When server mode is enabled, the
    VideoManager will set up a socket to accept incoming image requests and will
    serve the images over those sockets. When server mode is disabled, instead
    the VidMemWriter class will be used to write the images to /dev/shm/images,
    where the vision modules can collect them. The latter approach is the old
    behavior. While this works, it introduces more delay to the images because
    images are not served on request basis but depend on the frequency of the
    VideoManager. The new approach requests an image only when the vision module
    requires one, bringing the time of processing and the time of generation
    closer together. It also gives the opportunity to serve vision modules on
    a different host, by connecting over TCP/IP to port 59152. This will of
    course result in degraded performance because transmitting over the network
    takes a lot longer than over the Unix Domain Socket. Still, performance
    should be usable.
    """
    def __init__(self, pipe, profile=False):
        super(VideoManager, self).__init__()
        self.__vidmemwriter = None
        self.__pipe = pipe
        self.__heartbeat = time.time()
        self.__hb_interval = 2
        self.__camera = 0
        self.__nao_camera = None
        self.__logger = None
        self.__video_sources = []
        self.__profile = profile
        self.__server_mode = True

        self.__video_modules = {}
        self.__last_images = {}
        self.__video_locks = {}

        self.__image_queue = []
        self.__image_handlers = []

        try:
            import util.openni_kinectvideo
            self.__use_openni = False
        except:
            self.__use_openni = False

    def stop(self, signum=None, frame=None):
        """
        Process a stop request, usually used as a signal handler
        """
        self.__logger.warning("Caught signal, exiting")
        self._running = False

    def run(self):
        """
        The run method is the method started when the new process starts. It
        wraps the _run method, using the cProfile when requested to profile
        the VideoManager.
        """
        global vminstance

        iomanager.clear_IOM()
        vminstance = self
        if self.__profile:
            import cProfile, pstats
            cProfile.runctx('vminstance._run()', globals(), locals(), 'videomanager.prof')
        else:
            self._run()

    def _run(self):
        """
        The main loop of the VideoManager. It will communicate with the
        Communicator to start/stop video sources and to check which
        sources are available.
        """
        # Catch signals, shut down nicely
        signal.signal(signal.SIGTERM, self.stop)
        signal.signal(signal.SIGINT, self.stop)

        # Set up server listening on TCP socket and Unix Domain Socket
        self.__UD_server = VideoSourceServer(self, "VideoManager")
        # Use compression for TCP socket because that one will most likely
        # be used, if at all, by vision modules running on a different machine,
        # so bandwidth will be limited.
        self.__tcp_server = VideoSourceServer(self, 49953, compress=True, compress_level=6)

        # Set up the Queue for the image requests from vision modules
        self.__image_queue = Queue.Queue()

        self._ticker = Ticker(10)
        poller = iopoll.IOPoll()
        poller.register(self.__pipe)
        self.__logger = logging.getLogger('Borg.Brain.Communicator.VideoManager')
        self.__logger.info("VideoManager running")
        
        self._running = True
        #try:
        while self._running:
            self._ticker.tick()
            events = poller.poll(0)
            for fd, event in events:
                if event & iopoll.IO_ERROR:
                    self._running = False
                    break
                if event & iopoll.IO_READ:
                    self._handle_command()
                if event & iopoll.IO_WRITE:
                    self._send_heartbeat()
            if self.__vidmemwriter:
                self.__vidmemwriter.update()
        #except:
        #    self._running = False
        self._stop_video_sources()
        self.__logger.info("VideoManager stopping")
    
    def _send_heartbeat(self):
        """
        Send a heartbeat to the Communicator to indicate that
        the VideoManager has not crashed.
        """
        if time.time() > self.__heartbeat + self.__hb_interval:
            self.__pipe.send({"command": "heartbeat"})
            self.__heartbeat = time.time()

    def _handle_command(self):
        """
        Process a command from the Communicator: starting or stopping
        of video modules or returning which video sources are active
        or available.
        """
        data = self.__pipe.recv()
        cmd = data['command']
        if cmd == "start":
            # Start a video source
            source = data['source']
            ip = None
            inputres = None
            outputres = None
            if "ip" in data:
                ip = data['ip']
            if "inputres" in data:
                inputres = data['inputres']
            if "outputres" in data:
                outputres = data['outputres']
            if "camera" in data:
                self.__camera = data['camera']
            result = self._start_video_source(source, ip, inputres, outputres)
            self.__check_image_handlers()
            self.__pipe.send({"command": "source_started", "result": result})
        elif cmd == "stop":
            # Stop a video source
            source = data['source']
            result = self._stop_video_source(source)
            self.__pipe.send({"command": "source_stopped", "result": result})
        elif cmd == "set_nao_camera":
            # Change the camera the NAO is using: chin or top
            if "camera" in data:
                camera = data['camera']
                if self.__nao_camera:
                    self.__nao_camera.change_camera(camera)
                else:
                    self.__logger.warning("Nao Video camera source is None, cannot change camera")
        elif cmd == "check_sources":
            # Check which sources are available/connected to this machine
            sources = self._check_sources()
            self.__pipe.send({"command": "available_sources", "available_sources": sources})
        elif cmd == "set_speed":
            # Change the running frequency of the VideoManager
            if "frequency" in data:
                freq = data['frequency']
                self.__logger.info("Setting Video Manager speed to %d Hz" % freq)
                self._ticker.set_frequency(freq)
        elif cmd == "quit":
            # End the VideoManager
            self._running = False

    def _start_video_source(self, source, ip=None, inputres="640x480", outputres="640x480"):
        """
        This method starts a new video source, which some optional arguments
        specified such as input and output resolution and a IP address.
        Currently, these settings only apply to the naovideo source.
        """
        if source in self.__video_sources:
            return True

        if not source or source == "None":
            return True

        result = self._start_vidmemwriter(source, ip, inputres, outputres)

        return result

    def _start_vidmemwriter(self, camType, ip=None, inputres="640x480", outputres="640x480"):
        """
        Start the vidmemwriter and a video source, if required. If in server
        mode, no vidmemwriter will be started. Instead, the video module will
        be registered for with by the image_handlers. 
        """
        if not self.__vidmemwriter and not self.__server_mode:
            self.__vidmemwriter = vidmemwriter.VidMemWriter([], [])

        if camType in self.__video_sources:
            return True

        self.__logger.info("I'm starting %s" % camType)

        if ros_pattern.match(camType):
            #The first 4 characters "ros_" identify that is a specific ros image
            #The second part *** in "ros_***/topic" is the encoding:
            topic = camType[4:]
            encoding = "passthrough"
            self.__logger.info("camType !!!!!! %s" % camType)
            if not camType[4] == '/':
                str_list = camType.split("_")
                topic = '_'.join(str_list[2:])
                encoding = str_list[1]
            ros_image_source = rosimage.RosImage(topic, encoding)

            if self.__server_mode:
                self.__register_video_source(camType, ros_image_source)
            else:
                self.__vidmemwriter.add_video_source(ros_image_source, camType)
            self.__video_sources.append(camType)
            self.__logger.info("rosimage started for topic: %s, with encoding: %s" % (topic, encoding))
            return True
        elif camType == "webcam":
            self.__logger.debug("I'm starting webcam")
            webcamsource = takeimages.TakeImages(self.__camera)
            img = webcamsource.get_image()
            if type(img) is type(""):
                self.__logger.error("No camera found. Please check connection!")
                return False

            if webcamsource.Nocamera:
                if self.__camera == -1:
                    self.__logger.error("No camera found. Please check connection!")
                else:
                    self.__logger.error("Camera %d not found. Please check connection!" % self.__camera)
                return False
            if self.__server_mode:
                self.__register_video_source('webcam', webcamsource)
            else:
                self.__vidmemwriter.add_video_source(webcamsource, "webcam")
            self.__video_sources.append("webcam")
            self.__logger.info("Webcam started")
            return True
        elif camType == 'kinect_openni':
            self.__logger.debug("I'm starting kinect using openni")
            import util.openni_kinectvideo as kv
            depth_source = kv.OpenNIKinect("depth")
            rgb_source = kv.OpenNIKinect("rgb")

            try:
                depth_source.get_image()
            except:
                self.__logger.error("Kinect not found. Please check connection!")
                return False

            if self.__server_mode:
                self.__register_video_source('kinect_depth', depth_source)
                self.__register_video_source('kinect_rgb', rgb_source)
            else:
                self.__vidmemwriter.add_video_source(depth_source, "kinect_depth")
                self.__vidmemwriter.add_video_source(rgb_source, "kinect_rgb")

            self.__video_sources.append("kinect_depth")
            self.__video_sources.append("kinect_rgb")
            self.__video_sources.append("kinect")
            self.__video_sources.append("kinect_openni")
        
            self.__logger.info("Kinect started")
            return True
        elif camType == 'kinect' or camType == 'kinect_rgb' or camType == 'kinect_depth':
            if self.__use_openni:
                self.__logger.info("I'm starting kinect using openni")
                import util.openni_kinectvideo as kv
                depth_source = kv.OpenNIKinect("depth")
                rgb_source = kv.OpenNIKinect("rgb")

                try:
                    depth_source.get_image()
                except:
                    self.__logger.error("Kinect not found. Please check connection!")
                    return False
            else:
                self.__logger.info("I'm starting kinect using freenect")
                try:
                    import util.kinectmemwriter
                except:
                    self.__logger.error("Could not load kinectmemwriter module. Check modules.")
                    return False

                depth_source = util.kinectmemwriter.KinectDepthSource()
                rgb_source = util.kinectmemwriter.KinectRGBSource()

                try:
                    depth_source.get_image()
                except:
                    self.__logger.error("Kinect not found. Please check connection!")
                    return False

            if self.__server_mode:
                self.__register_video_source('kinect_depth', depth_source)
                self.__register_video_source('kinect_rgb', rgb_source)
            else:
                self.__vidmemwriter.add_video_source(depth_source, "kinect_depth")
                self.__vidmemwriter.add_video_source(rgb_source, "kinect_rgb")

            self.__video_sources.append("kinect_depth")
            self.__video_sources.append("kinect_rgb")
            self.__video_sources.append("kinect")
        
            self.__logger.info("Kinect started")
            return True
        elif camType == "naovideo":
            self.__logger.debug("I'm starting naovideo")
            try:
                import util.naovideo as naovideo
            except:
                self.__logger.error("Could not load naovideo module. Check modules")
                return False
            #get ip of nao:
            #TODO: fix this dirty hack (it should be read from the config file)
            naoip = "129.125.178.232"
            if ip:
                naoip = ip
    
            self.__logger.warn("Using input resolution %s and output resolution %s" % (inputres, outputres))
            #use the naovideo module:
            if self.__camera != 0 and self.__camera != 1:
                self.__camera = 0
            try:
                naocamsource = naovideo.VideoModule(naoip, inputres, outputres, camera=self.__camera)
                naocamsource.get_image()
            except:
                self.__logger.error("Something went wrong using the camera of the nao (check connection!)")
                traceback.print_exc()
                return False

            if self.__server_mode:
                self.__register_video_source('naovideo', naocamsource)
            else:
                self.__vidmemwriter.add_video_source(naocamsource, "naovideo")
            self.__video_sources.append("naovideo")
            self.__nao_camera = naocamsource
            self.__logger.info("Naovideo started")
            return True
        else:
            self.__logger.warning("Invalid video source specified: %s" % camType)
            return False
    
    def __register_video_source(self, name, source):
        """
        Register a running video source, for use in server mode.
        """
        self.__video_modules[name] = source
        self.__last_images[name] = (time.time(), source.get_image())
        self.__video_locks[name] = threading.Lock()

    def _stop_video_sources(self):
        """
        Stop all video sources.
        """
        for source in self.__video_sources:
            self._stop_video_source(source)

    def _stop_video_source(self, video_source):
        """
        Stop the specified video source
        """
        if not video_source in self.__video_sources:
            self.__logger.warning("Not stopping video source %s - not running" % video_source)
            return True

        if self.__server_mode:
            if video_source == "kinect":
                if "kinect_rgb" in self.__video_modules:
                    del self.__video_modules["kinect_rgb"]
                    del self.__last_images["kinect_rgb"]
                    del self.__video_locks["kinect_rgb"]
                if "kinect_depth" in self.__video_modules:
                    del self.__video_modules["kinect_depth"]
                    del self.__last_images["kinect_depth"]
                    del self.__video_locks["kinect_depth"]
            else:
                if video_source in self.__video_modules:
                    if video_source == "naovideo":
                        self.__video_modules[video_source].unsubscribe()

                    del self.__video_modules[video_source]
                    del self.__last_images[video_source]
                    del self.__video_locks[video_source]

        if video_source == "kinect":
            import util.kinectvideo
            if not self.__server_mode:
                self.__vidmemwriter.stop_video_source("kinect_rgb")
                self.__vidmemwriter.stop_video_source("kinect_depth")
            for src in ['kinect', 'kinect_rgb', 'kinect_depth', 'kinect_openni']:
                if src in self.__video_sources:
                    self.__video_sources.remove(src)
            util.kinectvideo.StopKinect()
        else:
            self.__video_sources.remove(video_source)
            if not self.__server_mode:
                self.__vidmemwriter.stop_video_source(video_source)
         
        if not self.__video_sources:
            self.__logger.info("No video source active. Stopping vidmemwriter.")
            self.__vidmemwriter = None
        return True

    def _check_sources(self):
        """
        Check the available video sources. Use the list of active video sources
        as a starting point. Then, for each known video source not currently
        active, try to start it, and if this succeeds, add it to the list.
        """
        avail_sources = []
        for source in self.__video_sources:
            avail_sources.append(source)

        # Try to start each video source known. Naovideo is not in this list,
        # because without an explicit request, no IP and port is known so no
        # connection can be made.
        for source in ["webcam", "kinect"]:
            if source in avail_sources:
                continue

            # Status unknown, try to start and stop
            self.__logger.info("Trying to see if source %s is available" % source)
            try:
                if self._start_video_source(source):
                    avail_sources.append(source)
                    self._stop_video_source(source)
            except Exception, e:
                self.__logger.info("Error starting source %s, (error: %s) must not be available" % (source, str(e)))
        
        return avail_sources

    def request_image(self, source, connection):
        """
        This method is for use by the VideoModuleHandler objects that handle
        incoming connection to the VideoSourceServer. It adds a new request
        for an image from a specified source to the queue.
        """
        try:
            self.__image_queue.put_nowait((source, connection))
            return True
        except Queue.Full:
            return False

    def __get_image(self, source):
        """
        This method tries to obtain a new image from a specified source. It
        locks the video module while doing this, to avoid retrieving two
        images at the same time. When an image has been retrieved less than
        0.05 seconds ago, that image is used instead to avoid requesting 
        more than 20 images per second. The hardware can't handle that
        anyway.
        """
        if not source in self.__video_modules:
            return (None, None)
        with self.__video_locks[source]:
            last_time, last_img = self.__last_images[source]
            age = time.time() - last_time
            if age > 0.05:
                new_image = self.__video_modules[source].get_image()
                try:
                    new_time = self.__video_modules[source].get_time()
                    print "Got time from ros: %f" % new_time
                except:
                    new_time = time.time()

                if new_image:
                    last_time = new_time 
                    last_img = new_image
                    self.__last_images[source] = (new_time, new_image)
            return (last_time, last_img)

    def __check_image_handlers(self):
        """
        This method checks the amount of video modules active and the
        amount of image handler threads active. Each video module should
        have at least one handler to process requests because they will block
        for a short period of time while the image is obtained from the camera.

        If too few threads are running, a new one is started.
        """
        active_workers = []
        for w in self.__image_handlers:
            if w.is_alive():
                active_workers.append(w)
        self.__image_handlers = active_workers

        if len(self.__video_modules) > len(self.__image_handlers):
            new_t = threading.Thread(target=self.__image_request_handler)
            new_t.start()
            self.__image_handlers.append(new_t)

    def __image_request_handler(self):
        """
        This method is the main loop for a image handler worker thread. It will
        try to get an image request from the Queue and if available, it will
        try to fulfill the request. 

        As soon as the new image is obtained from the video module, it will
        be sent over the socket that requested the image.
        """
        self.__logger.info("Image Request Handling Thread started")
        ticker = Ticker(2)
        while self._running:
            timeout = ticker.end_tick(False)
            try:
                task = self.__image_queue.get(True, timeout)
            except Queue.Empty:
                ticker.start_tick()
                continue

            # There is a task to process
            ticker.start_tick()
            source, connection = task

            # Check if the connection has been closed. If it was,
            # do not bother processing the request.
            if not connection.connected():
                self.__logger.info("Skipping request for image of source %s"   \
                                   " because requesting client disconnected"   \
                                   % source)
                self.__image_queue.task_done()
                continue 

            # Obtain new image
            error = "No image available"
            image = None
            mtime = time.time()
            if source in self.__video_modules:
                try:
                    mtime, image = self.__get_image(source)
                except Exception as err:
                    error = "Obtaining image failed: %s" % repr(err)
            else:
                error = "Video source %s has not been started" % source

            if connection.connected():
                if image:
                    # Valid image was obtained
                    img_str = image.tostring()
                    data = {'name':         'image',
                            'source':       source,
                            'time':         mtime,
                            'shape':        (image.width, image.height),
                            'depth':        image.depth,
                            'nChannels':    image.nChannels}
                else:
                    # An error occured, notify the vision module
                    self.__logger.info("Failed to obtain image for source %s. "\
                                       " Error message: %s" % (source, error))
                    img_str = ""
                    data = {'name':         'image',
                            'source':       source,
                            'time':         mtime,
                            'error':        error}
                # Send the data to the vision module.
                if not connection.sendall(data, img_str):
                    self.__logger.warning("Failed to send data to client. "   \
                                          "Probably disconnected")
            else:
                self.__logger.info("Image of source %s obtained but not "      \
                                   "sending because requesting client "        \
                                   "disconnected" % source)
            self.__image_queue.task_done()
        self.__logger.info("Image Request Handling Thread ended")
