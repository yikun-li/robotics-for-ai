#Import some python utilities
import cv
import logging
import random
import atexit

import util.nullhandler

logging.getLogger('Borg.Brain.Util.NaoVideo').addHandler(util.nullhandler.NullHandler())

##Set system paths
#path = `os.environ.get("ROBOCUP_DIR")`
#home = `os.environ.get("HOME")`
#
#if path == "None":
#        raise Exception("the environnement variable ROBOCUP_DIR is not set, aborting...")
#else:
#        sys.path.append((path + "/util").replace("~", home).replace("'", ""))

#Set path for aldebaran software & do some aldebaran imports
import config
config.set_aldebaran_path()
try:
    from naoqi import ALProxy
except:
    print "NAOQI is not available. Please install Aldebaran libs if you need this"
from vision_definitions import*

SequenceNumber = 0

all_instances = []

def clean_up():
    for proxy, nameId in all_instances:
        if proxy:
            logging.getLogger('Borg.Brain.Util.NaoVideo').                    \
                info("Unsubscribing Nao Video subscription %s" % nameId)
            proxy.unsubscribe(nameId)

atexit.register(clean_up)

class VideoModule():
    """Module that helps you get images from the NAO in your python program"""

    def __init__(self, IP, resolution="640x480", output="640x480", camera=0):
        """Create a proxy on video device, register themodule, and create image header"""
        self.logger = logging.getLogger('Borg.Brain.Util.NaoVideo')
        
        PORT = 9559

        ####
        # Create proxy on ALVideoDevice
        print IP
        try:
            self.camProxy = ALProxy("ALVideoDevice",IP,PORT)#TODO putting IP here for off-board, but no IP for on-board?
        except Exception,e:
            print "Error when creating vision proxy:"
            print str(e)
            exit(1)

        ####
        # Register a Generic Video Module
        input_resolution = kVGA
        img_res = (640, 480)
        if resolution == "320x240":
            input_resolution = kQVGA
            img_res = (320, 240)
        elif resolution == "160x120":
            input_resolution = kQQVGA
            img_res = (160, 120)

        out_res = (640, 480)
        if output == "320x240":
            out_res = (320, 240)
        elif output == "160x120":
            out_res = (160, 120)

        colorSpace = kBGRColorSpace
        fps = 30
        self.camProxy.setParam(kCameraSelectID, camera)

        global SequenceNumber
        mod_num = SequenceNumber
        SequenceNumber += 1
        mod_name = "python_GVM_%d" % mod_num
        self.logger.info("Subscription name: %s" % mod_name)
        self.nameId = self.camProxy.subscribe(mod_name, input_resolution, colorSpace, fps)
        self.subscribed = True
        self.iplImageHeader = cv.CreateImageHeader(img_res, 8, 3)
        self.output = cv.CreateImage(out_res, 8, 3)

        global all_instances
        all_instances.append((self.camProxy, self.nameId))

    def __del__(self):
        """Unsubscribe from proxy"""
        self.unsubscribe()

    def unsubscribe(self):
        global all_instances
        if hasattr(self, 'nameId'):
            ident = (self.camProxy, self.nameId)
            self.logger.info("Unsubscribing Nao Video subscription %s"        \
                             % self.nameId)
            self.camProxy.unsubscribe(self.nameId)
            self.subscribed = False
            if ident in all_instances:
                all_instances.remove(ident)

    def get_image(self):
        """
        Get image through proxy, copy it, and return the copy (to avoid memory
        problems)
        """
        result = self.camProxy.getImageRemote(self.nameId)
        #result is array with [width, height, numberOfLayers, colorSpace, timestamp(highest 32 bits), timestamp(lowest 32 bits), actual image]
        cv.SetData(self.iplImageHeader, result[6], result[0] * result[2]) #in the case data image depth = 8u only!
        cv.Resize(self.iplImageHeader, self.output)

        return self.output
        #return cv.CloneImage(self.iplImageHeader) #copy it just to be safe

    def change_camera(self, camera=0):
        self.camProxy.setParam(kCameraSelectID, camera)



##############
#IMAGE SIZES #    
##############
#
# small        medium      big
# 160x120      320x240     640x480
# 0 = kQQVGA   1 = kQVGA   2 = kVGA

if __name__ == "__main__":
    cameranumber = 0
    ip = "129.125.178.226" # Mies/Jan
    resolution = "640x480"
    output = "640x480"
    naovideo = VideoModule(ip, resolution, output, cameranumber)

    import time

    frametime = 0.0
    frames = 0

    while True:
        start = time.time()
        image = naovideo.get_image()
        print image
        cv.ShowImage("Camera index: " + str(cameranumber), image)
        
        k = cv.WaitKey(5)
        frametime += (time.time() - start)
        frames += 1 
        if frames > 0:
            avgframe = frametime / frames
            print "FPS: %f" % (1.0 / avgframe)
        if k == 0x1b:
            print 'ESC pressed. Exiting ...'
            clean_up()
            break

