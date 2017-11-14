import cv
import takeimages
import os
import subprocess
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Util.VidMemWriter').addHandler(util.nullhandler.NullHandler())

class VidMemWriter():
    """
    Writes video to shared memory to be used by vision modules.
    This should be updated by the visionController, so there are images available
    to the vision modules
    """

    def __init__(self, sourcelist, namelist, clone=""):
        """
        Initialize VidMemWriter with a list of the sources and a list
        of the names of the sources. Sources can be any object with a
        get_image function (should return an openCV image)
        @param sourcelist - a list of sources, can be whatever sends images in openCV format. Source has to have get_image() function implemented.
        @param namelist - the name (label) of the folder you want the source to have
        @param clone - list of host names of the laptops you want to clone the images to (for parallel training)
        """
        self.logger = logging.getLogger('Borg.Brain.Util.VidMemWriter')
        self.clone = clone
        self.sourcelist = sourcelist
        self.namelist = namelist
        self.cleanup()
        assert len(sourcelist) == len(namelist)
        self.idxlist = [0 for i in range(len(sourcelist))]
        self.max_n_images = 10000
        for name in namelist:
            # make local dir        
            command = "mkdir -p -m 0777 /dev/shm/images/" + name
            os.system(command)            

    def add_video_source(self, source, name):
        self.sourcelist.append(source)
        self.namelist.append(name)
        self.idxlist.append(0)
        # Create the image cache dir
        command = "mkdir -p -m 0777 /dev/shm/images/" + name
        print "RUNNING %s" % command
        os.system(command)

    def stop_video_source(self, name):
        try:
            idx = self.namelist.index(name)
        except ValueError:
            self.logger.warning("No video source known by name %s" % name)
            return False
            
        self.logger.info("Stopping video source %s" % name)
        self.namelist.pop(idx)
        self.idxlist.pop(idx)
        source = self.sourcelist.pop(idx)
        if hasattr(source, "__del__"):
            source.__del__()
        del source
        # Remove the image cache dir
        command = "rm -rf /dev/shm/images/" + name
        os.system(command)
        return True

    def update(self):
        """
        write a new image from each source to the shared memory. Lateron,
        might be improved by only writing if for that source a new image is
        requested (last image is longer ago than specified in frequency...)
        """
        if len(self.sourcelist) != len(self.namelist) or \
           len(self.idxlist) != len(self.sourcelist):
            self.logger.critical("Video source lists are not of equal length")
            return False

        for source_idx in range(len(self.sourcelist)):
            #get the image
            try:
                img = self.sourcelist[source_idx].get_image()
            except:
                self.logger.warning("Unable to retrieve image of source %s. " \
                                    "Please check connection" \
                                    % self.namelist[source_idx])
                continue
            #increment the counter
            new_image_n = (self.idxlist[source_idx] + 1) % self.max_n_images
            #write the image
            dirname = "/dev/shm/images/" + self.namelist[source_idx]
            filename = dirname + "/" + str(new_image_n) + ".png"
            linkname = os.path.join(dirname, "lastimage")
            try:
                cv.SaveImage(filename, img)
                os.system("chmod a+rwx " + filename)
                try:
                    os.symlink(filename, linkname)
                except:
                    os.unlink(linkname)
                    os.symlink(filename, linkname)
                self.idxlist[source_idx] = new_image_n
            except TypeError as e:
                # On errors, just continue and try again next round
                self.logger.error("Got invalid image of source %s (%s), not saving to shared memory" % (self.namelist[source_idx], repr(e)))
                continue
            except KeyboardInterrupt:
                self.logger.info("Keyboard Interrupt!\nShared memory folder contents before cleanup:\n")
                os.system("ls -la /dev/shm/images/" + self.namelist[source_idx])
                self.logger.warning("No valid image was received from source: " + self.namelist[source_idx])
                raise SystemExit
            
            #start rsync process cloning of files
            if self.idxlist[source_idx] == (self.max_n_images - 1):
                for host in self.clone:
                    subprocess.Popen(["rsync", "-rpxcz", "-e", "ssh -c arcfour", dirname, host + ":/dev/shm/images"])


    def __del__(self):
        """
        Clean up if the program is terminated. In that case, files in shared
        memory should be deleted. However, if the program is stopped
        prematurely this method might not be executed.
        """
        self.cleanup()

    def cleanup(self):
        """Remove the folder containing the images from /dev/shm/images"""
        os.system("rm -rf /dev/shm/images")
        for host in self.clone:
            subprocess.Popen(["ssh", host, "rm -rf /dev/shm/images"])

#Example of how this can be used:
if __name__ == "__main__":
    #ip = "129.125.178.234"
    #source = naovideo.VideoModule(ip)
    webcamsource = takeimages.TakeImages(cameranumber=1, source='camera', resolution='verylow')
    vmw = VidMemWriter([webcamsource], ["webcam"])
    while True:
        vmw.update()


