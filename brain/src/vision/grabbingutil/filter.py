import cv2.cv as cv
import sys
from grabbingutil.util import shared_roi

class TiltFilter:
    '''Filter that compensates the tilt of the (kinect) camera by extracting
    the difference in depth (far away from the camera the floor is further
    away that near the camera)'''
    def calibrate(self, img, mask_range = (50, 100)):
        # mask: ignore pixels that are really close or far away, like the
        # Nao (close, low values) and kinect noise (far far away, high values)
        _, _, width, height = cv.GetImageROI(img)
        mask = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(img, mask_range[0], mask_range[1], mask)

        # using the mask, create a new image containing only the pixels of
        # interest.
        self.base_img = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
        cv.SetImageROI(self.base_img, cv.GetImageROI(img))

        # get the minimum value, and subtract that from all pixels so only
        # the difference in depth in the image remains.
        minimum, _, _, _ = cv.MinMaxLoc(img, mask)
        cv.SubS(img, minimum, self.base_img)

    def filter(self, input_img):
        original_input_roi = cv.GetImageROI(input_img)
        filter_roi = shared_roi(input_img, self.base_img)
        
        if not filter_roi:
            raise Exception("input image and training image do not share a region of interest")
        
        cv.SetImageROI(input_img, filter_roi)

        # and that is it! Just subtract the difference we calibrated on from
        # the normal image, and tada!
        
        input_roi = cv.GetImageROI(input_img)
        cv.SetImageROI(self.base_img, input_roi)

        cv.Sub(input_img, self.base_img, input_img)
        cv.SetImageROI(input_img, original_input_roi)
    
class DummyFilter:
    '''Just as the name says, it is a dummy filter. It does nothing but
    implement the Filter interface.'''
    
    def calibrate(self, img):
        pass
    
    def filter(self, input_img):
        pass

if __name__ == '__main__':
    def render_with_histogram(img):
        '''Just a utility to draw a grayscale histogram next to an image.'''
        _, _, width, height = cv.GetImageROI(img)
        
        canvas = cv.CreateImage((width + 200, max(height, 255)), cv.IPL_DEPTH_8U, 1)
        cv.Rectangle(canvas, (width, 0), (width + 200, height), (0), cv.CV_FILLED)
        
        cv.SetImageROI(canvas, (0, 0, width, height))
        cv.Copy(img, canvas)
        
        cv.SetImageROI(canvas, (width, 0, 200, canvas.height))
        
        hist = cv.CreateHist([255], cv.CV_HIST_ARRAY, [(0,255)], 1)
        cv.CalcHist([img], hist)
        
        values = [cv.QueryHistValue_1D(hist, n) for n in range(255)]
        max_value = max(values)
        
        for n, value in enumerate(values):
            cv.Rectangle(canvas,
                (0, n),
                (int((value / max_value) * 200), n + 1),
                (255), cv.CV_FILLED)
        
        cv.SetImageROI(canvas, (0, 0, canvas.width, canvas.height))
        return canvas
    
    if len(sys.argv) < 3:
        print "Usage: %s calibration-img.png test-img.png [test-img.png 2 ...]"
        exit(0)
    
    filter = TiltFilter()
    input_img = cv.LoadImage(sys.argv[1], cv.CV_LOAD_IMAGE_GRAYSCALE)
    cv.SetImageROI(input_img, (0, 0, 630, 440))
    filter.calibrate(input_img)
    
    for path in sys.argv[2:]:
        img = cv.LoadImage(path, cv.CV_LOAD_IMAGE_GRAYSCALE)
        cv.ShowImage("Normal", render_with_histogram(img))
        filter.filter(img)
        cv.ShowImage("Filtered", render_with_histogram(img))
        
        if cv.WaitKey(0) in (-1, 27):
            break
