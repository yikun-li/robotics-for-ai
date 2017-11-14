import numpy
import sys
#import cv
import cv2

def convert_16to8(bit16):
    """
    Convert a 16 bit IplImage to an 8 bit IplImage, assuming
    that the source is the Kinect. The Kinect outputs
    16 bit values between 0 and +- 9500. But the minimum distance
    is 400mm, so effectively, the source range is 400 - 9500, which
    nearly fits into 13 bits (8192 max). So, 400 is subtracted from
    every value and then the result is bit-shifted 5 positions, to
    leave 8 bits of depth information.

    If a 8-bit image is passed it, it is returned right away.
    """
    if bit16.depth == '8L' or bit16.depth == '8':
        return bit16

    # Total weirdness, apparently those are not equal...
    dstr = str(bit16.depth)
    if dstr == "8L" or dstr == "8":
        return bit16

    depth = numpy.asarray(bit16[:,:])
    #For Gazebo kinect inputs
    if str(bit16.depth) == '32':
        depth *= 1000
        depth = depth.astype(numpy.uint16)
    depth -= 400
    depth >>= 5
    
    # Convert the numpy array back to an IplImage
    bit8 = cv2.cv.CreateImageHeader((depth.shape[1], depth.shape[0]), cv.IPL_DEPTH_16U, 1)
    cv2.cv.SetData(bit8, depth.tostring(), depth.dtype.itemsize * depth.shape[1])

    new = cv2.cv.CreateImage((bit8.width, bit8.height), cv.IPL_DEPTH_8U, 1)
    cv2.cv.Convert(bit8, new)
    
    return new
