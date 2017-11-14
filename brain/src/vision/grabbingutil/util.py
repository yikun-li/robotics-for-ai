import cv2.cv as cv
from grabbingutil.geometry import Rectangle

def clone_color_image(img):
    '''Create a new color image based on a grayscale image'''
    roi = cv.GetImageROI(img)

    color = cv.CreateImage((img.width, img.height), img.depth, 4)
    cv.SetImageROI(color, roi)

    empty = cv.CreateImage((img.width, img.height), img.depth, 1)
    cv.SetImageROI(empty, roi)

    cv.Merge(img, img, img, empty, color)
    return color

def create_empty_image(dims):
    img = cv.CreateImage(dims, cv.IPL_DEPTH_8U, 1)
    cv.Rectangle(img, (0,0), dims, (0), cv.CV_FILLED)
    return img

def copy_in_range(input_img, output_img, mask_range):
    '''Copy ,only the pixels with values between mask_range[0] and mask_range[1]
    from input_img to output_img.'''
    mask = cv.CreateImage((input_img.width, input_img.height), cv.IPL_DEPTH_8U, 1)
    cv.InRangeS(input_img, cv.Scalar(mask_range[0]), cv.Scalar(mask_range[1]), mask)
    cv.Copy(input_img, output_img, mask)

def shared_roi(*images):
    '''Return the part of the region of interest that all arguments share.
    Returns None if they don't share one rect all together.'''
    roi = Rectangle.from_cv_rect(cv.GetImageROI(images[0]))

    for image in images[1:]:
        roi = Rectangle.from_cv_rect(cv.GetImageROI(image)).union(roi)

    return roi.as_cv_rect() if roi.is_real else None