import cv2
import numpy as np

height = 32
width = 32
nClasses = 10


def preprocess(image):
    image = cv2.resize(image, (height, width))
    image = image.reshape((-1, height, width, 3))

    image = image.astype('float') / 255
    return image


def oneHot(size, idx):
    vec = np.zeros((size))
    vec[idx] = 1
    vec = vec.reshape((-1, nClasses))
    return vec
