#!/usr/bin/env python
import cv2
import numpy as np

from scriba_vision.cv_utils import undistord_fisheye

class ScribaCam:
    """Scriba camera object for a camera with a fisheye (also called equidistant) distortion model.

    Process the image by applying an homography transform to get a flat view of the floor plane.
    """

    def __init__(self, img_dim, K, D, H):
        """Initialize the object

        Keyword arguments:
        img_dim: (image_width, image_height) in pixels
        K: camera matrix, numpy matrix float[3,3]
        D: distortion coefficient (fisheye), numpy array float[4]
        H: Homography matrix for transform between the image plane to the floor plane
        """

        self.K = K
        self.D = D
        self.H = H
        self.img_dim = img_dim

    def process_image(self, img, min_threshold, ksize, border_type=0):
        """Process the image so it can be used for localization

        Keyword arguments:
        img: OpenCV image to be processed
        min_threshold : Adaptive threshold mean offset
        ksize: Gaussian kernel size to be used by the gaussian blur [height width]
        border_type: Gaussian kernel border type (OpenCV border type)
        """

        # Rectify fisheye distortion
        rect = undistord_fisheye(img, self.K, self.D, self.img_dim)
        # Flatten using homography
        flatten = cv2.warpPerspective(rect, self.H, self.img_dim)
        # Convert to greyscale
        gray = cv2.cvtColor(flatten, cv2.COLOR_BGR2GRAY)
        # Apply filter
        filtered = cv2.GaussianBlur(gray, (11, 11), 0)
        # Binarized the image
        thresh = cv2.adaptiveThreshold(filtered, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, min_threshold)

        return thresh