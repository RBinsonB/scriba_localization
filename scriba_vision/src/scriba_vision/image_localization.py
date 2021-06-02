#!/usr/bin/env python
import cv2
import numpy as np

from scriba_vision.feature_matching import FeatureMatcher, dist_to_cnt
from scriba_vision.exceptions import ImageLocalizationError, FeatureMatchingError

def get_homography_components(M):
    '''((translationx, translationy), rotation, (scalex, scaley), shear)'''
    p = np.sqrt(M[0,0]**2 + M[0,1]**2)
    r = (M[0,0]*M[1,1] - M[0,1]*M[1,0]) / p 

    translation = (M[0,2], M[1,2])
    scale = (p,r)
    shear = (M[0,0]*M[1,0]+M[0,1]*M[1,1]) / (M[0,0]*M[1,1] - M[0,1]*M[1,0])
    theta = np.arctan2(M[0,1], M[0,0])

    return (translation, theta, scale, shear)

class ImageLocalization:
    def __init__(self, img_map, min_matches=10, scale_diff_tolerance=0.15, shear_tolerance=0.1):
        '''Camera localization object'''

        # Create feature matcher with default arguments
        self.feature_matcher = FeatureMatcher()
        # Initialize map
        self.feature_matcher.set_map(img_map)

        self.scale_diff_tolerance = scale_diff_tolerance
        self.shear_tolerance = shear_tolerance
        self.min_matches = min_matches


    def homography_from_matches(self, kp_img, kp_map, matches):
        '''
        Get homography and returns origin and rotation
        kp_img: query image keypoints as output of OpenCV detector
        kp_map: image map keypoints as output of OpenCV detector
        matches: sorted matches
        '''

        # Get homography from matches
        src_pts = np.float32([ kp_img[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp_map[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

        return get_homography_components(M)


    def pose_from_match(self, kp_img, kp_map, matches):
        '''
        Retrieve pose of image in reference image from matches using homography
        kp_img: query image keypoints as output of OpenCV detector
        kp_map: image map keypoints as output of OpenCV detector
        matches: sorted matches
        '''

        # Get homography components from matches
        (translation, theta, scale, shear) = self.homography_from_matches(kp_img, kp_map, matches)

        # Check if the homography scale ratio (x/y) is within the tolerances
        if not (1-self.scale_diff_tolerance < (scale[0] / scale[1]) < 1+self.scale_diff_tolerance):
            raise ImageLocalizationError('Scale ratio ({0}) not within tolerance value ({1})'.format((scale[0] / scale[1]), self.scale_diff_tolerance))

        # Check if the homography shear is within the tolerance
        if abs(shear) > self.shear_tolerance:
            raise ImageLocalizationError('Shear value ({0}) above tolerance value ({1})'.format(abs(shear), self.shear_tolerance))

        # If all checks passed, return pose as (x, y, rotation)
        else:
            return translation[0], translation[1], theta


    def localize(self, img, lookup_area = None):
        '''Returns the pose of the input image in the image map'''

        try:
            kp_img, kp_map, matches = self.feature_matcher.detect_and_match(img, lookup_area)
            if matches:
                if len(matches) > self.min_matches:
                    return self.pose_from_match(kp_img, kp_map, matches)
                    
            #If not enough matches
            raise ImageLocalizationError("Not enough matches")

        # If error in matching, raised to image localization error
        except FeatureMatchingError as e:
            raise ImageLocalizationError(e)
