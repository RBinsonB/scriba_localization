#!/usr/bin/env python
import cv2
import numpy as np

from scriba_vision.exceptions import FeatureMatchingError

def dist_to_cnt(pt, contour):
    '''
    Returns distance to a contour
    returns zero if point is in contour and the distance as a positive value if point isn't in the contour
    '''
    
    dist = cv2.pointPolygonTest(contour, pt, True)
    if dist > 0:
        #print("is in cnt")
        return 0
    else:
        #print("is not in cnt, distance is {}".format(dist))
        return abs(dist)

class FeatureMatcher:
    def __init__(self, detector='sift', matcher='flann'):
        '''
        Initialize with a given feature detector and feature matcher
        detector: a supported feature detector and descriptor (string)
        matcher: a supported feature matcher (string)
        '''

        # Supported detectors and matchers
        supported_detectors = ['sift']
        supported_matchers = ['flann']

        # Init detector
        if detector in supported_detectors:
            # SIFT detector
            if detector == 'sift':
                self.detector = cv2.SIFT_create()

        # If given detector not supported, raise error
        else:
            raise FeatureMatchingError("The detector %s is not supported by FeatureMatcher, supported detectors are %s" 
                             % supported_detectors)


        # Init matcher
        if matcher in supported_matchers:
            # FLANN matcher
            if matcher == 'flann':
                FLANN_INDEX_KDTREE = 1
                index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
                search_params = dict(checks = 50)
                self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        # If given matcher not supported, raise error
        else:
            raise FeatureMatchingError("The matcher %s is not supported by FeatureMatcher, supported detectors are %s"
                             % supported_matcher)

    
    def set_map(self, img_map):
        '''
        Set image map reference
        img_map: image map
        '''

        self.img_map = img_map

        # Compute features on the map
        try:
            self.kp_map, self.des_map = self.detector.detectAndCompute(self.img_map,None)
        except AttributeError:
            raise FeatureMatchingError("Missing attribute in FeatureMatcher, please ensure object has been properly initialized")


    def detect_and_match(self, img, lookup_area = None, max_matches = 150):
        '''Returns matched features between given image and map
        img: image to be matched against the map
        lookup_area: opencv contour, closest matches to the lookup area are selected first
        max_matches: limit the number of good matches
        '''

        try:
            kp_img, des_img = self.detector.detectAndCompute(img,None)
            matches = self.matcher.knnMatch(des_img, self.des_map,k=2)
            
            # If a lookup area is given, weight and sort the matches by distance to the lookup area
            if lookup_area is not None:
                matches = sorted(matches, key = lambda match:dist_to_cnt(self.kp_map[match[0].trainIdx].pt, *lookup_area))

            # store all the good matches as per Lowe's ratio test.
            good_matches = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good_matches.append(m)
                    if len(good_matches) > max_matches:
                        break

            return kp_img, self.kp_map, good_matches

        except AttributeError:
            raise FeatureMatchingError("Missing attribute in FeatureMatcher, please ensure object has been properly initialized")