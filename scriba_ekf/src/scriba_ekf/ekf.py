#!/usr/bin/env python
import cv2
import numpy as np

from scriba_ekf.process_localization import LocalizationFixHandler


class ExtendedKalmanFilter:
    """Extended Kalman Filter (EKF) class for the Scriba robot
    The filter is simplified by the assumption of non-dynamic covariances 
    of the sensor readings (covariance of the sensor and not of the measurement)
    """

    def __init__(self, Fu, Fx, update_sources, prediction_sources):
        """
        Create the EKF given the updade source

        Keyword arguments:
        Fu: robot model jacobian to odometry vector, numpy matrix (2x3)
        Fx: robot model jacobian to state vector, numpy matrix (3x3)
        update_sources: absolute localization fix sources, given as dictionary.
            Every source is a dictionary with the following elements: 
                R: Covariance matrix of the localization, numpy matrix
                T: Transform between the sensor frame and the robot body frame
                max_distance: Maximum allowed distance value between previous estimation and innovation
        prediction_sources: dead-reckoning sources, given as dictionary.
            Every source is a dictionary with the following elements:
                Q: Covariance matrix of the prediction, numpy matrix
        """

        self.Fu = Fu
        self.Fx = Fx

        # # Create prediction source handles
        # self.predict_sources_dict = {}
        # for source, src_args in prediction_sources.items():
        #     self.predict_sources_dict.update(source : TODO(src_args))

        # Create update source handles
        self.update_sources_dict = {}
        for source, src_args in update_sources.items():
            self.update_sources_dict.update(source : LocalizationFixHandler(src_args))

        self.estimate = None 
        self.estimate_covariance = None

    def initialize(self, init_pose, init_cov):
        """Initialize the filter estimate and covariance matrix
        
        Keyword arguments:
        init_pose: 2D pose on the format numpy.array([x, y, theta])
        init_cov: Covariance matrix of the pose as a numpy matrix (3x3)
        """
        self.estimate = init_pose
        self.estimate_covariance = init_cov

    def position_update(self, update_source, localization_fix):
        """Innovation as difference between previous estimate and new value
        
        Keyword arguments:
        localization_fix: numpy.array([x, y, theta])
        """

        if update_source in self.update_sources_dict:
            # Get innovation from localization fix
            innovation = self.compute_innovation(self.update_sources_dict[update_source].LocalizationFixHandler(localization_fix))
            # Check if innovation is valid
            if self.update_sources_dict[update_source].check_validation_gate(innovation):
                # Update EKF
                self.update_filter(innovation, update_sources[update_source]['R'])

    def update_filter(self, innovation, innovation_cov):
        """Update the filter given an innovation and the associated covariance"""

        # Compute Kalman gain using sensor covariance
        K = self.compute_kalman_gain(innovation_cov)
        # Compute new estimate
        self.estimate = self.estimate + np.matmul(K, innovation)
        # Compute new estimate covariance
        self.estimate_covariance = self.estimate_covariance - np.matmul(K, np.matmul(innovation_cov, K.transpose()))


    def compute_innovation(self, localization_fix):
        """Innovation as difference between previous estimate and new value
        
        Keyword arguments:
        localization_fix: numpy.array([x, y, theta])
        """

        if self.estimate:
            return localization_fix - self.estimate

    def compute_kalman_gain(innovation_cov):
        """Compute the Kalman gain given the innovation covariance"""
        
        if self.estimate_covariance:
            return np.matmul(self.estimate_covariance, np.linalg.inv(innovation_cov))


    def position_prediction(self, prediction_source, u):
        """Predict the position given an odometry estimate"""

        if prediction_source in self.predict_sources_dict:
            if self.estimate and self.estimate_covariance:
                self.predict(u, self.predict_sources_dict[prediction_source]['Q'])

    def prefict(self, u, u_cov):
        """Predict step of the filter given a value and the associated covariance"""

        # Compute new estimate
        self.estimate = self.estimate + u
        # Compute new estimate covariance
        self.estimate_covariance = np.matmul(self.Fx, np.matlmul(self.estimate_covariance, self.Fx.transpose()) + np.matmul(self.Fu, np.matlmul(self.u_cov, self.Fu.transpose())


