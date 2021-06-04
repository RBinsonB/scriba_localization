#!/usr/bin/env python
import cv2
import numpy as np

from scriba_ekf.process_localization import LocalizationFixHandler

def angle_diff(a, b):
    """Return the relative difference between two angles (a-b) by wrapping around"""

    angle_diff = wrap_angle_positive(a) - wrap_angle_positive(b)
    if angle_diff < -np.pi:
        angle_diff += 2*np.pi
    if angle_diff > np.pi:
        angle_diff -= 2*np.pi

    return angle_diff

def wrap_angle_positive(a):
    """Wrap an angle to a positive [0, 360) representation"""

    a = np.fmod(a + 2*np.pi, 2*np.pi)
    if a < 0:
        a += 2*np.pi
    return a


class ExtendedKalmanFilter:
    """Extended Kalman Filter (EKF) class for the Scriba robot
    The filter is simplified by the assumption of non-dynamic covariances 
    of the sensor readings (covariance of the sensor and not of the measurement)
    """

    def __init__(self, robot_model, update_sources, prediction_sources):
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

        self.robot_model = robot_model
        self.update_sources = update_sources
        self.predict_sources = prediction_sources

        # # Create prediction source handles
        # self.predict_sources_dict = {}
        # for source, src_args in prediction_sources.items():
        #     self.predict_sources_dict.update(source : TODO(src_args))

        # Create update source handles
        self.update_sources_dict = {}
        for source, src_args in update_sources.items():
            self.update_sources_dict.update({source : LocalizationFixHandler(np.array(src_args['R']).reshape((3,3)),
                                                                             np.array(src_args['T']).reshape((4,4)),
                                                                             src_args['max_distance'])})

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
            innovation = self.compute_innovation(self.update_sources_dict[update_source].handle_localization_fix(localization_fix))
            # Check if innovation is valid
            if self.update_sources_dict[update_source].check_validation_gate(innovation):
                # Update EKF
                self.update_filter(innovation, np.array(self.update_sources[update_source]['R']).reshape((3,3)))

    def update_filter(self, innovation, measurement_cov):
        """Update the filter given an innovation and the associated covariance"""

        # Compute Kalman gain using sensor covariance
        innovation_cov = self.estimate_covariance + measurement_cov
        print("innovation_cov is \n{}",format(innovation_cov))
        K = self.compute_kalman_gain(innovation_cov)
        # Compute new estimate
        self.estimate = self.estimate + np.matmul(K, innovation)
        # Compute new estimate covariance
        print("cov variation")
        print(np.matmul(K, np.matmul(innovation_cov, K.transpose())))
        self.estimate_covariance = self.estimate_covariance - np.matmul(K, np.matmul(innovation_cov, K.transpose()))


    def compute_innovation(self, localization_fix):
        """Innovation as difference between previous estimate and new value
        
        Keyword arguments:
        localization_fix: numpy.array([x, y, theta])
        """
        print("Estimate is {0} and localization is {1}".format(self.estimate,localization_fix))

        if self.estimate is not None:
            innovation = np.zeros(3)
            innovation[:2] = localization_fix[:2] - self.estimate[:2]
            innovation[2] = angle_diff(localization_fix[2], self.estimate[2])
            return innovation

    def compute_kalman_gain(self, innovation_cov):
        """Compute the Kalman gain given the innovation covariance"""
        
        if self.estimate_covariance is not None:
            return np.matmul(self.estimate_covariance, np.linalg.inv(innovation_cov))


    def position_prediction(self, prediction_source, u):
        """Predict the position given an odometry estimate"""

        if prediction_source in self.predict_sources:
            if self.estimate is not None and self.estimate_covariance is not None:
                self.predict(u, np.array(self.predict_sources[prediction_source]['Q']).reshape((2,2)))

    def predict(self, u, u_cov):
        """Predict step of the filter given a value and the associated covariance"""

        # Compute new estimate
        self.estimate = self.robot_model.F(self.estimate, u)

        # Compute new estimate covariance
        Fx = self.robot_model.Fx(self.estimate, u)
        Fu = self.robot_model.Fu(self.estimate, u)
        self.estimate_covariance = np.matmul(Fx, np.matmul(self.estimate_covariance, Fx.transpose())) + np.matmul(Fu, np.matmul(u_cov, Fu.transpose()))