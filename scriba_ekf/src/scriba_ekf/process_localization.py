#!/usr/bin/env python
import numpy as np

def mahalanobis_distance(innovation, innovation_cov):
    """compute the Mahalanobis distance using innovation value and covariance
    
    Keyword arguments:
    innovation: localization difference between the previous estimate and the current localization fix numpy.array([x, y, theta])
    innovation_cov: innovation covariance as a numpy matrix (3,3)
    """
    print("Innovation is: \n{}".format(innovation))

    print("Mahalanobis distance is: {}".format(np.matmul(innovation.transpose(), np.matmul(np.linalg.inv(innovation_cov), innovation))))
    return np.matmul(innovation.transpose(), np.matmul(np.linalg.inv(innovation_cov), innovation))

def is_close(x, y, rtol=1.e-5, atol=1.e-8):
    """
    Code from crmccreary: https://gist.github.com/crmccreary/1593090
    """
    return abs(x-y) <= atol + rtol * abs(y)

def euler_angles_from_rotation_matrix(R):
    """
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix"

    Code from crmccreary: https://gist.github.com/crmccreary/1593090
    """
    theta_x = 0.0
    if is_close(R[2,0],-1.0):
        theta_y = np.pi/2.0
        theta_x = np.arctan2(R[0,1],R[0,2])
    elif is_close(R[2,0],1.0):
        theta_y = -np.pi/2.0
        theta_x = np.arctan2(-R[0,1],-R[0,2])
    else:
        theta_y = -np.arcsin(R[2,0])
        cos_theta = np.cos(theta_y)
        theta_x = np.arctan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        theta_z = np.arctan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return theta_x, theta_y, theta_z


class LocalizationFixHandler:
    """
    Handle localization from a absolute source and process it to be used in the update step of the EKF.
    The localization fix should be on the format numpy.array([x, y, theta])
    """

    def __init__(self, R, T, max_distance):
        """
        Keyword arguments:
        R: Covariance matrix of the localization, numpy matrix (3,3)
        T: 3D Transform between the sensor frame and the robot body frame, numpy matrix (4x4)
        max_distance: Maximum allowed distance value between previous estimation and innovation
        """

        self.R = R 
        self.T = T
        self.max_distance = max_distance

    def handle_localization_fix(self, localization_fix):
        """handle the localization fix (transform to proper frame, additional checks..."""

        if localization_fix is not None: # if no localization error
            return self.transform_to_body_frame(localization_fix)


    def transform_to_body_frame(self, localization_fix):
        """Transform a localization fix numpy.array([x, y, theta])from sensor frame to body frame
        sensor frame is assumed to be parallel with body frame (z=0), but can be inverted (computer vision convention).
        """

        # Create 3D vector for transform
        coord = np.append(localization_fix[:2], [0, 1]).reshape(4,1)

        # Apply transform matrix on coordinates
        transformed_coord = np.matmul(self.T, coord)

        # Apply rotation matrix on angle
        ## Create rotation matrix of theta angle
        R_theta = np.array([np.cos(localization_fix[2]), -np.sin(localization_fix[2]), 0,
                            np.sin(localization_fix[2]),  np.cos(localization_fix[2]), 0,
                            0, 0, 1]).reshape(3,3)
        transformed_rot = np.matmul(self.T[:3,:3], R_theta)
        roll, pitch, yaw = euler_angles_from_rotation_matrix(transformed_rot)

        # Return the transformed (x, y, theta)
        return np.append(coord[:2], yaw)


    def check_validation_gate(self, innovation):
        """Check the received localization fix innovation is within defined limits
            
        Keyword arguments:
        innovation: localization difference between the previous estimate and the current localization fix numpy.array([x, y, theta])
        """

        return mahalanobis_distance(innovation, self.R) <= self.max_distance




