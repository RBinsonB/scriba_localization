#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from scriba_msgs.msg import localization_fix, motion_odom

from scriba_ekf.ekf import ExtendedKalmanFilter
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2 import TransformBroadcaster

from scriba_ekf.robot_model import ScribaRobotModel

class ScribaEKFNode:
    """ROS node wrapper for the Scriba robot position EKF"""

    def __init__(self):
        # Get EKF parameters as a dict
        ekf_param = self.get_params()

        # Check that EKF parameters are valid
        if self.check_ekf_params(ekf_param):
            # Create filter
            self.position_ekf = ExtendedKalmanFilter(ScribaRobotModel(ekf_param['L']),      # robot model with length L
                                                     ekf_param['update_sources'],
                                                     ekf_param['prediction_sources'])

        # Initialize filter with param or from topic
        if self.init_pose_from_param:
            self.set_initial_from_param()
        else:
            # Wait for initial pose to start the filter
            rospy.loginfo("Scriba position EKF: waiting for initial position...")
            self.handle_initial_pose_msg(rospy.wait_for_message("initialpose", PoseWithCovarianceStamped, timeout=None))

        # Setup subscribers and publishers
        self.setup_pub_sub(ekf_param['update_sources'], ekf_param['prediction_sources'])

        # Main loop at fixed frequency
        self.main_loop(self.node_frequency)


    def main_loop(self, frequency):
        """Publishes the output of the EKF at a fixed rate.
        CAREFUL: frequency is only the publishing frequency, not the update/predict steps frequency

        Keyword arguments:
        frequency: loop frequency in Hz
        """
        r = rospy.Rate(frequency)
        while rospy.not_shutdown():
            # Publish output of filter
            self.publish_pose_estimate()

            # Sleep to respect loop time
            r.sleep()


    def setup_pub_sub(self, update_sources, prediction_sources):
        """Setup ROS subscribers and publishers

        Keyword arguments:
        update_sources: absolute localization fix sources, given as dictionary.
        prediction_sources: dead-reckoning sources, given as dictionary.
        """

        # Output publisher
        self.output_pub = rospy.Publisher("~estimated_pose", PoseWithCovarianceStamped)

        # Output TF broadcaster
        self.output_tf_broadcaster = TransformBroadcaster()

        # Update sources subscribers
        for key, source in update_sources.items():
            rospy.Subscriber(source['topic'], localization_fix, self.update_callback, key)

        # Prediction sources subscribers
        for key, source in prediction_sources.items():
            rospy.Subscriber(source['topic'], motion_odom, self.predict_callback, key)


    def set_initial_from_param(self):
        """Set initial EKF value using parameters"""

        set_initial(self.initial_pose, self.initial_cov)

    
    def handle_initial_pose_msg(self, initial_pose_msg):
        """Handle initial pose ROS message and initialize the EKF"""

        x = initial_pose_msg.pose.pose.point.x
        y = initial_pose_msg.pose.pose.point.y

        # Get euler angle from pose quaternion
        (roll, pitch, yaw) = euler_from_quaternion([initial_pose_msg.pose.pose.orientation.x,
                                                    initial_pose_msg.pose.pose.orientation.y,
                                                    initial_pose_msg.pose.pose.orientation.z,
                                                    initial_pose_msg.pose.pose.orientation.w])

        covariance_3d = np.array(initial_pose_msg.pose.covariance).reshape(6,6)
        covariance_2d = np.vstack([covariance_3d[:2,:2], [0, 0] ])
        covariance_2d = np.column_stack([covariance_2d, [0, 0, covariance_3d[2,2]]])

        # Set initial 2D pose and covariance as numpy.array([x, y, theta]) and numpy.array([cov_xx, cov_yy, cov_aa])
        self.set_initial(np.array([x, y, yaw]), covariance_2d)


    def set_initial(self, initial_pose, initial_cov):
        """Set initial value of EKF"""
        rospy.loginfo("Scriba position EKF: initializing with initial pose: (x: {0}, y:{1}, theta: {2}) and covariance:\n{3}".format(initial_pose[0],
                                                                                                                                     initial_pose[1],
                                                                                                                                     initial_pose[2],
                                                                                                                                     covariance_2d))
        self.position_ekf.initialize(initial_pose, initial_cov)


    def update_callback(self, loc_msg, source):
        """Update the filter everytime an absolute localization fix is received"""
        localization_fix = np.array([loc_msg.x, loc_msg.y, loc_msg.theta])
        self.position_ekf.position_update(source, localization_fix)


    def predict_callback(self, motion_msg, source):
        """Apply the filter prediction step everytime a motion is received"""

        u = np.array([motion_msg.steerAngle, motion_msg.frontWheelDistance])
        self.position_ekf.position_prediction(source, u)


    def get_params(self):
        """Get all needed parameters to configure the EKF from ROS"""

        self.node_frequency = rospy.get_param("~frequency", 50)         # in Hz
        self.init_pose_from_param = rospy.get_param("~init_pose_from_param", False)
        self.publish_tf = rospy.get_param("~publish_tf", True)

        # Initial pose parameters
        initial_pose_x  = rospy.get_param("~initial_pose_x ", 0.0)
        initial_pose_y  = rospy.get_param("~initial_pose_y ", 0.0)
        initial_pose_theta  = rospy.get_param("~initial_pose_theta ", 0.0)
        self.initial_pose = np.array([initial_pose_x, initial_pose_y, initial_pose_theta])

        # Initial pose covariance parameters
        initial_cov_xx = rospy.get_param("~initial_cov_xx ", 0.025)
        initial_cov_yy = rospy.get_param("~initial_cov_yy ", 0.025)
        initial_cov_aa = rospy.get_param("~initial_cov_aa ", 0.07)
        self.initial_cov = np.array([initial_cov_xx, 0, 0,
                                     0, initial_cov_yy, 0
                                     0, 0, initial_cov_aa]).reshape(3,3)

        #return EKF parameters dictionary
        return rospy.get_param("~ekf_params")

    def publish_pose_estimate(self):
        """Publish the pose estimate and its covariance as a ROS message"""

        # Populate message
        pose_output_msg = PoseWithCovarianceStamped()
        pose_output_msg.header.frame_id = 'map'
        pose_output_msg.header.stamp = rospy.Time.now()
        pose_output_msg.pose.pose.position.x = self.position_ekf.estimate[0]
        pose_output_msg.pose.pose.position.y = self.position_ekf.estimate[1]
        pose_output_msg.pose.pose.position.orientation = quaternion_from_euler(self.position_ekf.estimate[2].to_list())
        pose_output_msg.pose.covariance[0] = self.position_ekf.estimate_covariance[0,0]
        pose_output_msg.pose.covariance[7] = self.position_ekf.estimate_covariance[1,1]
        pose_output_msg.pose.covariance[35] = self.position_ekf.estimate_covariance[2,2]

        # Publish
        self.output_pub.publish(pose_output_msg)

        if self.publish_tf:
            broadcast_tf_from_pose(pose_output_msg)
            

    def broadcast_tf_from_pose(self, stamped_pose_with_cov_msg):
        """Broadcast a TF from a stamped pose with covariance message"""

        # Populate transform message
        tf_output_msg = TransformStamped()
        tf_output_msg.header = stamped_pose_with_cov_msg.header
        tf_output_msg.child_frame_id = 'base_link'
        tf_output_msg.transform.translation.x = stamped_pose_with_cov_msg.pose.pose.position.x
        tf_output_msg.transform.translation.y = stamped_pose_with_cov_msg.pose.pose.position.y
        tf_output_msg.transform.translation.z = 0.0         # flat-ground assumption
        tf_output_msg.transform.rotation = stamped_pose_with_cov_msg.pose.pose.orientation

        self.output_tf_broadcaster.sendTransform(tf_output_msg)


    def check_ekf_params(self, param_dict):
        """Check if all parameters needed for EKF creation are present"""

        param_keys = ['L', 'update_sources', 'prediction_sources']
        if not all([key in param_dict.keys() for key in param_keys]):
            rospy.logerr("Cannot initialize, missing parameter in position EKF")
            return False
        else:
            # Check update sources
            for source_key, source in param_dict['update_sources'].items():
                update_source_keys = ['R', 'T', 'max_distance', 'topic']
                if not all([key in source.keys() for key in update_source_keys]):
                    rospy.logerr("Cannot initialize, missing parameter in position EKF update source {0}".format(source_key))
                    return False

            # Check prediction sources
            for source_key, source in param_dict['predictions_sources'].items():
                predict_source_keys = ['Q', 'topic']
                if not all([key in source.keys() for key in predict_source_keys]):
                    rospy.logerr("Cannot initialize, missing parameter in position EKF predict source {0}".format(source_key))
                    return False
        # If all check passed, return true
        return True


if __name__ == '__main__':
    """Main"""

    rospy.init_node('scriba_camera', anonymous=False)
    try:
        scriba_ekf_node = ScribaEKFNode()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Scriba EKF node: %s" %(e))
        sys.exit(0)
    except rospy.exceptions.ROSInterruptException as e:
        rospy.loginfo("Scriba EKF node: %s" %(e))
        sys.exit(0)