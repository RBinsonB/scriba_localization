#!/usr/bin/env python
import rospy, rospkg
import cv2
import numpy as np
import sys, math, os
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from scriba_msgs.msg import localization_fix
from cv_bridge import CvBridge, CvBridgeError

# Dynamic reconfigure parameters
from dynamic_reconfigure.server import Server as DynReconfServer
from scriba_vision.cfg import ScribaVisionConfig

from scriba_vision.scriba_camera import ScribaCam

from scriba_vision.image_localization import ImageLocalization
from scriba_vision.exceptions import ImageLocalizationError

class scribaCamNode:
    """Main camera node, localize in relation to a given map"""
    
    def __init__(self):
        rospy.loginfo("Initializing scriba camera localization")
        # Get camera parameters
        self.init_params()

        # Create camera object from parameters
        self.scriba_cam = ScribaCam(self.img_dim, self.K, self.D, self.homography)

        # Setup dynamic reconfigure server
        dyn_reconfig_srv = DynReconfServer(ScribaVisionConfig, self.dyn_reconfig_callback)

        # OpenCV-ROS bridge
        self.cvbridge = CvBridge()

        self.lookup_area = None
        self.localization_msg = localization_fix()

        # Load map and create localizer object
        self.load_map(self.map_picture_path)
        self.localizer = ImageLocalization(self.img_map, self.min_matches, self.scale_diff_tolerance, self.shear_tolerance)

        # Initialize ROS publishers and subscribers
        self.setup_pub_sub()


    def handle_camera_stream(self, img_msg):
        """Handle the camera stream to extract localization"""

        try:
            # Get and process image for localization
            camera_img = self.scriba_cam.process_image(self.cvbridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough'), self.min_threshold, self.blur_ksize)

            # Publish processed image
            self.processed_img_pub.publish(self.cvbridge.cv2_to_imgmsg(camera_img, "mono8"))

            # Localization of processed image on the map
            self.localize_and_publish(camera_img, self.lookup_area)

        except CvBridgeError as e:
            rospy.logerr(e)


    def load_map(self, map_picture_path):
        """Read image file and load it as map"""

        # Load map
        self.img_map = cv2.imread(map_picture_path)

        # Check than map has been loaded properly
        if len(self.img_map) <= 0:
            raise ValueError("cannot initialize map, error in reading image file")


    def localize_and_publish(self, img, lookup_area):
        """localize the image within a map"""

        try:
            # Get localization
            (x, y, theta) = self.localizer.localize(img, lookup_area)
            rospy.logdebug("Got localization x: {0}, y: {1}, theta: {2}".format(x, y, theta))

            # Populate and publish message
            self.localization_msg.x = x
            self.localization_msg.y = y
            self.localization_msg.theta = theta
            self.localization_pub.publish(self.localization_msg)
        except ImageLocalizationError as e:
            rospy.logdebug(e)

    def setup_pub_sub(self):
        """Setup ROS publishers and subscribers"""

        # Processed image publisher
        self.processed_img_pub = rospy.Publisher("~processed_image", Image, queue_size = 20)
        # Localization publisher
        self.localization_pub = rospy.Publisher("~localization_fix", localization_fix, queue_size = 20)
        # Camera stream subscriber
        self.camera_stream_sub = rospy.Subscriber("image_raw", Image, self.handle_camera_stream)

        if self.use_lookup_area:
            self.robot_pose_sub = rospy.Subscriber("/robot_pose_estimate", PoseWithCovarianceStamped)

    def pose_estimate_callback(self, pose_msg = PoseWithCovarianceStamped):
        None #TODO


    def get_lookup_area(self, pose):
        """Generate lookup area based on camera resolution, pose estimate and m-to-pixel ratio"""

        None #TODO

    def init_params(self):
        """Get params from ROS param server, topics and config files"""

        # Get map file path
        self.map_picture_path = os.path.join(rospkg.RosPack().get_path(rospy.get_param('~map_file/package','scriba_vision')),
                                                                       rospy.get_param('~map_file/path','config/map/textmap.png'))

        # Image localization params
        self.min_matches = rospy.get_param("min_matches", 10)
        self.scale_diff_tolerance = rospy.get_param("scale_diff_tolerance", 0.15)
        self.shear_tolerance = rospy.get_param("shear_tolerance", 0.1)

        # Use location estimation to weight feature detection
        self.use_lookup_area = rospy.get_param('use_lookup_area', False)

        # Camera m to pixel scale
        self.camera_img_scale = rospy.get_param("camera_img_scale", 10000)

        # Image processing params
        self.min_threshold = rospy.get_param("min_threshold", 2)
        self.blur_ksize = (rospy.get_param("ksize_height", 10), rospy.get_param("ksize_width", 10))

        # Get camera info from topic
        rospy.loginfo("Waiting for camera info...")
        self.get_camera_info(rospy.wait_for_message("camera_info", CameraInfo))

        # Get camera homography matrix
        if rospy.has_param("homography_matrix"):
            param = rospy.get_param("homography_matrix")
            self.homography = np.array(param['data']).reshape(param['rows'],param['cols'])
        else:
            rospy.logwarn("Camera homography matrix parameter not found, using default value")
            self.homography = np.array([[ 1.04025083e+00, 7.49276645e-01, 1.51786298e+01],
                                        [-5.92353140e-03, 1.77367076e+00, 6.88502101e+01],
                                        [-1.17527770e-05, 7.66672902e-04, 1.00000000e+00]])
        
        rospy.loginfo("Camera homography matrix:\n%s" % str(self.homography))


    def dyn_reconfig_callback(self, config, level):
        """Dynamic reconfigure parameter callback"""

        rospy.loginfo("Reconfigure Request, min threshold: %i, blur ksize: (%i, %i)" % (config['min_threshold'], config['ksize_height'], config['ksize_width']))
        self.min_threshold = config['min_threshold']
        self.blur_ksize = (config['ksize_height'], config['ksize_width'])

        return config


    def get_camera_info(self, camera_info_msg):
        """Get camera info from ROS message"""

        # Only support 'equidistant' (also called 'fisheye') model
        if camera_info_msg.distortion_model == 'equidistant':
            self.K = np.array(camera_info_msg.K).reshape(-1,3)
            self.D = np.array(camera_info_msg.D).reshape(-1,1)
            self.P = np.array(camera_info_msg.P)
            self.img_dim = (camera_info_msg.width, camera_info_msg.height)
            rospy.loginfo("Camera info loaded")
            rospy.loginfo("Camera image dimensions:%s" % str(self.img_dim))
            rospy.loginfo("Camera matrix K:\n%s" % str(self.K))
            rospy.loginfo("Camera distortion D: \n%s" % str(self.D))

        else:
            rospy.logerr("Camera distortion model not supported: %s" % camera_info_msg.distortion_model)


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('scriba_camera', anonymous=False)
    try:
        scriba_cam_node = scribaCamNode()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Scriba camera node: %s" %(e))
        sys.exit(0)
    except rospy.exceptions.ROSInterruptException as e:
        rospy.loginfo("Scriba camera node: %s" %(e))
        sys.exit(0)