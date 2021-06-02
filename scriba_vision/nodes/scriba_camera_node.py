#!/usr/bin/env python
import rospy, rospkg
import cv2
import numpy as np
import sys, math, os
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

# Dynamic reconfigure parameters
from dynamic_reconfigure.server import Server as DynReconfServer
from scriba_vision.cfg import ScribaCamConfig
from scriba_vision.cv_utils import undistord_fisheye

from scriba_vision.image_localization import ImageLocalization
from scriba_vision.exceptions import ImageLocalizationError

class scribaCamImager:
    """Main camera node, localize in relation to a given map"""
    
    def __init__(self):
        rospy.loginfo("Initializing scriba camera localization")
        self.init_params()
        self.setup_pub_sub()

        dyn_reconfig_srv = DynReconfServer(ScribaCamConfig, self.dyn_reconfig_callback)
        self.cvbridge = CvBridge()

        self.i = 0

        # Load map and create localizer object
        self.textmap = cv2.imread(self.map_picture_path)
        self.localizer = ImageLocalization(self.textmap)


    def handle_camera_stream(self, img_msg):
        """Handle the camera stream to extract localization"""
        try:
            # Get and process image for localization
            camera_img = self.process_image(self.cvbridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough'))
            
            # Publish processed image
            self.processed_img_pub.publish(self.cvbridge.cv2_to_imgmsg(camera_img, "mono8"))

            self.localization(camera_img)

        except CvBridgeError as e:
            rospy.logerr(e)

        # Show camera stream
        self.i = self.show_and_save(camera_img, i)


    def localization(self, img):
        """localize the image within a map"""
        map_visu = self.textmap.copy()
        axis_length = 100
        try:
            (x, y, theta) = self.localizer.localize(img)
            print(x, y, theta)

            # Draw the coordinate frame
            # x
            cv2.arrowedLine(map_visu,
                            (int(x),int(y)),
                            (int(x + axis_length*math.cos(theta)), int(y + -axis_length*math.sin(theta) ) ),
                            (0, 0, 255), 3, cv2.LINE_AA)
            # y
            cv2.arrowedLine(map_visu,
                            (int(x),int(y)),
                            (int(x + (axis_length*math.sin(theta))), int(y+ axis_length*math.cos(theta))),
                            (0, 255, 0), 3, cv2.LINE_AA)

            cv2.imshow("localization", map_visu)
            cv2.waitKey(3)

        except ImageLocalizationError as e:
            print(e)


    def process_image(self, img):
        """Process the image so it can be used for localization"""

        # Rectify fisheye distortion
        rect = undistord_fisheye(img, self.K, self.D, self.img_dim)
        # Flatten using homography
        flatten = cv2.warpPerspective(rect, self.homography, self.img_dim)
        # Convert to greyscale
        gray = cv2.cvtColor(flatten, cv2.COLOR_BGR2GRAY)
        # Apply filter
        #filtered = cv2.bilateralFilter(gray, 5, 75, 75)
        filtered = cv2.GaussianBlur(gray, (11, 11), 0)
        # Binarized the image
        #thresh = cv2.threshold(gray, self.min_threshold, self.max_threshold, cv2.THRESH_BINARY_INV)[1]
        thresh = cv2.adaptiveThreshold(filtered, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, self.min_threshold)

        return thresh

    def show_and_save(self, img, i):
        """Show image and save if pushing key"""

        cv2.imshow("camera_stream", cv2.resize(img, (960, 540), interpolation = cv2.INTER_AREA))
        key = cv2.waitKey(6)
        if key:
            if key & 0xFF == ord('p'):
               pic_name = "saved_picture_"+str(i)+".png"
               print("saving picture %s..." % pic_name)
               cv2.imwrite(pic_name,thresh)
               i = i+1
            elif key & 0xFF == ord('q'):
                rospy.signal_shutdown('manual shutdown')
        return i

    def setup_pub_sub(self):
        """Setup ROS publishers and subscribers"""

        # Processed image publisher
        self.processed_img_pub = rospy.Publisher("process_image", Image, queue_size = 20)
        # Camera stream subscriber
        self.camera_stream_sub = rospy.Subscriber("image_raw", Image)

    def init_params(self):
        """Get params from ROS param server, topics and config files"""

        # Get map file path
        self.map_picture_path = os.path.join(rospkg.RosPack().get_path(rospy.get_param('~map_file/package','scriba_localization')),
                                                                       rospy.get_param('~map_file/path','config/map/textmap.png'))

        # Camera m to pixel scale
        self.camera_img_scale = rospy.get_param("camera_img_scale", 10000)

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
        rospy.loginfo("Reconfigure Request, min threshold: %i, max threshold: %i" % (config['min_threshold'], config['max_threshold']))
        self.min_threshold = config['min_threshold']
        self.max_threshold = config['max_threshold']

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
        scriba_cam_imager = scribaCamImager()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Scriba camera node: %s" %(e))
        sys.exit(0)
    except rospy.exceptions.ROSInterruptException as e:
        rospy.loginfo("Scriba camera node: %s" %(e))
        sys.exit(0)