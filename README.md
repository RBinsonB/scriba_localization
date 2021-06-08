# scriba_localization
The Scriba robot localization ROS packages.

- [**scriba_ekf**](/README.md#scriba_ekf) provides a position EKF for the robot
- [**scriba_vision**](/README.md#scriba_vision)  provides a position estimation from the robot camera

Next steps:
- Use the calibration rig to get sensor covariance
- Transform the camera localization fix in the camera node instead of the EKF node
- Implement dynamic covariances dependent on the measurement rather than the sensor
  - Based on good matches/outlier ratio?
  - Based on homography difference to perfect rectangle?


# scriba_ekf

## Overview
Provides an EKF to fuse position estimation from cameras and odometry.

### Estimation architecture
The estimation diagram of the robot is given below. The steer angle of the front wheel is estimated from the steer angle stepper motor command and the steer angle potentiometer value using a 1-D Kalman filter. Localization from one or more cameras can be fused to obtain the robot position estimation.

<img src="/documentation/pictures/ekf_diagram.png" align="center" width="800"/>

### Robot model

The robot is modeled using a bicycle model with the front wheel steer angle (φ) and front wheel traveled distance (d_fw) being measured. The robot model class is implemented in [/scriba_ekf/src/scriba_ekf/robot_model.py](/scriba_ekf/src/scriba_ekf/robot_model.py).

<img src="/documentation/pictures/scriba_robot_model.png" align="center" width="350"/>

The robot odometry vector is <img src="/documentation/formulas/u.png" align="center" border="0" alt="u_{t} =  \begin{bmatrix} \varphi_{t} \\ d_{fw_{t}} \end{bmatrix} " width="93" height="40" /> and robot state vector is <img src="/documentation/formulas/x.png" align="center" border="0" alt="x_{t} =  \begin{bmatrix} x_{t} \\ y_{t} \\ \theta_{t} \end{bmatrix} " width="83" height="68" />

The robot position prediction is <img src="/documentation/formulas/f.png" align="center" border="0" alt="\widehat{x_{t}}  = f(x_{t-1}, u_t) =  \begin{bmatrix}x_{t-1} \\y_{t-1} \\  \theta_{t-1}  \end{bmatrix} +  \begin{bmatrix}cos( \theta_{t} )\times d_{x_{t}} - sin(\theta_{t})\times d_{y_{t}} \\sin(\theta_{t})\times d_{x_{t}} +  cos( \theta_{t} )\times d_{y_{t}}\\  \omega_{t} \end{bmatrix}" width="420" height="68" />

with:

<img src="/documentation/formulas/dx.png" align="center" border="0" alt="d_{x_{t}} =  \frac{L}{tan(\varphi_{t})} \times sin(\omega_{t})" width="194" height="46" />

<img src="/documentation/formulas/dy.png" align="center" border="0" alt="d_{y_{t}} =  \frac{L}{tan(\varphi_{t})} \times (1-cos(\omega_{t}))" width="230" height="46" />

<img src="/documentation/formulas/omega.png" align="center" border="0" alt="\omega_{t} = \big( \frac {d_{fw_{t}}\times sin(\varphi_{t})}{L} \big)" width="172" height="43" />

As needed for the EKF, the Jacobians of the position prediction to respectively the state vector and the odometry vector are:

<img src="/documentation/formulas/Fx.png" align="center" border="0" alt="F_{x} =  \begin{bmatrix}1 & 0 & -sin(\theta)\times d_{x} - cos(\theta)\times d_{y} \\0 & 1 & cos(\theta)\times d_{x} -  sin(\theta)\times d_{y} \\ 0 & 0 & 1\end{bmatrix} " width="333" height="68" />

and

<img src="/documentation/formulas/Fu.png" align="center" border="0" alt="Fu =  \begin{bmatrix}cos(\theta + \omega)*cos(\varphi) & (L*cos(\omega)*sin(\theta) - L*sin(\theta) + L*sin(\omega)*cos(\theta) - d_{fw}*cos(\omega)*cos(\varphi)^2*cos(\theta)*sin(\varphi) + d_{fw}*sin(\omega)*cos(\varphi)^2*sin(\varphi)*sin(\theta))/(cos(\varphi)^2 - 1)\\sin(\theta + \omega)*cos(\varphi) &-(L*cos(\omega)*cos(\theta) - L*cos(\theta) - L*sin(\omega)*sin(\theta) + d_{fw}*cos(\omega)*cos(\varphi)^2*sin(\varphi)*sin(\theta) + d_{fw}*sin(\omega)*cos(\varphi)^2*cos(\theta)*sin(\varphi))/(cos(\varphi)^2 - 1)\\\fraq{sin(\varphi)}{L}&\fraq{(d_{fw}*cos(\varphi))}{L} \end{bmatrix} " width="1544" height="68" />


## Usage

## Config files
- **scriba_ekf_params.yaml** Default EKF configuration for fusing odometry and localization measurement from the front camera.

## Launch files
- **scriba_ekf.launch**: Run the EKF node with the default configuration for fusing odometery and localization measurement from the front camra.
    - `init_pose_from_param` If true, get initial pose and covariance from parameters. Default: `true`.

## Nodes
### ekf_scriba_node.py
Position EKF node for the Scriba robot. Publishes a position estimation at a fixed frequency using inputs from the odometry and external localization sources (camera, ...). The node inputs are modular an allow for a range of sensor to be fused as long as they provide their localization measurement on the same format.

#### Subscribed Topics
- `update source topic` ([scriba_msgs/localization_fix](/TODO))
    
    A topic for every update source for the filter. Update is on the form of a "localization_fix", a 2D pose with covariance (x, y, yaw angle). Topic name is set by the parameter `update_sources/<update_source>/topic`. The topic triggers the update step of the EKF.
    
- `prediction source topic` ([scriba_msgs/motion_odom](/TODO))
    
    A topic for every prediction source for the filter. Prediction data is on the form of a "motion_odom" message, a motion data vector with covariance (front wheel steer angle `phi`, front wheel traveled distance `dfw`). Topic name is set by the parameter `prediction_sources/<prediction_source>/topic`. The topic triggers the prediction step of the EKF.
    
- `initialpose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    
    Initial pose and covariance of the filter. Only taken in account if parameter `~init_pose_from_param` is set to false.
    
#### Published Topics
- `~estimated_pose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    
    Estimated pose by the EKF with covariance.

- `tf` ([geometry_msgs/TransformStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html))
    
    If parameters `publish_tf` set to true, publishes the pose as a transform from `map` to `base_link`.
    
#### Parameters
- `~frequency` (int, default: 50)

    Node publish frequency. Careful: not the filter update/prediction step frequency (message triggered).
    
- `~init_pose_from_param` (bool, default: false)

    If true, will use the initial pose and covariance parameters to initialize the filter. If false, will get initial pose and covariance from topic.
    
- `~publish_tf` (bool, default: true)
.
    If true, published estimated pose as transform between `map` and `base_link`
    
- `~initial_pose_x` (double, default: 0.0 meters) 
 
    Initial pose mean (x), used to initialize filter with Gaussian distribution.

- `~initial_pose_ỳ` (double, default: 0.0 meters) 
 
    Initial pose mean (y), used to initialize filter with Gaussian distribution.
    
- `~initial_pose_theta` (double, default: 0.0 radians) 
 
    Initial pose mean (yaw), used to initialize filter with Gaussian distribution.

- `~initial_cov_xx` (double, default: 0.05\*0.05 meters)

    Initial pose covariance (x\*x), used to initialize filter with Gaussian distribution.

- `~initial_cov_yy` (double, default: 0.05\*0.05 meters)

    Initial pose covariance (y\*y), used to initialize filter with Gaussian distribution.
    
- `~initial_cov_aa` (double, default: 0.07 radians²)

    Initial pose covariance (theta\*theta), used to initialize filter with Gaussian distribution.

- `~ekf_params/L` (double)

    Length of the robot from rear wheel axle to front wheel axis.
    
- `~ekf_params/update_sources` (dictionary)

   Dictionary of all update sources (absolute measurement sources). Each source is a dictionary of its own of the format:
   - `<update_source>`
     - `topic` (string): topic name for the update source
     - `R` (double[9]): 3x3 covariance matrix for the sensor noise. will be used if the sensor doesn't provide covariance for its measurement.
     - `T` (double[16]): Transform matrix (4x4) between localization fix frame and robot body frame.
     - `max_distance` (double): Max Mahalanobis distance for the validation gate.
      
- `~ekf_params/prediction_sources` (dictionary)

   Dictionary of all prediction sources (relative measurement sources). Each source is a dictionary of its own of the format:
   - `<prediction_source>`
      - `topic` (string): topic name for the prediction source.
      - `Q` (double[4]): 2x2 covariance matrix for the predicion step noise. will be used if the sensor doesn't provide covariance for its measurement.

# scriba_vision
Provide a localization fix using the robot camera. The localization is based on SIFT feature identification and matching using FLANN algorithm.

<img src="/documentation/pictures/localization_screenshot.png" align="center" width="600"/>

*The camera image localized on the map*

<img src="/documentation/pictures/localization_test.png" align="center" width="600"/>

*The robot camera pointed at the test map*

## Overview

### Image processing

The robot localization is based on binarized image (black and white) as the robot is only able to print black ink on white paper (whitout half-toning).

The image from the camera is flatten using an homography to correct for the camera position. The homography matrix is obtained during a calibration phase. The flatten imaged is then blurred and binarized using an adaptive threshold.

### Localization

Features are identified using SIFT on both the camera picture and the map. Features are then matched using FLANN and sorted. A maximum number of match is kept, with a feature allowing to weight matches relative to their distance to an area. At last, homography search using RANSAC algorithm allows to get rid of outliers.

<img src="/documentation/pictures/feature_matching.png" align="center" width="800"/>

*The matches after outliers have been removed with the homography between the camera picture and the map in red*

The homography matrix from the camera picture to the map is decomposed to get its translation, rotation and shear components. The height/width ratio and the shear value are compared to tolerance values to accept or discard the localization reading.

## Config files
- **front_camera_params.yaml** Front camera parameters including homography matrix from the camera image to the floor plane.

- **front_camera.yaml** Front camera base parameters.

- **localization_params.yaml** Default localization parameters.

- **map** Folder of the map images

## Launch files
- **camera_localization.launch**: Run the localization node with the cv_camera node.
    - `device_id` Camera ID number for the cv_camera node. Default: `2`.

    - `camera_name` Camera name for the namespace. Default: `camera`.

- **front_camera_localization.launch**: Run the localization node with the cv_camera node with the the front camera parameters.

## Nodes

### scriba_camera_node.py
Localize an image in a image map using SIFT feature identification and FLANN matching algorithm.

#### Subscribed Topics

- `image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
    
    Image to be localized on the map.

- `camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html))

    Camera info used to initialize the node.

- `/robot_pose_estimate` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    
    Robot pose estimate use to weight matches to favor matches within the estimated observed region.

#### Published Topics
- `~processed_image` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
    
    Rectified, flatten and binarized image using as localization input picture.

- `~localization_fix` ([scriba_msgs/localization_fix](/TODO))
    
    Localization from the node.

#### Parameters
- `map_file/package` (string, default: "scriba_vision")

    ROS package containing the map.

- `map_file/path` (string, default: "config/map/textmap.png")

    Map file path relative to the root of the package

- `min_matches` (int, default: 10)

    Minimum number of matches needed for a reading to be valid.

- `scale_diff_tolerance` (double, default: 0.15)

    Tolerance on the x-scale/y-scale ratio for a reading to be valid.

- `shear_tolerance` (double, default: 0.1)

    Tolerance on the shear value for a reading to be valid.

- `use_lookup_area` (bool, default: false)

    If true, the matches will be sorted to favour those within or close to the lookup area estimated from the pose on `/robot_pose_estimate`.

- `camera_img_scale` (double, default: 10000.0)

    meters-to-pixels scale ratio.

- `min_threshold` (int, default: 2)

    Adaptive threshold mean offset for image binarization.

- `ksize_height` (int, default: 10)

    Gaussian kernel height for the gaussian blur.

- `ksize_width` (int, default: 10)

    Gaussian kernel width for the gaussian blur.

- `homography_matrix` (int, default: 10)

    Homography matrix to flatten the camera image to the floor plane.