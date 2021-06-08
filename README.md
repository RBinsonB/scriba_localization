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

## Nodes
### ekf_scriba_node.py
Position EKF node for the Scriba robot. Publishes a position estimation at a fixed frequency using inputs from the odometry and external localization sources (camera, ...). The node inputs are modular an allow for a range of sensor to be fused as long as they provide their localization measurement on the same format.

#### Subscribed Topics
- `update source topic` ([scriba_msgs/localization_fix](/TODO)
    
    A topic for every update source for the filter. Update is on the form of a "localization_fix", a 2D pose with covariance (x, y, yaw angle). Topic name is set by the parameter `update_sources/<update_source>/topic`. The topic triggers the update step of the EKF.
    
- `prediction source topic` ([scriba_msgs/motion_odom](/TODO)
    
    A topic for every prediction source for the filter. Prediction data is on the form of a "motion_odom" message, a motion data vector with covariance (front wheel steer angle `phi`, front wheel traveled distance `dfw`). Topic name is set by the parameter `prediction_sources/<prediction_source>/topic`. The topic triggers the prediction step of the EKF.
    
- `initialpose` ([scriba_msgs/motion_odom]([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) 
    
    Initial pose and covariance of the filter. Only taken in account if parameter `~init_pose_from_param` is set to false.
    
#### Published Topics
- `~estimated_pose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    
    Estimated pose by the EKF with covariance.

- `tf` ([geometry_msgs/TransformStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html)
    
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

- `~initial_cov_xx` (double, default: 0.05*0.05 meters)

    Initial pose covariance (x*x), used to initialize filter with Gaussian distribution.

- `~initial_cov_yy` (double, default: 0.05*0.05 meters)

    Initial pose covariance (y*y), used to initialize filter with Gaussian distribution.
    
- `~initial_cov_aa` (double, default: 0.07 radians²)

    Initial pose covariance (theta*theta), used to initialize filter with Gaussian distribution.

- `~ekf_params/L` (double)

    Length of the robot from rear wheel axle to front wheel axis.
    
- `~ekf_params/update_sources` (dictionary)

   Dictionary of all update sources (absolute measurement sources). Each source is a dictionary of its own of the format:
   `<update_source>`
     - `topic` (string): topic name for the update source
     - `R` (double[9]): 3x3 covariance matrix for the sensor noise. will be used if the sensor doesn't provide covariance for its measurement.
     - `T` (double[16]): Transform matrix (4x4) between localization fix frame and robot body frame.
     - `max_distance` (double): Max Mahalanobis distance for the validation gate.
      
- `~ekf_params/prediction_sources` (dictionary)

   Dictionary of all prediction sources (relative measurement sources). Each source is a dictionary of its own of the format:
   `<prediction_source>`.
     - `topic` (string): topic name for the prediction source.
     - `Q` (double[4]): 2x2 covariance matrix for the predicion step noise. will be used if the sensor doesn't provide covariance for its measurement.
      

## Launch files


# scriba_vision
Provide a localization fix using the robot camera. The localization is based on SIFT feature identification and matching
