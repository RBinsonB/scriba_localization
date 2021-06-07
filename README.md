# scriba_localization
The Scriba robot localization ROS packages.

- scriba_ekf provides a position EKF for the robot
- scriba_vision provides a position estimation from the robot camera

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

<img src="/documentation/pictures/scriba_robot_model.png" align="center" width="500"/>

The robot odometry vector is <img src="http://www.sciweavers.org/tex2img.php?eq=u_%7Bt%7D%20%3D%20%20%5Cbegin%7Bbmatrix%7D%20%5Cvarphi_%7Bt%7D%20%5C%5C%20d_%7Bfw_%7Bt%7D%7D%20%5Cend%7Bbmatrix%7D%20&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0" align="center" border="0" alt="u_{t} =  \begin{bmatrix} \varphi_{t} \\ d_{fw_{t}} \end{bmatrix} " width="93" height="40" /> and robot state vector is <img src="https://bit.ly/3cqhguj" align="center" border="0" alt="x_{t} =  \begin{bmatrix} x_{t} \\ y_{t} \\ \theta_{t} \end{bmatrix} " width="83" height="68" />

The robot position prediction is <img src="https://bit.ly/3vYW5HG" align="center" border="0" alt="\widehat{x_{t}}  = f(x_{t-1}, u_t) =  \begin{bmatrix}x_{t-1} \\y_{t-1} \\  \theta_{t-1}  \end{bmatrix} +  \begin{bmatrix}cos( \theta_{t} )\times d_{x_{t}} - sin(\theta_{t})\times d_{y_{t}} \\sin(\theta_{t})\times d_{x_{t}} +  cos( \theta_{t} )\times d_{y_{t}}\\  \omega_{t} \end{bmatrix}" width="380" height="68" />

with:

<img src="http://www.sciweavers.org/tex2img.php?eq=d_%7Bx_%7Bt%7D%7D%20%3D%20%20%5Cfrac%7BL%7D%7Btan%28%5Cvarphi_%7Bt%7D%29%7D%20%5Ctimes%20sin%28%5Comega_%7Bt%7D%29&bc=White&fc=Black&im=png&fs=12&ff=arev&edit=0" align="center" border="0" alt="d_{x_{t}} =  \frac{L}{tan(\varphi_{t})} \times sin(\omega_{t})" width="194" height="46" />

<img src="https://bit.ly/2S8DESg" align="center" border="0" alt="d_{y_{t}} =  \frac{L}{tan(\varphi_{t})} \times (1-cos(\omega_{t}))" width="230" height="46" />

<img src="http://www.sciweavers.org/tex2img.php?eq=%5Comega_%7Bt%7D%20%3D%20%5Cbig%28%20%5Cfrac%20%7Bd_%7Bfw_%7Bt%7D%7D%5Ctimes%20sin%28%5Cvarphi_%7Bt%7D%29%7D%7BL%7D%20%5Cbig%29%0A&bc=White&fc=Black&im=png&fs=12&ff=arev&edit=0" align="center" border="0" alt="\omega_{t} = \big( \frac {d_{fw_{t}}\times sin(\varphi_{t})}{L} \big)" width="172" height="43" />

As needed for the EKF, the Jacobians of the position prediction to respectively the state vector and the odometry vector are:

<img src="https://bit.ly/3fWIVFi" align="center" border="0" alt="F_{x} =  \begin{bmatrix}1 & 0 & -sin(\theta)\times d_{x} - cos(\theta)\times d_{y} \\0 & 1 & cos(\theta)\times d_{x} -  sin(\theta)\times d_{y} \\ 0 & 0 & 1\end{bmatrix} " width="255" height="68" />

and

<img src="https://bit.ly/3pvxQOY" align="center" border="0" alt="Fu =  \begin{bmatrix}cos(\theta + \omega)*cos(\varphi) & (L*cos(\omega)*sin(\theta) - L*sin(\theta) + L*sin(\omega)*cos(\theta) - d_{fw}*cos(\omega)*cos(\varphi)^2*cos(\theta)*sin(\varphi) + d_{fw}*sin(\omega)*cos(\varphi)^2*sin(\varphi)*sin(\theta))/(cos(\varphi)^2 - 1)\\sin(\theta + \omega)*cos(\varphi) &-(L*cos(\omega)*cos(\theta) - L*cos(\theta) - L*sin(\omega)*sin(\theta) + d_{fw}*cos(\omega)*cos(\varphi)^2*sin(\varphi)*sin(\theta) + d_{fw}*sin(\omega)*cos(\varphi)^2*cos(\theta)*sin(\varphi))/(cos(\varphi)^2 - 1)\\\fraq{sin(\varphi)}{L}&\fraq{(d_{fw}*cos(\varphi))}{L} \end{bmatrix} " width="1544" height="68" />


## Usage

## Nodes

## Launch files


# scriba_vision
Provide a localization fix using the robot camera. The localization is based on SIFT feature identification and matching
