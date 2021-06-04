# scriba_localization
The Scriba robot localization packages

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

### Robot model



# scriba_vision
Provide a localization fix using the robot camera. The localization is based on SIFT feature identification and matching
