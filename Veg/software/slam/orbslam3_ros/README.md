
# ORB-SLAM3 ROS Integration (Monocular-Inertial)

This directory contains configuration and launch files for running ORB-SLAM3 in monocular-inertial mode on the वेग drone.

## 📦 Requirements

- ROS Noetic
- OpenCV 4.x
- Pangolin
- Eigen, Sophus
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)

## 🔧 Setup

1. Clone and build ORB-SLAM3 (ROS version)
2. Place `ORBvoc.txt` vocabulary file in `Vocabulary/`
3. Use the launch file:

```bash
roslaunch orbslam3_ros monocular_inertial.launch
```

Make sure the camera and IMU topics are publishing as:
- `/usb_cam/image_raw`
- `/imu/data`

## 🔍 Calibration

The config file `EuRoC.yaml` contains camera intrinsics and IMU noise parameters. You may update them based on calibration results.
