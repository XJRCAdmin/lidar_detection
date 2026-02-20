<div align="center">

<img src="img/lidar-detection.png" alt="lidar-detection" width="60%"/>

<p align="center">
  <a href="README.md">English</a> | <a href="README_ZH.md">中文</a>
</p>

[![License](https://img.shields.io/badge/License-CC%20BY%204.0-blue?style=for-the-badge)](https://creativecommons.org/licenses/by/4.0/)
[![Static Badge](https://img.shields.io/badge/arXiv-2602.07363-red?style=for-the-badge&logo=arxiv)](https://arxiv.org/abs/2602.07363)
![Static Badge](https://img.shields.io/badge/c%2B%2B-%2300599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Static Badge](https://img.shields.io/badge/ROS2-Humble-orange?style=for-the-badge&logo=ROS)

</div>

`foxy, ubuntu 20.04` and `humble ubuntu 22.04` are supported.for humble see 'humble' branch.

## Overview
## TODO
- [ ] 完善README


<div align="center">
  <video src="https://github.com/user-attachments/assets/8f529ec4-a3c2-4c4d-8a57-bd930af39ada" width="100%" controls></video>
</div>

## Usage
```
git clone https://github.com/XJRCAdmin/lidar_detection.git -b humble
cd lidar_detection
git submodule update --init  --recursive
colcon build --symlink-install
```

Debug:
```
colcon build \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Debug
```

## 记录rosbag

```bash
ros2 bag record -s mcap -a -o my_mcap_bag

```
- [lidarDetection README](lidarDetection/README.md)
- [CHANGELOG](lidarDetection/CHANGELOG.md)
- [yolo_realsense README](realsense/yolo_realsense/README.md)

# Acknowledgement
[lidar_obstacle_detector](https://github.com/SS47816/lidar_obstacle_detector)
