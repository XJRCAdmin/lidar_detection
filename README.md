# ROS2 lidarDetection package

`foxy, ubuntu 20.04`

在workspace(不是`lidarDetection/`目录)下运行:
```
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
