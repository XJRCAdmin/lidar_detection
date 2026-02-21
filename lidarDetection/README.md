# lidar Detection

> [!TIP]
> Fits for: humble,ubuntu 22.04

## File structure
```
lidarDetection
├── src
│   ├── lidarDetection
|   |        |── config
|   |        |     └── go2.yaml # important parameters for tuning the algorithm.
|   |        |── launch
|   |        |     └── go2.launch.py # launch file for lidar obstacle detection.
|   |        |── src
|   |        |    |── odom_to_baselink_transformer.cpp # the node for transforming the odom msg topic to baselink position,which doesn't important.
|   |        |    |── interface.cpp # interface of upper and lower level codes in uerebot project.
|   |        |    |── obstacle_to_other_frame_transformer.cpp # the node for transforming the obstacle position from lidar frame to other frame,use 'visualization_mode' in go2.yaml to enable which mode(frame) you want to visualize bbox.
|   |        |    |── map_launcher.cpp # the node for loading static map, and publishing the map to the map topic, you need to use fastlio or Point_LIO to build the static map before using it.
|   |        |    └── obstacle_detector.cpp # the main node for lidar obstacle detection, including point cloud pre-processing, clustering and obstacle information publishing.
|   |        |── include
|   |        |     |── lidar_detection 
|   |        |            |── ukf.hpp # header file for ukf.cpp, which implements the unscented kalman filter for obstacle tracking,some parameters of kalman filter can be tuned in it.
|   |        |            └── obstacle_detector.hpp # header file for obstacle_detector.cpp.
|   |        |
|   |        |── static
|   |        |── msgs
|   |        |── rviz
|   |── fastlio 
|   |── lidarSORT # Original version of lidar detection , Currently unavailable to use.
|   |── livox_ros_driver2
|   └── Point_LIO
├── scripts
|── README.md
└── CHANGELOG.md
```

## Parameters
Main Parameters can be found in [`./src/lidarDetection/config/go2.yaml`](config/go2.yaml),and the parameters are explained in the comments in the yaml file,and currently fits mid360 lidar, you can adjust them according to your own application scenarios. 

Some other parameters about kalman filter can be found in [`include/ukf.hpp`](include/ukf.hpp).

## Modification of fastlio and Point_LIO(compared to the original github repository)
### FAST_LIO
We use Pointcloud2 message for detection, so the `lidar_type` parameter in `config/mid360.yaml` is set to 2.

`config/mid360.yaml`:
```yaml
lidar_type: 2                # (modifed) for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 4 for any other pointcloud input
map_file_path:    "xxx/xxx"  # (modified) the path of the static map built by FAST_LIO, which will be loaded by the map_launcher node and published to the map topic.
```

`launch/mapping.launch.py`:
```python
  #  ld.add_action(rviz_node)
```
In addition, we modified the frame naming convention. In the original FAST-LIO repository, the reference frames were `camera_init` and `body`. To avoid ambiguity (e.g., whether `body` refers to the robot body or the LiDAR body), we renamed them to `odom` and `lidar_body`, respectively, so as to better align with the lidarDetection codebase.

### Point_LIO
The original repository is based on [Point_LIO](https://github.com/dfloreaa/point_lio_ros2) repository.(**Not hku mars Point_LIO repository**).

We use Pointcloud2 message for detection, so the `lidar_type` parameter in `config/mid360.yaml` is set to 2.
```yaml
lidar_type: 2      # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR
```

In addition, we modified the frame naming convention. In the original Point_LIO repository, the reference frames were `camera_init` and `body`. To avoid ambiguity (e.g., whether `body` refers to the robot body or the LiDAR body), we renamed them to `odom` and `lidar_body`, respectively, so as to better align with the lidarDetection codebase.

## livox_ros_driver2
`launch_ROS2/msg_MID360_launch.py`:
```python
xfer_format = 0
publish_freq = 15.0  # original 10.0 is also ok
```

`config/MID360_config.json`(lots of tutorials on the internet):
```json
  "lidar_type": 8 
  "ip" : "192.168.1.155" // change it to your lidar ip
```

## Dynamic Reconfigure

Before launching the `obstacle_detector_node`, run the following command to start the dynamic parameter configuration interface:

```bash
cd <workspace>
./lidarDetection/scripts/reconfigure.sh
```

Since ROS 2 Foxy does not provide a package equivalent to ROS 1’s `dynamic_reconfigure`, a custom dynamic parameter configuration interface is implemented here. It allows real-time tuning of parameters related to point cloud preprocessing and clustering. However, after adjustment, you need to manually write the updated parameters back to the `go2.yaml` file :)

The current parameters in `go2.yaml` have been empirically tuned and provide satisfactory performance.

# log
Lidar点云障碍物检测,并发布障碍物位置话题.
![](src/lidarDetection/static/rviz.png)
## dependency
```bash
sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions libpcl-dev
git submodule update --init --recursive
wget http://fishros.com/install -O fishros && . fishros # 选择安装rosdep

cd <workspace>
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro foxy
```

## build
```bash
colcon build --symlink-install --paths .
colcon build --symlink-install --packages-select sensing_msgs
```
![](src/lidarDetection/static/rqt.png)


