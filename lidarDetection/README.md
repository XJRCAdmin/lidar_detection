# lidar Detection

> [!TIP]
> humble,ubuntu 22.04

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
|   |── Point_LIO
|   |── README.md
```

## Parameters
Main Parameters can be found in [`config/go2.yaml`](config/go2.yaml),and the parameters are explained in the comments in the yaml file,and currently fits mid360 lidar, you can adjust them according to your own application scenarios. 

Some other parameters about kalman filter can be found in [`include/ukf.hpp`](include/ukf.hpp).

## Modification of fastlio and Point_LIO(compared to the original github repository)

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
## dynamic reconfigure
在开启obstacle_detector_node节点前,运行以下命令启动动态参数配置界面:
```bash
./lidarDetection/scripts/reconfigure.sh
```
ROS2 foxy没有像ROS1 的dynamic_reconfigure package,所以这里使用了一个自定义的动态参数配置界面,可以动态调整点云预处理和聚类的相关参数,但是调整的手感感觉稀烂,有时候参数会设置失败.目前在`go2.yaml`的参数是一组效果还可以的参数.


