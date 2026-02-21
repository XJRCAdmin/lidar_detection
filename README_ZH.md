# 概览

> [!NOTE]
> 本仓库支持 `humble ubuntu 22.04`，更精确地说，已在 `x86_64` 架构上测试通过。关于 `foxy ubuntu 20.04`（尚未完成）的内容，请查看 `foxy` 分支。

本仓库包含一个基于激光雷达与相机的障碍物检测系统实现。完整检测系统包括一个 YOLO–RealSense 的二维检测器和一个基于几何的点云包围盒激光雷达障碍物检测器，旨在利用激光雷达数据（当前仅支持 **Mid360**）与视觉信息实时检测与分类障碍物。四足机器人 “Uerebot” 的所有感知相关代码及文件结构位于此处：[uerebot perception code](https://github.com/XJRCAdmin/lidar_detection)。

`./lidar_detection/src/lidarDetection` 文件夹是一个用于基于激光雷达障碍检测的 ROS 2 包，其中包含基于 Mid360 的激光雷达障碍检测器，具备以下特性：

- 可定制的障碍检测感兴趣区域（ROI）
- 基于卡尔曼滤波的障碍跟踪
- 为便于调参以适配不同应用，算法的所有关键参数均可通过 `.yaml` 文件中的 ros param 在线调整
- 多种点云滤波以降低噪声并去除离群点
- 地面平面与障碍点云分割

整个系统演示见下方视频：

<div align="center">
  <video src="https://github.com/user-attachments/assets/8f529ec4-a3c2-4c4d-8a57-bd930af39ada" width="100%" controls></video>
</div>

## 待办事项（TODOs）

- [ ] 更健壮的地面分割算法
- [ ] 完善 foxy 分支

# 整体感知系统（本仓库）

你可以仅使用 lidar_detection 包（见 `./lidarDetection/src/lidarDetection`）来使用激光雷达数据检测障碍物，或者使用包含激光雷达障碍检测与 YOLO-RealSense 2D 检测的完整感知系统。基于我们的实验与部署经验，我们采用了将激光雷达与相机处理解耦的感知策略。两部分彼此独立，你可以根据需求选择只用其中之一或两者同时使用。

## 激光雷达障碍检测部分

```bash
git clone https://github.com/XJRCAdmin/lidar_detection.git -b humble
cd lidar_detection/lidarDetection
```

你可能需要安装一些依赖（如 rviz2、rqt），具体请在 `colcon build` 时根据依赖的报错提示进行安装：

```bash
sudo apt update
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions libpcl-dev
```

然后使用以下命令构建包：

```bash
colcon build --symlink-install
ros2 launch lidar_detection go2.launch.py
```

关于激光雷达障碍检测部分的更多信息，如参数设置、流程与文件说明，请参阅 [lidarDetection README](lidarDetection/README.md)。

### rqt Graph

![](./lidarDetection/src/lidarDetection/static/nodes.png)

## Yolo-RealSense 2D 检测部分

```bash
git clone https://github.com/XJRCAdmin/lidar_detection.git -b humble # 同上
cd lidar_detection/realsense
git submodule update --init  --recursive # 初始化 realsense 子模块

cd realsense-ros
git fetch --all --tags
git checkout 4.56.4 # 支持 ros2 humble 的 realsense-ros 版本
```

你可能还需要安装一些依赖：

安装 RealSense SDK 2.0：见文档链接 [link](https://github.com/vanderbiltrobotics/realsense-ros?tab=readme-ov-file#step-2-install-librealsense2)

另外：

```bash
sudo apt install ros-foxy-rqt-image-view
sudo apt install ros-foxy-sensor-msgs
pip install -r lidar_detection/realsense/yolo_realsense/requirements.txt
```

然后使用以下命令构建包：

```bash
colcon build --symlink-install 
```

运行以下命令启动 yolo_realsense 包：

```bash
ros2 launch yolo_realsense camera_yolo.launch.py
```

关于 yolo_realsense 部分的更多信息，如参数设置，请参阅 [yolo_realsense README](./realsense/yolo_realsense/README.md)。

## 接口文件夹

与下游模块交互的消息话题使用了自定义消息格式。这些格式属于 Uerebot 项目特定定义，不具有通用参考价值。

## 问与答

**问**：为什么不采用将激光雷达与相机数据融合的方案，而要把它们分成两个独立流水线？

**答**：将激光雷达与 RealSense 相机感知部分分离的原因包括：

* 由于边缘计算平台（如 NVIDIA Jetson）的算力瓶颈与严格的实时性要求，整体感知帧率（FPS）出现了不可接受的下降。此外，消息缓存队列管理与时间戳同步也变得棘手。
* 四足机器人特有的高频振动以及反射引起的伪影，导致 FAST-LIO 与 Point-LIO 的里程计漂移（即点云漂移），进而使基于融合的配准失败并产生空间错位。
* 感知视野（FOV）的性价比也是关键考量。虽然 Mid360 提供 360° 全向覆盖，但前置 RealSense 相机的有效水平视场仅约 80°–90°。采用融合方案只能在正前方获得高置信度的数据，而剩余 270° 仍需独立的仅基于雷达的几何检测流水线。这样的“半融合、半解耦”混合架构会给边缘设备带来额外计算负担——需要同时运行 YOLO、前向融合节点以及独立的全向雷达处理线程。与其维持这种割裂的系统，不如采用彻底解耦的设计，充分发挥 Mid360 的全向几何优势，更加高效且实用。

**问**：为什么不使用基于学习的激光雷达检测方法？

**答**：尽管基于学习的 3D 激光雷达检测方法（如 PointPillars、CenterPoint）在自动驾驶基准上表现优异，但将它们部署到像 Uerebot 这样的四足机器人上，会带来显著的领域差异与工程瓶颈。
首先，在 KITTI 或 nuScenes 等数据集上训练的模型高度适配车顶旋转式激光雷达的水平扫描模式，难以泛化到 Mid360 这种非重复、密集玫瑰花瓣式扫描模式以及贴近地面的视角。
其次，基于学习的方法通常是闭集的，关注预定义类别（例如车辆、行人）。而我们的四足机器人需要避让开放世界中、不规则的几何障碍（例如岩石、散落箱子、低台阶）——这类目标很难通过有限类别的学习模型覆盖。
