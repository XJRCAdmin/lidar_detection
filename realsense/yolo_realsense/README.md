首先安装[realsense sdk2](https://github.com/vanderbiltrobotics/realsense-ros?tab=readme-ov-file#step-2-install-librealsense2).

一些依赖:
```
sudo apt install ros-foxy-rqt-image-view
sudo apt install ros-foxy-sensor-msgs
pip install -r requirements.txt
```
# Foxy 不被realsense-ros最新支持版本支持，需切换4.0.4版本：
```
git fetch --all --tags
# 切换到 4.0.4 或更早版本，这是支持 Foxy 的最后一个稳定大版本
git checkout 4.0.4
```

# run realsense + yolo detector
```
ros2 launch yolo_realsense camera_yolo.launch.py
```
纯yolo.pt;推理用时:
```
[yolo_realsense_node-2] [INFO] [1766913026.079859364] [yolov11_node]: YOLO Time: Total 161.84ms (Pre: 3.01ms, Infer: 157.29ms, Post: 1.54ms)
```