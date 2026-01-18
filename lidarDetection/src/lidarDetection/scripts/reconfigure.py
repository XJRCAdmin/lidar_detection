import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QSlider, QCheckBox, QLineEdit, QPushButton,
                             QScrollArea, QGroupBox)
from PyQt5.QtCore import Qt, QThread
import os
import copy


class RosNodeThread(QThread):
    """在单独线程中运行 rclpy.spin"""
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()


class ParameterReconfigureGUI(QWidget):
    def __init__(self, config_path):
        super().__init__()
        self.config_path = config_path
        self.config = self.load_config()
        self.widgets = {}  # 存放参数名到widget的映射
        self.default_params = {}
        self.param_ranges = {
            'rotate_x': (-10.0, 10.0), 'rotate_y': (-10.0, 10.0), 'rotate_z': (-10.0, 10.0),
            'rotate_roll': (-3.14, 3.14), 'rotate_pitch': (-3.14, 3.14), 'rotate_yaw': (-3.14, 3.14),
            'roi_max_x': (0.0, 8.0), 'roi_max_y': (0.0, 8.0), 'roi_max_z': (0.0, 3.0),
            'roi_min_x': (-8.0, 0.0), 'roi_min_y': (-8.0, 0.0), 'roi_min_z': (-1.0, 0.0),
            'cluster_threshold': (0.1, 3.0), 'cluster_max_size': (10, 2000), 'cluster_min_size': (3, 50),
            'ground_threshold': (0.0, 1.0), 'voxel_grid_size': (0.05, 1.0),
            'displacement_threshold': (0.1, 5.0), 'iou_threshold': (0.1, 1.0),
            'statistical_nb_neighbors': (10, 100), 'statistical_std_ratio': (0.1, 4.0),
            'max_dimension_ratio': (5.0, 10.0),
            'min_x': (-1.0, 1.0), 'min_y': (-1.0, 1.0), 'min_z': (-1.0, 1.0),
            'max_x': (-1.0, 1.0), 'max_y': (-1.0, 1.0), 'max_z': (-1.0, 1.0),
            'ground_distance_threshold': (0.0, 5.0),
            'min_height_and_longest_edge_ratio': (0.0, 1.0),
            'box_max_length_threshold': (0.0, 10.0),
            'wall_distance_threshold': (0.0, 1.0),
            'ground_normal_angle_thresh_rad': (0.0, 1.0),
            'wall_normal_angle_thresh_rad': (0.0, 1.0)
        }
        self.param_descriptions = {
            'cluster_threshold': '点间最大距离(米),越大越粗糙,越小越精细',
            'cluster_max_size': '单个障碍物最大点数,越大可检测越大的障碍物',
            'cluster_min_size': '单个障碍物最小点数,越小越易检测小障碍物',
            
            'roi_max_x': 'ROI前向最大距离(米),越大检测范围越远',
            'roi_max_y': 'ROI横向最大距离(米),越大检测范围越宽',
            'roi_max_z': 'ROI高度最大距离(米),越大检测高度越高',
            'roi_min_x': 'ROI后向最小距离(米),越小检测范围越广',
            'roi_min_y': 'ROI横向最小距离(米),越小检测范围越宽',
            'roi_min_z': 'ROI高度最小距离(米),越小检测高度越低',
            
            'ground_threshold': '地面分割阈值(米),越大越容易将障碍物误判为地面',
            
            'voxel_grid_size': '体素网格大小(米),越大点云越稀疏,处理越快',
            
            'displacement_threshold': '位移阈值(米),越大允许目标移动距离越大',
            'iou_threshold': 'IOU阈值(0-1),越大要求框重叠度越高',
            
            'statistical_nb_neighbors': '统计邻域点数,越大越平滑但可能丢失细节',
            'statistical_std_ratio': '统计标准差比例,越大保留更多噪点',
            
            'rotate_x': 'X轴旋转角度(度),调整LiDAR俯仰',
            'rotate_y': 'Y轴旋转角度(度),调整LiDAR横滚',
            'rotate_z': 'Z轴旋转角度(度),调整LiDAR航向',
            'rotate_roll': '滚转角(弧度),调整LiDAR滚转',
            'rotate_pitch': '俯仰角(弧度),调整LiDAR俯仰',
            'rotate_yaw': '偏航角(弧度),调整LiDAR航向',
            
            'max_dimension_ratio': '边界框最大长宽高比例,越大允许长宽比越大的障碍物',
            
            'min_x': '自身点云过滤最小X值',
            'min_y': '自身点云过滤最小Y值',
            'min_z': '自身点云过滤最小Z值',
            'max_x': '自身点云过滤最大X值',
            'max_y': '自身点云过滤最大Y值',
            'max_z': '自身点云过滤最大Z值',
            
            'ground_distance_threshold': '过滤高于该高度的障碍物bbox',
            'min_height_and_longest_edge_ratio': 'bbox最小高度与最长边的比例',
            'box_max_length_threshold': 'bbox最大长度阈值(米)',
            
            'wall_distance_threshold': '点到墙壁平面的距离阈值',
            'ground_normal_angle_thresh_rad': '地面法向量与Z轴最大夹角(弧度)',
            'wall_normal_angle_thresh_rad': '墙面法向量与XY平面最大夹角(弧度)'
        }
        rclpy.init()
        self.node = Node('parameter_reconfigure_gui')
        self.param_client = self.node.create_client(SetParameters, 'obstacle_detector_node/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            print('set_parameters service not available, waiting...')

        self.ros_thread = RosNodeThread(self.node)
        self.ros_thread.start()

        self.init_ui()

    def load_config(self):
        if not os.path.exists(self.config_path):
            print(f"Error: Config file not found at {self.config_path}")
            return None
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)

    def init_ui(self):
        self.setWindowTitle('Obstacle Detector Reconfigure')
        self.setGeometry(100, 100, 800, 800)

        main_layout = QVBoxLayout()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)

        if not self.config or 'obstacle_detector_node' not in self.config:
            layout.addWidget(QLabel("Failed to load or parse config file."))
            self.setLayout(layout)
            return

        params = self.config['obstacle_detector_node']['ros__parameters']
        self.default_params = copy.deepcopy(params)

        layout.addWidget(self.create_group_box("LiDAR Transform", [
            'rotate_x', 'rotate_y', 'rotate_z', 'rotate_roll', 'rotate_pitch', 'rotate_yaw'
        ], params))

        layout.addWidget(self.create_group_box("Self Point Cloud Filter", [
            'min_x', 'min_y', 'min_z', 'max_x', 'max_y', 'max_z'
        ], params))

        layout.addWidget(self.create_group_box("ROI Filter", [
            'roi_max_x', 'roi_max_y', 'roi_max_z', 'roi_min_x', 'roi_min_y', 'roi_min_z'
        ], params))

        layout.addWidget(self.create_group_box("Clustering", [
            'cluster_threshold', 'cluster_max_size', 'cluster_min_size'
        ], params))

        layout.addWidget(self.create_group_box("Ground Segmentation", [
            'ground_threshold', 'ground_segment'
        ], params))

        layout.addWidget(self.create_group_box("Downsampling", [
            'voxel_grid_size'
        ], params))

        layout.addWidget(self.create_group_box("Tracking", [
            'use_tracking', 'displacement_threshold', 'iou_threshold'
        ], params))

        layout.addWidget(self.create_group_box("Statistical Outlier Filter", [
            'enable_statistical_filter', 'statistical_nb_neighbors', 'statistical_std_ratio'
        ], params))

        layout.addWidget(self.create_group_box("Height and Range Filter", [
            'enable_height_range_filter', 'ground_distance_threshold', 
            'min_height_and_longest_edge_ratio', 'box_max_length_threshold'
        ], params))

        layout.addWidget(self.create_group_box("Wall Segmentation", [
            'enable_wall_segmentation', 'wall_distance_threshold', 
            'ground_normal_angle_thresh_rad', 'wall_normal_angle_thresh_rad'
        ], params))

        layout.addWidget(self.create_group_box("Other", [
            'use_pca_box', 'max_dimension_ratio'
        ], params))

        # 恢复默认按钮
        btn_layout = QHBoxLayout()
        reset_btn = QPushButton("恢复默认")
        reset_btn.clicked.connect(self.reset_defaults)
        btn_layout.addStretch()
        btn_layout.addWidget(reset_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def create_group_box(self, title, param_names, params):
        group_box = QGroupBox(title)
        layout = QVBoxLayout()
        for name in param_names:
            if name in params:
                value = params[name]
                if isinstance(value, bool):
                    widget = self.create_checkbox(name, value)
                elif isinstance(value, str):
                    # 字符串类型参数，只显示不提供滑块
                    widget = self.create_label_only(name, value)
                else:
                    widget = self.create_slider(name, value)
                layout.addWidget(widget)
        group_box.setLayout(layout)
        return group_box

    def create_checkbox(self, name, value):
        container = QWidget()
        layout = QHBoxLayout(container)
        checkbox = QCheckBox(name)
        checkbox.setChecked(value)

        # stateChanged 处理：将 0/2（Qt）转换为 bool
        checkbox.stateChanged.connect(lambda state, n=name: self.set_parameter(n, bool(state)))

        layout.addWidget(checkbox)
        layout.addStretch()

        # 存储引用
        self.widgets[name] = {'type': 'bool', 'checkbox': checkbox}
        return container

    def create_slider(self, name, value):
        container = QWidget()
        layout = QHBoxLayout(container)

        if name in self.param_descriptions:
            label_text = f"{name}: {self.param_descriptions[name]}"
        else:
            label_text = f"{name}:"
        
        label = QLabel(label_text)
        label.setFixedWidth(300) 
        label.setWordWrap(True) 
        layout.addWidget(label)

        slider = QSlider(Qt.Horizontal)
        line_edit = QLineEdit(str(value))
        line_edit.setFixedWidth(80)

        if name in self.param_ranges:
            min_val, max_val = self.param_ranges[name]
        else:
            is_float_guess = isinstance(value, float)
            min_val, max_val = (0.0, 10.0) if is_float_guess else (0, 100)

        is_float = isinstance(value, float)
        multiplier = 100 if is_float else 1

        slider.setRange(int(min_val * multiplier), int(max_val * multiplier))
        slider.setValue(int(value * multiplier))

        # 当滑动改变时更新文本框显示
        slider.valueChanged.connect(
            lambda val, le=line_edit, m=multiplier, is_f=is_float:
            le.setText(f"{val/m:.2f}" if is_f else str(int(val)))
        )
        # 当释放滑块或回车时通过文本设置参数
        slider.sliderReleased.connect(lambda n=name, le=line_edit: self.set_parameter_from_text(n, le.text()))
        line_edit.returnPressed.connect(lambda n=name, le=line_edit: self.set_parameter_from_text(n, le.text()))

        layout.addWidget(slider)
        layout.addWidget(line_edit)

        self.widgets[name] = {
            'type': 'scalar',
            'slider': slider,
            'line_edit': line_edit,
            'multiplier': multiplier,
            'is_float': is_float
        }
        return container

    def create_label_only(self, name, value):
        """创建只显示的参数控件（用于字符串等非数值类型）"""
        container = QWidget()
        layout = QHBoxLayout(container)
        
        if name in self.param_descriptions:
            label_text = f"{name}: {self.param_descriptions[name]}"
        else:
            label_text = f"{name}:"
        
        label = QLabel(label_text)
        label.setFixedWidth(300)
        label.setWordWrap(True)
        layout.addWidget(label)
        
        value_label = QLabel(str(value))
        value_label.setFixedWidth(80)
        layout.addWidget(value_label)
        
        layout.addStretch()
        
        self.widgets[name] = {'type': 'string', 'label': label, 'value_label': value_label}
        return container

    def set_parameter_from_text(self, name, text_value):
        try:
            if '.' in text_value:
                value = float(text_value)
            else:
                value = int(text_value)

            if name in self.param_ranges:
                min_val, max_val = self.param_ranges[name]
                if value < min_val or value > max_val:
                    print(f"Value {value} for {name} out of range [{min_val}, {max_val}]")
                    return
            self.set_parameter(name, value)
        except ValueError:
            print(f"Invalid value for {name}: {text_value}")

    def set_parameter(self, name, value):
        try:
            parameters = []
            param = Parameter(name, value=value)
            parameters.append(param.to_parameter_msg())

            req = SetParameters.Request()
            req.parameters = parameters

            future = self.param_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

            if future.result() is not None:
                result = future.result()
                if result.results[0].successful:
                    print(f"Set parameter {name} to {value}")
                else:
                    print(f"Failed to set parameter {name}: {result.results[0].reason}")
            else:
                print(f"Service call failed for parameter {name}")
        except Exception as e:
            print(f"Exception when setting parameter {name}: {e}")

    def reset_defaults(self):
        """将所有参数恢复为 YAML 中的默认值并更新 UI"""
        if not self.default_params:
            print("No default params available to reset.")
            return

        for name, default_val in self.default_params.items():
            if name not in self.widgets:
                continue

            w = self.widgets[name]
            if w['type'] == 'bool':
                checkbox = w['checkbox']
                checkbox.blockSignals(True)
                checkbox.setChecked(bool(default_val))
                checkbox.blockSignals(False)
                self.set_parameter(name, bool(default_val))
            elif w['type'] == 'scalar':
                slider = w['slider']
                line_edit = w['line_edit']
                multiplier = w['multiplier']
                is_float = w['is_float']

                slider.blockSignals(True)
                if is_float:
                    slider.setValue(int(float(default_val) * multiplier))
                    line_edit.setText(f"{float(default_val):.2f}")
                else:
                    slider.setValue(int(default_val))
                    line_edit.setText(str(int(default_val)))
                slider.blockSignals(False)
                val_to_set = float(default_val) if is_float else int(default_val)
                self.set_parameter(name, val_to_set)
            elif w['type'] == 'string':
                value_label = w['value_label']
                value_label.setText(str(default_val))

        print("All displayed parameters restored to defaults.")

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()


if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file_path = os.path.join(script_dir, '..', 'config', 'go2.yaml')

    app = QApplication(sys.argv)
    gui = ParameterReconfigureGUI(config_file_path)
    gui.show()
    sys.exit(app.exec_())