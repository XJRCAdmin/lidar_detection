import torch
import torch.onnx
from pathlib import Path
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
model.export(format="onnx", imgsz=640, opset=11, simplify=True)
print("Exported to yolov11n.onnx")

# ov_fp32_dir = model.export(format="openvino", imgsz=640, half=False)
# print(f"OpenVINO FP32 IR 导出目录: {ov_fp32_dir}")
