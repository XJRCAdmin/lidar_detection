import torch
import torch.onnx
from pathlib import Path
from ultralytics import YOLO

model = YOLO("yolov11s.pt")
model.export(format="onnx", imgsz=640, opset=11, simplify=True)
print("Exported to yolov11s.onnx")