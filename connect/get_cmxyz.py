from ultralytics import YOLO
import cv2
from collections import defaultdict

# 模型配置
model = YOLO('/home/jetson/xxf/engine/best111.pt')
FOCAL_LENGTH = 1530
CLASS_WIDTHS = {
    "ear": 9.0,    # 单位：cm
    "green": 9.0,
    "yellow": 14.0
}

class DetectionSystem:
    def __init__(self):
        self.cap = None
        self._init_camera()

    def _init_camera(self):
        """Jetson CSI摄像头初始化"""
        self.cap = cv2.VideoCapture(
            'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, '
            'format=NV12, framerate=20/1 ! nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! appsink',
            cv2.CAP_GSTREAMER
        )


    
    def _calculate_distance(self, cls_name, pixel_width):
        """距离计算核心方法"""
        return (CLASS_WIDTHS[cls_name] * FOCAL_LENGTH) / pixel_width

    def get_target_detections(self, target_class, sample_frames=20):
        """定向检测指定类别"""
        # 验证输入类别有效性
        if target_class not in CLASS_WIDTHS:
            raise ValueError(f"无效类别，可选类别: {list(CLASS_WIDTHS.keys())}")
        
        # 初始化数据存储
        bboxes = []
        distances = []
        
        # 采集指定帧数
        for _ in range(sample_frames):
            ret, frame = self.cap.read()
            if not ret: continue

            # 执行检测并处理结果
            results = model(frame, verbose=False)
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                cls_name = model.names[int(box.cls)]
                
                # 仅处理目标类别
                if cls_name == target_class:
                    distance = self._calculate_distance(cls_name, x2-x1)
                    bboxes.append((x1, y1, x2, y2))
                    distances.append(distance)
        
        # 结果处理
        if len(bboxes) == 0:
            return False
        
        # 计算平均数据
        avg_box = [
            int(sum(coords) / len(bboxes))
            for coords in zip(*bboxes)
        ]
        avg_distance = round(sum(distances) / len(distances), 1)
        
        return {
            'target_class': target_class,
            'avg_bbox': tuple(avg_box),
            'avg_distance': avg_distance
        }

    def get_specific_class_average(self, target_class, sample_frames=20):
        """新增方法：定向检测指定类别 (保持原有函数不变)"""
        # 验证输入有效性
        if target_class not in CLASS_WIDTHS:
            raise ValueError(f"无效类别，可选: {list(CLASS_WIDTHS.keys())}")

        # 数据收集容器
        target_data = {
            'bboxes': [],
            'distances': []
        }

        # 采集指定帧数
        for _ in range(sample_frames):
            ret, frame = self.cap.read()
            if not ret: continue

            # 复用原有检测流程
            results = model(frame, verbose=False)
            current_detections = self._process_detections(results)  # 使用原函数

            # 筛选目标数据
            for cls_name, bbox, distance in current_detections:
                if cls_name == target_class:
                    target_data['bboxes'].append(bbox)
                    target_data['distances'].append(distance)

        # 结果判断
        if not target_data['bboxes']:
            return None

        # 计算平均值（复用原有逻辑）
        avg_box = [
            int(sum(coords) / len(target_data['bboxes']))
            for coords in zip(*target_data['bboxes'])
        ]
        avg_distance = round(
            sum(target_data['distances']) / len(target_data['distances']), 
            1
        )

        return {
            'class': target_class,
            'bbox': tuple(avg_box),
            'distance': avg_distance
        }

    def __del__(self):
        """资源清理"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
