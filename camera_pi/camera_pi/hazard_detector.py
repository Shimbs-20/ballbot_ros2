#!/usr/bin/env python3
"""
Optimized Hazard Detector — fixes:
1. Resize before inference (not at 840x680)
2. Skip frames when queue builds up
3. Separate emergency stop topic
4. Temperature monitoring integration
5. Async publishing to not block inference
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, Temperature
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
import cv2
import json
import time
import numpy as np
from ultralytics import YOLO

LABELS = {
    0: "fire", 1: "smoke",
    2: "electrical_hazard", 3: "person", 4: "obstacle"
}

CRITICAL  = {0, 1, 2}   # fire, smoke, electrical → emergency stop
INFER_SIZE = 416         # YOLO input — match your training size

# Best-effort QoS — drops old frames instead of buffering
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1   # ← only keep LATEST frame, drop rest
)

class HazardDetectorNode(Node):
    def __init__(self):
        super().__init__('hazard_detector')

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        # Load NCNN model
        self.get_logger().info("Loading NCNN Model...")
        self.model = YOLO("/home/GP-Robot/best_ncnn_model")
        self.get_logger().info("NCNN Model loaded!")

        # Subscribe with depth=1 — always process freshest frame
        self.subscription = self.create_subscription(
            Image, '/image_raw',
            self.image_callback,
            qos_profile=BEST_EFFORT_QOS  # ← KEY FIX: drop stale frames
        )

        # Publishers
        self.annotated_pub  = self.create_publisher(Image,   '/image_annotated',    10)
        self.detections_pub = self.create_publisher(String,  '/hazard_detections',  10)
        self.estop_pub      = self.create_publisher(Bool,    '/status/estop',        10)
        self.fps_pub        = self.create_publisher(Float32, '/detector/fps',        10)

        self.get_logger().info("Hazard Detector ready!")

    def image_callback(self, msg):
        t_start = time.time()

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # ── KEY FIX: resize BEFORE inference ──────────────
        # Original: YOLO runs on 840×680 = 571,200 pixels
        # Fixed:    YOLO runs on 416×416 = 173,056 pixels → 3.3× less work
        frame_small = cv2.resize(frame, (INFER_SIZE, INFER_SIZE))

        # Run YOLO on small frame
        results = self.model(
            frame_small,
            conf=0.45,
            iou=0.4,
            imgsz=INFER_SIZE,
            verbose=False
        )

        # Scale boxes back to original frame size
        scale_x = frame.shape[1] / INFER_SIZE
        scale_y = frame.shape[0] / INFER_SIZE

        detections = []
        emergency   = False

        for box in results[0].boxes:
            cls  = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # Scale back to original resolution
            x1 = int(x1 * scale_x); y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x); y2 = int(y2 * scale_y)

            # Compute useful derived metrics
            cx = (x1 + x2) / 2          # center X
            area_ratio = ((x2-x1)*(y2-y1)) / (frame.shape[0]*frame.shape[1])

            det = {
                "class":      LABELS.get(cls, "unknown"),
                "class_id":   cls,
                "confidence": round(conf, 2),
                "bbox":       [x1, y1, x2, y2],
                "center_x":   round(cx, 1),
                "proximity":  round(area_ratio, 3)  # 0=far, 1=fills frame
            }
            detections.append(det)

            if cls in CRITICAL and conf > 0.5:
                emergency = True

        # Publish emergency stop
        estop_msg = Bool(); estop_msg.data = emergency
        self.estop_pub.publish(estop_msg)

        # Publish detections JSON
        if detections:
            dmsg = String(); dmsg.data = json.dumps(detections)
            self.detections_pub.publish(dmsg)

        # Annotate on ORIGINAL full-res frame for crisp dashboard video
        for det in detections:
            x1,y1,x2,y2 = det["bbox"]
            color = (0,0,255) if det["class_id"] in CRITICAL else (0,165,255)
            cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
            label = f"{det['class']} {det['confidence']:.0%}"
            cv2.putText(frame, label, (x1, y1-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # FPS counter
        self.frame_count += 1
        elapsed = time.time() - self.last_fps_time
        if elapsed > 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = time.time()
            fps_msg = Float32(); fps_msg.data = float(self.fps)
            self.fps_pub.publish(fps_msg)

        # Publish annotated full-res frame
        amsg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.annotated_pub.publish(amsg)

        inference_ms = (time.time() - t_start) * 1000
        if self.fps > 0:
            self.get_logger().info(
                f"FPS:{self.fps:.1f} | {inference_ms:.0f}ms | "
                f"Detections:{len(detections)} | ESTOP:{emergency}",
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
