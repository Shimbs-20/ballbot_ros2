#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class EasyCameraNode(Node):
    def __init__(self):
        super().__init__('easy_camera')

        # 1. Declare parameters with default values
        self.declare_parameter('width', 840)
        self.declare_parameter('height', 680)
        self.declare_parameter('framerate', 30)

        # 2. Get the values of the parameters
        cam_width = self.get_parameter('width').value
        cam_height = self.get_parameter('height').value
        cam_fps = self.get_parameter('framerate').value

        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.bridge = CvBridge()

        # 3. Inject the variables into the GStreamer string using an f-string
        gstreamer_pipeline = (
            f"libcamerasrc ! "
            f"video/x-raw, width={cam_width}, height={cam_height}, framerate={cam_fps}/1 ! "
            f"videoconvert ! appsink"
        )

        self.get_logger().info(f"Attempting to start camera at {cam_width}x{cam_height} @ {cam_fps} FPS...")
        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

        # Fallback just in case the pipeline fails
        if not self.cap.isOpened():
            self.get_logger().warning("GStreamer failed. Falling back to /dev/video0...")
            self.cap = cv2.VideoCapture(0)

        self.get_logger().info("Python Camera Node Started! Streaming to /image_raw...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Failed to grab a frame from the camera!")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EasyCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
