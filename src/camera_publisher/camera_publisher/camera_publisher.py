#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 FPS
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            raise RuntimeError("Cannot open camera")
            
        self.get_logger().info("Camera publisher node started")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Can't receive frame (stream end?). Exiting...")
            return
            
        # Convert OpenCV image to ROS2 message
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()