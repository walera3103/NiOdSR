#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPointPublisher(Node):
    def __init__(self):
        super().__init__('camera_point_publisher')
        self.bridge = CvBridge()
        self.point = None

        self.subscription = self.create_subscription(
            Image,
            '/image', 
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Point, '/point', 10)
        self.window_name = "Camera"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("CameraPointPublisher started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        if self.point is not None:
            cv2.circle(cv_image, self.point, 5, (0, 255, 0), -1)

            pt_msg = Point()
            pt_msg.x = float(self.point[0])
            pt_msg.y = float(self.point[1])
            pt_msg.z = 0.0
            self.publisher_.publish(pt_msg)
            self.get_logger().info(f"Published point: ({pt_msg.x}, {pt_msg.y}, {pt_msg.z})")

        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)  

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)
            self.get_logger().info(f"Mouse click at: ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
