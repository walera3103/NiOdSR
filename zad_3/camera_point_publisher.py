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

        self.declare_parameter('square_size', 200)
        self.square_size = self.get_parameter('square_size').value
        self.get_logger().info(f'Square size set to: {self.square_size}')

        self.publisher = self.create_publisher(Point, '/point', 10)

        self.bridge = CvBridge()
        self.point = None
        self.window_name = "camera"
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', 
            self.listener_callback,
            10
        )

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info('CameraPointPublisher started')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        if self.point is not None:
            x, y = self.point
            cv2.rectangle(cv_image, (x, y),
                          (x + self.square_size, y + self.square_size),
                          (0, 255, 0), 2)

            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0
            self.publisher.publish(point_msg)

        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)


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
