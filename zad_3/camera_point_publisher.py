import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

class CameraPointPublisher(Node):
    def __init__(self):
        super().__init__('camera_point_publisher')

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(Point, '/point', 10)

        self.bridge = CvBridge()
        self.cv_image = None

        self.get_logger().info('CameraPointPublisher started')

    def image_callback(self, data):
        # Convert ROS image to OpenCV format
        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Show window
        cv2.imshow('Camera', self.cv_image)
        cv2.setMouseCallback('Camera', self.on_mouse_click)

        # Required for OpenCV event loop
        cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if self.cv_image is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            # Draw a dot on the image
            cv2.circle(self.cv_image, (x, y), 5, (0, 255, 0), -1)

            # Publish point coordinates
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            self.publisher.publish(point)

            self.get_logger().info(f'Clicked point published: ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
