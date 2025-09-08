import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import (
    PointCloud2,
    Image
)
from cv_bridge import (
    CvBridge,
    CvBridgeError
)


class Go2LidarTest(Node):
    def __init__(self):
        super().__init__('go2_cv_test')

        self._lidar_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic='/unitree_lidar/points',
            callback=self._lidar_callback,
            qos_profile=10
        )

        self._camera_sub = self.create_subscription(
            msg_type=Image,
            topic='/rgb_image',
            callback=self._camera_callback,
            qos_profile=10
        )

        self.cv_bridge = CvBridge()


    def _lidar_callback(self, msg: PointCloud2):
        pass


    def _camera_callback(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            edges = self._get_canny_edges(cv_image)

            cv2.imshow('Camera Image', edges)
            cv2.waitKey(1)          
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')


    def _get_canny_edges(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return cv2.Canny(gray, 100, 200)


def main(args=None):
    rclpy.init(args=args)

    node = Go2LidarTest()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()