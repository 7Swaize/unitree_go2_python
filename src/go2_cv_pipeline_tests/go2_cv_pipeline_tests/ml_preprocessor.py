import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (
    PointCloud2,
    Image,
    Imu
)
from cv_bridge import (
    CvBridge,
    CvBridgeError
)


class MLPreprocessor(Node):
    def __init__(self):
        super().__init__('ml_preprocessor')

        self._unitree_lidar_sub = self.create_subscription(
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

        self._velodyne_points_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic='/velodyne_points/points',
            callback=self._velodyne_callback,
            qos_profile=10
        )

        self._imu_sub = self.create_subscription(
            msg_type=Imu,
            topic='/imu/data',
            callback=self._imu_callback,
            qos_profile=10
        )

        self._odometry_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self._odometry_callback,
            qos_profile=10
        )

        self.cv_bridge = CvBridge()


    def _lidar_callback(self, msg: PointCloud2):
        pass

    def _camera_callback(self, msg: Image):
        pass

    def _velodyne_callback(self, msg: PointCloud2):
        pass

    def _imu_callback(self, msg: Imu):
        pass

    def _odometry_callback(self, msg: Odometry):
        pass
