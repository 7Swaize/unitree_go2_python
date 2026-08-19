import numpy as np
import numpy.typing as npt
from typing import Any, Optional
from dataclasses import dataclass
import fast_pointcloud as fp

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from .lidar_bridge import LidarBridge


POINTFIELD_TO_INTERNAL_CTYPE = {
    PointField.INT8: fp.PointFieldType["INT8"],
    PointField.UINT8: fp.PointFieldType["UINT8"],
    PointField.INT16: fp.PointFieldType["INT16"],
    PointField.UINT16: fp.PointFieldType["UINT16"],
    PointField.INT32: fp.PointFieldType["INT32"],
    PointField.UINT32: fp.PointFieldType["UINT32"],
    PointField.FLOAT32: fp.PointFieldType["FLOAT32"],
    PointField.FLOAT64: fp.PointFieldType["FLOAT64"]
}


@dataclass
class CollectionConfig:
    optimize_collection: bool
    skip_nans: bool 


@dataclass(frozen=True)
class PointCloudLayout:
    has_intensity: bool
    x_offset: int
    y_offset: int
    z_offset: int
    intensity_offset: int
    xyz_internal_type: int
    intensity_internal_type: int


class LidarDecoderNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_decoder")
        self._declare_parameters()
        self._config = self._load_configuration()

        self._qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        ) 

        self._pc_layout: Optional[PointCloudLayout] = None
        self._bridge = LidarBridge()

        self.setup_subscriptions()
    

    def _declare_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("collection.optimize_collection", rclpy.Parameter.Type.BOOL),
                ("collection.skip_nans", rclpy.Parameter.Type.BOOL),
            ]
        )


    def _load_configuration(self) -> CollectionConfig:
        optimize_collection = self.get_parameter('collection.optimize_collection').get_parameter_value().bool_value 
        skip_nans = self.get_parameter('collection.skip_nans').get_parameter_value().bool_value 

        return CollectionConfig(
            optimize_collection=optimize_collection,
            skip_nans=skip_nans,
        )


    def setup_subscriptions(self) -> None:
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            "/utlidar/cloud",
            self.lidar_callback_optimized if self._config.optimize_collection else self.lidar_callback_unoptimized,
            self._qos_profile
        )


    def _get_layout(self, msg: PointCloud2) -> PointCloudLayout:
        if self._pc_layout is None:
            self._pc_layout = self._init_pointcloud_layout(msg)
        return self._pc_layout


    def _init_pointcloud_layout(self, msg: PointCloud2) -> PointCloudLayout:
        fields: dict[str, PointField] = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            raise ValueError("PointCloud2 missing XYZ fields")

        dtype_xyz = fields["x"].datatype
        if not all(fields[k].datatype == dtype_xyz for k in ("x", "y", "z")):
            raise TypeError("Mixed XYZ datatypes not supported")
        
        xyz_internal = POINTFIELD_TO_INTERNAL_CTYPE.get(dtype_xyz)
        if xyz_internal is None:
            raise TypeError(f"Unsupported XYZ datatype: {dtype_xyz}")

        has_intensity = "intensity" in fields
        if has_intensity:
            intensity_dtype = fields["intensity"].datatype
            intensity_internal = POINTFIELD_TO_INTERNAL_CTYPE.get(intensity_dtype)
            if intensity_internal is None:
                raise TypeError(f"Unsupported intensity datatype: {intensity_dtype}")
            intensity_offset = fields["intensity"].offset
        else:
            intensity_internal = fp.PointFieldType["INT8"]
            intensity_offset = -1

        return PointCloudLayout(
            has_intensity=has_intensity,
            x_offset=fields["x"].offset,
            y_offset=fields["y"].offset,
            z_offset=fields["z"].offset,
            intensity_offset=intensity_offset,
            xyz_internal_type=xyz_internal,
            intensity_internal_type=intensity_internal
        )


    def lidar_callback_unoptimized(self, msg: PointCloud2) -> None:
        try:
            layout = self._get_layout(msg)

            if layout.has_intensity:
                data = point_cloud2.read_points_numpy(
                    msg,
                    field_names=["x", "y", "z", "intensity"],
                    skip_nans=self._config.skip_nans
                ).astype(np.float32, copy=False)
            else:
                data = point_cloud2.read_points_numpy(
                    msg,
                    field_names=["x", "y", "z"],
                    skip_nans=self._config.skip_nans
                ).astype(np.float32, copy=False)

            self._send_to_bridge(data, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR data: {e}")
            

    def lidar_callback_optimized(self, msg: PointCloud2) -> None:
        try:
            layout = self._get_layout(msg)
            data = fp.decode_xyz_intensity(
                msg.data,
                msg.point_step,
                layout.x_offset,
                layout.y_offset,
                layout.z_offset,
                layout.intensity_offset,
                msg.is_bigendian,
                layout.xyz_internal_type,
                layout.intensity_internal_type,
                self._config.skip_nans
            )

            self._send_to_bridge(data, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR data: {e}")
            

    def _send_to_bridge(self, points: npt.NDArray[np.float32], src_pc_header: Header) -> None:
        stamp_ns = src_pc_header.stamp.sec * 1_000_000_000 + src_pc_header.stamp.nanosec
        points = np.asfortranarray(points.T)

        self._bridge.send_decoded(stamp_ns, points)


    def shutdown_ext(self) -> None:
        self._bridge.shutdown()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = LidarDecoderNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f"Error running lidar decoder: {e}")
    finally:
        node.shutdown_ext()
        node.destroy_node() 
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
