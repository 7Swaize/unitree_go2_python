import unittest
import numpy as np
from unittest.mock import MagicMock, patch

from sensor_msgs.msg import PointField, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from lidar_processor.lidar_decoder_node import LidarDecoderNode, CollectionConfig


def make_node(optimize_collection: bool, skip_nans: bool = True) -> LidarDecoderNode:
    with patch('lidar_processor.lidar_decoder_node.Node.__init__', return_value=None):
        node = LidarDecoderNode.__new__(LidarDecoderNode)
        node._config = CollectionConfig(optimize_collection=optimize_collection, skip_nans=skip_nans)
        node._pc_layout = None
        node._bridge = MagicMock()
        node.get_logger = MagicMock(return_value=MagicMock(
            info=lambda msg: None,
            warn=lambda msg: None,
            error=lambda msg: None,
            debug=lambda msg: None,
        ))
        return node
    

def make_pointcloud2(xyz: np.ndarray, intensity: np.ndarray = None) -> PointCloud2:
    header = Header(frame_id='lidar')
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    if intensity is not None:
        fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
        points = np.hstack([
            xyz.astype(np.float32),
            intensity.reshape(-1, 1).astype(np.float32)
        ])
    else:
        points = xyz.astype(np.float32)

    return point_cloud2.create_cloud(header, fields, points)


class DecoderBehaviorMixin:
    optimize_collection: bool

    def setUp(self):
        self.node = make_node(optimize_collection=self.optimize_collection)

    def _run(self, cloud: PointCloud2) -> None:
        if self.optimize_collection:
            self.node.lidar_callback_optimized(cloud)
        else:
            self.node.lidar_callback_unoptimized(cloud)

    def test_forwards_xyz_only_points_to_bridge(self):
        xyz = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ], dtype=np.float32)
        cloud = make_pointcloud2(xyz)

        self._run(cloud)

        self.node._bridge.send_decoded.assert_called_once()
        _, points = self.node._bridge.send_decoded.call_args[0]
        self.assertEqual(points.shape, (3, 3))

        np.testing.assert_allclose(points, xyz, atol=1e-4)
        self.assertTrue(points.flags.c_contiguous)
        self.assertEqual(points.dtype, np.float32)

    def test_forwards_xyzi_points_to_bridge(self):
        xyz = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
        ], dtype=np.float32)
        intensity = np.array([100.0, 200.0], dtype=np.float32)
        cloud = make_pointcloud2(xyz, intensity)

        self._run(cloud)

        self.node._bridge.send_decoded.assert_called_once()
        _, points = self.node._bridge.send_decoded.call_args[0]
        self.assertEqual(points.shape, (2, 4))

        np.testing.assert_allclose(points[:, :3], xyz, atol=1e-4)
        np.testing.assert_allclose(points[:, 3], intensity, atol=1e-4)
        self.assertTrue(points.flags.c_contiguous)
        self.assertEqual(points.dtype, np.float32)

    def test_handles_empty_cloud_with_inten(self):
        xyz = np.empty((0, 3), dtype=np.float32)
        intensity = np.empty((0,), dtype=np.float32)
        cloud = make_pointcloud2(xyz, intensity)

        self._run(cloud)

        self.node._bridge.send_decoded.assert_called_once()
        _, points = self.node._bridge.send_decoded.call_args[0]

        self.assertEqual(points.shape, (0, 4))
        self.assertTrue(points.flags.c_contiguous)
        self.assertEqual(points.dtype, np.float32)

    def test_drops_nan_rows_so_only_valid_points_are_forwarded(self):
        xyz = np.array([
            [1.0, 2.0, 3.0],
            [np.nan, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ], dtype=np.float32)
        cloud = make_pointcloud2(xyz)

        self._run(cloud)

        self.node._bridge.send_decoded.assert_called_once()
        _, points = self.node._bridge.send_decoded.call_args[0]

        self.assertEqual(points.shape, (2, 3))
        np.testing.assert_allclose(points, xyz[[0, 2], :], atol=1e-4)
        self.assertTrue(points.flags.c_contiguous)
        self.assertEqual(points.dtype, np.float32)

    def test_uses_correct_stamp(self):
        xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud = make_pointcloud2(xyz)
        cloud.header.stamp.sec = 5
        cloud.header.stamp.nanosec = 500

        self._run(cloud)

        stamp_ns, _ = self.node._bridge.send_decoded.call_args[0]
        self.assertEqual(stamp_ns, 5 * 1_000_000_000 + 500)

    def test_layout_is_only_computed_once(self):
        xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud_a = make_pointcloud2(xyz)
        cloud_b = make_pointcloud2(xyz)

        with patch.object(LidarDecoderNode, '_init_pointcloud_layout', wraps=self.node._init_pointcloud_layout) as spy:
            self._run(cloud_a)
            self._run(cloud_b)
            self.assertEqual(spy.call_count, 1)

    def test_malformed_cloud_is_caught_and_logged_not_raised(self):
        bad_cloud = PointCloud2()
        bad_cloud.header = Header()
        bad_cloud.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)]
        bad_cloud.data = b''
        bad_cloud.point_step = 4
        bad_cloud.is_bigendian = False

        self._run(bad_cloud)
        self.node._bridge.send_decoded.assert_not_called()


class TestLidarDecoderNodeOptimized(DecoderBehaviorMixin, unittest.TestCase):
    optimize_collection = True

class TestLidarDecoderNodeUnoptimized(DecoderBehaviorMixin, unittest.TestCase):
    optimize_collection = False


if __name__ == '__main__':
    unittest.main()