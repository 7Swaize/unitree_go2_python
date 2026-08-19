import time
import ctypes
import pytest
import unittest
import threading
import numpy as np
import iceoryx2 as iox2
from unittest.mock import patch
from iceoryx_interfaces.qos import LidarQoS
from iceoryx_interfaces.lidar_data import LidarHeader_

from lidar_processor.lidar_bridge import LidarBridge


PUBLISH_HZ = 10

def _make_decoded_ipc_client() -> tuple[iox2.Node, iox2.Service, iox2.Client, iox2.PendingResponse]:
    iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new() \
        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
        .create(iox2.ServiceType.Local)
    
    service = node.service_builder(iox2.ServiceName.new(LidarQoS.TOPIC_LIDAR_DECODED)) \
        .request_response(ctypes.c_uint32, iox2.Slice[ctypes.c_float]) \
        .response_header(LidarHeader_) \
        .open_or_create()
    
    client = service.client_builder().create()

    sample = client.loan_uninit()
    sample = sample.write_payload(ctypes.c_uint32(PUBLISH_HZ))
    pending_response = sample.send()
    
    return node, service, client, pending_response
    

def _init_lidar_bridge_with_local_service(self) -> None:
    iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
    self._node = iox2.NodeBuilder.new() \
        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
        .create(iox2.ServiceType.Local)
    
    self._decoded_service = self._node.service_builder(iox2.ServiceName.new(LidarQoS.TOPIC_LIDAR_DECODED)) \
        .request_response(ctypes.c_uint32, iox2.Slice[ctypes.c_float]) \
        .response_header(LidarHeader_) \
        .open_or_create()

    self._decoded_server = self._decoded_service.server_builder() \
        .initial_max_slice_len(LidarQoS.RESPONSE_INITIAL_SLICE_LEN_HINT) \
        .allocation_strategy(iox2.AllocationStrategy.PowerOfTwo) \
        .create()

    self._cycle_time = iox2.Duration.from_millis(10)
    self._active_request = None

    self._stop_event = threading.Event()
    self._worker_thread = threading.Thread(target=self._run, daemon=True)
    self._worker_thread.start()


def _build_complex_xyzi() -> np.ndarray:
    x = np.linspace(0.0, 9.0, 10)
    y = np.linspace(0.0, 9.0, 10)
    z = np.linspace(0.0, 9.0, 10)
    Xgrid, Ygrid, Zgrid = np.meshgrid(x, y, z, indexing='ij')

    xyz = np.vstack((Xgrid.flatten(), Ygrid.flatten(), Zgrid.flatten())).T
    intensity = np.linspace(100.0, 500.0, 1000)
    dataset = np.column_stack((xyz, intensity))

    # (N, channels) layout, C-contiguous - matches the decoder's native output layout.
    return np.ascontiguousarray(dataset, dtype=np.float32)


def _wait_for_response(pending_response: iox2.PendingResponse, timeout: float) -> iox2.Response:
    start_time = time.time()

    while time.time() - start_time < timeout:
        response = pending_response.receive()
        if response is not None:
            return response
        
    assert False, "Failed to receive IPC response from server."



class TestLidarBridgeSendDecoded(unittest.TestCase):
    def test_send_decoded_does_not_raise_for_various_shapes_c_contiguous(self):
        bridge = LidarBridge()

        bridge.send_decoded(
            stamp_ns=1,
            array=np.zeros((0, 3), dtype=np.float32, order="C"),
        )
        bridge.send_decoded(
            stamp_ns=2,
            array=np.ones((10, 3), dtype=np.float32, order="C"),
        )
        bridge.send_decoded(
            stamp_ns=3,
            array=np.ones((10, 4), dtype=np.float32, order="C"),
        )

    def test_send_decoded_sub_raises_for_array_f_contiguous(self):
        bridge = LidarBridge()

        with pytest.raises(AssertionError, match="Array must be float32 and C-contiguous"):
            bridge.send_decoded(
                stamp_ns=1,
                array=np.zeros((3, 2), dtype=np.float32, order="F"),
            )

    def test_subscriber_receives_exactly_what_was_sent_xyz_simple(self):
        # We have this patch because we want to use INTRA-process communication. These tests all execute within one process.
        with patch.object(LidarBridge, "__init__", _init_lidar_bridge_with_local_service):
            bridge = LidarBridge()

        _, _, _, pending_response = _make_decoded_ipc_client()

        for _ in range(50):
            if bridge._active_request is not None:
                break
            time.sleep(0.01)

        assert bridge._active_request is not None, "Bridge never received the client's request"

        points = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0]
        ], dtype=np.float32, order='C')

        bridge.send_decoded(stamp_ns=99, array=points)
        response = _wait_for_response(pending_response=pending_response, timeout=5)
        self._assert_response_matches_sent(response, points, expected_stamp_ns=99)

    def test_subscriber_receives_exactly_what_was_sent_xyzi_complex(self):
        # We have this patch because we want to use INTRA-process communication. These tests all execute within one process.
        with patch.object(LidarBridge, "__init__", _init_lidar_bridge_with_local_service):
            bridge = LidarBridge()

        _, _, _, pending_response = _make_decoded_ipc_client()

        for _ in range(50):
            if bridge._active_request is not None:
                break
            time.sleep(0.01)
            
        assert bridge._active_request is not None, "Bridge never received the client's request"

        points = _build_complex_xyzi()

        bridge.send_decoded(stamp_ns=99, array=points)
        response = _wait_for_response(pending_response=pending_response, timeout=5)
        self._assert_response_matches_sent(response, points, expected_stamp_ns=99)

    def _assert_response_matches_sent(self, response: iox2.Response, points:np.ndarray, expected_stamp_ns: int = 99):
        data_ptr = ctypes.cast(response.payload().as_ptr(), ctypes.POINTER(ctypes.c_float))
        rows = response.user_header().contents.rows
        cols = response.user_header().contents.cols
        itemsize = np.dtype(np.float32).itemsize
        stamp_ns = response.user_header().contents.stamp_ns

        received = np.ndarray(
            shape=(rows, cols),
            dtype=np.float32,
            buffer=(ctypes.c_float * (rows * cols)).from_address(ctypes.addressof(data_ptr.contents)),
            strides=(cols * itemsize, itemsize)
        ).copy(order='C')

        assert stamp_ns == expected_stamp_ns
        assert rows == points.shape[0]
        assert cols == points.shape[1]

        np.testing.assert_equal(received, points)
        self.assertTrue(points.flags.c_contiguous)
        self.assertEqual(points.dtype, np.float32)



class TestLidarBridgeSingleton(unittest.TestCase):
    def test_bridge_is_a_process_wide_singleton(self):
        first = LidarBridge()
        second = LidarBridge()
        assert first is second