import ctypes
import threading
import numpy as np
import numpy.typing as npt
import iceoryx2 as iox2
from typing import Optional

from iceoryx_interfaces.qos import LidarQoS
from iceoryx_interfaces.lidar_data import LidarHeader_

from .utils.singleton import Singleton
    

class LidarBridge(metaclass=Singleton):
    def __init__(self) -> None:
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
        self._node = iox2.NodeBuilder.new() \
            .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
            .create(iox2.ServiceType.Ipc)

        self._decoded_service = self._node.service_builder(iox2.ServiceName.new(LidarQoS.TOPIC_LIDAR_DECODED)) \
            .request_response(ctypes.c_uint32, iox2.Slice[ctypes.c_float]) \
            .response_header(LidarHeader_) \
            .open_or_create()
        
        self._decoded_server = self._decoded_service.server_builder() \
            .initial_max_slice_len(LidarQoS.RESPONSE_INITIAL_SLICE_LEN_HINT) \
            .allocation_strategy(iox2.AllocationStrategy.PowerOfTwo) \
            .create()
        
        self._cycle_time = iox2.Duration.from_millis(10)
        self._active_request: Optional[iox2.ActiveRequest] = None
        
        self._stop_event = threading.Event()
        self._worker_thread = threading.Thread(target=self._run, daemon=True)
        self._worker_thread.start()


    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._node.wait(self._cycle_time)

            self._active_request = self._decoded_server.receive()
            if self._active_request is not None:
                break
            

    def send_decoded(self, stamp_ns: int, array: npt.NDArray[np.float32]) -> None:
        assert array.dtype == np.float32 and array.flags.f_contiguous, "Array must be float32 and F-contiguous"

        if not self._active_request or not self._active_request.is_connected:
            return

        response = self._active_request.loan_slice_uninit(array.size)
        rows, cols = array.shape
        response.user_header().contents.stamp_ns = stamp_ns
        response.user_header().contents.rows = rows
        response.user_header().contents.cols = cols
        
        ctypes.memmove(response.payload().as_ptr(), array.ctypes.data, array.nbytes)

        response = response.assume_init()
        response.send()


    def shutdown(self) -> None:
        self._stop_event.set()
        self._active_request = None
