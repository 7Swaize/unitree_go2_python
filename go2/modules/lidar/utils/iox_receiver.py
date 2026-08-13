import ctypes
import threading
import numpy as np
import iceoryx2 as iox2
import numpy as np

from typing_extensions import override
from iceoryx_interfaces.qos import LidarQoS
from iceoryx_interfaces.lidar_data import LidarHeader_

from .callback_dispatcher import CallbackDispatcher


class IoxReceiver(threading.Thread):
    def __init__(self, dispatcher: CallbackDispatcher, publish_hz: int) -> None:
        super().__init__(daemon=True)
        self._publish_hz = publish_hz
        self._dispatcher: CallbackDispatcher = dispatcher
        self._stop_event: threading.Event = threading.Event()
        self._init_iox2()

    def _init_iox2(self) -> None:
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
        self._cycle_time = iox2.Duration.from_millis(1000 // self._publish_hz)
        self._node = iox2.NodeBuilder.new() \
                .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
                .create(iox2.ServiceType.Ipc)
        
        self._decoded_service = self._node.service_builder(iox2.ServiceName.new(LidarQoS.TOPIC_LIDAR_DECODED)) \
                                    .request_response(ctypes.c_uint32, iox2.Slice[ctypes.c_float]) \
                                    .response_header(LidarHeader_) \
                                    .open_or_create()

        self._decoded_client = self._decoded_service.client_builder().create()

    @override
    def run(self) -> None:
        sample = self._decoded_client.loan_uninit()
        sample = sample.write_payload(ctypes.c_uint32(self._publish_hz))
        pending_response = sample.send()

        while not self._stop_event.is_set():
            self._node.wait(self._cycle_time)

            while True:
                response = pending_response.receive()
                if response is None:
                    break

                data_ptr = ctypes.cast(response.payload().as_ptr(), ctypes.POINTER(ctypes.c_float))
                rows = response.user_header().contents.rows
                cols = response.user_header().contents.cols
                itemsize = np.dtype(np.float32).itemsize

                self._dispatcher._emit_decoded(
                    response.user_header().contents.stamp_ns,
                    np.ndarray(
                        shape=(rows, cols),
                        dtype=np.float32,
                        buffer=(ctypes.c_float * (rows * cols)).from_address(ctypes.addressof(data_ptr.contents)),
                        strides=(itemsize, rows * itemsize)
                    ).copy(order='F')
                )


    def _shutdown(self) -> None:
        self._stop_event.set()