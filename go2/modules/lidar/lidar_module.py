import numpy as np
import numpy.typing as npt
from abc import ABC
from typing import Callable

from ...core.module import DogModule
from .utils.iox_receiver import IoxReceiver
from .utils.callback_dispatcher import CallbackDispatcher


class LIDARModule(DogModule, ABC):
    """
    ``LIDARModule`` provides a simple API for:
        - Receiving decoded ``PointCloud2`` structures as xyz-(optionally intensity) numpy arrays
    
    Users should not access or construct this class directly.
    Rather, they should access it through the :class:`~core.controller.Go2Controller` instance.
 
    Parameters
    ----------
    publish_hz : int
        The publishing frequency (in Hz) of decoded point cloud frames. This dictates 
        how many times per second the registered callback is triggered. A lower publishing rate 
        results in a longer point accumulation time per frame, yielding a denser point cloud 
        with greater field-of-view (FOV) coverage, but introduces higher delay between publishings.
        (Although, the data transfer will always be an *O(1)* pointer-swap internally).
        Must be in the range [5, 100]. Default is 10.
        
        - When running in ``HardwareType.NATIVE`` mode, ``publish_hz`` does **not** control 
          the actual publish rate of the physical LIDAR sensor. You must configure that rate 
          manually via the LIDAR's own driver/firmware settings. In this mode, ``publish_hz`` 
          only governs how frequently this wrapper internally polls for new data over IPC 
          from the ROS2 publisher node to this receiver. 
        - When running in ``HardwareType.VIRTUAL`` mode, ``publish_hz`` **does** matter, since 
          we are mocking the real LIDAR and fully control when it publishes data.
    """

    def __init__(self, publish_hz: int = 10) -> None:
        super().__init__("LIDAR")
        self._publish_hz: int = publish_hz

    def _launch_bridge(self) -> None:
        self._dispatcher = CallbackDispatcher()
        self._iox_receiver = IoxReceiver(self._dispatcher, self._publish_hz)
        self._iox_receiver.start()

    def register_decoded_pointcloud_callback(self, callback: Callable[[int, npt.NDArray[np.float32]], None]) -> None:
        """
        Register a callback to receive decoded PointCloud2 data.
        The callback is triggered whenever a new raw point cloud sample is received via Iceoryx2.

        Parameters
        ----------
        callback : Callable[[int, np.ndarray], None]
            A function to be called with:
                - **timestamp** (int): The source timestamp in nanoseconds.
                - **points** (np.ndarray): A **Fortran-contiguous** ``float32`` array of shape ``(3, N)``
                    for [x, y, z] or ``(4, N)`` if intensity is supported [x, y, z, intensity].

        Important
        ---------
        - **Shared Views:** 
            During each cycle, all subscribers receive **views** of 
            the same underlying data. This is done for performance. Modifying data 
            through the returned view is unsafe, as it affects the shared data. If 
            you need to modify the data, create a **copy** first.
        - **Cache Locality & Iteration:**
            Because the array is Fortran-contiguous, the 
            data is laid out column-by-column in memory (e.g., x0, y0, z0, x1, y1, z1...). 
            This means all coordinates (plus intensity) for a single point are tightly packed together. 
            Therefore, column-major iteration should be greatly prefered over row-major iteration to maximize cache efficiency.
            If you *need* a C-contiguous array (row-major) you can use use ``numpy.ascontiguousarray(array)``.
        """
        self._dispatcher._register_decoded(callback)