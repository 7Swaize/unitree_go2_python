import threading
import numpy as np
import numpy.typing as npt
import pyrealsense2 as rs
from typing import Optional
from typing_extensions import override

from ..frame_result import FrameResult
from .camera_source import CameraSource

# Basic Camera Use: https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
# Align: https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
# Optimzation/Configs: https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance

class RealSenseDepthCameraSource(CameraSource):
    def __init__(self) -> None:
        self._width: int = 848
        self._height: int = 480
        self._fps: int = 30

        self._thread: Optional[threading.Thread] = None
        self._latest_frames: Optional[tuple[npt.NDArray[np.uint8], npt.NDArray[np.uint16]]] = None
        self._stop_event = threading.Event()

        self._initialize_pipeline()

    def _initialize_pipeline(self) -> None:
        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        self._config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)


    @override
    def _start(self) -> None:
        self._thread = threading.Thread(target=self._rs_thread, daemon=True)
        self._thread.start()

    def _rs_thread(self) -> None:
        self._pipeline.start(self._config)
        align = rs.align(rs.stream.color)

        while not self._stop_event.is_set():
            frames = self._pipeline.wait_for_frames()
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Here, we can be more 'lazy' when copying data, and only copy when we KNOW the frame will outlive (1000ms/fps).
            # See ref: https://dev.realsenseai.com/docs/frame-management/
            rgb = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            self._latest_frames = (rgb, depth)

        self._pipeline.stop()


    @override
    def _get_frames(self) -> FrameResult:
        pair, self._latest_frames = self._latest_frames, None
        if pair is None:
            return FrameResult.pending()

        return FrameResult.color_and_depth(color=pair[0].copy(), depth=pair[1].copy())
        

    @override
    def _shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join()

        self._latest_frames = None