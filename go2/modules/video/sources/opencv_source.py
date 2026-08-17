import cv2
import threading
import numpy as np
import numpy.typing as npt
from typing import Optional, cast
from typing_extensions import override

from ..frame_result import FrameResult
from .camera_source import CameraSource


class OpenCVCameraSource(CameraSource):
    def __init__(self, camera_index: int = 0) -> None:
        self._camera_index = camera_index

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_rgb: Optional[npt.NDArray[np.uint8]] = None
        self._initialize_source()

    def _initialize_source(self) -> None:
        self._capture = cv2.VideoCapture(self._camera_index)
        if not self._capture.isOpened():
            raise RuntimeError(f"[OpenCVCamera] Failed to open camera at index {self._camera_index}")

    @override
    def _start(self) -> None:
        self._thread = threading.Thread(target=self._capture_thread, daemon=True)
        self._thread.start()

    def _capture_thread(self) -> None:
        while not self._stop_event.is_set():
            ret, frame = self._capture.read()
            if not ret:
                continue
            
            self._latest_rgb = cast(npt.NDArray[np.uint8], frame)
            
        self._capture.release()

    @override
    def _get_frames(self) -> FrameResult:
        ret, self._latest_rgb = self._latest_rgb, None
        if ret is None:
            return FrameResult.pending()
            
        return FrameResult.color_only(ret)

    @override
    def _shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join()

        self._latest_rgb = None
