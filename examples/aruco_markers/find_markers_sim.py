import cv2
import time
import threading
import numpy as np
from typing import Optional
from typing_extensions import override
 
from go2.core import ModuleType, HardwareType, VirtualGo2Controller, create_controller
from go2.modules.video import CameraSourceFactory
 
from aruco_helpers import (
    get_aruco_marker,
    extract_data_from_marker_by_id,
    draw_marker_bounds,
)


FRAME_POLL_INTERVAL_S = 0.01


class VideoThreadWorker(threading.Thread):
    PREVIEW_WINDOW_NAME: str = "Simulator Stream"
    MARKER_LABEL_OFFSET_PX: int = 10

    def __init__(self, controller: VirtualGo2Controller, shutdown_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self._controller = controller
        self._shutdown_event = shutdown_event
        self._latest_frame: Optional[np.ndarray] = None


    @override
    def run(self) -> None:
        while not self._shutdown_event.is_set():
            frame_result = self._controller.video.get_frames()
            if not frame_result.has_color():
                time.sleep(FRAME_POLL_INTERVAL_S)
                continue

            self._latest_frame = frame_result.color
            self._render_preview(frame_result.color)

        cv2.destroyWindow(self.PREVIEW_WINDOW_NAME)


    def _render_preview(self, frame: np.ndarray) -> None:
        corners, ids, _ = get_aruco_marker(frame)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                bounds = [(int(x), int(y)) for x, y in corners[i][0]]
                draw_marker_bounds(frame, bounds, color=(0, 255, 0))
                label_origin = (bounds[0][0], bounds[0][1] - self.MARKER_LABEL_OFFSET_PX)
                cv2.putText(frame, f"ID: {int(marker_id[0])}", label_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow(self.PREVIEW_WINDOW_NAME, frame)
        cv2.waitKey(1)


    def get_latest_frame(self) -> Optional[np.ndarray]:
        # Technically, this is safe.
        # A pointer swap is atomic via the GIL, so no thread can ever observe a torn struct
        # Additionally, this returns a strong reference, which calls Py_INCREF, incrementing its ref count.
        # When this worker thread reassigns 'self._latest_frame', it only rebinds the pointer.
        # The underlying data (prior to reassign) is still prevented from being GC collected, due to the returned strong reference.
        #
        # If you are untrustworthy of me, use a lock.
        return self._latest_frame



"""
Go2 robot program that scans the simulation for a sequence of Aruco markers 
(IDs 0 through NUM_MARKERS_TO_SCAN - 1) and physically aligns itself with each one in turn.

Behavior:
- Spins up a VirtualGo2Controller (via the 'create_controller' factory) in
  VIRTUAL hardware mode with the video (virtual camera) module attached.
- Runs a background thread (VideoThreadWorker) that continuously pulls color
  frames from the camera, detects Aruco markers in each frame, draws bounding
  boxes and ID labels on a live preview window, and caches the latest raw frame for consumers.
- For each target marker ID, the main thread:
    1. Rotates in place (SEARCH_ROTATE_STEP per iteration) until the target
       marker is detected in the latest frame.
    2. Once found, repeatedly nudges rotation (CORRECT_ALIGNMENT_STEP_AMOUNT)
       toward the marker based on its horizontal pixel offset from center,
       until that offset falls within H_OFFSET_THRESHOLD pixels.
    3. Stops movement, sits down, stands back up, and moves on to the next
       marker ID.
- On completion (or shutdown), stops the video thread, closes the OpenCV
  preview window, and calls safe_shutdown() on the controller.
"""
class MarkerScanProgram:
    CORRECT_ALIGNMENT_VELOCITY: float = 0.5
    H_OFFSET_THRESHOLD: float = 50.0
    SEARCH_ROTATE_VELOCITY: float = 0.8
    NUM_MARKERS_TO_SCAN: int = 6


    def __init__(self) -> None:
        self._controller: VirtualGo2Controller = create_controller(hardware_type=HardwareType.VIRTUAL)
        self._controller.register_cleanup_callback_pre_module_shutdown(self._shutdown_callback)
        self._controller.add_module(ModuleType.VIDEO, camera_source=CameraSourceFactory.create_virtual_camera())
 
        self._shutdown_event = threading.Event()
        self._video_worker = VideoThreadWorker(self._controller, self._shutdown_event)
        self._video_worker.start()


    def _shutdown_callback(self) -> None:
        self._shutdown_event.set()
        self._video_worker.join(timeout=2.0)
        cv2.destroyAllWindows()


    def _scan_for_marker(self, target_marker_id: int) -> None:
        while not self._shutdown_event.is_set():
            _, found, marker_id, _, h_offset, _ = self._get_marker_state(target_marker_id)
            if not found:
                self._controller.movement.move(0, 0, MarkerScanProgram.SEARCH_ROTATE_VELOCITY)
                continue

            self._controller.movement.stop_move().wait()

            while not self._shutdown_event.is_set():
                aligned = self._correct_alignment_to_marker(
                    marker_id, h_offset, MarkerScanProgram.H_OFFSET_THRESHOLD, MarkerScanProgram.CORRECT_ALIGNMENT_VELOCITY
                )

                if aligned:
                    self._controller.movement.stop_move().wait()
                    return
                
                _, _, marker_id, _, h_offset, _ = self._get_marker_state(target_marker_id)


    def _get_marker_state(self, target_marker_id: int):
        frame = self._wait_for_frame()
        found, marker_id, bounds, center, h_offset, area = self._find_marker_in_frame(
            frame, target_marker_id
        )
        return frame, found, marker_id, bounds, h_offset, area


    def _wait_for_frame(self) -> np.ndarray:
        while not self._shutdown_event.is_set():
            frame = self._video_worker.get_latest_frame()
            if frame is not None:
                return frame
            time.sleep(FRAME_POLL_INTERVAL_S)


    def _find_marker_in_frame(self, image: np.ndarray, target_marker_id: int):
        corners, ids, _ = get_aruco_marker(image)
        marker_id, bounds, center, h_offset, area = extract_data_from_marker_by_id(
            image, corners, ids, target_marker_id
        )
        found = marker_id == target_marker_id
        return found, marker_id, bounds, center, h_offset, area


    def _correct_alignment_to_marker(self, marker_id: int, h_offset: float, h_offset_threshold: float, angular_velocity: float) -> bool:
        if marker_id == -1:
            return False
 
        if abs(h_offset) <= h_offset_threshold:
            return True
        if h_offset < -h_offset_threshold:
            self._controller.movement.move(0, 0, -angular_velocity)
        else:
            self._controller.movement.move(0, 0, angular_velocity)
        return False


    def main(self) -> None:
        self._controller.movement.stand_up().wait()
 
        for marker_id in range(MarkerScanProgram.NUM_MARKERS_TO_SCAN):
            print(f"\n--- Scanning for Target Marker: {marker_id} ---")
            self._scan_for_marker(marker_id)
 
            self._controller.movement.stop_move().wait()
            self._controller.movement.stand_down().wait()
            self._controller.movement.stand_up().wait()
 
        self._controller.safe_shutdown()



if __name__ == "__main__":
    MarkerScanProgram().main()