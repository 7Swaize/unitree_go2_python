import cv2
from collections import deque

from go2.core import ModuleType, HardwareType, VirtualGo2Controller, create_controller
from go2.modules.video import CameraSourceFactory
from go2.modules.ocr import OCRConfig


# Number of frames to buffer before running an OCR pass.
FRAMES_PER_ANALYSIS = 4
# Max number of frames to keep in the buffer at once.
QUEUE_MAXLEN = 8


def init_controller() -> VirtualGo2Controller:
    # Create a Go2 Controller instance to access all provided funtionalities.
    # It's recommended to use the 'create_controller' factory method to create the controller (as shown below).
    controller = create_controller(hardware_type=HardwareType.VIRTUAL)

    # The VIDEO module is not a default module on the controller. Therefore, we must add it explicitly.
    # The module contructor takes in a 'camera_source' target where we say when camera we want to read frames from.
    # Here, we create an external camera to read frames from a connected camera via OpenCV.
    controller.add_module(ModuleType.VIDEO, camera_source=CameraSourceFactory.create_opencv_camera(0))

    # The OCR module is not a default module on the controller. Therefore, we must add it explicitly.
    # It takes no additional paramters for construction
    controller.add_module(ModuleType.OCR)

    return controller


def main(controller: VirtualGo2Controller) -> None:
    queue = deque(maxlen=QUEUE_MAXLEN)

    while True:
        # Get a FrameResult object from the video module.
        frame_result = controller.video.get_frames()

        # If the result doesn't contain a color frame, skip this iteration.
        if not frame_result.has_color():
            continue

        queue.append(frame_result.color)

        # Once we've buffered enough frames, run temporal-voting OCR on the
        # most recent batch and print any recognized text.
        if len(queue) == FRAMES_PER_ANALYSIS:
            batch = [queue.pop() for _ in range(FRAMES_PER_ANALYSIS)]
            
            # Create an OCRConfig object defining the minimum number of occurrences of a word
            # in a sequence of frames, for it to be considered valid.
            config = OCRConfig(temporal_voting_threshold=FRAMES_PER_ANALYSIS)
            
            # Use the temporal voting function instead, passing in our custom configuration
            seqs, frames = controller.ocr.extract_text_from_images_temporal_voting(batch, config=config)

            if seqs:
                print(" ".join(seqs))

        # Show the live camera feed.
        cv2.imshow("Capture", frame_result.color)

        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            break


if __name__ == "__main__":
    # Create a controller instance.
    controller = init_controller()

    main(controller)

    # Safely shutdown the controller and release resources
    controller.safe_shutdown()