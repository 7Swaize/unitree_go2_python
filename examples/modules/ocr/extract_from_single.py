import cv2

from go2.core import ModuleType, HardwareType, VirtualGo2Controller, create_controller
from go2.modules.video import CameraSourceFactory

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


if __name__ == "__main__":
    # Create a controller instance.
    controller = init_controller()

    while True:
        # Get a 'FrameResult' object from the video module.
        frame_result = controller.video.get_frames()

        # If the container object doesn't have a frame, we continue.
        if not frame_result.has_color():
            continue

        # Extract text from the image.
        # Optionally, we can pass in an OCRConfig object. However, we can just use the default here.
        (seqs, frame) = controller.ocr.extract_text_from_image(frame_result.color)

        # Print any characters sequences found that pass the threshold
        if len(seqs) > 0:
            print(" ".join(seqs))

        # Display the frame used for OCR
        cv2.imshow("Capture", frame)

        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            break

    # Safely shutdown the controller and release resources
    controller.safe_shutdown()