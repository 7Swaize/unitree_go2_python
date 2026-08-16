import cv2

from go2.core import ModuleType, HardwareType, VirtualGo2Controller, create_controller
from go2.modules.video import CameraSourceFactory


def init_controller() -> VirtualGo2Controller:
    # Create a Go2 Controller instance to access all provided funtionalities.
    # Here, we pass HardwareType.VIRTUAL because we want to execute commands with respect to the simulator.
    # It's recommended to use the 'create_controller' factory method to create the controller (as shown below).
    controller = create_controller(hardware_type=HardwareType.VIRTUAL)

    # The VIDEO module is not a default module on the controller. Therefore, we must add it explicitly.
    # The module contructor takes in a 'camera_source' target where we say when camera we want to read frames from.
    # Here, we create an external camera to read frames from via OpenCV
    controller.add_module(ModuleType.VIDEO, camera_source=CameraSourceFactory.create_opencv_camera(0))

    # If any of the cameras cannot be accessed, a runtime exception will be thrown.

    return controller


if __name__ == "__main__":
    # Create a controller instance.
    controller = init_controller()

    while True:
        # Get a 'FrameResult' object from the video module.
        # This is a queryable container for a captured frame.
        frame_result = controller.video.get_frames()

        # If the container object doesn't have a frame, we continue.
        # This happens when we ask for frames faster than the capture rate on the camera or if an internal capture exception occurred.
        if not frame_result.has_color():
            continue

        # Display the frame using OpenCV
        cv2.imshow("External Camera", frame_result.color)

        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            break

    # Safely shutdown the controller and release resources
    controller.safe_shutdown()