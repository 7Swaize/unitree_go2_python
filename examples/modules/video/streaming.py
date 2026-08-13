import cv2
import numpy as np

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

    return controller


def start_streaming(controller: VirtualGo2Controller) -> None:
    # Start the streaming server - only accessible on LAN devices
    controller.video.start_stream_server()

    # Print a link to the view the camera stream
    print(f"WebRTC streaming at: http://{controller.video.get_stream_server_local_ip()}:{controller.video.get_stream_server_port()}")


if __name__ == "__main__":
    # Create a controller instance
    controller = init_controller()

    start_streaming(controller)

    while True:
        # Get a 'FrameResult' object from the video module.
        frame_result = controller.video.get_frames()

        # If the container object doesn't have a frame, we continue.
        if not frame_result.has_color():
            continue

        # Access frame
        color = frame_result.color

        # Annotate frame with something
        cv2.putText(
            color, 
            "This is an Annotation", 
            (50, 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (0, 255, 0), 
            2
        )

        # Send the annotated frame to the streamer
        controller.video.send_frame(color)

        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            break

    # Safely shutdown controller and release resources
    controller.safe_shutdown()
