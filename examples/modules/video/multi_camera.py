import cv2
import numpy as np

from go2.core import ModuleType, HardwareType, VirtualGo2Controller, create_controller
from go2.modules.video import CameraSourceFactory, FrameResult


def init_controller() -> VirtualGo2Controller:
    # Create a Go2 Controller instance to access all provided funtionalities.
    # Here, we pass HardwareType.VIRTUAL because we want to execute commands with respect to the simulator.
    # It's recommended to use the 'create_controller' factory method to create the controller (as shown below).
    controller = create_controller(HardwareType.VIRTUAL)

    # The VIDEO module is not a default module on the controller. Therefore, we must add it explicitly.
    # The module contructor takes in a 'camera_source' target where we say when camera we want to read frames from.
    # We can pass in a camera group if we want to read frames from multiple cameras.
    controller.add_module(ModuleType.VIDEO, camera_source=CameraSourceFactory.create_camera_group({
        "sim": CameraSourceFactory.create_virtual_camera(), # Access the simulated Go2 camera.
        "cam": CameraSourceFactory.create_opencv_camera(0) # Access an external camera via OpenCV.
    }))

    # If any of the cameras cannot be accessed, a runtime exception will be thrown.

    return controller


def read_sim_camera(frame_result: FrameResult) -> None:
    # The simulator camera returns an RGB and Depth frame pair.

    if frame_result.is_fully_valid():
        color, depth = frame_result.color, frame_result.depth

        # Apply a heat colormap to the depth frame
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03),
            cv2.COLORMAP_JET
        )

        combined = np.hstack((
            color,
            cv2.resize(depth_colormap, (color.shape[1], color.shape[0]))
        ))

        cv2.imshow("Simulator Camera", combined)

    # Do something with frame.


def read_external_camera(frame_result: FrameResult) -> None:
    # Process the frame.
    pass


if __name__ == "__main__":
    # Create a controller instance
    controller = init_controller()

    while True:
        # Since, we are reading from multiple camera targets, we get a 'MultiFrameResult' instead.
        multi_res = controller.video.get_frames()

        # Access the 'FrameResult' from the sim camera capture.
        sim_res = multi_res["sim"] # Key defined from when we added the camera source initially.

        # Access the 'FrameResult' from the external camera capture.
        external_res = multi_res["cam"] # Key defined from when we added the camera source initially.

        # Do something with each of the frames
        read_sim_camera(sim_res)
        read_external_camera(external_res)

        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            break

    # Safely shutdown the controller and release resources.
    controller.safe_shutdown()

