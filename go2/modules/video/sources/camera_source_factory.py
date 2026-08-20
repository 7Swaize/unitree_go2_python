from typing import Dict

from .camera_source import CameraSource
from .realsense_source import RealSenseDepthCameraSource
from .opencv_source import OpenCVCameraSource
from .native_source import NativeCameraSource
from .virtual_source import VirtualCameraSource
from .camera_group import CameraGroup


class CameraSourceFactory:
    """
    Factory class for creating camera sources.

    ``CameraSourceFactory`` provides a simple, safe way for users to
    select which camera hardware to use without needing to understand
    low-level camera implementations.

    The returned objects are compatible with :class:`VideoModule` and
    should not be used directly.

    Note
    ----
    Users should **not** instantiate camera source classes directly.
    Always use this factory instead.
    """


    @staticmethod
    def create_native_camera() -> CameraSource:
        """
        Create a camera source backed by the robot's internal camera.

        This camera is typically used when running code directly on
        the robot hardware.

        The returned frame contains a BGR color image as a NumPy ``uint8`` array.
        """
        return NativeCameraSource()


    @staticmethod
    def create_opencv_camera(camera_index: int = 0) -> CameraSource:
        """
        Create a camera source using OpenCV's ``VideoCapture``.

        This is the recommended option for:
            - USB webcams
            - Laptop cameras
            - Development on personal machines

        The returned frame contains a BGR color image as a NumPy ``uint8`` array.

        Parameters
        ----------
        camera_index : int, optional
            Index of the OpenCV camera (default is 0).
        """
        return OpenCVCameraSource(camera_index)


    @staticmethod
    def create_depth_camera() -> CameraSource:
        """
        Create an RGB-Depth camera source using an Intel RealSense D345i.

        This camera provides both color and depth frames. The returned
        camera source automatically aligns depth data to the color frame.

        The returned ``FrameResult`` includes:
            - ``color``: BGR image as a NumPy ``uint8`` array
            - ``depth``: Z16 aligned depth image as a NumPy ``uint16`` array (D345i depth values in millimeters)

        Note
        ----
        Requires an Intel RealSense camera and the RealSense SDK.
        """
        return RealSenseDepthCameraSource()
    

    @staticmethod
    def create_virtual_camera() -> CameraSource:
        """
        Create an RGB-Depth camera source inside of the robot simulation.

        This camera provides both color and depth frames. The returned
        camera source automatically aligns depth data to the color frame.

        The returned ``FrameResult`` includes:
            - ``color``: BGR image as a NumPy ``uint8`` array
            - ``depth``: Depth image as a NumPy ``uint16`` array

        Note
        ----
        Runs only within the Mujoco Simulation.
        """
        return VirtualCameraSource()

    @staticmethod
    def create_camera_group(sources: Dict[str, CameraSource]) -> CameraGroup:
        """
        Create a named group of camera sources for multi-camera setups.

        Parameters
        ----------
        sources : Dict[str, CameraSource]
            Mapping of camera name to camera source. Names are used
            to key frames in the returned MultiFrameResult.
        """
        return CameraGroup(sources)
