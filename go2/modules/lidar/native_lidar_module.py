import os
import signal
import subprocess
from typing import Optional
from typing_extensions import override

from .lidar_module import LIDARModule


class NativeLIDARModule(LIDARModule):
    """
    ``NativeLIDARModule`` runs the LIDAR pipeline against the real Go2 hardware.
 
    Important
    ---------
    When executing on ``HardwareType.NATIVE`` this module launches ROS2 nodes.
    In such a case, it is **critical** that users **ALWAYS** call :meth:`Go2Controller.safe_shutdown` after normal (error free) script exit.
    """

    @override
    def __init__(self, publish_hz = 10) -> None:
        super().__init__(publish_hz)
        self._ros_proc: Optional[subprocess.Popen] = None 

    @override
    def _initialize(self) -> None:
        if self._initialized:
            return

        if self._publish_hz < 5 or self._publish_hz > 100:
            raise ValueError(f"Parameter 'publish_hz' must be in the range [5, 100]; got {self._publish_hz}")

        self._launch_ros()
        self._launch_bridge()
        self._initialized = True

    def _launch_ros(self) -> None:
        kwargs = dict()
        # To detach child process: https://stackoverflow.com/questions/45911705/why-use-os-setsid-in-python
        kwargs["start_new_session"] = True

        self._ros_proc = subprocess.Popen(
            ["ros2", "launch", "bringup", "lidar_processor.launch.py"],
            **kwargs
        )

    @override
    def _shutdown(self):
        if self._ros_proc and self._ros_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._ros_proc.pid), signal.SIGINT)
                self._ros_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._ros_proc.terminate()
                self._ros_proc.wait(timeout=5)

        if self._iox_receiver:
            self._iox_receiver._shutdown()
            self._iox_receiver.join(timeout=2)

        self._initialized = False