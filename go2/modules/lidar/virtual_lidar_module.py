from typing_extensions import override

from .lidar_module import LIDARModule


class VirtualLIDARModule(LIDARModule):
    """
    Runs the LIDAR pipeline against the MuJoCo simulation.

    The simulated LIDAR approximates the [Livox Mid-360](https://www.livoxtech.com/mid-360),including its scan pattern
    and 40 meter maximum range. The simulated sensor produces 200,000 points per second, distributed across
    publishes according to ``publish_hz``. Therefore, ``publish_hz`` controls the point cloud publication rate, while the
    overall point generation rate remains fixed at 200,000 points per second.
    """

    @override
    def _initialize(self) -> None:
        if self._initialized:
            return

        if self._publish_hz < 5 or self._publish_hz > 100:
            raise ValueError(f"Parameter 'publish_hz' must be in the range [5, 100]; got {self._publish_hz}")

        self._launch_bridge()
        self._initialized = True

    @override
    def _shutdown(self) -> None:
        if self._iox_receiver:
            self._iox_receiver._shutdown()
            self._iox_receiver.join(timeout=2)

        self._initialized = False
