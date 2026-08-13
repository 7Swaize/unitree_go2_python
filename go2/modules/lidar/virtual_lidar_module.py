from typing_extensions import override

from .lidar_module import LIDARModule


class VirtualLIDARModule(LIDARModule):
    """
    ``VirtualLIDARModule`` runs the LIDAR pipeline against the MuJoCo simulation,
    where ``publish_hz`` fully controls the point cloud publish rate.
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
