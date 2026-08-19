import numpy as np
import matplotlib.pyplot as plt
from typing import Optional

from go2.core import ModuleType, HardwareType, ExecutionMode, VirtualGo2Controller, create_controller


def init_controller() -> VirtualGo2Controller:
    # We must pass 'ExecutionMode.Advanced' to have access to the LIDAR Module.
    controller = create_controller(hardware_type=HardwareType.VIRTUAL, execution_mode=ExecutionMode.ADVANCED)

    # The LIDAR module is not a default module, so we must add it explicitly.
    controller.add_module(ModuleType.LIDAR)

    return controller


class Renderer:
    POINT_SIZE: float = 0.5
    MAX_POINTS: int = 10000

    def __init__(self) -> None:
        self._latest_points: Optional[np.ndarray] = None
        self._latest_stamp: Optional[np.ndarray] = None

        plt.ion()
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(projection='3d')
        self._scatter = None


    def _decoded_callback(self, stamp_ns: int, points: np.ndarray) -> None:
        self._latest_points = points
        self._latest_stamp = stamp_ns


    def draw_latest(self) -> None:
            if self._latest_points is None:
                return
            
            pts = self._latest_points
            if Renderer.MAX_POINTS is not None and pts.shape[0] > Renderer.MAX_POINTS:
                idx = np.random.choice(pts.shape[0], Renderer.MAX_POINTS, replace=False)
                pts = pts[idx]

            self._ax.cla()
            self._ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=Renderer.POINT_SIZE, c=pts[:, 2], cmap='viridis')
            self._ax.set_xlabel('X')
            self._ax.set_ylabel('Y')
            self._ax.set_zlabel('Z')
            self._ax.set_title(f"stamp_ns={self._latest_stamp}, n={pts.shape[0]}")

            # Lidar in simulator has a max range of 40 meters.
            R = 40
            self._ax.set_xlim(-R, R)
            self._ax.set_ylim(-R, R)
            self._ax.set_zlim(-1, R)

            self._fig.canvas.draw()
            self._fig.canvas.flush_events()


def main() -> None:
    controller = init_controller()
    renderer = Renderer()

    # Register a callback that gets invoked whenever the controller gets a new set of points.
    controller.lidar.register_decoded_pointcloud_callback(renderer._decoded_callback)
    controller.movement.stand_up().wait()

    try:
        while True:   
            renderer.draw_latest()
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()