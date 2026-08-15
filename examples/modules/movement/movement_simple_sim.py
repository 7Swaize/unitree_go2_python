import time

from go2.core import HardwareType, VirtualGo2Controller, create_controller


def init_controller() -> VirtualGo2Controller:
    # Create a Go2 Controller instance to access all provided funtionalities.
    # Here, we choose HardwareType.VIRTUAL because we want movements commands to be executed in the simulation.
    # It's recommended to use the 'create_controller' factory method to create the controller (as shown below).
    controller = create_controller(hardware_type=HardwareType.VIRTUAL)

    # NOTE: The movement module is a default module. It is already added onto the controller.
    # Therefore, we do not need to explictly add it.

    return controller


def run_movement_sequence(controller: VirtualGo2Controller) -> None:
    # Simulated movement commands return a 'VirtualCommandHandle' which can be used to await an operation.
    # This blocks the current thread until the command finishes.
    # Here we wait for the stand up to complete, so we don't send movement commands during the stand-up sequence.
    controller.movement.stand_up().wait()

    for _ in range(4):
        # Move forward for about 3 seconds at 0.5 m/s.
        # Since 'move' commands are retained for 1 second internally, we need to continously refresh the timeout.
        controller.movement.move(0.5, 0, 0)
        time.sleep(0.75)

    # Stop all active motion
    controller.movement.stop_move().wait()

    for _ in range(4):
        # Rotate counter-clockwise for about 3 seconds at 0.8 rad/s.
        controller.movement.move(0, 0, 0.8)
        time.sleep(0.75)

    controller.movement.stop_move().wait()

    # Sit back down
    controller.movement.stand_down().wait()


if __name__ == "__main__":
    # Create a controller instance.
    controller = init_controller()

    # Run our test movement sequence.
    run_movement_sequence(controller)

    # Shutdown the controller.
    # It is critical to always shutdown the control on script exit.
    controller.safe_shutdown()