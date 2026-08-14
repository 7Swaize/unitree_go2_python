import time

from go2.core import HardwareType, NativeGo2Controller, create_controller


def init_controller() -> NativeGo2Controller:
    # Create a Go2 Controller instance to access all provided funtionalities.
    # Here, we choose HardwareType.NATIVE because we want movements commands to be executed on the actual robot.
    # It's recommended to use the 'create_controller' factory method to create the controller (as shown below).
    controller = create_controller(hardware_type=HardwareType.NATIVE)

    # NOTE: The movement module is a default module. It is already added onto the controller.
    # Therefore, we do not need to explictly add it.

    return controller


def run_movement_sequence(controller: NativeGo2Controller) -> None:
    # Stand Up
    controller.movement.stand_up()
    time.sleep(3)

    # Move forward
    # Velocity is held for 1s. We loop to maintain motion for approx 3 seconds
    for _ in range(4):
        # Paramter in units of m/s
        controller.movement.move(1, 0, 0)
        time.sleep(0.75)

    # Clear internal velocity command buffer
    controller.movement.stop_move()

    # Rotate
    # Rotation is held for 1s. We loop to maintain motion for approx 3 seconds
    for _ in range(4):
        # Parameter in units of rad/s
        controller.movement.move(0, 0, 0.5)
        time.sleep(0.75)

    # Clear internal velocity command buffer
    controller.movement.stop_move()

    # Stand down
    controller.movement.stand_down()


if __name__ == "__main__":
    # Create a controller instance.
    controller = init_controller()

    # Run our test movement sequence.
    run_movement_sequence(controller)

    # Shutdown the controller.
    # It is critical to always shutdown the control on script exit.
    controller.safe_shutdown()