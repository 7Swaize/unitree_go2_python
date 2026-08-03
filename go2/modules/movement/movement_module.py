from typing import Tuple
from typing_extensions import override

from ...core.module import DogModule
from ...hardware.hardware_interface_movement import HardwareInterfaceMovement


class MovementModule(DogModule):
    """
    Users should not access or construct this class directly.
    Rather, they should access it through the :class:`~core.controller.Go2Controller` instance.

    Parameters
    ----------
    hardware : HardwareInterfaceMovement
        The underlying movement hardware interface that communicates with the robot.

    Important
    ---------
    - Virtual and native movement commands may not function exactly the same way.
      Their behavior is not fully transitive across backends, so they should be used with care when switching between them.
    - Commands are intended to wrap or imitate the ``SportClient`` class provided by the Unitree SDK.
      For details see `Motion Services Interface V2.0 <https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0>`_.
    """

    def __init__(self, hardware: HardwareInterfaceMovement) -> None:
        """
        Create the MovementModule. 

        Sets up maximum safe limits for rotation and translation.
        """
        super().__init__("Movement")
        self.hardware = hardware
        self.max_rotation = 5.0
        self.max_translation = 5.0

    @override
    def _initialize(self) -> None:
        """
        Prepare the movement module for use. This is called internally,
        and should not be called directly by users.

        This marks the module as initialized. It must be called
        before issuing movement commands.
        """
        self._initialized = True

    def move(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """
        Move the robot forward/backward, laterally, and rotate about its vertical axis.

        Parameters
        ----------
        x : float, optional
            Forward/backward movement amount. Positive is forward.
            Automatically clamped to [-max_translation, max_translation].
        y : float, optional
            Lateral movement amount. Positive is right.
            Automatically clamped to [-max_translation, max_translation].
        yaw : float, optional
            Rotation about the vertical (z) axis. Positive is counterclockwise.
            Automatically clamped to [-max_yaw, max_yaw].

        Notes
        -----
        - Translational movements are relative to the robot's current orientation.
        - Translation and yaw rotation are executed as a single relative motion.
        """
        x = max(-self.max_translation, min(x, self.max_translation))
        y = max(-self.max_translation, min(y, self.max_translation))
        yaw = max(-self.max_rotation, min(yaw, self.max_rotation))
        self.hardware._move(x, y, yaw)

    def stand_up(self) -> None:
        """
        Command the robot to stand up.

        Notes
        -----
        - It's best practice to wait a second after standing up before issuing movement commands.
        """
        self.hardware._stand_up()

    def stand_down(self) -> None:
        """
        Command the robot to lay down.
        """
        self.hardware._stand_down()

    def stop(self) -> None:
        """
        Stop all robot movement immediately.

        Notes
        -----
        - Can be called at any time to halt translation and rotation. Can be used to clear internal command buffer.
        """
        self.hardware._stop_move()

    def damp(self) -> None:
        """
        Command the robot to damp.
        """
        self.hardware._damp()

    def balance_stand(self) -> None:
        """
        Command the robot to balance stand.
        """
        self.hardware._balance_stand()

    def recovery_stand(self) -> None:
        """
        Command the robot to recovery stand.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._recovery_stand()

    def euler(self, roll: float, pitch: float, yaw: float) -> None:
        """
        Set the robot's Euler angles.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._euler(roll, pitch, yaw)

    def sit(self) -> None:
        """
        Command the robot to sit.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._sit()

    def rise_sit(self) -> None:
        """
        Command the robot to rise from sit.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._rise_sit()

    def speed_level(self, level: int) -> None:
        """
        Set the robot's speed level.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._speed_level(level)

    def hello(self) -> None:
        """
        Command the robot to perform hello.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._hello()

    def stretch(self) -> None:
        """
        Command the robot to stretch.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._stretch()

    def content(self) -> None:
        """
        Command the robot to perform content.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._content()

    def dance1(self) -> None:
        """
        Command the robot to perform dance1.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._dance1()

    def dance2(self) -> None:
        """
        Command the robot to perform dance2.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._dance2()

    def switch_joystick(self, on: bool) -> None:
        """
        Switch joystick mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._switch_joystick(on)

    def pose(self, flag: bool) -> None:
        """
        Set pose mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._pose(flag)

    def scrape(self) -> None:
        """
        Command the robot to scrape.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._scrape()

    def front_flip(self) -> None:
        """
        Command the robot to perform front flip.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._front_flip()

    def front_jump(self) -> None:
        """
        Command the robot to perform front jump.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._front_jump()

    def front_pounce(self) -> None:
        """
        Command the robot to perform front pounce.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._front_pounce()

    def heart(self) -> None:
        """
        Command the robot to perform heart.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._heart()

    def left_flip(self) -> None:
        """
        Command the robot to perform left flip.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._left_flip()

    def back_flip(self) -> None:
        """
        Command the robot to perform back flip.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._back_flip()

    def free_walk(self) -> None:
        """
        Command the robot to free walk.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._free_walk()

    def free_bound(self, flag: bool) -> None:
        """
        Set free bound mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._free_bound(flag)

    def free_jump(self, flag: bool) -> None:
        """
        Set free jump mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._free_jump(flag)

    def free_avoid(self, flag: bool) -> None:
        """
        Set free avoid mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._free_avoid(flag)

    def walk_upright(self, flag: bool) -> None:
        """
        Set walk upright mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._walk_upright(flag)

    def cross_step(self, flag: bool) -> None:
        """
        Set cross step mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._cross_step(flag)

    def static_walk(self) -> None:
        """
        Command the robot to static walk.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._static_walk()

    def trot_run(self) -> None:
        """
        Command the robot to trot run.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._trot_run()

    def hand_stand(self, flag: bool) -> None:
        """
        Set hand stand mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._hand_stand(flag)

    def classic_walk(self, flag: bool) -> None:
        """
        Set classic walk mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._classic_walk(flag)

    def auto_recovery_set(self, enabled: bool) -> None:
        """
        Set auto recovery mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._auto_recovery_set(enabled)

    def get_auto_recovery(self) -> Tuple[int, bool | None]:
        """
        Get auto recovery status.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        return self.hardware._auto_recovery_get()

    def switch_avoid_mode(self) -> None:
        """
        Switch avoid mode.

        Important
        ---------
        This command is not supported when :attr:`HardwareType.VIRTUAL <go2.hardware.hardware_type.HardwareType.VIRTUAL>` is selected and will raise NotImplementedError.
        """
        self.hardware._switch_avoid_mode()

    @override
    def _shutdown(self) -> None:
        """
        Shut down the movement module safely. This should not be called by users

        Stops all movement and marks the module as uninitialized.
        """
        self.stop()
        self._initialized = False