from typing_extensions import override

from unitree_sdk2py.go2.sport.sport_client import SportClient

from .movement_module import MovementModule


class NativeMovementModule(MovementModule[None]):
    """
    Hardware-backed :class:`MovementModule` implementation that drives the physical Go2 directly through the
    Unitree SDK's ``SportClient``.

    Note
    ----
    Commands issued here take effect on real hardware immediately and are not simulated or sandboxed in any way.
    The underlying ``SportClient`` calls are fire-and-forget from this wrapper's perspective.
    """

    def __init__(self) -> None:
        super().__init__()

    @override
    def _initialize(self) -> None:
        if self._initialized:
            return

        self._sport_client = SportClient()
        self._sport_client.Init() # type: ignore[no-untyped-call]
        self._sport_client.SetTimeout(3.0)

        self._initialized = True

    @override
    def _shutdown(self) -> None:
        pass

    @override
    def damp(self) -> None:
        """
        Enter damping state, stopping all motor joint movement.

        Note
        ----
        This mode has the highest priority of any command and is intended for emergency stops in unexpected
        situations. It will override whatever the robot is currently doing.
        """
        self._sport_client.Damp() # type: ignore[no-untyped-call]

    @override
    def balance_stand(self) -> None:
        """
        Release the joint motor lock and switch to balance standing mode.

        Note
        ----
        Unlike the plain locked-joint standing mode, the robot's posture and height will actively remain balanced
        regardless of the terrain underneath it. Once in this mode, posture can be adjusted in real time via
        :meth:`euler`.
        """
        self._sport_client.BalanceStand() # type: ignore[no-untyped-call]

    @override
    def stop_move(self) -> None:
        """
        Stop the current motion and reset motion parameters to their defaults.

        Note
        ----
        This halts whatever movement is in progress and resets Go2's internal motion parameters back to their
        default values. This is the recommended way to terminate a velocity command issued via :meth:`move` when you
        are done with it, rather than letting it simply time out.
        """
        self._sport_client.StopMove() # type: ignore[no-untyped-call]

    @override
    def stand_up(self) -> None:
        """
        Lock the joints and stand tall at the default standing height.

        Note
        ----
        Unlike balance standing mode, the robot's posture will not actively compensate for uneven terrain here --
        the joints are simply locked at a fixed standing height of 0.33m. Because the posture is rigidly locked,
        this mode can easily cause motor overheating if held too long, so it should be exited (e.g. via
        :meth:`balance_stand` or :meth:`damp`) promptly once it is no longer needed.
        """
        self._sport_client.StandUp() # type: ignore[no-untyped-call]

    @override
    def stand_down(self) -> None:
        """
        Lock the joints and crouch down low.

        Note
        ----
        This is the locked-joint counterpart to :meth:`stand_up`, lowering the robot's body close to the ground
        instead of standing tall.
        """
        self._sport_client.StandDown() # type: ignore[no-untyped-call]

    def recovery_stand(self) -> None:
        """
        Recover to a balanced standing position from any current state.

        Note
        ----
        This works regardless of whether the robot is currently fallen over, crouched, or in some other irregular
        pose. It will always attempt to bring the robot back to a balanced standing position, making it a useful
        general-purpose recovery command after a fall or unexpected disturbance.
        """
        self._sport_client.RecoveryStand() # type: ignore[no-untyped-call]

    def euler(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        """
        Set the robot's posture (Euler) angles while standing or moving.

        Parameters
        ----------
        roll : float
            Roll angle in radians, in the range [-0.75, 0.75].
        pitch : float
            Pitch angle in radians, in the range [-0.75, 0.75].
        yaw : float
            Yaw angle in radians, in the range [-0.6, 0.6].

        Note
        ----
        This only has an effect while the robot is in balance standing mode (see :meth:`balance_stand`) or actively
        moving. The three angles are applied as rotations about the robot's own relative axes, composed in z-y-x
        order (yaw, then pitch, then roll).
        """
        self._sport_client.Euler(roll, pitch, yaw)

    @override
    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> None:
        """
        Command a movement velocity, expressed in the robot's own coordinate frame.

        Parameters
        ----------
        vx : float
            Forward/backward velocity in m/s, in the range [-2.5, 3.8].
        vy : float
            Lateral (side-to-side) velocity in m/s, in the range [-1.0, 1.0].
        vyaw : float
            Yaw turning rate in rad/s, in the range [-4, 4].

        Note
        ----
        This interface has two properties worth being aware of: the underlying motion controller does not apply any
        filtering to the requested velocities, so abrupt changes in vx/vy/vyaw will be passed straight through to
        the gait controller; and the most recently sent command is held and kept active for 1 second, even if this
        method is not called again. When you are done commanding movement, explicitly call this
        with all zeros or call :meth:`stop_move`, rather than relying on the 1-second hold to expire.
        """
        self._sport_client.Move(vx, vy, vyaw)

    def sit(self) -> None:
        """
        Perform the sit-down special action.

        Note
        ----
        This is a scripted special action rather than a continuous control mode. As with other special actions, it
        should only be triggered once the robot's previous action has fully completed. Issuing it mid-action can
        produce abnormal or jerky movement.
        """
        self._sport_client.Sit() # type: ignore[no-untyped-call]

    def rise_sit(self) -> None:
        """
        Recover from a seated position back to a balanced standing position.

        Note
        ----
        This is the counterpart to :meth:`sit`, used to bring the robot from sitting back up onto its feet in a
        balanced stance.
        """
        self._sport_client.RiseSit() # type: ignore[no-untyped-call]

    def speed_level(self, level: int) -> None:
        """
        Set the overall movement speed level.

        Parameters
        ----------
        level : int
            Speed level enumeration value: -1 for slow, 0 for normal, and 1 for fast.
        """
        self._sport_client.SpeedLevel(level)

    def hello(self) -> None:
        """Perform a greeting gesture."""
        self._sport_client.Hello() # type: ignore[no-untyped-call]

    def stretch(self) -> None:
        """Perform a stretching motion."""
        self._sport_client.Stretch() # type: ignore[no-untyped-call]

    def switch_joystick(self, flag: bool) -> None:
        """
        Enable or disable the robot's native joystick response.

        Parameters
        ----------
        flag : bool
            True to enable the native joystick response, False to disable it.

        Note
        ----
        With the native joystick response disabled, moving a physically connected joystick will have no effect on
        the robot and will not interfere with whatever program is currently controlling it (useful when a custom
        controller (such as this module) should have exclusive control).
        """
        self._sport_client.SwitchJoystick(flag)

    def content(self) -> None:
        """Perform an expression of happiness."""
        self._sport_client.Content() # type: ignore[no-untyped-call]

    def heart(self) -> None:
        """Make a heart shape with the front paws."""
        self._sport_client.Heart() # type: ignore[no-untyped-call]

    def pose(self, flag: bool) -> None:
        """
        Strike a pose, or return to normal stance.

        Parameters
        ----------
        flag : bool
            True to strike a pose, False to return to normal stance.
        """
        self._sport_client.Pose(flag)

    def scrape(self) -> None:
        """Bow as a greeting gesture."""
        self._sport_client.Scrape() # type: ignore[no-untyped-call]

    def front_flip(self) -> None:
        """
        Perform a front flip.

        Note
        ----
        This action is potentially dangerous. Maintain a safe distance from people and objects before executing
        it, since Unitree does not accept liability for accidents caused by improper use. It may also accelerate
        hardware wear and reduce the robot's operational lifespan, so it should be used sparingly and with caution.
        """
        self._sport_client.FrontFlip() # type: ignore[no-untyped-call]

    def front_jump(self) -> None:
        """Perform a front jump."""
        self._sport_client.FrontJump() # type: ignore[no-untyped-call]

    def front_pounce(self) -> None:
        """Lunge forward."""
        self._sport_client.FrontPounce() # type: ignore[no-untyped-call]

    def dance1(self) -> None:
        """Perform dance sequence 1."""
        self._sport_client.Dance1() # type: ignore[no-untyped-call]

    def free_bound(self, flag: bool) -> None:
        """
        Enter or exit bound run mode.

        Parameters
        ----------
        flag : bool
            True to enter bound run mode, False to exit it and return to agile mode.

        Note
        ----
        Bound run mode uses a bounding gait, where the front and rear leg pairs move together in near-unison rather
        than the more typical diagonal trot pattern.
        """
        self._sport_client.FreeBound(flag)

    def free_jump(self, flag: bool) -> None:
        """
        Enter or exit jump mode.

        Parameters
        ----------
        flag : bool
            True to enter jump mode, False to exit it and return to agile mode.

        Note
        ----
        Jump mode uses a jump-running gait, with a more pronounced airborne phase between strides than the robot's
        regular gaits.
        """
        self._sport_client.FreeJump(flag)

    def free_avoid(self, flag: bool) -> None:
        """
        Enter or exit obstacle avoidance mode.

        Parameters
        ----------
        flag : bool
            True to enter avoidance mode, False to exit it and return to agile mode.

        Note
        ----
        While in this mode and moving, the robot will actively steer around obstacles it detects. While stationary,
        it can also dodge out of the way of objects approaching it from in front.
        """
        self._sport_client.FreeAvoid(flag)

    def walk_upright(self, flag: bool) -> None:
        """
        Enter or exit rear-leg (upright) standing mode.

        Parameters
        ----------
        flag : bool
            True to enter rear-leg standing mode, False to exit it and return to agile mode.

        Note
        ----
        In this mode the robot balances on its rear legs in an upright stance. The motors involved are prone to
        overheating in this gait, so calling code should manage timing carefully and avoid holding this mode for
        extended periods, to prevent the robot falling due to motor overheating.
        """
        self._sport_client.WalkUpright(flag)

    def cross_step(self, flag: bool) -> None:
        """
        Enter or exit cross-step mode.

        Parameters
        ----------
        flag : bool
            True to enter cross-step mode, False to exit it and return to agile mode.

        Note
        ----
        As with :meth:`walk_upright`, the motors involved in this gait are prone to overheating, so calling code
        should manage timing carefully and avoid holding this mode for extended periods, to prevent the robot
        falling due to motor overheating.
        """
        self._sport_client.CrossStep(flag)

    def classic_walk(self, flag: bool) -> None:
        """
        Enter or exit classic AI gait mode.

        Parameters
        ----------
        flag : bool
            True to enter classic gait mode, False to exit it and return to agile mode.

        Note
        ----
        This is Go2's AI-driven classic gait, with strong adaptability to complex terrain. It supports climbing
        stairs, and traversing gravel, meadows, and slippery surfaces, all while maintaining a stable and visually
        elegant walking posture.
        """
        self._sport_client.ClassicWalk(flag)

    def trot_run(self) -> None:
        """
        Enter regular running mode.

        Note
        ----
        This is a running gait with a maximum speed of 3.7 m/s. It carries some risk due to its high speed, so it
        should be used cautiously, and it has no special handling for complex terrain, making it prone to falling on
        uneven ground.
        """
        self._sport_client.TrotRun() # type: ignore[no-untyped-call]

    def static_walk(self) -> None:
        """
        Enter regular walking mode.

        Note
        ----
        This is Go2's default gait in normal (non-AI) mode. Like :meth:`trot_run`, it has no special handling for
        complex terrain, but it produces an elegant, stable-looking walking posture on flat ground.
        """
        self._sport_client.StaticWalk() # type: ignore[no-untyped-call]

    def switch_avoid_mode(self) -> None:
        """
        Toggle obstacle-avoidance suppression while in avoidance mode.

        Note
        ----
        While already in obstacle avoidance mode (see :meth:`free_avoid`), this additionally disables front-facing
        avoidance whenever the joystick is not being pushed, as well as disabling rear-facing avoidance entirely.
        This is a fairly niche behavior and is generally not recommended unless you specifically understand and need
        this interaction.
        """
        self._sport_client.SwitchAvoidMode() # type: ignore[no-untyped-call]

    def auto_recovery_set(self, flag: bool) -> None:
        """
        Enable or disable automatic recovery after a fall.

        Parameters
        ----------
        flag : bool
            True to enable auto-recovery, False to disable it.

        Note
        ----
        If the robot is carrying a payload (such as a gimbal, camera, or other sensor mounted on its head) it's
        recommended to disable this setting. A violent auto-recovery flip after a fall could otherwise damage that
        payload, so disabling auto-recovery avoids triggering that flip.
        """
        self._sport_client.AutoRecoverySet(flag)

    def auto_recovery_get(self) -> bool:
        """
        Check whether automatic recovery after a fall is currently enabled.

        Returns
        -------
        bool
            True if auto-recovery is enabled, False otherwise.
        """
        _, flag = self._sport_client.AutoRecoveryGet() # type: ignore[no-untyped-call]
        return bool(flag)