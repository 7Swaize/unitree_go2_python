from typing_extensions import override

import iceoryx2 as iox2
from iceoryx_interfaces.mappings import SportCommand
from iceoryx_interfaces.qos import SportQoS
from iceoryx_interfaces.sport_cmds import (
    SportCommandHeader_,
    NoArgsData_,
    FloatArgsData_,
    CommandResponse_
)

from .movement_module import MovementModule


class VirtualCommandHandle:
    """
    Handle for tracking completion of an asynchronous command sent to the simulator.

    Returned by every :class:`VirtualMovementModule` command method. Call :meth:`wait` to block until the
    simulator has processed the request (or the connection is dropped), since sends are (by default) non-blocking.

    Important
    ---------
    This handle should **NEVER** be cached or stored beyond its immediate use. Call :meth:`wait` on it (or
    otherwise consume it) and let it go out of scope.

    The underlying ``pending_response`` and ``node`` bindings rely on their Python wrapper's refcount to know
    when the corresponding native resources are safe to release. Holding an extra reference to this handle
    (e.g. storing it in a list, an instance attribute, or any other long-term container) keeps that refcount
    artificially elevated, which prevents the GC from collecting the handle and, in turn, prevents the resources
    it owns from being freed.
    """
    def __init__(self, pending_response: iox2.PendingResponse, node: iox2.Node, cycle_time: iox2.Duration) -> None:
        self._pending_response = pending_response
        self._node = node
        self._cycle_time = cycle_time

    def wait(self) -> None:
        """
        Block until the simulator has processed this command, or the connection is dropped.

        Repeatedly polls the underlying response channel, sleeping for one cycle time between checks.
        Releases the GIL during sleep.
        """
        while True:
            if not self._pending_response.is_connected:
                return

            self._node.wait(self._cycle_time)
            response = self._pending_response.receive()

            if response is not None:
                return


class VirtualMovementModule(MovementModule):
    """
    Simulator-backed :class:`MovementModule` implementation that issues sport commands over iceoryx2 IPC
    request/response services rather than the Unitree SDK.

    Notes
    -----
    Each command is fire-and-forget and returns a :class:`VirtualCommandHandle` that can be awaited for completion. 
    """

    def __init__(self) -> None:
        super().__init__()
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)

    @override
    def _initialize(self) -> None:
        self._node = iox2.NodeBuilder.new() \
                        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
                        .create(iox2.ServiceType.Ipc)
        
        self._noargs_service = self._node.service_builder(iox2.ServiceName.new(SportQoS.TOPIC_SIM_NOARGS_CMD)) \
                                    .request_response(NoArgsData_, CommandResponse_) \
                                    .request_header(SportCommandHeader_) \
                                    .open_or_create()
        
        self._floatargs_service = self._node.service_builder(iox2.ServiceName.new(SportQoS.TOPIC_SIM_FLOATARGS_CMD)) \
                                    .request_response(FloatArgsData_, CommandResponse_) \
                                    .request_header(SportCommandHeader_) \
                                    .open_or_create()
        
        self._noargs_client = self._noargs_service.client_builder().create()
        self._floatargs_client = self._floatargs_service.client_builder().create()
        self._cycle_time = iox2.Duration.from_millis(50)
        self._initialized = True

    @override
    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> VirtualCommandHandle:
        """
        Command a movement velocity, expressed in the robot's own coordinate frame.

        Parameters
        ----------
        vx : float
            Forward/backward velocity in m/s, in the range [-0.5, 0.5].
        vy : float
            Lateral (side-to-side) velocity in m/s, in the range [-0.5, 0.5].
        vyaw : float
            Yaw turning rate in rad/s, in the range [-0.8, 0.8].

        Notes
        -----
        This interface has two properties worth being aware of: the underlying motion controller does not apply any
        filtering to the requested velocities, so abrupt changes in vx/vy/vyaw will be passed straight through to
        the gait controller; and the most recently sent command is held and kept active for 1 second, even if this
        method is not called again. When you are done commanding movement, explicitly call this
        with all zeros or call :meth:`stop_move`, rather than relying on the 1-second hold to expire.
        """
        sample = self._floatargs_client.loan_uninit()
        sample.user_header().contents.command = SportCommand.MOVE
        sample = sample.write_payload(
            FloatArgsData_(arg1=vx, arg2=vy, arg3=vyaw)
        )

        return VirtualCommandHandle(sample.send())

    @override
    def stand_up(self) -> VirtualCommandHandle:
        """
        Lock the joints and stand tall at the default standing height.

        Notes
        -----
        This is the locked-joint counterpart to :meth:`stand_down`, standing tall instead of lowering
        the robots body close to the ground.
        """
        sample = self._noargs_client.loan_uninit()
        sample.user_header().contents.command = SportCommand.STAND_UP
        sample = sample.write_payload(
            NoArgsData_(null=0)
        )

        return VirtualCommandHandle(sample.send())

    @override
    def stand_down(self) -> VirtualCommandHandle:
        """
        Lock the joints and crouch down low (essentially sitting down).

        Notes
        -----
        This is the locked-joint counterpart to :meth:`stand_up`, lowering the robot's body close to the ground
        instead of standing tall.
        """
        sample = self._noargs_client.loan_uninit()
        sample.user_header().contents.command = SportCommand.STAND_DOWN
        sample = sample.write_payload(
            NoArgsData_(null=0)
        )

        return VirtualCommandHandle(sample.send())

    @override
    def stop_move(self) -> VirtualCommandHandle:
        """
        Stop the current motion and reset motion parameters to their defaults.

        This is the recommended way to terminate a velocity command issued via :meth:`move` when you
        are done with it, rather than letting it simply time out.
        """
        sample = self._noargs_client.loan_uninit()
        sample.user_header().contents.command = SportCommand.STOP_MOVE
        sample = sample.write_payload(
            NoArgsData_(null=0)
        )

        return VirtualCommandHandle(sample.send())

    @override
    def damp(self) -> VirtualCommandHandle:
        """
        Enter damping state, stopping all motor joint movement.

        Notes
        -----
        This mode has the highest priority of any command and is intended for emergency stops in unexpected
        situations. It will override whatever the robot is currently doing.
        """
        sample = self._noargs_client.loan_uninit()
        sample.user_header().contents.command = SportCommand.DAMP
        sample = sample.write_payload(
            NoArgsData_(null=0)
        )

        return VirtualCommandHandle(sample.send())

    @override
    def balance_stand(self) -> VirtualCommandHandle:
        """
        Release the joint motor lock and switch to balance standing mode. 
        Within the simulator, this functions the same as :meth:`stand_up`.
        """
        sample = self._noargs_client.loan_uninit()
        sample.user_header().contents.command = SportCommand.BALANCE_STAND
        sample = sample.write_payload(
            NoArgsData_(null=0)
        )

        return VirtualCommandHandle(sample.send())

    @override
    def _shutdown(self) -> None:
        pass