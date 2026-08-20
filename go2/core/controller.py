import os
import sys
import signal
import threading
from abc import ABC, abstractmethod
from types import FrameType
from typing import Callable, List, Dict, TypeVar, Type, Optional, Any
from typing_extensions import override

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from ..modules.input import InputSignal
from ..modules.audio import AudioModule
from ..modules.input import InputModule
from ..modules.movement import NativeMovementModule, VirtualMovementModule
from ..modules.ocr import OCRModule
from ..modules.video import VideoModule
from ..modules.lidar import NativeLIDARModule, VirtualLIDARModule
from ..logging import get_logger
from .module import DogModule
from .registry import ModuleRegistry, ModuleType, ExecutionMode, HardwareType

logger = get_logger(__name__)

# TTS: https://medium.com/@vndee.huynh/build-your-own-voice-assistant-and-run-it-locally-whisper-ollama-bark-c80e6f815cba
# Digging into Dog: https://www.darknavy.org/darknavy_insight/the_jailbroken_unitree_robot_dog

# Some very interesting turtorial with the G02: https://hackmd.io/@c12hQ00ySVi6JYIERU7bCg/ByAOr12qJg

# Update to JetPack 6.x on Dog's Jetson: https://theroboverse.com/unitree-go2-edu-jetpack-6-2-1-update/

# TETHERING + ROS2 SETUP: https://www.youtube.com/watch?v=Q_dqPLJDPms


T = TypeVar("T", bound=DogModule)

class Go2Controller(ABC):
    """    
    Primary control interface for the Unitree Go2 robot.

    This is the shared base for :class:`NativeGo2Controller` and :class:`VirtualGo2Controller`.
    It manages hardware initialization, module lifecycles, safety checks, and shutdown.

    Modules are accessed via properties rather than direct instantiation.

    Notes
    -----
        - All hardware access is routed through this controller.
        - Modules creation, initialization, and shutdown is handled automatically.
        - Supports both SDK-backed hardware and a Mujoco simulation.

    See Also
    --------
    DogModule
    ModuleRegistry
    NativeGo2Controller
    VirtualGo2Controller
    """
    def __init__(self, hardware_type: HardwareType, execution_mode: ExecutionMode = ExecutionMode.BASIC) -> None:
        """
        Parameters
        ----------
        hardware_type : HardwareType
            Selects the backend (NATIVE or VIRTUAL). Set implicitly by the concrete subclass.
        execution_mode : ExecutionMode
            Governs which optional modules (e.g. LIDAR) are available.

        Raises
        ------
        RuntimeError
            If hardware backend initialization fails.
        """
        self._hardware_type = hardware_type
        self._execution_mode = execution_mode

        self._emit_execution_mode_info()

        self._shutdown_event = threading.Event()
        self._shutdown_lock = threading.Lock()
        self._cleanup_callbacks_pre_module_shutdown: List[Callable[[], None]] = []
        self._cleanup_callbacks_post_module_shutdown: List[Callable[[], None]] = []

        self._install_signal_handlers()

        self._modules: Dict[ModuleType, DogModule] = {}


    def _emit_execution_mode_info(self) -> None:
        logger.info(f"[Controller] Executing in {'BASIC' if self._execution_mode == ExecutionMode.BASIC else 'ADVANCED'} execution mode.")
        if self._execution_mode == ExecutionMode.BASIC:
            logger.warning("[Controller] LIDAR functionalities not available in 'BASIC' execution")


    # Automatic shutdown on exception incase its not done by users
    def _install_signal_handlers(self) -> None:
        def handler(signum: int, frame: Optional[FrameType]) -> None:
            try:
                self.safe_shutdown()
            finally:
                signal.signal(signum, signal.SIG_DFL)  # Hands control back to default handler
                os.kill(os.getpid(), signum)

        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)


    @abstractmethod
    def _register_default_modules(self) -> None:
        pass


    def add_module(self, module_type: ModuleType, **kwargs: Any) -> None:
        """
        Add a module to the controller. Modules are initialized immediately upon addition.
        Manual module initialization is discouraged and should not be attempted.
        Duplicate module addition is prevented explicitly.

        Modules are identified using :class:`ModuleType` enums.

        Parameters
        ----------
        module_type : ModuleType
            The type of module to add.
        **kwargs
            Constructor arguments passed to the module implementation.

        Raises
        ------
        ValueError
            If the module type is not registered, requires native hardware support
            the controller doesn't provide, or requires advanced execution mode
            the controller isn't running under.
        """
        if module_type in self._modules:
            logger.info(f"[Controller] Module type '{module_type.name}' is already loaded. Returning existing instance.")
            return

        descriptor = ModuleRegistry.get_descriptor(module_type)
        if descriptor is None:
            raise ValueError(f"[Controller] Module type '{module_type.name}' is not registered")
        
        if descriptor.requires_native_hardware and self._hardware_type != HardwareType.NATIVE:
            raise ValueError(f"[Controller] Module type '{module_type.name}' requires native hardware support")

        if descriptor.requires_advanced_execution and self._execution_mode != ExecutionMode.ADVANCED:
            raise ValueError(
                f"[Controller] Module type '{module_type.name}' requires advanced execution mode. "
                "You may need to install ROS2 Humble libraries.\n"
                "[Controller] For more information see here: https://go2-control.readthedocs.io/en/latest/api/core.html#go2.core.registry.ExecutionMode"
            )

        module: DogModule = descriptor.create_instance(self._hardware_type, **kwargs)
        module._initialize()
        
        self._modules[module_type] = module


    def has_module(self, module_type: ModuleType) -> bool:
        """Check whether a module is currently loaded."""
        return module_type in self._modules


    def _get_module(self, module_type: ModuleType, expected: Type[T]) -> T:
        module = self._modules.get(module_type)

        if not isinstance(module, expected):
            raise RuntimeError(f"{expected.__name__} not loaded")
        if self.is_shutdown_requested():
            logger.debug(f"Cannot access {expected.__name__} after shutdown has been requested")

        return module


    @property
    def video(self) -> VideoModule:
        """
        Access the video capture module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.VIDEO, VideoModule)


    @property
    def audio(self) -> AudioModule:
        """
        Access the audio control module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.AUDIO, AudioModule)


    @property
    def ocr(self) -> OCRModule:
        """
        Access the OCR module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.OCR, OCRModule)


    def is_shutdown_requested(self) -> bool:
        """Check if shutdown has been requested"""
        return self._shutdown_event.is_set()


    def register_cleanup_callback_pre_module_shutdown(self, callback: Callable[[], None]) -> None:
        """
        Register a cleanup callback to be executed during safe shutdown before modules are stopped and before hardware is released.

        Parameters
        ----------
        callback : callable
            Zero-argument function to execute during shutdown.
        """
        self._cleanup_callbacks_pre_module_shutdown.append(callback)


    def register_cleanup_callback_post_module_shutdown(self, callback: Callable[[], None]) -> None:
        """
        Register a cleanup callback to be executed during safe shutdown after modules are stopped, but before hardware is released.

        Parameters
        ----------
        callback : callable
            Zero-argument function to execute during shutdown.
        """
        self._cleanup_callbacks_post_module_shutdown.append(callback)


    def safe_shutdown(self) -> None:
        """
        Perform a coordinated and safe shutdown.

        This method:
            - Signals shutdown to all subsystems
            - Shuts down all modules
            - Executes registered cleanup callbacks
            - Releases hardware resources   

        Important
        ---------
        Select modules depend on ROS2 nodes. These lifetime of these nodes must handled cleanly on process termination.
        Therefore, it is **critical** that students **ALWAYS** call :meth:`safe_shutdown` to ensure proper shutdown of ROS2.
        This called automatically on process crash (via `SIGINT` or `SIGTERM`). However, it is **fully** the 
        user's responsibility to call this method upon normal (error free) script exit.
        Failing to follow this can (**and probably will**) result in zombie ROS2 processes, unreleased resources, and UB.

        Notes
        -----
        - This method is **idempotent** and may be safely called multiple times.
        """
        logger.info("\n[Controller] Starting safe shutdown...")

        with self._shutdown_lock:
            if self._shutdown_event.is_set():
                return
            self._shutdown_event.set()

            for callback in self._cleanup_callbacks_pre_module_shutdown:
                try:
                    callback()
                except Exception:
                    logger.exception("[Controller] Cleanup callback failed")

            for module_type, module in self._modules.items():
                try:
                    module._shutdown()
                except Exception:
                    logger.exception(f"[Controller] Failed to shutdown {module_type.name}")

            for callback in self._cleanup_callbacks_post_module_shutdown:
                try:
                    callback()
                except Exception:
                    logger.exception("[Controller] Cleanup callback failed")
            
        logger.info("[Controller] Shutdown complete")



class NativeGo2Controller(Go2Controller):
    """
    Controller for a physical Go2 robot.

    Exposes every module, including :attr:`input`, and returns :class:`NativeMovementModule`
    from :attr:`movement`.
    """

    def __init__(self, execution_mode: ExecutionMode = ExecutionMode.BASIC) -> None:
        super().__init__(HardwareType.NATIVE, execution_mode)
        self._init_cyclonedds_services()
        self._register_default_modules()
        self._initialize_input_bindings()


    def _init_cyclonedds_services(self) -> None:
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])


    @override
    def _register_default_modules(self) -> None:
        self.add_module(ModuleType.MOVEMENT)
        self.add_module(ModuleType.INPUT)


    def _initialize_input_bindings(self) -> None:
        if self._hardware_type == HardwareType.NATIVE:
            self.input.register_callback(
                InputSignal.BUTTON_A,
                lambda _: self._shutdown_event.set(),
                "emergency_stop"
            )


    @property
    def movement(self) -> NativeMovementModule:
        """
        Access the native, movement control module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.MOVEMENT, NativeMovementModule)


    @property
    def input(self) -> InputModule:
        """
        Access the input control module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.INPUT, InputModule)


    @property
    def lidar(self) -> NativeLIDARModule:
        """
        Access the native, LIDAR control module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.LIDAR, NativeLIDARModule)



class VirtualGo2Controller(Go2Controller):
    """
    Controller for the simulated Go2 (Mujoco).

    Exposes every module except :attr:`~NativeGo2Controller.input` (no physical controller to bind to),
    and returns :class:`VirtualMovementModule` from :attr:`movement`.
    """

    def __init__(self, execution_mode: ExecutionMode = ExecutionMode.BASIC) -> None:
        super().__init__(HardwareType.VIRTUAL, execution_mode)
        self._register_default_modules()


    @override
    def _register_default_modules(self) -> None:
        self.add_module(ModuleType.MOVEMENT)


    @property
    def movement(self) -> VirtualMovementModule:
        """
        Access the virtual, movement control module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.MOVEMENT, VirtualMovementModule)


    @property
    def lidar(self) -> VirtualLIDARModule:
        """
        Access the virtual, LIDAR control module.

        Raises
        ------
        RuntimeError
            If the module is not loaded or shutdown has been requested.
        """
        return self._get_module(ModuleType.LIDAR, VirtualLIDARModule)