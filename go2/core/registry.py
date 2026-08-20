from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Dict, Generic, Optional, Type, TypeVar, Callable

from ..modules.audio import AudioModule
from ..modules.input import InputModule
from ..modules.lidar import NativeLIDARModule, VirtualLIDARModule
from ..modules.movement import NativeMovementModule, VirtualMovementModule
from ..modules.ocr import OCRModule
from ..modules.video import VideoModule
from .module import DogModule


T = TypeVar('T', bound=DogModule)

class ExecutionMode(Enum):
    """
    Enumeration passed upon creation of a :class:`~core.controller.Go2Controller` instance.
    Dicates the available modules based on system installed packages.

    Installation for all dependencies can be found here: https://github.com/7Swaize/go2-control/blob/main/docs/public/install/installation.md
    """
    BASIC = auto() #: Restricts access to the :class:`~core.registry.ModuleType.LIDAR` module.
    ADVANCED = auto() #: Allows access to **ALL** modules.



class ModuleType(Enum):
    """
    Enumeration of all supported default module categories.

    This enum is used by the module registry to identify and construct
    modules dynamically.
    """
    VIDEO = auto()
    MOVEMENT = auto()
    INPUT = auto()
    OCR = auto()
    AUDIO = auto()
    LIDAR = auto()



class HardwareType(Enum):
    """Enumeration passed upon creation of a :class:`~core.controller.Go2Controller` instance."""
    NATIVE = auto() #: Commands should act upon an actual Unitree-Go2 robot.
    VIRTUAL = auto() #: Commands should act upon the simulator (launched on controller creation in `VIRTUAL`` mode).



@dataclass
class ModuleDescriptor(Generic[T]):
    """
    Descriptor linking a module type to its implementation.

    Contains all metadata required to construct and display a module in the system.
    """

    module_type: ModuleType  #: Enum identifying the module
    module_class: Optional[Type[T]]  #: Optional concrete implementation class
    display_name: str  #: Human-readable name
    requires_native_hardware: bool = False  #: Whether this module needs native hardware
    requires_advanced_execution: bool = False  #: Whether this module needs advanced execution mode
    class_resolver: Optional[Callable[[HardwareType], Type[T]]] = None #: Resolver used during resolution

    def __post_init__(self) -> None:
        if self.module_class is None and self.class_resolver is None:
            raise ValueError(f"{self.module_type}: must provide 'module_class' or 'class_resolver'")

    def _resolve_class(self, hardware_type: HardwareType) -> Type[T]:
        return self.class_resolver(hardware_type) if self.class_resolver else self.module_class # type: ignore[return-value]
 
    def create_instance(self, hardware_type: HardwareType, *args: Any, **kwargs: Any) -> T:
        return self._resolve_class(hardware_type)(*args, **kwargs)
    

class ModuleRegistry:
    """
    Central registry for all available modules.

    The registry maps ``ModuleType`` values to their corresponding implementations and metadata. It is responsible for controlling
    which modules are available based on system configuration.

    Note
    ----
    - This is an internal system component.
    - Modules are registered at startup.
    """

    _descriptors: Dict[ModuleType, ModuleDescriptor[DogModule]] = {}

    @classmethod
    def _register(cls, descriptor: ModuleDescriptor[DogModule]) -> None:
        """
        Register a module descriptor.

        Parameters
        ----------
        descriptor : ModuleDescriptor
            Descriptor to register.
        """
        cls._descriptors[descriptor.module_type] = descriptor

    @classmethod
    def get_descriptor(cls, module_type: ModuleType) -> Optional[ModuleDescriptor[DogModule]]:
        """Retrieve the descriptor for a module type."""
        return cls._descriptors.get(module_type)
 
    @classmethod
    def is_registered(cls, module_type: ModuleType) -> bool:
        """Check whether a module type is registered."""
        return module_type in cls._descriptors

    @classmethod
    def _register_defaults(cls) -> None:
        """
        Register all default system modules.
 
        Called automatically at import time to populate the module registry
        with all built-in modules.
        """
        cls._register(ModuleDescriptor(
            ModuleType.VIDEO,
            VideoModule,
            "Video Capture",
            requires_native_hardware=False,
            requires_advanced_execution=False,
        ))
 
        cls._register(ModuleDescriptor(
            ModuleType.MOVEMENT,
            None,
            "Movement Control",
            requires_native_hardware=False,
            requires_advanced_execution=False,
            class_resolver=lambda ht: (
                NativeMovementModule if ht == HardwareType.NATIVE else VirtualMovementModule
            ),
        ))
 
        cls._register(ModuleDescriptor(
            ModuleType.OCR,
            OCRModule,
            "Optical Character Recognition",
            requires_native_hardware=False,
            requires_advanced_execution=False,
        ))
 
        cls._register(ModuleDescriptor(
            ModuleType.AUDIO,
            AudioModule,
            "Text-to-Speech",
            requires_native_hardware=False,
            requires_advanced_execution=False,
        ))
 
        cls._register(ModuleDescriptor(
            ModuleType.INPUT,
            InputModule,
            "Controller Input",
            requires_native_hardware=True,
            requires_advanced_execution=False,
        ))
 
        cls._register(ModuleDescriptor(
            ModuleType.LIDAR,
            None,
            "LIDAR Capture",
            requires_native_hardware=False,
            requires_advanced_execution=True,
            class_resolver=lambda ht: (
                NativeLIDARModule if ht == HardwareType.NATIVE else VirtualLIDARModule
            )
        ))


ModuleRegistry._register_defaults()