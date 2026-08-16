from typing import Literal
from typing_extensions import overload

from .registry import ExecutionMode, HardwareType
from .controller import Go2Controller, NativeGo2Controller, VirtualGo2Controller


@overload
def create_controller(hardware_type: Literal[HardwareType.NATIVE], execution_mode: ExecutionMode = ...) -> NativeGo2Controller: ...

@overload
def create_controller(hardware_type: Literal[HardwareType.VIRTUAL], execution_mode: ExecutionMode = ...) -> VirtualGo2Controller: ...

def create_controller(hardware_type: HardwareType, execution_mode: ExecutionMode = ExecutionMode.BASIC) -> Go2Controller:
    """Factory method to create a Go2Controller instance."""
    cls = NativeGo2Controller if hardware_type == HardwareType.NATIVE else VirtualGo2Controller
    return cls(execution_mode)