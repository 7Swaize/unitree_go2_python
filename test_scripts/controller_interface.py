
import struct
import threading

from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ # specific to using Go2
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

from dataclasses import dataclass
from collections.abc import Callable
from typing import Dict, List, Optional
from abc import ABC, abstractmethod
from enum import Enum, auto
from concurrent.futures import ThreadPoolExecutor


class CallbackPriority(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

class RobotState(Enum):
    IDLE = 1
    MOVEMENT = 2
    PATHFINDING = 3
    EMERGENCY = 4
    CUSTOM = 5

@dataclass
class ControllerState:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    
    l1: float = 0.0
    l2: float = 0.0
    r1: float = 0.0
    r2: float = 0.0
    
    a: float = 0.0
    b: float = 0.0
    x: float = 0.0
    y: float = 0.0
    
    up: float = 0.0
    down: float = 0.0
    left: float = 0.0
    right: float = 0.0
    
    select: float = 0.0
    start: float = 0.0
    f1: float = 0.0
    f3: float = 0.0

    changed: bool = False
    
@dataclass
class CallbackInfo:
    callback: Callable[[ControllerState], None]
    priority : CallbackPriority
    states: List[RobotState]
    name: Optional[str] = None
    enabled: bool = True

# https://docs.python.org/3/library/abc.html
class StateHandler(ABC):
    def __init__(self, sport_client: SportClient) -> None:
        super().__init__()

        self.sport_client = sport_client

    @abstractmethod
    def on_enter(self, previous_state: RobotState) -> None:
        pass

    @abstractmethod
    def on_exit(self, next_state: RobotState) -> None:
        pass

    @abstractmethod
    def handle_controller_input(self, controller_state: ControllerState) -> None:
        pass

class IdleStateHandler(StateHandler):
    def on_enter(self, previous_state: RobotState) -> None:
        pass
    
    def on_exit(self, next_state: RobotState) -> None:
        pass
    
    def handle_controller_input(self, controller_state: ControllerState) -> None:
        pass

class MovementStateHandler(StateHandler):
    def on_enter(self, previous_state: RobotState) -> None:
        return super().on_enter(previous_state)
    
    def on_exit(self, next_state: RobotState) -> None:
        self.sport_client.Move(0, 0, 0)

    def handle_controller_input(self, controller_state: ControllerState) -> None:
        pass

class UnitreeRemoteControllerInputParser:
    def __init__(self) -> None:
        self._state = ControllerState()
        self._previous_state = ControllerState()

    def _parse_buttons(self, data1: int, data2: int) -> None:
        mapping1 = {
            0: "r1", 1: "l1", 2: "start", 3: "select",
            4: "r2", 5: "l2", 6: "f1", 7: "f3"
        }
        mapping2 = {
            0: "a", 1: "b", 2: "x", 3: "y",
            4: "up", 5: "right", 6: "down", 7: "left"
        }

        for i, attr in mapping1.items():
            setattr(self._state, attr, (data1 >> i) & 1)

        for i, attr in mapping2.items():
            setattr(self._state, attr, (data2 >> i) & 1)

    def _parse_analog(self, data: bytes):
        lx_offset = 4
        self.lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        L2_offset = 16
        L2 = struct.unpack('<f', data[L2_offset:L2_offset + 4])[0] # Placeholder，unused
        ly_offset = 20
        self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

    def parse(self, remote_data: bytes) -> ControllerState:
        self._previous_state = ControllerState(**self._state.__dict__)
        
        self._parse_analog(remote_data)
        self._parse_buttons(remote_data[2], remote_data[3])
          
        self._state.changed = self._detect_changes()

        return self._state
    
    def _detect_changes(self) -> bool:
        current_dict = {k: v for k, v in self._state.__dict__.items() 
                       if k not in ['timestamp', 'changed']}
        previous_dict = {k: v for k, v in self._previous_state.__dict__.items() 
                        if k not in ['timestamp', 'changed']}
        return current_dict != previous_dict


class CallbackManager:
    def __init__(self) -> None:
        self._callbacks: Dict[str, CallbackInfo] = {}
        self._execution_lock = threading.Lock()
        self._execution_pool = ThreadPoolExecutor(max_workers=4)

    def register_callback(
            self, 
            callback: Callable[[ControllerState], None],
            priority: CallbackPriority = CallbackPriority.MEDIUM,
            states: Optional[List[RobotState]] = None,
            name: Optional[str] = None
    ) -> str:
        if states is None:
            states = list(RobotState)

        callback_id = name or callback.__name__

        self._callbacks[callback_id] = CallbackInfo(
            callback=callback,
            priority=priority,
            states=states,
            name=callback_id,
            enabled=True
        )

        return callback_id

    def unregister_callback(self, callback_id: str) -> bool:
        return self._callbacks.pop(callback_id, None) is not None
    
    def enable_callback(self, callback_id: str, enabled: bool = True) -> bool:
        if (callback_id in self._callbacks):
            self._callbacks[callback_id].enabled = enabled
            return True
        
        return False
    
    def clear_callbacks(self) -> None:
        self._callbacks.clear()
    
    def execute_callbacks(self, controller_state: ControllerState, current_state: RobotState) -> None:
        if not controller_state.changed:
            return
        
        to_call = [
            callback for callback in self._callbacks.values()
            if callback.enabled and current_state in callback.states
        ]

        to_call.sort(key=lambda x: x.priority.value, reverse=True)

        with self._execution_lock:
            for callback_info in to_call:
                if callback_info.priority == CallbackPriority.CRITICAL:
                    self._execute_callback_sync(callback_info, controller_state)
                else:
                    self._execution_pool.submit(
                        self._execute_callback_sync,
                        callback_info,
                        controller_state
                    )

    def _execute_callback_sync(self, callback_info: CallbackInfo, controller_state: ControllerState) -> None:
        try:
            callback_info.callback(controller_state)
        except Exception as e:
            print(f"Callback {callback_info.name} failed: {e}")


