import numpy as np
import numpy.typing as npt
from typing import Callable


class CallbackDispatcher:
    __slots__ = ("_decoded_callbacks")
    
    def __init__(self) -> None:
        self._decoded_callbacks: list[Callable[[int, npt.NDArray[np.float32]], None]] = []

    def _register_decoded(self, cb: Callable[[int, npt.NDArray[np.float32]], None]) -> None:
        self._decoded_callbacks.append(cb)

    def _emit_decoded(self, stamp_ns: int, array: npt.NDArray[np.float32]) -> None:
        for cb in self._decoded_callbacks:
            cb(stamp_ns, array)