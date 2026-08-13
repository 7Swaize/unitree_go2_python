import numpy as np
from typing import Callable


class CallbackDispatcher:
    def __init__(self) -> None:
        self._decoded_callbacks: list[Callable[[int, np.ndarray], None]] = []

    def _register_decoded(self, cb: Callable[[int, np.ndarray], None]) -> None:
        self._decoded_callbacks.append(cb)

    def _emit_decoded(self, stamp_ns: int, array: np.ndarray) -> None:
        for cb in self._decoded_callbacks:
            cb(stamp_ns, array)