from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from ...core.module import DogModule

AbstractCommandResult = TypeVar('AbstractCommandResult')


class MovementModule(DogModule, ABC, Generic[AbstractCommandResult]):
    """
    Abstract base class defining the movement/sport control interface for the Go2, implemented by both
    native (hardware) and virtual (simulated) backends.

    Users should not access or construct this class (or any subclass) directly.
    Rather, they should access it through the :class:`~core.controller.Go2Controller` instance.

    Important
    ---------
    - Virtual and native movement commands may not function exactly the same way.
      Their behavior is not fully transitive across backends, so they should be used with care when switching between them.
    - Commands are intended to wrap or imitate the ``SportClient`` class provided by the Unitree SDK.
      For details see `Motion Services Interface V2.0 <https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0>`_.
    """

    def __init__(self) -> None:
        super().__init__("Movement")

    @abstractmethod
    def _initialize(self) -> None:
        """
        Prepare the movement module for use. This is called internally, and should not be called directly by users.

        This marks the module as initialized. It must be called before issuing movement commands.
        """
        pass

    @abstractmethod
    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> AbstractCommandResult:
        pass

    @abstractmethod
    def stand_up(self) -> AbstractCommandResult:
        pass

    @abstractmethod
    def stand_down(self) -> AbstractCommandResult:
        pass

    @abstractmethod
    def stop_move(self) -> AbstractCommandResult:
        pass

    @abstractmethod
    def damp(self) -> AbstractCommandResult:
        pass

    @abstractmethod
    def balance_stand(self) -> AbstractCommandResult:
        pass

    @abstractmethod
    def _shutdown(self) -> None:
        """
        Shut down the movement module safely. This should not be called by users
        """
        pass