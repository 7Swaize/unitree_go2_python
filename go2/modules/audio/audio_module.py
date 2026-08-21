import queue
import pyttsx3
import threading
from typing import Optional
from typing_extensions import override

from ...core.module import DogModule


class AudioCommandHandle:
    """
    Handle for tracking completion of an asynchronous audio command sent to the ``pyttsx3`` engine.

    Returned by :meth:`AudioModule.say`.
    Call :meth:`wait` to block until the input text has been fully spoken, since commands are (by default) non-blocking.

    Note
    ----
    This handle does not have same restrictions as :class:`VirtualCommandHandle`.
    It doesn't wrap any underlying resources nor lock the ``pyttsx3`` engine.
    """
    def __init__(self, text: str) -> None:
        self._text = text
        self._done = threading.Event()

    @property
    def text(self) -> str:
        """Access queued string to be spoken for this handle."""
        return self._text

    def wait(self, timeout: Optional[float] = None) -> bool:
        """Block until ``pyttsx3`` has processed this text or shutdown is ran."""
        return self._done.wait(timeout)
    
    def _mark_done(self) -> None:
        self._done.set()


class AudioModule(DogModule):
    """
    Text-to-speech playback for the Go2 robot, backed by ``pyttsx3``.
    Specifically, it hands text off to a dedicated background thread that pulls requests off a
    queue (max 5) and feeds them to ``pyttsx3`` one at a time.

    Users should not access or construct this class directly. 
    Rather, they should access it through a :class:`~core.controller.Go2Controller` instance.
    """
    MAX_QUEUE: int = 5

    def __init__(self) -> None:
        super().__init__("Audio")
        self._queue: queue.Queue[AudioCommandHandle] = queue.Queue(maxsize=AudioModule.MAX_QUEUE)
        self._worker_thread_ref: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()


    @override
    def _initialize(self) -> None:
        if self._initialized:
            return

        self._engine = pyttsx3.init()
        self._worker_thread_ref = threading.Thread(target=self._run, daemon=True)
        self._worker_thread_ref.start()

        self._initialized = True


    def get_engine(self) -> pyttsx3.Engine:
        """
        Access the underlying pyttsx3 engine.

        Note
        ----
        Advanced use only (e.g. tweaking rate/voice/volume). Prefer :meth:`say` for normal playback.
        """
        if not self._initialized:
            raise RuntimeError("[Audio] Audio module must be initialized before accessing pyttsx3 engine")
        
        return self._engine


    def say(self, text: str, override: bool = False) -> AudioCommandHandle:
        """
        Queue ``text`` to be spoken.

        Parameters
        ----------
        text : str
            The text to speak.
        override: bool
            If ``True``, stop any current playback, drop everything still queued, and speak ``text`` immediately.
        """
        handle = AudioCommandHandle(text)

        if override:
            self._engine.stop()
            self._drain_and_cancel()
            self._queue.put_nowait(handle)
        else:
            try:
                self._queue.put_nowait(handle)
            except queue.Full:
                self._queue.get_nowait()._mark_done()
                self._queue.put_nowait(handle)

        return handle


    def _run(self) -> None:
        while True:
            handle = self._queue.get()
            if self._shutdown_event.is_set():
                return

            self._engine.say(handle.text)
            self._engine.runAndWait()
            handle._mark_done()


    def _drain_and_cancel(self) -> None:
        while True:
            try:
                self._queue.get_nowait()._mark_done()
            except queue.Empty:
                return


    @override
    def _shutdown(self) -> None:
        self._engine.stop()
        self._shutdown_event.set()
        self._drain_and_cancel()

        if self._worker_thread_ref:
            self._worker_thread_ref.join(timeout=1.0)