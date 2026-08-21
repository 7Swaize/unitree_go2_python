import logging

DEFAULT_LOG_LEVEL = logging.INFO
DEFAULT_LOG_FORMAT = "%(message)s"
# DEFAULT_LOG_FORMAT = "%(levelname)s: %(message)s"


def _create_default_handler() -> logging.Handler:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(DEFAULT_LOG_FORMAT))
    return handler


def configure_logging(level: int = DEFAULT_LOG_LEVEL) -> None:
    root_logger = logging.getLogger()
    if not root_logger.handlers:
        root_logger.addHandler(_create_default_handler())

    root_logger.setLevel(level)


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)