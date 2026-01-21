"""
Logging Handlers.

This module provides custom logging handlers for dronesdk.
"""

from __future__ import annotations

import logging
import sys


class ErrprinterHandler(logging.Handler):
    """
    A logging handler that prints to stderr.

    This handler is used for autopilot status messages and
    error reporting.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        """Emit a log record to stderr."""
        try:
            msg = self.format(record)
            print(msg, file=sys.stderr)
        except Exception:
            self.handleError(record)


def setup_dronesdk_logging(
    level: int = logging.INFO,
    autopilot_level: int = logging.WARNING,
) -> None:
    """
    Set up dronesdk logging.

    Args:
        level: Log level for dronesdk logger
        autopilot_level: Log level for autopilot logger
    """
    # dronesdk logger
    dronesdk_logger = logging.getLogger("dronesdk")
    dronesdk_logger.setLevel(level)

    # Autopilot logger
    autopilot_logger = logging.getLogger("autopilot")
    autopilot_logger.setLevel(autopilot_level)
    autopilot_logger.addHandler(ErrprinterHandler())


def get_dronesdk_logger() -> logging.Logger:
    """Get the main dronesdk logger."""
    return logging.getLogger("dronesdk")


def get_autopilot_logger() -> logging.Logger:
    """Get the autopilot message logger."""
    return logging.getLogger("autopilot")
