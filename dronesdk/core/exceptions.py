"""
Exception Classes.

This module defines the base exception classes used throughout dronesdk.
"""

from __future__ import annotations


class APIException(Exception):
    """
    Base class for dronesdk related exceptions.

    :param message: Message string describing the exception
    """

    pass


class TimeoutError(APIException):
    """
    Raised by operations that have timeouts.

    This exception is raised when a dronesdk operation exceeds its
    configured timeout period, such as:
    - Waiting for vehicle connection
    - Waiting for attribute readiness
    - Parameter operations
    - Mission upload/download
    """

    pass
