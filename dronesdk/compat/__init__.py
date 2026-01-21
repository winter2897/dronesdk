"""
Compatibility Layer.

This module provides backwards compatibility for existing dronesdk code.
"""

from dronesdk.compat.facade import Vehicle
from dronesdk.compat.connect import connect

__all__ = ["Vehicle", "connect"]
