"""
Status Data Models.

This module provides dataclasses for vehicle mode and system status.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class VehicleMode:
    """
    Vehicle flight mode.

    The flight mode determines the behaviour of the vehicle and what
    commands it can obey.

    Recommended modes for dronesdk-Python apps:
    - Copter: Use AUTO for waypoint missions, GUIDED otherwise.
    - Plane/Rover: Use AUTO in all cases.
    - RETURN_TO_LAUNCH works on all platforms.

    Example:
        >>> vehicle.mode = VehicleMode("GUIDED")
        >>> if vehicle.mode == "GUIDED":
        ...     print("In guided mode")

    Attributes:
        name: The mode name as a string (e.g., "GUIDED", "AUTO", "LOITER").
    """

    name: str

    def __str__(self) -> str:
        return f"VehicleMode:{self.name}"

    def __eq__(self, other: Any) -> bool:
        """
        Compare mode to a string or another VehicleMode.

        This allows comparisons like: vehicle.mode == "GUIDED"
        """
        if isinstance(other, VehicleMode):
            return self.name == other.name
        return self.name == other

    def __ne__(self, other: Any) -> bool:
        return not self.__eq__(other)

    def __hash__(self) -> int:
        return hash(self.name)

    @classmethod
    def from_mavlink(cls, mode_name: str) -> "VehicleMode":
        """
        Create from MAVLink mode string.

        Args:
            mode_name: Mode name string

        Returns:
            VehicleMode instance
        """
        return cls(name=mode_name)


@dataclass
class SystemStatus:
    """
    System status information.

    An object of this type is returned by Vehicle.system_status.

    The state values correspond to the MAV_STATE enum:
    - UNINIT: Uninitialized
    - BOOT: System booting
    - CALIBRATING: Calibrating sensors
    - STANDBY: Ready, motors not armed
    - ACTIVE: Active, motors armed
    - CRITICAL: Critical condition
    - EMERGENCY: Emergency condition
    - POWEROFF: Powering off

    Attributes:
        state: The system state as a string.
    """

    state: str

    def __str__(self) -> str:
        return f"SystemStatus:{self.state}"

    def __eq__(self, other: Any) -> bool:
        """
        Compare status to a string or another SystemStatus.

        This allows comparisons like: vehicle.system_status == "STANDBY"
        """
        if isinstance(other, SystemStatus):
            return self.state == other.state
        return self.state == other

    def __ne__(self, other: Any) -> bool:
        return not self.__eq__(other)

    def __hash__(self) -> int:
        return hash(self.state)

    @classmethod
    def from_mavlink(cls, state_name: str) -> "SystemStatus":
        """
        Create from MAVLink state string.

        Args:
            state_name: State name string

        Returns:
            SystemStatus instance
        """
        return cls(state=state_name)

    @property
    def is_ready(self) -> bool:
        """Check if system is ready (STANDBY or ACTIVE)."""
        return self.state in ("STANDBY", "ACTIVE")

    @property
    def is_armed(self) -> bool:
        """Check if system is armed (ACTIVE state)."""
        return self.state == "ACTIVE"

    @property
    def is_critical(self) -> bool:
        """Check if system is in critical or emergency state."""
        return self.state in ("CRITICAL", "EMERGENCY")
