"""
Version and Capabilities Data Models.

This module provides dataclasses for autopilot version information
and capability flags.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pymavlink import mavutil


@dataclass
class Version:
    """
    Autopilot version and type information.

    An object of this type is returned by Vehicle.version.

    The version number can be read in different formats. To get it in
    a human-readable format, just print the object (e.g., "APM:Copter-3.3.2-rc4").

    Attributes:
        raw_version: Raw 32-bit version number from autopilot.
        autopilot_type: MAV_AUTOPILOT type (e.g., MAV_AUTOPILOT_ARDUPILOTMEGA).
        vehicle_type: MAV_TYPE vehicle type (e.g., MAV_TYPE_QUADROTOR).
        major: Major version number.
        minor: Minor version number.
        patch: Patch version number.
        release: Release type (see FIRMWARE_VERSION_TYPE enum).
    """

    raw_version: int | None
    autopilot_type: int
    vehicle_type: int
    major: int | None = field(init=False)
    minor: int | None = field(init=False)
    patch: int | None = field(init=False)
    release: int | None = field(init=False)

    def __post_init__(self) -> None:
        """Parse version components from raw_version."""
        if self.raw_version is None:
            self.major = None
            self.minor = None
            self.patch = None
            self.release = None
        else:
            self.major = (self.raw_version >> 24) & 0xFF
            self.minor = (self.raw_version >> 16) & 0xFF
            self.patch = (self.raw_version >> 8) & 0xFF
            self.release = self.raw_version & 0xFF

    def is_stable(self) -> bool:
        """
        Check if current firmware is a stable release.

        Returns:
            True if stable release, False otherwise.
        """
        return self.release == 255

    def release_version(self) -> int | None:
        """
        Get the version within the release type.

        Returns:
            Release version number (e.g., 23 for "Copter-3.3rc23").
            Returns None if release is unknown, 0 if stable.
        """
        if self.release is None:
            return None
        if self.release == 255:
            return 0
        return self.release % 64

    def release_type(self) -> str | None:
        """
        Get text describing the release type.

        Returns:
            Release type string ("dev", "alpha", "beta", "rc", or "stable").
            Returns None if release is unknown.
        """
        if self.release is None:
            return None
        if self.release == 255:
            return "stable"
        types = ["dev", "alpha", "beta", "rc"]
        return types[self.release >> 6]

    def __str__(self) -> str:
        """Return human-readable version string."""
        # Import here to avoid circular imports
        from pymavlink import mavutil

        prefix = ""

        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            prefix += "APM:"
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            prefix += "PX4"
        else:
            prefix += "UnknownAutoPilot"

        if self.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
            prefix += "Copter-"
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            prefix += "Plane-"
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            prefix += "Rover-"
        else:
            prefix += f"UnknownVehicleType{self.vehicle_type}-"

        if self.release_type() is None:
            release_type = "UnknownReleaseType"
        elif self.is_stable():
            release_type = ""
        else:
            # e.g., "-rc23"
            release_type = f"-{self.release_type()}{self.release_version()}"

        return f"{prefix}{self.major}.{self.minor}.{self.patch}{release_type}"


@dataclass(frozen=True)
class Capabilities:
    """
    Autopilot capabilities (supported message types and functionality).

    An object of this type is returned by Vehicle.capabilities.

    See MAV_PROTOCOL_CAPABILITY enum for details.

    Attributes:
        mission_float: Supports MISSION float message type.
        param_float: Supports PARAM float message type.
        mission_int: Supports MISSION_INT scaled integer message type.
        command_int: Supports COMMAND_INT scaled integer message type.
        param_union: Supports PARAM_UNION message type.
        ftp: Supports FTP for file transfers.
        set_attitude_target: Supports commanding attitude offboard.
        set_attitude_target_local_ned: Supports position/velocity in local NED.
        set_altitude_target_global_int: Supports position/velocity in global int.
        terrain: Supports terrain protocol.
        set_actuator_target: Supports direct actuator control.
        flight_termination: Supports flight termination command.
        compass_calibration: Supports onboard compass calibration.
    """

    mission_float: bool
    param_float: bool
    mission_int: bool
    command_int: bool
    param_union: bool
    ftp: bool
    set_attitude_target: bool
    set_attitude_target_local_ned: bool
    set_altitude_target_global_int: bool
    terrain: bool
    set_actuator_target: bool
    flight_termination: bool
    compass_calibration: bool

    @classmethod
    def from_mavlink(cls, capabilities: int) -> "Capabilities":
        """
        Create from MAVLink AUTOPILOT_VERSION capabilities bitmask.

        Args:
            capabilities: 64-bit capabilities bitmask

        Returns:
            Capabilities instance
        """
        return cls(
            mission_float=bool((capabilities >> 0) & 1),
            param_float=bool((capabilities >> 1) & 1),
            mission_int=bool((capabilities >> 2) & 1),
            command_int=bool((capabilities >> 3) & 1),
            param_union=bool((capabilities >> 4) & 1),
            ftp=bool((capabilities >> 5) & 1),
            set_attitude_target=bool((capabilities >> 6) & 1),
            set_attitude_target_local_ned=bool((capabilities >> 7) & 1),
            set_altitude_target_global_int=bool((capabilities >> 8) & 1),
            terrain=bool((capabilities >> 9) & 1),
            set_actuator_target=bool((capabilities >> 10) & 1),
            flight_termination=bool((capabilities >> 11) & 1),
            compass_calibration=bool((capabilities >> 12) & 1),
        )
