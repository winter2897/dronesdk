"""
Sensor Data Models.

This module provides dataclasses for various sensor readings
including GPS, rangefinder, and wind.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GPSInfo:
    """
    Standard information about GPS.

    If there is no GPS lock the parameters are set to None.

    Attributes:
        eph: GPS horizontal dilution of position (HDOP).
        epv: GPS vertical dilution of position (VDOP).
        fix_type: 0-1: no fix, 2: 2D fix, 3: 3D fix, 4+: RTK
        satellites_visible: Number of satellites visible.
    """

    eph: int | None
    epv: int | None
    fix_type: int | None
    satellites_visible: int | None

    def __str__(self) -> str:
        return f"GPSInfo:fix={self.fix_type},num_sat={self.satellites_visible}"

    @classmethod
    def from_mavlink(
        cls,
        eph: int,
        epv: int,
        fix_type: int,
        satellites_visible: int,
    ) -> "GPSInfo":
        """
        Create from MAVLink GPS_RAW_INT message values.

        Args:
            eph: GPS HDOP horizontal dilution of position
            epv: GPS VDOP vertical dilution of position
            fix_type: GPS fix type
            satellites_visible: Number of satellites visible

        Returns:
            GPSInfo instance
        """
        return cls(
            eph=eph,
            epv=epv,
            fix_type=fix_type,
            satellites_visible=satellites_visible,
        )

    @property
    def has_fix(self) -> bool:
        """Check if GPS has a valid fix (2D or better)."""
        return self.fix_type is not None and self.fix_type >= 2

    @property
    def has_3d_fix(self) -> bool:
        """Check if GPS has a 3D fix or better."""
        return self.fix_type is not None and self.fix_type >= 3

    @property
    def fix_type_str(self) -> str:
        """Return human-readable fix type string."""
        if self.fix_type is None:
            return "Unknown"
        fix_types = {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed",
        }
        return fix_types.get(self.fix_type, f"Type {self.fix_type}")


@dataclass
class Rangefinder:
    """
    Rangefinder readings.

    An object of this type is returned by Vehicle.rangefinder.

    Attributes:
        distance: Distance in meters. None if no rangefinder.
        voltage: Voltage in volts. None if no rangefinder.
    """

    distance: float | None
    voltage: float | None

    def __str__(self) -> str:
        return f"Rangefinder: distance={self.distance}, voltage={self.voltage}"

    @classmethod
    def from_mavlink(
        cls,
        distance: float,
        voltage: float,
    ) -> "Rangefinder":
        """
        Create from MAVLink RANGEFINDER message values.

        Args:
            distance: Distance in meters
            voltage: Raw voltage in volts

        Returns:
            Rangefinder instance
        """
        return cls(distance=distance, voltage=voltage)

    @property
    def is_valid(self) -> bool:
        """Check if rangefinder reading is valid (distance not None)."""
        return self.distance is not None


@dataclass
class Wind:
    """
    Wind information.

    An object of this type is returned by Vehicle.wind.

    Attributes:
        wind_direction: Wind direction in degrees (0=North, 90=East).
        wind_speed: Wind speed in m/s.
        wind_speed_z: Vertical wind speed in m/s.
    """

    wind_direction: float | None
    wind_speed: float | None
    wind_speed_z: float | None

    def __str__(self) -> str:
        return (
            f"Wind: wind direction: {self.wind_direction}, "
            f"wind speed: {self.wind_speed}, "
            f"wind speed z: {self.wind_speed_z}"
        )

    @classmethod
    def from_mavlink(
        cls,
        direction: float,
        speed: float,
        speed_z: float,
    ) -> "Wind":
        """
        Create from MAVLink WIND message values.

        Args:
            direction: Wind direction in degrees
            speed: Wind speed in m/s
            speed_z: Vertical wind speed in m/s

        Returns:
            Wind instance
        """
        return cls(
            wind_direction=direction,
            wind_speed=speed,
            wind_speed_z=speed_z,
        )
