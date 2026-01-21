"""
Location Data Models.

This module provides immutable dataclasses for representing vehicle location
in different coordinate frames.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass
class LocationGlobal:
    """
    A global location object.

    The latitude and longitude are relative to the WGS84 coordinate system.
    The altitude is relative to mean sea-level (MSL).

    Example:
        >>> loc = LocationGlobal(-34.364114, 149.166022, 30)
        >>> print(loc)
        LocationGlobal:lat=-34.364114,lon=149.166022,alt=30

    Attributes:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        alt: Altitude in meters relative to mean sea-level (MSL).
    """

    lat: float | None
    lon: float | None
    alt: float | None = None
    # Backwards compatibility attributes
    local_frame: Any = None
    global_frame: Any = None

    def __str__(self) -> str:
        return f"LocationGlobal:lat={self.lat},lon={self.lon},alt={self.alt}"

    @classmethod
    def from_mavlink(
        cls,
        lat_e7: int,
        lon_e7: int,
        alt_mm: int,
    ) -> "LocationGlobal":
        """
        Create from MAVLink message values.

        Args:
            lat_e7: Latitude in 1E7 degrees
            lon_e7: Longitude in 1E7 degrees
            alt_mm: Altitude in millimeters

        Returns:
            LocationGlobal instance
        """
        return cls(
            lat=lat_e7 / 1.0e7,
            lon=lon_e7 / 1.0e7,
            alt=alt_mm / 1000.0,
        )


@dataclass
class LocationGlobalRelative:
    """
    A global location object, with altitude relative to home location.

    The latitude and longitude are relative to the WGS84 coordinate system.
    The altitude is relative to the home position.

    Example:
        >>> loc = LocationGlobalRelative(-34.364114, 149.166022, 30)
        >>> print(loc)
        LocationGlobalRelative:lat=-34.364114,lon=149.166022,alt=30

    Attributes:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        alt: Altitude in meters relative to home location.
    """

    lat: float | None
    lon: float | None
    alt: float | None = None
    # Backwards compatibility attributes
    local_frame: Any = None
    global_frame: Any = None

    def __str__(self) -> str:
        return f"LocationGlobalRelative:lat={self.lat},lon={self.lon},alt={self.alt}"

    @classmethod
    def from_mavlink(
        cls,
        lat_e7: int,
        lon_e7: int,
        relative_alt_mm: int,
    ) -> "LocationGlobalRelative":
        """
        Create from MAVLink message values.

        Args:
            lat_e7: Latitude in 1E7 degrees
            lon_e7: Longitude in 1E7 degrees
            relative_alt_mm: Relative altitude in millimeters

        Returns:
            LocationGlobalRelative instance
        """
        return cls(
            lat=lat_e7 / 1.0e7,
            lon=lon_e7 / 1.0e7,
            alt=relative_alt_mm / 1000.0,
        )


@dataclass
class LocationLocal:
    """
    A local location object in NED (North-East-Down) frame.

    The coordinates are relative to the EKF origin, which is most likely
    the location where the vehicle was turned on.

    Attributes:
        north: Position north of the EKF origin in meters.
        east: Position east of the EKF origin in meters.
        down: Position down from the EKF origin in meters
              (i.e., negative altitude).
    """

    north: float | None
    east: float | None
    down: float | None

    def __str__(self) -> str:
        return f"LocationLocal:north={self.north},east={self.east},down={self.down}"

    def distance_home(self) -> float | None:
        """
        Calculate distance from home (EKF origin).

        Returns:
            3D distance if `down` is known, otherwise 2D distance.
            Returns None if north and east are not available.
        """
        if self.north is not None and self.east is not None:
            if self.down is not None:
                return math.sqrt(self.north**2 + self.east**2 + self.down**2)
            else:
                return math.sqrt(self.north**2 + self.east**2)
        return None

    @classmethod
    def from_mavlink(
        cls,
        x: float,
        y: float,
        z: float,
    ) -> "LocationLocal":
        """
        Create from MAVLink LOCAL_POSITION_NED message values.

        Args:
            x: North position in meters
            y: East position in meters
            z: Down position in meters

        Returns:
            LocationLocal instance
        """
        return cls(north=x, east=y, down=z)
