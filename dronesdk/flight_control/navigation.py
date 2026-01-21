"""
Navigation Utilities.

This module provides utility functions for navigation calculations.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dronesdk.models.location import (
        LocationGlobal,
        LocationGlobalRelative,
        LocationLocal,
    )

# Earth radius in meters
EARTH_RADIUS = 6378137.0


def get_distance_metres(
    location1: "LocationGlobal | LocationGlobalRelative",
    location2: "LocationGlobal | LocationGlobalRelative",
) -> float:
    """
    Calculate the ground distance between two locations.

    Uses the Haversine formula for accurate results over short distances.

    Args:
        location1: First location
        location2: Second location

    Returns:
        Distance in meters

    Example:
        >>> loc1 = LocationGlobal(-35.36, 149.17, 0)
        >>> loc2 = LocationGlobal(-35.37, 149.18, 0)
        >>> distance = get_distance_metres(loc1, loc2)
    """
    lat1 = location1.lat
    lon1 = location1.lon
    lat2 = location2.lat
    lon2 = location2.lon

    if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
        raise ValueError("Location coordinates cannot be None")

    # Convert to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    # Haversine formula
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS * c


def get_location_metres(
    original_location: "LocationGlobal | LocationGlobalRelative",
    dNorth: float,
    dEast: float,
) -> "LocationGlobal | LocationGlobalRelative":
    """
    Calculate a location offset by a given number of meters.

    This function is useful for calculating waypoints relative to
    the current position or home location.

    Args:
        original_location: Starting location
        dNorth: Offset north in meters (negative = south)
        dEast: Offset east in meters (negative = west)

    Returns:
        New location offset by the specified amounts

    Example:
        >>> home = LocationGlobal(-35.36, 149.17, 0)
        >>> waypoint = get_location_metres(home, 100, 50)  # 100m N, 50m E
    """
    from dronesdk.models.location import LocationGlobal, LocationGlobalRelative

    lat = original_location.lat
    lon = original_location.lon

    if lat is None or lon is None:
        raise ValueError("Location coordinates cannot be None")

    # Coordinate offsets in radians
    dLat = dNorth / EARTH_RADIUS
    dLon = dEast / (EARTH_RADIUS * math.cos(math.radians(lat)))

    # New position in decimal degrees
    new_lat = lat + math.degrees(dLat)
    new_lon = lon + math.degrees(dLon)

    # Return same type as input
    if isinstance(original_location, LocationGlobalRelative):
        return LocationGlobalRelative(new_lat, new_lon, original_location.alt)
    else:
        return LocationGlobal(new_lat, new_lon, original_location.alt)


def get_bearing(
    location1: "LocationGlobal | LocationGlobalRelative",
    location2: "LocationGlobal | LocationGlobalRelative",
) -> float:
    """
    Calculate the bearing from location1 to location2.

    Args:
        location1: Starting location
        location2: Target location

    Returns:
        Bearing in degrees (0-360, 0 = North, 90 = East)

    Example:
        >>> home = LocationGlobal(-35.36, 149.17, 0)
        >>> target = LocationGlobal(-35.37, 149.18, 0)
        >>> bearing = get_bearing(home, target)
    """
    lat1 = location1.lat
    lon1 = location1.lon
    lat2 = location2.lat
    lon2 = location2.lon

    if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
        raise ValueError("Location coordinates cannot be None")

    # Convert to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)

    # Calculate bearing
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
        lat2_rad
    ) * math.cos(dlon)

    bearing = math.degrees(math.atan2(x, y))

    # Normalize to 0-360
    return (bearing + 360) % 360


def get_distance_3d(
    location1: "LocationGlobal | LocationGlobalRelative",
    location2: "LocationGlobal | LocationGlobalRelative",
) -> float:
    """
    Calculate the 3D distance between two locations including altitude.

    Args:
        location1: First location
        location2: Second location

    Returns:
        3D distance in meters
    """
    ground_distance = get_distance_metres(location1, location2)

    alt1 = location1.alt or 0
    alt2 = location2.alt or 0
    dalt = alt2 - alt1

    return math.sqrt(ground_distance**2 + dalt**2)


def is_within_radius(
    location: "LocationGlobal | LocationGlobalRelative",
    target: "LocationGlobal | LocationGlobalRelative",
    radius: float,
) -> bool:
    """
    Check if a location is within a radius of a target.

    Args:
        location: Location to check
        target: Center of the radius
        radius: Radius in meters

    Returns:
        True if location is within radius of target
    """
    distance = get_distance_metres(location, target)
    return distance <= radius


def get_ground_course(vx: float, vy: float) -> float:
    """
    Calculate ground course from velocity components.

    Args:
        vx: North velocity (m/s)
        vy: East velocity (m/s)

    Returns:
        Course in degrees (0-360, 0 = North, 90 = East)
    """
    course = math.degrees(math.atan2(vy, vx))
    return (course + 360) % 360


def get_ground_speed(vx: float, vy: float) -> float:
    """
    Calculate ground speed from velocity components.

    Args:
        vx: North velocity (m/s)
        vy: East velocity (m/s)

    Returns:
        Ground speed in m/s
    """
    return math.sqrt(vx**2 + vy**2)
