"""
Battery Data Model.

This module provides the Battery dataclass for representing
system battery information.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Battery:
    """
    System battery information.

    An object of this type is returned by Vehicle.battery.

    Attributes:
        voltage: Battery voltage in volts.
        current: Battery current in amperes. None if the autopilot
                does not support current measurement.
        level: Remaining battery energy as percentage (0-100).
              None if the autopilot cannot estimate remaining battery.
    """

    voltage: float | None
    current: float | None
    level: int | None

    def __str__(self) -> str:
        return (
            f"Battery:voltage={self.voltage},"
            f"current={self.current},"
            f"level={self.level}"
        )

    @classmethod
    def from_mavlink(
        cls,
        voltage_mv: int,
        current_ca: int,
        battery_remaining: int,
    ) -> "Battery":
        """
        Create from MAVLink SYS_STATUS message values.

        Args:
            voltage_mv: Battery voltage in millivolts
            current_ca: Battery current in 10 * milliamperes
            battery_remaining: Remaining battery energy percentage (-1 if unknown)

        Returns:
            Battery instance
        """
        voltage = voltage_mv / 1000.0

        if current_ca == -1:
            current = None
        else:
            current = current_ca / 100.0

        if battery_remaining == -1:
            level = None
        else:
            level = battery_remaining

        return cls(voltage=voltage, current=current, level=level)

    @property
    def is_low(self) -> bool | None:
        """
        Check if battery level is low (below 20%).

        Returns:
            True if level is below 20%, False otherwise.
            None if level is unknown.
        """
        if self.level is None:
            return None
        return self.level < 20

    @property
    def is_critical(self) -> bool | None:
        """
        Check if battery level is critical (below 10%).

        Returns:
            True if level is below 10%, False otherwise.
            None if level is unknown.
        """
        if self.level is None:
            return None
        return self.level < 10
