"""
Health Monitor.

This module monitors vehicle health including EKF status,
system status, version, and capabilities.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

from dronesdk.models.status import SystemStatus, VehicleMode
from dronesdk.models.version import Capabilities, Version

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent

logger = logging.getLogger(__name__)


@dataclass
class EKFStatus:
    """
    EKF (Extended Kalman Filter) status.

    Attributes:
        velocity_variance: Velocity variance
        pos_horiz_variance: Horizontal position variance
        pos_vert_variance: Vertical position variance
        compass_variance: Compass variance
        terrain_alt_variance: Terrain altitude variance
        flags: EKF status flags
    """

    velocity_variance: float
    pos_horiz_variance: float
    pos_vert_variance: float
    compass_variance: float
    terrain_alt_variance: float
    flags: int

    @property
    def is_ok(self) -> bool:
        """Check if EKF is healthy (flags indicate OK status)."""
        # Check EKF_ATTITUDE, EKF_VELOCITY_HORIZ, EKF_VELOCITY_VERT
        required_flags = 0x01 | 0x02 | 0x04
        return (self.flags & required_flags) == required_flags


class HealthMonitor:
    """
    Monitors vehicle health status.

    This module tracks:
    - System status from HEARTBEAT
    - Armed state
    - Vehicle mode
    - EKF status
    - Autopilot version and capabilities

    Args:
        on_mode_change: Optional callback when mode changes
        on_armed_change: Optional callback when armed state changes
    """

    __slots__ = (
        "_system_status",
        "_mode",
        "_armed",
        "_ekf_status",
        "_version",
        "_capabilities",
        "_autopilot_type",
        "_vehicle_type",
        "_on_mode_change",
        "_on_armed_change",
        "_unsubscribe_fns",
    )

    def __init__(
        self,
        on_mode_change: Callable[[VehicleMode], None] | None = None,
        on_armed_change: Callable[[bool], None] | None = None,
    ) -> None:
        self._system_status: SystemStatus | None = None
        self._mode: VehicleMode | None = None
        self._armed: bool | None = None
        self._ekf_status: EKFStatus | None = None
        self._version: Version | None = None
        self._capabilities: Capabilities | None = None
        self._autopilot_type: int | None = None
        self._vehicle_type: int | None = None
        self._on_mode_change = on_mode_change
        self._on_armed_change = on_armed_change
        self._unsubscribe_fns: list[Callable[[], None]] = []

    @property
    def name(self) -> str:
        """Module name."""
        return "health"

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive health-related messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        subscriptions = [
            ("HEARTBEAT", self._handle_heartbeat),
            ("EKF_STATUS_REPORT", self._handle_ekf_status),
            ("AUTOPILOT_VERSION", self._handle_autopilot_version),
        ]

        for msg_type, handler in subscriptions:
            unsub = event_bus.subscribe_message(msg_type, handler)
            self._unsubscribe_fns.append(unsub)

        logger.debug("HealthMonitor attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        for unsub in self._unsubscribe_fns:
            unsub()
        self._unsubscribe_fns.clear()
        logger.debug("HealthMonitor detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_heartbeat(self, event: "MAVLinkMessageEvent") -> None:
        """Handle HEARTBEAT message."""
        from pymavlink import mavutil

        msg = event.message

        # Ignore non-vehicle heartbeats
        if msg.type in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_GIMBAL,
            mavutil.mavlink.MAV_TYPE_ADSB,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        ):
            return

        # Store autopilot and vehicle type
        self._autopilot_type = msg.autopilot
        self._vehicle_type = msg.type

        # Update system status
        state_map = {
            mavutil.mavlink.MAV_STATE_UNINIT: "UNINIT",
            mavutil.mavlink.MAV_STATE_BOOT: "BOOT",
            mavutil.mavlink.MAV_STATE_CALIBRATING: "CALIBRATING",
            mavutil.mavlink.MAV_STATE_STANDBY: "STANDBY",
            mavutil.mavlink.MAV_STATE_ACTIVE: "ACTIVE",
            mavutil.mavlink.MAV_STATE_CRITICAL: "CRITICAL",
            mavutil.mavlink.MAV_STATE_EMERGENCY: "EMERGENCY",
            mavutil.mavlink.MAV_STATE_POWEROFF: "POWEROFF",
        }
        state_name = state_map.get(msg.system_status, "UNKNOWN")
        self._system_status = SystemStatus.from_mavlink(state_name)

        # Update armed state
        new_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if self._armed != new_armed:
            self._armed = new_armed
            if self._on_armed_change:
                try:
                    self._on_armed_change(new_armed)
                except Exception:
                    logger.exception("Error in armed change callback")

        # Update mode
        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            mode_name = mavutil.interpret_px4_mode(
                msg.base_mode,
                msg.custom_mode,
            )
        else:
            mode_mapping = mavutil.mode_mapping_bynumber(self._vehicle_type)
            if mode_mapping:
                mode_name = mode_mapping.get(msg.custom_mode, "UNKNOWN")
            else:
                mode_name = "UNKNOWN"

        if self._mode is None or self._mode.name != mode_name:
            self._mode = VehicleMode.from_mavlink(mode_name)
            if self._on_mode_change:
                try:
                    self._on_mode_change(self._mode)
                except Exception:
                    logger.exception("Error in mode change callback")

    def _handle_ekf_status(self, event: "MAVLinkMessageEvent") -> None:
        """Handle EKF_STATUS_REPORT message."""
        msg = event.message
        self._ekf_status = EKFStatus(
            velocity_variance=msg.velocity_variance,
            pos_horiz_variance=msg.pos_horiz_variance,
            pos_vert_variance=msg.pos_vert_variance,
            compass_variance=msg.compass_variance,
            terrain_alt_variance=msg.terrain_alt_variance,
            flags=msg.flags,
        )

    def _handle_autopilot_version(self, event: "MAVLinkMessageEvent") -> None:
        """Handle AUTOPILOT_VERSION message."""
        msg = event.message

        self._version = Version(
            raw_version=msg.flight_sw_version,
            autopilot_type=self._autopilot_type or 0,
            vehicle_type=self._vehicle_type or 0,
        )

        self._capabilities = Capabilities.from_mavlink(msg.capabilities)

    @property
    def system_status(self) -> SystemStatus | None:
        """Get current system status."""
        return self._system_status

    @property
    def mode(self) -> VehicleMode | None:
        """Get current vehicle mode."""
        return self._mode

    @property
    def armed(self) -> bool | None:
        """Get armed state."""
        return self._armed

    @property
    def ekf_status(self) -> EKFStatus | None:
        """Get current EKF status."""
        return self._ekf_status

    @property
    def ekf_ok(self) -> bool:
        """Check if EKF is healthy."""
        return self._ekf_status is not None and self._ekf_status.is_ok

    @property
    def version(self) -> Version | None:
        """Get autopilot version."""
        return self._version

    @property
    def capabilities(self) -> Capabilities | None:
        """Get autopilot capabilities."""
        return self._capabilities

    @property
    def autopilot_type(self) -> int | None:
        """Get autopilot type (MAV_AUTOPILOT enum)."""
        return self._autopilot_type

    @property
    def vehicle_type(self) -> int | None:
        """Get vehicle type (MAV_TYPE enum)."""
        return self._vehicle_type

    @property
    def is_armable(self) -> bool:
        """
        Check if vehicle is armable.

        Returns True if EKF is OK and system is ready.
        """
        if not self.ekf_ok:
            return False
        if self._system_status is None:
            return False
        return self._system_status.is_ready
