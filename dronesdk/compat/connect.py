"""
Connect Function.

This module provides the connect() function for establishing
vehicle connections.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

import monotonic

from dronesdk.compat.facade import Vehicle
from dronesdk.core.exceptions import APIException, TimeoutError
from dronesdk.datalink.connection import MAVConnection

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


def connect(
    ip: str,
    wait_ready: bool | list[str] | None = None,
    timeout: float = 30,
    still_waiting_callback: Any = None,
    still_waiting_interval: float = 1,
    status_printer: Any = None,
    baud: int = 115200,
    heartbeat_timeout: float = 30,
    source_system: int = 255,
    source_component: int = 0,
    rate: int = 4,
    use_native: bool = False,
) -> Vehicle:
    """
    Connect to a vehicle.

    This function establishes a MAVLink connection to a vehicle
    and returns a Vehicle object for interaction.

    Args:
        ip: Connection string. Examples:
            - "127.0.0.1:14550" (UDP)
            - "udpin:0.0.0.0:14550" (UDP input)
            - "tcp:127.0.0.1:5760" (TCP)
            - "/dev/ttyUSB0" (Serial)
            - "com14" (Serial on Windows)
        wait_ready: If True, wait for default attributes.
                   If list, wait for specific attributes.
                   If None/False, return immediately.
        timeout: Connection timeout in seconds
        still_waiting_callback: Callback while waiting
        still_waiting_interval: Interval between callbacks
        status_printer: Deprecated, use logging instead
        baud: Baud rate for serial connections
        heartbeat_timeout: Timeout waiting for heartbeat
        source_system: MAVLink source system ID
        source_component: MAVLink source component ID
        rate: Requested telemetry rate (Hz)
        use_native: Use native MAVLink implementation

    Returns:
        Vehicle object

    Raises:
        APIException: If connection fails
        TimeoutError: If connection times out

    Example:
        >>> from dronesdk import connect
        >>> vehicle = connect('127.0.0.1:14550', wait_ready=True)
        >>> print(vehicle.mode)
    """
    # Create connection
    handler = MAVConnection(
        ip,
        baud=baud,
        target_system=0,
        source_system=source_system,
        source_component=source_component,
        use_native=use_native,
    )

    # Start the connection threads
    handler.start()

    # Add heartbeat listener to detect target system
    from pymavlink import mavutil

    def _heartbeat_listener(_: Any, msg: Any) -> None:
        """Update target system when heartbeat is received."""
        if msg.get_type() == "HEARTBEAT":
            # Ignore non-vehicle heartbeats
            if msg.type not in (
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_TYPE_GIMBAL,
                mavutil.mavlink.MAV_TYPE_ADSB,
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            ):
                if handler.target_system == 0:
                    handler.target_system = msg.get_srcSystem()
                    handler.master.target_system = msg.get_srcSystem()
                    handler.master.target_component = msg.get_srcComponent()

    handler.forward_message(_heartbeat_listener)

    # Wait for heartbeat
    logger.info("Connecting to vehicle on: %s", ip)
    start = monotonic.monotonic()
    last_callback = start

    while True:
        # Check for heartbeat
        if handler.target_system != 0:
            break

        # Check timeout
        if (monotonic.monotonic() - start) > heartbeat_timeout:
            handler.close()
            raise TimeoutError(f"No heartbeat received in {heartbeat_timeout}s")

        # Still waiting callback
        if still_waiting_callback:
            now = monotonic.monotonic()
            if (now - last_callback) >= still_waiting_interval:
                still_waiting_callback(handler)
                last_callback = now

        time.sleep(0.1)

    logger.info(
        "Connected to vehicle (system %d, component %d)",
        handler.target_system,
        handler.master.target_component,
    )

    # Create vehicle
    vehicle = Vehicle(handler)

    # Request data streams
    _request_data_streams(handler, rate)

    # Request autopilot version/capabilities
    _request_autopilot_capabilities(handler)

    # Wait for attributes if requested
    if wait_ready:
        if wait_ready is True:
            # Default attributes
            attrs = ["parameters", "gps_0", "armed", "mode", "attitude"]
        else:
            attrs = list(wait_ready)

        try:
            vehicle.wait_ready(*attrs, timeout=timeout)
        except TimeoutError:
            handler.close()
            raise

    return vehicle


def _request_autopilot_capabilities(handler: MAVConnection) -> None:
    """Request autopilot version and capabilities."""
    from pymavlink import mavutil

    handler.master.mav.command_long_send(
        handler.target_system,
        handler.master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0,  # confirmation
        1,  # request (1 = request)
        0,
        0,
        0,
        0,
        0,
        0,
    )


def _request_data_streams(handler: MAVConnection, rate: int) -> None:
    """Request data streams from the vehicle."""
    # Request all data streams
    handler.master.mav.request_data_stream_send(
        handler.target_system,
        handler.master.target_component,
        0,  # MAV_DATA_STREAM_ALL
        rate,
        1,  # start
    )

    # Request specific streams at higher rates for important data
    streams = [
        (1, rate * 2),  # RAW_SENSORS
        (2, rate),  # EXTENDED_STATUS
        (3, rate),  # RC_CHANNELS
        (4, rate),  # RAW_CONTROLLER
        (6, rate * 2),  # POSITION
        (10, rate),  # EXTRA1 (attitude)
        (11, rate),  # EXTRA2 (VFR_HUD)
        (12, rate),  # EXTRA3
    ]

    for stream_id, stream_rate in streams:
        handler.master.mav.request_data_stream_send(
            handler.target_system,
            handler.master.target_component,
            stream_id,
            stream_rate,
            1,
        )
