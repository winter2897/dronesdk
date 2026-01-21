"""
Command Sequence.

This module provides the CommandSequence class for managing
mission waypoints.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, Callable, Iterator

import monotonic

from dronesdk.core.exceptions import APIException
from dronesdk.models.command import Command

if TYPE_CHECKING:
    from dronesdk.core.events import EventBus, MAVLinkMessageEvent
    from dronesdk.datalink.connection import MAVConnection

logger = logging.getLogger(__name__)


class CommandSequence:
    """
    A sequence of mission commands/waypoints.

    This class manages the mission command list, supporting:
    - Download from vehicle
    - Upload to vehicle
    - List-like access to commands
    - Clearing the mission

    Example:
        >>> cmds = vehicle.commands
        >>> cmds.download()
        >>> cmds.wait_ready()
        >>> for cmd in cmds:
        ...     print(cmd)
        >>> cmds.clear()
        >>> cmds.add(Command(...))
        >>> cmds.upload()
    """

    __slots__ = (
        "_commands",
        "_connection",
        "_next_command",
        "_count",
        "_downloading",
        "_uploading",
        "_download_complete",
        "_upload_complete",
        "_on_download_complete",
        "_on_upload_complete",
        "_unsubscribe_fns",
    )

    def __init__(
        self,
        on_download_complete: Callable[[], None] | None = None,
        on_upload_complete: Callable[[], None] | None = None,
    ) -> None:
        self._commands: list[Command] = []
        self._connection: MAVConnection | None = None
        self._next_command: int = 0
        self._count: int = -1
        self._downloading = False
        self._uploading = False
        self._download_complete = False
        self._upload_complete = False
        self._on_download_complete = on_download_complete
        self._on_upload_complete = on_upload_complete
        self._unsubscribe_fns: list[Callable[[], None]] = []

    @property
    def name(self) -> str:
        """Module name."""
        return "commands"

    def set_connection(self, connection: "MAVConnection") -> None:
        """Set the MAVLink connection."""
        self._connection = connection

    def attach(self, event_bus: "EventBus") -> None:
        """
        Attach to an event bus to receive mission messages.

        Args:
            event_bus: The event bus to subscribe to
        """
        subscriptions = [
            ("MISSION_COUNT", self._handle_mission_count),
            ("MISSION_ITEM", self._handle_mission_item),
            ("MISSION_ACK", self._handle_mission_ack),
            ("MISSION_CURRENT", self._handle_mission_current),
        ]

        for msg_type, handler in subscriptions:
            unsub = event_bus.subscribe_message(msg_type, handler)
            self._unsubscribe_fns.append(unsub)

        logger.debug("CommandSequence attached to event bus")

    def detach(self) -> None:
        """Detach from the event bus."""
        for unsub in self._unsubscribe_fns:
            unsub()
        self._unsubscribe_fns.clear()
        logger.debug("CommandSequence detached from event bus")

    def initialize(self) -> None:
        """Initialize the module."""
        pass

    def _handle_mission_count(self, event: "MAVLinkMessageEvent") -> None:
        """Handle MISSION_COUNT message."""
        msg = event.message
        self._count = msg.count
        self._commands = []

        if self._count == 0:
            self._downloading = False
            self._download_complete = True
            if self._on_download_complete:
                self._on_download_complete()
        else:
            # Request first item
            self._request_mission_item(0)

    def _handle_mission_item(self, event: "MAVLinkMessageEvent") -> None:
        """Handle MISSION_ITEM message."""
        msg = event.message
        cmd = Command.from_mavlink(msg)
        self._commands.append(cmd)

        # Request next item or complete
        if len(self._commands) < self._count:
            self._request_mission_item(len(self._commands))
        else:
            self._downloading = False
            self._download_complete = True
            if self._on_download_complete:
                self._on_download_complete()

    def _handle_mission_ack(self, event: "MAVLinkMessageEvent") -> None:
        """Handle MISSION_ACK message."""
        from pymavlink import mavutil

        msg = event.message
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            self._uploading = False
            self._upload_complete = True
            if self._on_upload_complete:
                self._on_upload_complete()
        else:
            logger.error("Mission upload failed with code: %d", msg.type)
            self._uploading = False

    def _handle_mission_current(self, event: "MAVLinkMessageEvent") -> None:
        """Handle MISSION_CURRENT message."""
        msg = event.message
        self._next_command = msg.seq

    def _request_mission_item(self, seq: int) -> None:
        """Request a specific mission item."""
        if not self._connection:
            return
        self._connection.master.mav.mission_request_send(
            self._connection.target_system,
            0,  # component
            seq,
        )

    def download(self) -> None:
        """
        Start downloading the mission from the vehicle.

        Use wait_ready() to block until download completes.
        """
        if not self._connection:
            raise APIException("No connection available")

        self._downloading = True
        self._download_complete = False
        self._commands = []
        self._count = -1

        # Request mission count
        self._connection.master.mav.mission_request_list_send(
            self._connection.target_system,
            0,  # component
        )

    def upload(self) -> None:
        """
        Upload the mission to the vehicle.

        The mission must be built first using add() and clear().
        """
        if not self._connection:
            raise APIException("No connection available")

        self._uploading = True
        self._upload_complete = False

        # Send count
        self._connection.master.mav.mission_count_send(
            self._connection.target_system,
            0,  # component
            len(self._commands),
        )

        # Vehicle will request items via MISSION_REQUEST
        # We need to handle that separately (not implemented in this basic version)
        # For now, send all items
        for i, cmd in enumerate(self._commands):
            msg = cmd.to_mavlink(
                self._connection.target_system,
                0,  # component
            )
            self._connection.master.mav.send(msg)
            time.sleep(0.01)  # Small delay between items

    def wait_ready(self, timeout: float = 30.0) -> bool:
        """
        Wait for download to complete.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if download completed, False if timeout

        Raises:
            APIException: If connection is lost
        """
        start = monotonic.monotonic()
        while self._downloading or not self._download_complete:
            if (monotonic.monotonic() - start) > timeout:
                return False
            if self._connection and not self._connection.is_alive:
                raise APIException("Connection lost during mission download")
            time.sleep(0.1)
        return True

    def clear(self) -> None:
        """Clear all commands from the local list."""
        self._commands = []

    def add(self, cmd: Command) -> None:
        """
        Add a command to the mission.

        Args:
            cmd: Command to add
        """
        # Update sequence number
        cmd.seq = len(self._commands)
        self._commands.append(cmd)

    @property
    def next(self) -> int:
        """Get the sequence number of the next command to execute."""
        return self._next_command

    @next.setter
    def next(self, value: int) -> None:
        """Set the next command to execute."""
        if not self._connection:
            raise APIException("No connection available")

        self._connection.master.mav.mission_set_current_send(
            self._connection.target_system,
            0,  # component
            value,
        )

    @property
    def count(self) -> int:
        """Get the number of commands."""
        return len(self._commands)

    def __len__(self) -> int:
        return len(self._commands)

    def __getitem__(self, index: int) -> Command:
        return self._commands[index]

    def __setitem__(self, index: int, cmd: Command) -> None:
        cmd.seq = index
        self._commands[index] = cmd

    def __delitem__(self, index: int) -> None:
        del self._commands[index]
        # Update sequence numbers
        for i, cmd in enumerate(self._commands):
            cmd.seq = i

    def __iter__(self) -> Iterator[Command]:
        return iter(self._commands)

    def __str__(self) -> str:
        return f"CommandSequence({len(self._commands)} commands)"
