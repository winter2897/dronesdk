"""
MAVLink Connection Implementation.

This module provides the MAVConnection class for establishing and managing
MAVLink communication with vehicles.
"""

from __future__ import annotations

import atexit
import copy
import errno
import logging
import os
import platform
import socket
import time
from queue import Empty, Queue
from threading import Thread
from typing import Any, Callable

from pymavlink import mavutil

from dronesdk.core.exceptions import APIException

if platform.system() == "Windows":
    from errno import WSAECONNRESET as ECONNABORTED
else:
    from errno import ECONNABORTED

logger = logging.getLogger(__name__)


class MAVWriter:
    """
    Thread-safe message writer for MAVLink.

    This provides an indirection layer to ensure all messages are
    written from a single thread.
    """

    __slots__ = ("_logger", "queue")

    def __init__(self, queue: Queue[bytes]) -> None:
        self._logger = logging.getLogger(__name__)
        self.queue = queue

    def write(self, pkt: bytes) -> None:
        """Queue a packet for writing."""
        self.queue.put(pkt)

    def read(self) -> None:
        """Read is not supported on the writer."""
        self._logger.critical("writer should not have had a read request")
        os._exit(43)


class MAVUDPMulti(mavutil.mavfile):
    """
    A UDP MAVLink socket supporting multiple addresses.

    This extends the pymavlink mavfile to support receiving from
    and sending to multiple UDP endpoints.
    """

    def __init__(
        self,
        device: str,
        baud: int | None = None,
        input: bool = True,
        broadcast: bool = False,
        source_system: int = 255,
        source_component: int = 0,
        use_native: bool = mavutil.default_native,
    ) -> None:
        self._logger = logging.getLogger(__name__)
        a = device.split(":")
        if len(a) != 2:
            self._logger.critical("UDP ports must be specified as host:port")
            raise ValueError("UDP ports must be specified as host:port")

        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server = input
        self.broadcast = False
        self.addresses: set[tuple[str, int]] = set()
        self.destination_addr: tuple[str, int] | None = None

        if input:
            self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.port.bind((a[0], int(a[1])))
        else:
            self.destination_addr = (a[0], int(a[1]))
            if broadcast:
                self.port.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.broadcast = True

        mavutil.set_close_on_exec(self.port.fileno())
        self.port.setblocking(False)
        mavutil.mavfile.__init__(
            self,
            self.port.fileno(),
            device,
            source_system=source_system,
            source_component=source_component,
            input=input,
            use_native=use_native,
        )

    def close(self) -> None:
        """Close the UDP socket."""
        self.port.close()

    def recv(self, n: int | None = None) -> bytes:
        """Receive data from the UDP socket."""
        try:
            try:
                data, new_addr = self.port.recvfrom(65535)
            except socket.error as e:
                if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                    return b""
                raise
            if self.udp_server:
                self.addresses.add(new_addr)
            elif self.broadcast:
                self.addresses = {new_addr}
            return data
        except Exception:
            self._logger.exception("Exception while reading data", exc_info=True)
            return b""

    def write(self, buf: bytes) -> None:
        """Write data to the UDP socket."""
        try:
            try:
                if self.udp_server:
                    for addr in self.addresses:
                        self.port.sendto(buf, addr)
                else:
                    if self.addresses and self.broadcast:
                        self.destination_addr = list(self.addresses)[0]
                        self.broadcast = False
                        self.port.connect(self.destination_addr)
                    if self.destination_addr:
                        self.port.sendto(buf, self.destination_addr)
            except socket.error:
                pass
        except Exception:
            self._logger.exception("Exception while writing data", exc_info=True)

    def recv_msg(self) -> Any:
        """Receive and parse a MAVLink message."""
        self.pre_message()
        s = self.recv()
        if len(s) > 0:
            if self.first_byte:
                self.auto_mavlink_version(s)

        m = self.mav.parse_char(s)
        if m is not None:
            self.post_message(m)

        return m


# Type alias for message callback
MessageCallback = Callable[["MAVConnection", Any], None]
LoopCallback = Callable[["MAVConnection"], None]


class MAVConnection:
    """
    MAVLink connection manager.

    This class handles the low-level MAVLink communication including:
    - Connection establishment (serial, UDP, TCP)
    - Message sending and receiving
    - Thread management for I/O
    - Message listener registration

    Args:
        ip: Connection string (e.g., "udpin:localhost:14550", "tcp:127.0.0.1:5760")
        baud: Baud rate for serial connections
        target_system: Target system ID
        source_system: Source system ID
        source_component: Source component ID
        use_native: Use native MAVLink implementation
    """

    __slots__ = (
        "_logger",
        "master",
        "out_queue",
        "target_system",
        "loop_listeners",
        "message_listeners",
        "_accept_input",
        "_alive",
        "_death_error",
        "mavlink_thread_in",
        "mavlink_thread_out",
    )

    def __init__(
        self,
        ip: str,
        baud: int = 115200,
        target_system: int = 0,
        source_system: int = 255,
        source_component: int = 0,
        use_native: bool = False,
    ) -> None:
        self._logger = logging.getLogger(__name__)

        # Create the mavlink connection
        if ip.startswith("udpin:"):
            self.master = MAVUDPMulti(
                ip[6:],
                input=True,
                baud=baud,
                source_system=source_system,
                source_component=source_component,
            )
        else:
            self.master = mavutil.mavlink_connection(
                ip,
                baud=baud,
                source_system=source_system,
                source_component=source_component,
            )

        # Message output queue
        self.out_queue: Queue[bytes] = Queue()
        self.master.mav = mavutil.mavlink.MAVLink(
            MAVWriter(self.out_queue),
            srcSystem=self.master.source_system,
            srcComponent=self.master.source_component,
            use_native=use_native,
        )

        # Monkey-patch MAVLink object for fix_targets
        sendfn = self.master.mav.send

        def newsendfn(mavmsg: Any, *args: Any, **kwargs: Any) -> Any:
            self.fix_targets(mavmsg)
            return sendfn(mavmsg, *args, **kwargs)

        self.master.mav.send = newsendfn

        # Target system
        self.target_system = target_system

        # Listeners
        self.loop_listeners: list[LoopCallback] = []
        self.message_listeners: list[MessageCallback] = []

        # State flags
        self._accept_input = True
        self._alive = True
        self._death_error: Exception | None = None

        # Thread handles
        self.mavlink_thread_in: Thread | None = None
        self.mavlink_thread_out: Thread | None = None

        # Register cleanup
        atexit.register(self._onexit)

        # Create threads
        self._create_threads()

    def _onexit(self) -> None:
        """Cleanup on exit."""
        self._alive = False
        self.stop_threads()

    def _create_threads(self) -> None:
        """Create the I/O threads."""
        t = Thread(target=self._mavlink_thread_in)
        t.daemon = True
        self.mavlink_thread_in = t

        t = Thread(target=self._mavlink_thread_out)
        t.daemon = True
        self.mavlink_thread_out = t

    def _mavlink_thread_out(self) -> None:
        """Output thread - sends messages from queue."""
        try:
            while self._alive:
                try:
                    msg = self.out_queue.get(True, timeout=0.01)
                    self.master.write(msg)
                except Empty:
                    continue
                except socket.error as error:
                    if error.errno == ECONNABORTED:
                        raise APIException("Connection aborting during write")
                    raise
                except Exception as e:
                    self._logger.exception("mav send error: %s", e)
                    break
        except APIException as e:
            self._logger.exception("Exception in MAVLink write loop")
            self._alive = False
            self.master.close()
            self._death_error = e
        except Exception as e:
            if self._alive:
                self._alive = False
                self.master.close()
                self._death_error = e

        # Clear output queue
        self.out_queue = Queue()

    def _mavlink_thread_in(self) -> None:
        """Input thread - receives and dispatches messages."""
        try:
            while self._alive:
                # Loop listeners
                for fn in self.loop_listeners:
                    fn(self)

                # Sleep/select
                self.master.select(0.05)

                while self._accept_input:
                    try:
                        msg = self.master.recv_msg()
                    except socket.error as error:
                        if error.errno == ECONNABORTED:
                            raise APIException("Connection aborting during read")
                        raise
                    except mavutil.mavlink.MAVError as e:
                        self._logger.debug("mav recv error: %s", e)
                        msg = None
                    except Exception:
                        self._logger.exception(
                            "Exception while receiving message: ",
                            exc_info=True,
                        )
                        msg = None

                    if not msg:
                        break

                    # Dispatch to message listeners
                    for fn in self.message_listeners:
                        try:
                            fn(self, msg)
                        except Exception:
                            self._logger.exception(
                                "Exception in message handler for %s",
                                msg.get_type(),
                                exc_info=True,
                            )

        except APIException as e:
            self._logger.exception("Exception in MAVLink input loop")
            self._alive = False
            self.master.close()
            self._death_error = e
        except Exception as e:
            if self._alive:
                self._alive = False
                self.master.close()
                self._death_error = e

    def stop_threads(self) -> None:
        """Stop the I/O threads."""
        if self.mavlink_thread_in is not None:
            self.mavlink_thread_in.join(timeout=2.0)
            self.mavlink_thread_in = None
        if self.mavlink_thread_out is not None:
            self.mavlink_thread_out.join(timeout=2.0)
            self.mavlink_thread_out = None

    def reset(self) -> None:
        """Reset the connection."""
        self.out_queue = Queue()
        if hasattr(self.master, "reset"):
            self.master.reset()
        else:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = mavutil.mavlink_connection(self.master.address)

    def fix_targets(self, message: Any) -> None:
        """Set correct target IDs for our vehicle."""
        if hasattr(message, "target_system"):
            message.target_system = self.target_system

    def forward_loop(self, fn: LoopCallback) -> LoopCallback:
        """
        Decorator for event loop callbacks.

        The callback is called on each iteration of the message loop.
        """
        self.loop_listeners.append(fn)
        return fn

    def forward_message(self, fn: MessageCallback) -> MessageCallback:
        """
        Decorator for message callbacks.

        The callback is called for each received message.
        """
        self.message_listeners.append(fn)
        return fn

    def start(self) -> None:
        """Start the I/O threads."""
        if self.mavlink_thread_in and not self.mavlink_thread_in.is_alive():
            self.mavlink_thread_in.start()
        if self.mavlink_thread_out and not self.mavlink_thread_out.is_alive():
            self.mavlink_thread_out.start()

    def close(self) -> None:
        """Close the connection."""
        self._alive = False
        # Wait for output queue to drain
        timeout = time.monotonic() + 5.0
        while not self.out_queue.empty() and time.monotonic() < timeout:
            time.sleep(0.1)
        self.stop_threads()
        self.master.close()

    def pipe(self, target: "MAVConnection") -> "MAVConnection":
        """
        Pipe messages between this connection and another.

        Args:
            target: The target connection to pipe to

        Returns:
            The target connection
        """
        target.target_system = self.target_system

        # vehicle -> self -> target
        @self.forward_message
        def callback_to_target(_: MAVConnection, msg: Any) -> None:
            try:
                target.out_queue.put(msg.pack(target.master.mav))
            except Exception:
                try:
                    assert len(msg.get_msgbuf()) > 0
                    target.out_queue.put(msg.get_msgbuf())
                except Exception:
                    self._logger.exception(
                        "Could not pack this object on receive: %s",
                        type(msg),
                        exc_info=True,
                    )

        # target -> self -> vehicle
        @target.forward_message
        def callback_from_target(_: MAVConnection, msg: Any) -> None:
            msg = copy.copy(msg)
            target.fix_targets(msg)
            try:
                self.out_queue.put(msg.pack(self.master.mav))
            except Exception:
                try:
                    assert len(msg.get_msgbuf()) > 0
                    self.out_queue.put(msg.get_msgbuf())
                except Exception:
                    self._logger.exception(
                        "Could not pack this object on forward: %s",
                        type(msg),
                        exc_info=True,
                    )

        return target

    @property
    def is_alive(self) -> bool:
        """Check if the connection is alive."""
        return self._alive

    @property
    def death_error(self) -> Exception | None:
        """Get the error that caused connection death, if any."""
        return self._death_error
