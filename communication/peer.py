"""
communication/peer.py
=======================
Each robot's own UDP communication endpoint (project spec section 10).

This is a genuinely decentralized transport: every robot binds its own
socket to its own port and sends datagrams directly to its peers'
addresses. There is no broker, relay, or central socket anywhere - if
you strace this process you'd see N independent UDP sockets exchanging
packets directly with each other, exactly like N separate machines would.

Simulated network imperfections (packet loss, delay) are implemented
here since this is the layer that owns the actual socket I/O, per
project spec section 37.
"""

from __future__ import annotations

import logging
import random
import socket
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import config

Address = Tuple[str, int]

logger = logging.getLogger("peer")


@dataclass
class DelayedMessage:
    ready_at: float
    data: bytes


class Peer:
    """A single robot's UDP socket, bound to its own address.

    Usage per simulation tick:
        peer.send(data, peer_address)      # or peer.broadcast(data, [...])
        for raw in peer.receive_all():      # non-blocking drain
            ...
    """

    def __init__(self, bind_address: Address,
                 packet_loss_rate: float = config.PACKET_LOSS_RATE,
                 network_delay: float = config.NETWORK_DELAY,
                 rng: Optional[random.Random] = None) -> None:
        self.bind_address = bind_address
        self.packet_loss_rate = packet_loss_rate
        self.network_delay = network_delay
        self.rng = rng or random.Random()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(bind_address)
        self.socket.setblocking(False)

        # Simulated network delay is implemented as a local delivery
        # queue: an outgoing send still happens immediately at the OS
        # level (loopback delivers it near-instantly regardless), but we
        # hold *incoming* datagrams here until their simulated arrival
        # time, so a receiver perceives the configured NETWORK_DELAY.
        self._delayed_inbox: List[DelayedMessage] = []

        logger.info("Peer bound to %s", bind_address)

    def send(self, data: bytes, address: Address) -> bool:
        """Send one datagram to a specific peer address. Returns False
        (without raising) if the simulated packet-loss roll drops it, or
        if the OS-level send fails (e.g. peer socket not yet up)."""
        if self.rng.random() < self.packet_loss_rate:
            return False
        try:
            self.socket.sendto(data, address)
            return True
        except OSError as e:
            logger.debug("send() to %s failed: %s", address, e)
            return False

    def broadcast(self, data: bytes, addresses: List[Address]) -> int:
        """Send one datagram to every address in `addresses`. Returns the
        number that were actually sent (not dropped by simulated loss)."""
        return sum(1 for addr in addresses if self.send(data, addr))

    def _drain_socket(self) -> None:
        """Pull everything currently sitting in the OS socket buffer into
        our own delayed-delivery queue."""
        while True:
            try:
                data, _addr = self.socket.recvfrom(config.UDP_BUFFER_SIZE)
            except BlockingIOError:
                break
            except OSError:
                break
            ready_at = time.time() + self.network_delay
            self._delayed_inbox.append(DelayedMessage(ready_at=ready_at, data=data))

    def receive_all(self) -> List[bytes]:
        """Non-blocking: return every datagram that has both arrived at
        the OS socket AND whose simulated network delay has elapsed."""
        self._drain_socket()
        now = time.time()
        ready = [m for m in self._delayed_inbox if m.ready_at <= now]
        self._delayed_inbox = [m for m in self._delayed_inbox if m.ready_at > now]
        return [m.data for m in ready]

    def close(self) -> None:
        try:
            self.socket.close()
        except OSError:
            pass
