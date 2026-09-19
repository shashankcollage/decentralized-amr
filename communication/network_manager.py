"""
communication/network_manager.py
===================================
Per-robot network manager - the piece a Robot actually talks to. Wraps:
  * discovery.PeerDiscovery  (who are my peers, and their addresses)
  * peer.Peer                (the actual UDP socket)
  * protocol.validate_message (security/validity checks)
  * protocol.sign_message     (HMAC signing of outgoing messages)

...and adds the higher-level behaviour project spec section 10 asks for:
send_message(), broadcast(), receive_message(), validate_message(),
handle_message(), plus peer-timeout tracking (a peer that stops sending
heartbeats is eventually treated as unavailable, per section 10's "If a
robot stops sending messages, mark it as unavailable after configurable
timeout").

One NetworkManager instance = one robot's entire communication stack.
Nothing here is shared between robots; each robot's copy is completely
independent, which is what actually makes the architecture decentralized
even though, for simulation convenience, all these independent instances
happen to run in the same Python process.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import config
from communication.discovery import PeerDiscovery, Address
from communication.message import Message
from communication.peer import Peer
from communication.protocol import validate_message, sign_message

logger = logging.getLogger("network_manager")


@dataclass
class PeerLivenessTracker:
    """Tracks the last time we heard from each peer, so a silent peer can
    be marked unavailable after config.ROBOT_TIMEOUT, per spec section 10.
    """
    last_seen: Dict[str, float] = field(default_factory=dict)

    def mark_seen(self, robot_id: str, now: float) -> None:
        self.last_seen[robot_id] = now

    def is_available(self, robot_id: str, now: float, timeout: float = config.ROBOT_TIMEOUT) -> bool:
        """A peer we have never heard from yet is given the benefit of the
        doubt (treated as available) rather than assumed failed - "no
        data yet" and "confirmed timed out" are different situations, and
        conflating them would make every peer look unavailable for the
        first ROBOT_TIMEOUT seconds of a run, before its first message
        has even had a chance to arrive. Only a peer we HAVE heard from
        before, and who has since gone silent past `timeout`, is reported
        unavailable.
        """
        last = self.last_seen.get(robot_id)
        if last is None:
            return True
        return (now - last) <= timeout

    def unavailable_peers(self, known_peer_ids: List[str], now: float,
                            timeout: float = config.ROBOT_TIMEOUT) -> List[str]:
        return [rid for rid in known_peer_ids if not self.is_available(rid, now, timeout)]


class NetworkManager:
    def __init__(self, self_robot_id: str, all_robot_ids: List[str],
                 require_signature: bool = True, port_offset: int = 0) -> None:
        self.self_robot_id = self_robot_id
        self.all_robot_ids = all_robot_ids
        self.require_signature = require_signature

        self.discovery = PeerDiscovery(self_robot_id, all_robot_ids, port_offset=port_offset)
        self.peer_addresses: Dict[str, Address] = self.discovery.discover_peers()

        self.socket_peer = Peer(bind_address=self.discovery.self_address())
        self.liveness = PeerLivenessTracker()

        self.messages_sent = 0
        self.messages_received = 0
        self.messages_rejected = 0

        self._seen_message_keys: set = set()
        self._max_seen_keys = 5000

    def send_message(self, message: Message, target_robot_id: str) -> bool:
        if self.require_signature:
            sign_message(message)
        address = self.peer_addresses.get(target_robot_id)
        if address is None:
            logger.warning("%s: unknown target robot %s", self.self_robot_id, target_robot_id)
            return False
        sent = self.socket_peer.send(message.to_json().encode("utf-8"), address)
        if sent:
            self.messages_sent += 1
        return sent

    def broadcast(self, message: Message) -> int:
        if self.require_signature:
            sign_message(message)
        data = message.to_json().encode("utf-8")
        count = self.socket_peer.broadcast(data, list(self.peer_addresses.values()))
        self.messages_sent += count
        return count

    def receive_messages(self, now: Optional[float] = None) -> List[Message]:
        now = now if now is not None else time.time()
        valid_messages: List[Message] = []

        for raw_bytes in self.socket_peer.receive_all():
            try:
                raw_str = raw_bytes.decode("utf-8")
            except UnicodeDecodeError:
                self.messages_rejected += 1
                continue

            msg, error = validate_message(
                raw_str,
                known_robot_ids=self.all_robot_ids,
                now=now,
                require_signature=self.require_signature,
            )
            if msg is None:
                logger.debug("%s: rejected message (%s)", self.self_robot_id, error)
                self.messages_rejected += 1
                continue

            dedup_key = (msg.robot_id, msg.type.value, msg.timestamp)
            if dedup_key in self._seen_message_keys:
                continue
            self._seen_message_keys.add(dedup_key)
            if len(self._seen_message_keys) > self._max_seen_keys:
                self._seen_message_keys = set(list(self._seen_message_keys)[self._max_seen_keys // 2:])

            self.liveness.mark_seen(msg.robot_id, now)
            self.messages_received += 1
            valid_messages.append(msg)

        return valid_messages

    def peer_ids(self) -> List[str]:
        return list(self.peer_addresses.keys())

    def unavailable_peers(self, now: Optional[float] = None) -> List[str]:
        now = now if now is not None else time.time()
        return self.liveness.unavailable_peers(self.peer_ids(), now)

    def close(self) -> None:
        self.socket_peer.close()
