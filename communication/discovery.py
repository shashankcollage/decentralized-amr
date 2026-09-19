"""
communication/discovery.py
=============================
Peer discovery, per project spec section 11.

For the simulation, there is no central robot registry: every robot is
simply given the full list of peer UDP addresses up front (localhost
ports, one per robot - R1=127.0.0.1:5001, R2=127.0.0.1:5002, ...), computed
deterministically from config.robot_address(). This is equivalent to a
static config file distributed to every robot ahead of time, which is a
completely standard (and still decentralized) discovery mechanism - no
robot asks a server "who else is out there", they already know.

The PeerDiscovery class below is intentionally a thin wrapper (rather
than inlined logic in NetworkManager) so a later deployment can swap in
a different discovery mechanism - UDP broadcast, mDNS, or ROS 2's DDS
discovery - by implementing the same `discover_peers()` interface,
without touching any of the code that consumes the peer list.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import config

Address = Tuple[str, int]


@dataclass
class PeerDiscovery:
    self_robot_id: str
    all_robot_ids: List[str]
    host: str = "127.0.0.1"
    port_offset: int = 0

    def discover_peers(self) -> Dict[str, Address]:
        """Return {robot_id: (host, port)} for every OTHER known robot.

        A future implementation backed by UDP broadcast/mDNS/ROS 2 would
        replace this method's body with an actual discovery handshake but
        keep returning the same {robot_id: address} shape.
        """
        peers: Dict[str, Address] = {}
        for i, robot_id in enumerate(self.all_robot_ids):
            if robot_id == self.self_robot_id:
                continue
            peers[robot_id] = config.robot_address(i, self.host)
            peers[robot_id] = (peers[robot_id][0], peers[robot_id][1] + self.port_offset)
        return peers

    def self_address(self) -> Address:
        index = self.all_robot_ids.index(self.self_robot_id)
        host, port = config.robot_address(index, self.host)
        return (host, port + self.port_offset)
