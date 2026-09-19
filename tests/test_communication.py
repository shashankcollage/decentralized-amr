"""
tests/test_communication.py
==============================
Tests for communication/message.py, protocol.py, and network_manager.py,
using REAL UDP sockets on localhost (not mocks) - these tests actually
exercise the decentralized transport.
"""

import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from communication.message import Message, MessageType, make_robot_state_message
from communication.protocol import sign_message, verify_signature, validate_message
from communication.network_manager import NetworkManager


def test_valid_message_round_trip():
    msg = make_robot_state_message(
        robot_id="R1", position=(1, 1), velocity=(1, 0), destination=(5, 5),
        battery=90.0, status="MOVING", priority=0.5, path=[(1, 1), (2, 1)],
        current_task="T1", timestamp=1000.0,
    )
    sign_message(msg)
    parsed, error = validate_message(msg.to_json(), known_robot_ids=["R1", "R2"], now=1000.5)
    assert error is None
    assert parsed.robot_id == "R1"
    assert parsed.payload["position"] == [1, 1]


def test_invalid_json_rejected():
    parsed, error = validate_message("not valid json{{{", known_robot_ids=["R1"], now=0.0)
    assert parsed is None
    assert error == "MALFORMED_MESSAGE"


def test_unknown_robot_id_rejected():
    msg = make_robot_state_message(
        robot_id="R99", position=(1, 1), velocity=(0, 0), destination=None,
        battery=100.0, status="IDLE", priority=0.0, path=[], current_task=None, timestamp=1000.0,
    )
    sign_message(msg)
    parsed, error = validate_message(msg.to_json(), known_robot_ids=["R1", "R2"], now=1000.0)
    assert parsed is None
    assert error == "UNKNOWN_ROBOT_ID"


def test_stale_message_rejected():
    msg = make_robot_state_message(
        robot_id="R1", position=(1, 1), velocity=(0, 0), destination=None,
        battery=100.0, status="IDLE", priority=0.0, path=[], current_task=None, timestamp=0.0,
    )
    sign_message(msg)
    parsed, error = validate_message(msg.to_json(), known_robot_ids=["R1"], now=1000.0, max_age=5.0)
    assert parsed is None
    assert error == "STALE_TIMESTAMP"


def test_bad_signature_rejected():
    msg = make_robot_state_message(
        robot_id="R1", position=(1, 1), velocity=(0, 0), destination=None,
        battery=100.0, status="IDLE", priority=0.0, path=[], current_task=None, timestamp=1000.0,
    )
    sign_message(msg, secret=b"wrong-secret")
    parsed, error = validate_message(msg.to_json(), known_robot_ids=["R1"], now=1000.0)
    assert parsed is None
    assert error == "BAD_SIGNATURE"


def test_real_udp_broadcast_between_two_network_managers():
    ids = ["R1", "R2"]
    nm1 = NetworkManager("R1", ids)
    nm2 = NetworkManager("R2", ids)
    try:
        msg = make_robot_state_message(
            robot_id="R1", position=(3, 3), velocity=(1, 0), destination=(9, 9),
            battery=80.0, status="MOVING", priority=0.4, path=[(3, 3), (4, 3)],
            current_task="T1", timestamp=time.time(),
        )
        sent = nm1.broadcast(msg)
        assert sent == 1
        time.sleep(0.05)
        received = nm2.receive_messages()
        assert len(received) == 1
        assert received[0].robot_id == "R1"
        assert received[0].payload["position"] == [3, 3]
    finally:
        nm1.close()
        nm2.close()


def test_duplicate_message_filtered():
    ids = ["R1", "R2"]
    nm1 = NetworkManager("R1", ids)
    nm2 = NetworkManager("R2", ids)
    try:
        msg = make_robot_state_message(
            robot_id="R1", position=(1, 1), velocity=(0, 0), destination=None,
            battery=100.0, status="IDLE", priority=0.0, path=[], current_task=None,
            timestamp=time.time(),
        )
        sign_message(msg)
        raw = msg.to_json().encode("utf-8")
        # Send the exact same datagram twice.
        nm1.socket_peer.send(raw, nm1.peer_addresses["R2"])
        nm1.socket_peer.send(raw, nm1.peer_addresses["R2"])
        time.sleep(0.05)
        received = nm2.receive_messages()
        assert len(received) == 1  # duplicate filtered, not delivered twice
    finally:
        nm1.close()
        nm2.close()
