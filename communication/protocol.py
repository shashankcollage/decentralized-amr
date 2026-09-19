"""
communication/protocol.py
===========================
Message validation and a basic security model (project spec section 38):

    - message validation (well-formed JSON, required fields present)
    - robot ID validation (sender must be a known peer)
    - timestamp validation (not too old, not from the future)
    - message type validation (must be a known MessageType)
    - optional shared-secret HMAC authentication

This is deliberately NOT a full cryptographic security architecture -
the goal (per the spec) is to demonstrate awareness of secure
robot-to-robot communication, not to build a production PKI. HMAC-SHA256
with a shared secret is enough to detect tampering/spoofing by anyone
who doesn't have the secret, without the key-management complexity of
asymmetric crypto.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import logging
import time
from typing import Iterable, Optional

import config
from communication.message import Message, MessageType

logger = logging.getLogger("protocol")


class ValidationError(str):
    """Simple string-subclass "enum" of validation failure reasons, kept
    as plain strings (rather than an Enum) so callers/logs can print them
    directly without a .value lookup."""
    MALFORMED = "MALFORMED_MESSAGE"
    UNKNOWN_TYPE = "UNKNOWN_MESSAGE_TYPE"
    UNKNOWN_ROBOT = "UNKNOWN_ROBOT_ID"
    STALE_TIMESTAMP = "STALE_TIMESTAMP"
    FUTURE_TIMESTAMP = "FUTURE_TIMESTAMP"
    BAD_SIGNATURE = "BAD_SIGNATURE"


def _canonical_signing_payload(msg: Message) -> bytes:
    """Deterministic byte representation of a message's content (excluding
    the signature field itself) used for HMAC signing/verification."""
    d = msg.to_dict()
    d.pop("signature", None)
    return json.dumps(d, sort_keys=True).encode("utf-8")


def sign_message(msg: Message, secret: bytes = config.HMAC_SHARED_SECRET) -> Message:
    digest = hmac.new(secret, _canonical_signing_payload(msg), hashlib.sha256).hexdigest()
    msg.signature = digest
    return msg


def verify_signature(msg: Message, secret: bytes = config.HMAC_SHARED_SECRET) -> bool:
    if msg.signature is None:
        return False
    expected = hmac.new(secret, _canonical_signing_payload(msg), hashlib.sha256).hexdigest()
    # Constant-time comparison to avoid leaking signature bytes via timing.
    return hmac.compare_digest(expected, msg.signature)


def validate_message(
    raw: str,
    known_robot_ids: Iterable[str],
    now: Optional[float] = None,
    max_age: float = config.MESSAGE_MAX_AGE,
    require_signature: bool = True,
    secret: bytes = config.HMAC_SHARED_SECRET,
) -> "tuple[Optional[Message], Optional[str]]":
    """Parse and fully validate a raw message string.

    Returns (message, None) on success, or (None, error_reason) on
    failure, where error_reason is one of the ValidationError constants.
    Never raises - malformed input from the network is expected and must
    be handled gracefully (project spec section 10: "handle invalid
    messages").
    """
    now = now if now is not None else time.time()

    try:
        msg = Message.from_json(raw)
    except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
        logger.warning("Rejected malformed message: %s", e)
        return None, ValidationError.MALFORMED

    if msg.type not in MessageType:
        return None, ValidationError.UNKNOWN_TYPE

    if msg.robot_id not in known_robot_ids:
        return None, ValidationError.UNKNOWN_ROBOT

    age = now - msg.timestamp
    if age > max_age:
        return None, ValidationError.STALE_TIMESTAMP
    if age < -1.0:  # small tolerance for clock skew between local threads
        return None, ValidationError.FUTURE_TIMESTAMP

    if require_signature and not verify_signature(msg, secret):
        return None, ValidationError.BAD_SIGNATURE

    return msg, None
