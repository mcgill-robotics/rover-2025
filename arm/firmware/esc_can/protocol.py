"""
esc_can_protocol.py — CAN FD protocol definitions for the B-G431B-ESC1

Encodes the 11-bit standard CAN ID layout and payload formats used by
CAN_processing_v2 on the ESC firmware. Import this wherever you need to
build or parse CAN frames for the ESC bus.

CAN ID bit layout (11-bit standard):
    Bit [10]    Sender        0 = Master, 1 = Slave/ESC
    Bit [9]     Action        0 = Run,    1 = Read
    Bit [8]     MotorConfig   0 = Multiple motors, 1 = Single motor
    Bit [7]     MotorType     0 = Drive,  1 = Steering
    Bits [6:4]  Specification RunSpec (Action=Run) or ReadSpec (Action=Read)
    Bits [3:0]  DeviceID      0–15, matched against ESC_ID on the target

Bus parameters:
    Nominal (arbitration) bitrate : 500 kbps
    Data-phase bitrate            : 2 Mbps
    Frame format                  : CAN FD with bit-rate switching (BRS)
    ID type                       : 11-bit standard (not extended)
"""

from __future__ import annotations

import struct
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional


# ---------------------------------------------------------------------------
# Bus constants
# ---------------------------------------------------------------------------

ARB_BITRATE = 500_000       # nominal / arbitration phase  (bps)
DATA_BITRATE = 2_000_000    # data phase                   (bps)


# ---------------------------------------------------------------------------
# Enums — mirror the C firmware definitions exactly
# ---------------------------------------------------------------------------

class Sender(IntEnum):
    """Bit 10 — who originated the frame."""
    MASTER = 0
    SLAVE  = 1   # ESC response


class Action(IntEnum):
    """Bit 9 — command category."""
    RUN  = 0
    READ = 1


class MotorConfig(IntEnum):
    """Bit 8 — addressing mode."""
    MULTIPLE = 0   # payload carries data for several ESCs
    SINGLE   = 1   # payload targets one ESC


class MotorType(IntEnum):
    """Bit 7 — motor role."""
    DRIVE    = 0
    STEERING = 1


class RunSpec(IntEnum):
    """Bits [6:4] when Action == RUN.

    Mirrors the RunSpec enum in CAN_processing_v2.h.
    Values 0–7 (3 bits).
    """
    STOP               = 0
    ACKNOWLEDGE_FAULTS = 1
    SPEED              = 2
    POSITION           = 3
    CALIBRATION        = 4
    # 5–7 reserved


class ReadSpec(IntEnum):
    """Bits [6:4] when Action == READ.

    Mirrors the ReadSpec enum in CAN_processing_v2.h.
    Values 0–7 (3 bits).
    """
    SPEED         = 0   # was CALIBRATION
    POSITION      = 1
    VOLTAGE       = 2
    CURRENT       = 3
    TEMPERATURE   = 4   # was CURRENT_STATE
    CURRENT_STATE = 5   # was TEMPERATURE
    PING          = 6
    CONTROL_MODE  = 7


class MotorID(IntEnum):
    """Well-known Device IDs (bits [3:0]).

    Drive motors typically use IDs 0–7.
    Arm joints use IDs 8–10 (waist, shoulder, elbow).
    """
    DRIVE_0   = 0
    DRIVE_1   = 1
    DRIVE_2   = 2
    DRIVE_3   = 3
    DRIVE_4   = 4
    DRIVE_5   = 5
    DRIVE_6   = 6
    DRIVE_7   = 7
    WAIST     = 8
    SHOULDER  = 9
    ELBOW     = 10


# Parsed CAN ID
@dataclass
class ParsedCANID:
    """Decoded fields from an 11-bit CAN arbitration ID."""
    sender: Sender
    action: Action
    motor_config: MotorConfig
    motor_type: MotorType
    spec: int            # RunSpec or ReadSpec value (3 bits)
    device_id: int       # 0–15

    @property
    def arb_id(self) -> int:
        """Encode back to an 11-bit CAN ID integer."""
        return encode_can_id(
            sender=self.sender,
            action=self.action,
            motor_config=self.motor_config,
            motor_type=self.motor_type,
            spec=self.spec,
            device_id=self.device_id,
        )

    def as_slave_response(self) -> ParsedCANID:
        """Return a copy with the Sender bit flipped to SLAVE.

        The ESC mirrors the received ID but sets bit 10 = 1 so that
        other ESCs on the bus ignore the response.
        """
        return ParsedCANID(
            sender=Sender.SLAVE,
            action=self.action,
            motor_config=self.motor_config,
            motor_type=self.motor_type,
            spec=self.spec,
            device_id=self.device_id,
        )

    def __repr__(self) -> str:
        spec_name = ""
        try:
            if self.action == Action.RUN:
                spec_name = f" ({RunSpec(self.spec).name})"
            else:
                spec_name = f" ({ReadSpec(self.spec).name})"
        except ValueError:
            pass

        return (
            f"ParsedCANID(0x{self.arb_id:03X} | "
            f"sender={self.sender.name}, "
            f"action={self.action.name}, "
            f"config={self.motor_config.name}, "
            f"type={self.motor_type.name}, "
            f"spec={self.spec}{spec_name}, "
            f"dev={self.device_id})"
        )


# ID encode / decode
def encode_can_id(
    sender: Sender | int = Sender.MASTER,
    action: Action | int = Action.RUN,
    motor_config: MotorConfig | int = MotorConfig.SINGLE,
    motor_type: MotorType | int = MotorType.DRIVE,
    spec: int = 0,
    device_id: int = 0,
) -> int:
    """Build an 11-bit CAN arbitration ID from individual fields.

    >>> hex(encode_can_id(device_id=1, action=Action.READ, spec=ReadSpec.TEMPERATURE))
    '0x221'
    """
    if not 0 <= spec <= 7:
        raise ValueError(f"spec must be 0–7, got {spec}")
    if not 0 <= device_id <= 15:
        raise ValueError(f"device_id must be 0–15, got {device_id}")

    return (
        (int(sender)       & 0x1) << 10
        | (int(action)     & 0x1) << 9
        | (int(motor_config) & 0x1) << 8
        | (int(motor_type) & 0x1) << 7
        | (int(spec)       & 0x7) << 4
        | (int(device_id)  & 0xF)
    )


def decode_can_id(arb_id: int) -> ParsedCANID:
    """Parse an 11-bit CAN ID into its component fields.

    >>> decode_can_id(0x221)
    ParsedCANID(0x221 | sender=MASTER, action=READ, config=MULTIPLE, type=DRIVE, spec=2 (TEMPERATURE), dev=1)
    """
    return ParsedCANID(
        sender=Sender((arb_id >> 10) & 0x1),
        action=Action((arb_id >> 9) & 0x1),
        motor_config=MotorConfig((arb_id >> 8) & 0x1),
        motor_type=MotorType((arb_id >> 7) & 0x1),
        spec=(arb_id >> 4) & 0x7,
        device_id=arb_id & 0xF,
    )


# Payload builders data that goes into the CAN frame's data bytes

def pack_single_float(value: float) -> bytes:
    """Pack a 32-bit float into 8 bytes (4 data + 4 zero padding).

    Matches SingleExtractFloatFromCAN() on the firmware side:
    little-endian float at bytes 0–3.
    """
    return struct.pack("<f", value) + b"\x00" * 4


def pack_multi_speeds(speeds: dict[int, int]) -> bytes:
    """Pack per-ESC int16 speed values into a CAN FD payload.

    ``speeds`` maps device_id → int16 speed value.
    Matches extract_multiple_speeds(): 2 bytes at offset ESC_ID * 2,
    little-endian signed 16-bit.

    Returns up to 64 bytes (zero-padded).
    """
    buf = bytearray(64)
    for dev_id, speed in speeds.items():
        offset = dev_id * 2
        if offset + 2 > 64:
            raise ValueError(f"device_id {dev_id} exceeds 64-byte payload")
        struct.pack_into("<h", buf, offset, speed)
    return bytes(buf)


def pack_multi_positions_arm(positions: dict[int, float]) -> bytes:
    """Pack per-ESC half-float positions for arm joints.

    ``positions`` maps device_id (8=waist, 9=shoulder, 10=elbow) → float.
    Matches extract_multiple_positions_arm(): 2 bytes at offset
    (ESC_ID - 8) * 2, IEEE-754 half-precision, little-endian.

    Returns up to 64 bytes (zero-padded).
    """
    buf = bytearray(64)
    for dev_id, pos in positions.items():
        offset = (dev_id - 8) * 2
        if offset < 0 or offset + 2 > 64:
            raise ValueError(f"device_id {dev_id} out of arm-joint range")
        half_bytes = struct.pack("<e", pos)
        buf[offset : offset + 2] = half_bytes
    return bytes(buf)


# ---------------------------------------------------------------------------
# Payload parsers — read data from received CAN frame bytes
# ---------------------------------------------------------------------------

def unpack_single_float(data: bytes | bytearray) -> float:
    """Extract a 32-bit float from bytes 0–3 of a CAN payload."""
    return struct.unpack("<f", data[:4])[0]


def unpack_multi_speeds(data: bytes | bytearray, device_id: int) -> int:
    """Extract a single ESC's int16 speed from a multi-motor payload."""
    offset = device_id * 2
    return struct.unpack_from("<h", data, offset)[0]


def unpack_multi_positions_arm(data: bytes | bytearray, device_id: int) -> float:
    """Extract a single arm joint's half-float position from a multi-motor payload."""
    offset = (device_id - 8) * 2
    return struct.unpack_from("<e", data, offset)[0]


# ---------------------------------------------------------------------------
# Convenience frame builders (returns arb_id + data, ready for python-can)
# ---------------------------------------------------------------------------

def build_single_run_frame(
    device_id: int,
    run_spec: RunSpec | int,
    value: float,
    motor_type: MotorType | int = MotorType.DRIVE,
) -> tuple[int, bytes]:
    """Build a single-motor Run command.

    Returns (arb_id, data) suitable for passing to can.Message().
    """
    arb_id = encode_can_id(
        sender=Sender.MASTER,
        action=Action.RUN,
        motor_config=MotorConfig.SINGLE,
        motor_type=motor_type,
        spec=int(run_spec),
        device_id=device_id,
    )
    return arb_id, pack_single_float(value)


def build_single_read_frame(
    device_id: int,
    read_spec: ReadSpec | int,
    motor_type: MotorType | int = MotorType.DRIVE,
) -> tuple[int, bytes]:
    """Build a single-motor Read request.

    Returns (arb_id, data). Payload is 8 zero bytes (read requests
    carry no data; the ESC responds with the requested value).
    """
    arb_id = encode_can_id(
        sender=Sender.MASTER,
        action=Action.READ,
        motor_config=MotorConfig.SINGLE,
        motor_type=motor_type,
        spec=int(read_spec),
        device_id=device_id,
    )
    return arb_id, b"\x00" * 8


def build_multi_speed_frame(
    speeds: dict[int, int],
    motor_type: MotorType | int = MotorType.DRIVE,
) -> tuple[int, bytes]:
    """Build a multi-motor speed Run command.

    ``speeds`` maps device_id → int16 speed.
    The arb_id device_id field is set to 0 (broadcast convention).
    Returns (arb_id, data).
    """
    arb_id = encode_can_id(
        sender=Sender.MASTER,
        action=Action.RUN,
        motor_config=MotorConfig.MULTIPLE,
        motor_type=motor_type,
        spec=int(RunSpec.SPEED),
        device_id=0,
    )
    return arb_id, pack_multi_speeds(speeds)


def build_multi_position_arm_frame(
    positions: dict[int, float],
) -> tuple[int, bytes]:
    """Build a multi-motor arm position Run command.

    ``positions`` maps device_id (8/9/10) → float degrees.
    Returns (arb_id, data).
    """
    arb_id = encode_can_id(
        sender=Sender.MASTER,
        action=Action.RUN,
        motor_config=MotorConfig.MULTIPLE,
        motor_type=MotorType.STEERING,
        spec=int(RunSpec.POSITION),
        device_id=0,
    )
    return arb_id, pack_multi_positions_arm(positions)


# ---------------------------------------------------------------------------
# Quick self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Round-trip encode/decode
    cid = encode_can_id(
        sender=Sender.MASTER,
        action=Action.READ,
        motor_config=MotorConfig.SINGLE,
        motor_type=MotorType.DRIVE,
        spec=ReadSpec.TEMPERATURE,
        device_id=3,
    )
    parsed = decode_can_id(cid)
    print(f"Encoded  : 0x{cid:03X}")
    print(f"Decoded  : {parsed}")
    print(f"Re-encode: 0x{parsed.arb_id:03X}")
    print()

    # Build a single-motor speed command
    arb, data = build_single_run_frame(device_id=1, run_spec=RunSpec.SPEED, value=1500.0)
    print(f"Run SPEED to dev 1 : id=0x{arb:03X}  data={list(data)}")
    print(f"  -> float back    : {unpack_single_float(data)}")
    print()

    # Build a read-temperature request
    arb, data = build_single_read_frame(device_id=2, read_spec=ReadSpec.TEMPERATURE)
    print(f"Read TEMP from dev 2: id=0x{arb:03X}  data={list(data)}")
    print()

    # Build a multi-speed frame
    arb, data = build_multi_speed_frame({0: 1000, 1: -500, 2: 2000})
    print(f"Multi-speed         : id=0x{arb:03X}  first 12 bytes={list(data[:12])}")
    print(f"  -> dev 0 speed   : {unpack_multi_speeds(data, 0)}")
    print(f"  -> dev 1 speed   : {unpack_multi_speeds(data, 1)}")
    print(f"  -> dev 2 speed   : {unpack_multi_speeds(data, 2)}")