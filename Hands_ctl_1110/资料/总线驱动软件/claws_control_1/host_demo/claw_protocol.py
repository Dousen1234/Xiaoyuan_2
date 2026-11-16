"""Pure-Python codec for the claw's 11-bit Classical CAN protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

PROTOCOL_VERSION = 1
CAN_ID_DEFAULT_BASE = 11
CAN_ID_FAULT = CAN_ID_DEFAULT_BASE + 5
CAN_ID_COMMAND = CAN_ID_DEFAULT_BASE
CAN_ID_POSITION = CAN_ID_DEFAULT_BASE + 1
CAN_ID_STATUS = CAN_ID_DEFAULT_BASE + 2
CAN_ID_DIAGNOSTIC = CAN_ID_DEFAULT_BASE + 3
CAN_ID_PARAMETER = CAN_ID_DEFAULT_BASE + 4

FLAG_RELATIVE = 0x01


class Opcode(IntEnum):
    SET_TARGET = 0x01
    STOP = 0x02
    CLEAR_FAULT = 0x03
    SET_ZERO = 0x04
    HEARTBEAT = 0x05
    SET_TARGET_M1 = 0x06
    SET_TARGET_M2 = 0x07
    SET_TARGET_M3 = 0x08
    JOG = 0x09
    SEEK_OPEN = 0x0A
    GRIP = 0x0D
    RUN_IN = 0x0E
    SET_CAN_ID = 0x0F


TARGET_OPCODES = {
    Opcode.SET_TARGET,
    Opcode.SET_TARGET_M1,
    Opcode.SET_TARGET_M2,
    Opcode.SET_TARGET_M3,
}


class State(IntEnum):
    IDLE = 0
    MOVING = 1
    HOLD = 2
    FAULT = 3


class Result(IntEnum):
    OK = 0
    BUSY = 1
    FAULT = 2
    RANGE = 3
    INVALID = 4


@dataclass(frozen=True)
class PositionFeedback:
    state: State
    angles_deg: tuple[float, float, float]


@dataclass(frozen=True)
class StatusFeedback:
    sequence: int
    state: State
    fault_mask: int
    synchronization_error_deg: float
    command_age_ms: int
    result: Result
    target_reached: bool


@dataclass(frozen=True)
class DiagnosticFeedback:
    hall_codes: tuple[int, int, int]
    duty_permille: tuple[int, int, int]
    counter: int


def crc8(data: bytes) -> int:
    """CRC-8/SAE-J1850 over command bytes 0..6."""
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def encode_command(
    sequence: int,
    opcode: Opcode,
    *,
    target_deg: float = 0.0,
    speed_rpm: float = 0.0,
    relative: bool = False,
) -> bytes:
    if opcode in (Opcode.JOG, Opcode.SEEK_OPEN):
        raise ValueError("use encode_jog or encode_seek_open for commissioning commands")
    if opcode == Opcode.GRIP and (target_deg != 0 or speed_rpm != 0 or relative):
        raise ValueError("grip has no position or speed fields")
    target_cdeg = round(target_deg * 100.0)
    speed_drpm = round(speed_rpm * 10.0)
    if not -32768 <= target_cdeg <= 32767:
        raise ValueError("target angle does not fit the protocol's int16 centidegrees")
    if not 0 <= speed_drpm <= 255:
        raise ValueError("speed does not fit the protocol's uint8 deci-rpm field")
    if opcode in TARGET_OPCODES and speed_drpm != 0 and not 40 <= speed_drpm <= 170:
        raise ValueError("motion speed must be 0 (default) or between 4.0 and 17.0 rpm")
    flags = FLAG_RELATIVE if relative else 0
    payload = struct.pack(
        "<BBBBhB", PROTOCOL_VERSION, sequence & 0xFF, int(opcode), flags,
        target_cdeg, speed_drpm,
    )
    return payload + bytes((crc8(payload),))


def encode_jog(sequence: int, motor: int, direction: int) -> bytes:
    """Firmware-bounded pulse: 200 permille / 200 ms, board-specific travel threshold."""
    if motor not in (1, 2, 3) or direction not in (0, 1):
        raise ValueError("jog requires motor 1..3 and raw DIR level 0/1")
    payload = struct.pack("<BBBBhB", PROTOCOL_VERSION, sequence & 0xFF,
                          Opcode.JOG, direction, 0, motor)
    return payload + bytes((crc8(payload),))


def encode_run_in(sequence: int, motor: int, direction: int) -> bytes:
    """Continuous full-duty run-in; motor is 0 for all or 1..3."""
    if motor not in (0, 1, 2, 3) or direction not in (0, 1):
        raise ValueError("run-in requires motor 0..3 and direction 0/1")
    mask = 0 if motor == 0 else 1 << (motor - 1)
    data = bytes((PROTOCOL_VERSION, sequence & 255, Opcode.RUN_IN, direction, 0, 0, mask))
    return data + bytes((crc8(data),))


def encode_set_can_id(sequence: int, base: int, save: bool = True) -> bytes:
    if not 11 <= base <= 120:
        raise ValueError("CAN ID base must be 11..120")
    data = struct.pack("<BBBBhB", PROTOCOL_VERSION, sequence & 255,
                       Opcode.SET_CAN_ID, 1 if save else 0, base, 0)
    return data + bytes((crc8(data),))


def decode_feedback(arbitration_id: int, data: bytes):
    if len(data) != 8 or data[0] != PROTOCOL_VERSION:
        raise ValueError("invalid feedback version or DLC")

    if arbitration_id == CAN_ID_POSITION:
        angles = struct.unpack_from("<hhh", data, 2)
        return PositionFeedback(State(data[1]), tuple(value / 100.0 for value in angles))

    if arbitration_id in (CAN_ID_STATUS, CAN_ID_FAULT):
        sync_cdeg = struct.unpack_from("<h", data, 4)[0]
        return StatusFeedback(
            sequence=data[1],
            state=State(data[2]),
            fault_mask=data[3],
            synchronization_error_deg=sync_cdeg / 100.0,
            command_age_ms=data[6] * 10,
            result=Result(data[7] & 0x0F),
            target_reached=bool(data[7] & 0x80),
        )

    if arbitration_id == CAN_ID_DIAGNOSTIC:
        return DiagnosticFeedback(
            hall_codes=(data[1], data[2], data[3]),
            duty_permille=(data[4] * 4, data[5] * 4, data[6] * 4),
            counter=data[7],
        )
    raise ValueError(f"unsupported CAN identifier 0x{arbitration_id:03X}")


def encode_seek_open(sequence: int, motor: int) -> bytes:
    if motor not in (0, 1, 2, 3):
        raise ValueError("motor must be 0 (all) or 1..3")
    data = bytes((PROTOCOL_VERSION, sequence & 255, Opcode.SEEK_OPEN, 0, 0, 0, motor))
    return data + bytes((crc8(data),))
