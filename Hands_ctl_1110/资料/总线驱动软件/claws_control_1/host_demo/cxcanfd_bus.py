"""Chuang Xin USBCANFD backend using the vendor ControlCANFD SDK.

The adapter sends and receives Classical CAN frames through the vendor's
ZCAN_* API.  CAN-FD transmit functions are intentionally not used here.
"""

from __future__ import annotations

from collections import deque
import ctypes
import os
from pathlib import Path
import time
from typing import Deque, Optional

import can


USBCANFD_200U = 41
STATUS_OK = 1
INVALID_HANDLE = 0
TYPE_CAN = 0
TYPE_CANFD = 1
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_ID_MASK = 0x1FFFFFFF


class _CanInitConfig(ctypes.Structure):
    _fields_ = [
        ("acc_code", ctypes.c_uint),
        ("acc_mask", ctypes.c_uint),
        ("reserved", ctypes.c_uint),
        ("filter", ctypes.c_ubyte),
        ("timing0", ctypes.c_ubyte),
        ("timing1", ctypes.c_ubyte),
        ("mode", ctypes.c_ubyte),
    ]


class _CanFdInitConfig(ctypes.Structure):
    _fields_ = [
        ("acc_code", ctypes.c_uint),
        ("acc_mask", ctypes.c_uint),
        ("abit_timing", ctypes.c_uint),
        ("dbit_timing", ctypes.c_uint),
        ("brp", ctypes.c_uint),
        ("filter", ctypes.c_ubyte),
        ("mode", ctypes.c_ubyte),
        ("pad", ctypes.c_ushort),
        ("reserved", ctypes.c_uint),
    ]


class _ChannelConfig(ctypes.Union):
    _fields_ = [("can", _CanInitConfig), ("canfd", _CanFdInitConfig)]


class _ChannelInitConfig(ctypes.Structure):
    _fields_ = [("can_type", ctypes.c_uint), ("config", _ChannelConfig)]


class _CanFrame(ctypes.Structure):
    _fields_ = [
        ("can_id", ctypes.c_uint),
        ("can_dlc", ctypes.c_ubyte),
        ("_pad", ctypes.c_ubyte),
        ("_res0", ctypes.c_ubyte),
        ("_res1", ctypes.c_ubyte),
        ("data", ctypes.c_ubyte * 8),
    ]


class _TransmitData(ctypes.Structure):
    _fields_ = [("frame", _CanFrame), ("transmit_type", ctypes.c_uint)]


class _ReceiveData(ctypes.Structure):
    _fields_ = [("frame", _CanFrame), ("timestamp", ctypes.c_ulonglong)]


def parse_channel(value: str | int) -> int:
    """Convert ``0``, ``1`` or ``can0``-style CLI values to a channel index."""

    text = str(value).strip().lower()
    if text.startswith("can"):
        text = text[3:]
    try:
        channel = int(text, 10)
    except ValueError as exc:
        raise ValueError(f"invalid Chuang Xin channel {value!r}; use 0, 1, can0 or can1") from exc
    if channel < 0:
        raise ValueError("CAN channel must be non-negative")
    return channel


def frame_from_message(message: can.Message) -> _TransmitData:
    """Translate a python-can message into the vendor's classic CAN struct."""

    if message.is_fd:
        raise can.CanError("Chuang Xin backend only sends Classical CAN frames")
    if len(message.data) > 8:
        raise can.CanError("Classical CAN payload cannot exceed 8 bytes")
    if message.arbitration_id < 0 or message.arbitration_id > CAN_ID_MASK:
        raise can.CanError("CAN identifier is outside the vendor API range")

    flags = 0
    if message.is_extended_id:
        flags |= CAN_EFF_FLAG
    if message.is_remote_frame:
        flags |= CAN_RTR_FLAG
    if message.is_error_frame:
        flags |= CAN_ERR_FLAG

    packet = _TransmitData()
    packet.transmit_type = 0  # normal transmit
    packet.frame.can_id = message.arbitration_id | flags
    packet.frame.can_dlc = len(message.data)
    for index, value in enumerate(message.data):
        packet.frame.data[index] = value
    return packet


def message_from_frame(frame: _CanFrame) -> can.Message:
    """Translate a vendor receive frame into a python-can message."""

    raw_id = int(frame.can_id)
    return can.Message(
        arbitration_id=raw_id & CAN_ID_MASK,
        is_extended_id=bool(raw_id & CAN_EFF_FLAG),
        is_remote_frame=bool(raw_id & CAN_RTR_FLAG),
        is_error_frame=bool(raw_id & CAN_ERR_FLAG),
        is_fd=False,
        data=bytes(frame.data[: min(int(frame.can_dlc), 8)]),
        check=False,
    )


def _candidate_dlls(dll_path: str | None) -> list[Path]:
    name = "ControlCANFD.dll" if os.name == "nt" else "libcontrolcanfd.so"
    candidates: list[Path] = []
    if dll_path:
        candidates.append(Path(dll_path))
    env_path = os.environ.get("CXCANFD_DLL")
    if env_path:
        candidates.append(Path(env_path))
    candidates.extend([Path.cwd() / name, Path(__file__).resolve().parent / name])
    return candidates


def resolve_dll(dll_path: str | None) -> Path:
    for candidate in _candidate_dlls(dll_path):
        if candidate.is_file():
            return candidate
    searched = ", ".join(str(path) for path in _candidate_dlls(dll_path))
    raise FileNotFoundError(
        "ControlCANFD SDK library not found. Pass --dll <ControlCANFD.dll> "
        f"or set CXCANFD_DLL. Searched: {searched}"
    )


class CxCanFdBus:
    """Small python-can-like bus wrapper around ControlCANFD.dll/.so."""

    def __init__(
        self,
        *,
        channel: str | int = 0,
        bitrate: int = 1_000_000,
        device_index: int = 0,
        dll_path: str | None = None,
    ) -> None:
        self.channel_index = parse_channel(channel)
        self.bitrate = int(bitrate)
        self._pending: Deque[can.Message] = deque()
        self._device = None
        self._channel = None
        library_path = resolve_dll(dll_path)
        loader = ctypes.WinDLL if os.name == "nt" else ctypes.CDLL
        self._dll = loader(str(library_path))
        self._bind_functions()
        try:
            self._device = self._dll.ZCAN_OpenDevice(
                USBCANFD_200U, int(device_index), 0
            )
            if not self._device:
                raise can.CanError(
                    f"ZCAN_OpenDevice failed for device index {device_index}"
                )

            self._check(self._dll.ZCAN_SetAbitBaud(self._device, self.channel_index, self.bitrate),
                        "ZCAN_SetAbitBaud")
            # The channel is initialized as FD-capable by the vendor API, but
            # all frames below use ZCAN_Transmit/ZCAN_Receive (classic CAN).
            self._check(self._dll.ZCAN_SetDbitBaud(self._device, self.channel_index, self.bitrate),
                        "ZCAN_SetDbitBaud")
            self._check(self._dll.ZCAN_SetCANFDStandard(self._device, self.channel_index, 0),
                        "ZCAN_SetCANFDStandard")

            config = _ChannelInitConfig()
            config.can_type = TYPE_CANFD
            config.config.canfd.mode = 0
            self._channel = self._dll.ZCAN_InitCAN(
                self._device, self.channel_index, ctypes.byref(config)
            )
            if not self._channel:
                raise can.CanError("ZCAN_InitCAN failed")
            self._check(self._dll.ZCAN_StartCAN(self._channel), "ZCAN_StartCAN")
        except Exception:
            self.shutdown()
            raise

    def _bind_functions(self) -> None:
        self._dll.ZCAN_OpenDevice.argtypes = (ctypes.c_uint, ctypes.c_uint, ctypes.c_uint)
        self._dll.ZCAN_OpenDevice.restype = ctypes.c_void_p
        self._dll.ZCAN_SetAbitBaud.argtypes = (ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint)
        self._dll.ZCAN_SetAbitBaud.restype = ctypes.c_uint
        self._dll.ZCAN_SetDbitBaud.argtypes = (ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint)
        self._dll.ZCAN_SetDbitBaud.restype = ctypes.c_uint
        self._dll.ZCAN_SetCANFDStandard.argtypes = (ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint)
        self._dll.ZCAN_SetCANFDStandard.restype = ctypes.c_uint
        self._dll.ZCAN_InitCAN.argtypes = (ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(_ChannelInitConfig))
        self._dll.ZCAN_InitCAN.restype = ctypes.c_void_p
        self._dll.ZCAN_StartCAN.argtypes = (ctypes.c_void_p,)
        self._dll.ZCAN_StartCAN.restype = ctypes.c_uint
        self._dll.ZCAN_ResetCAN.argtypes = (ctypes.c_void_p,)
        self._dll.ZCAN_ResetCAN.restype = ctypes.c_uint
        self._dll.ZCAN_Transmit.argtypes = (ctypes.c_void_p, ctypes.POINTER(_TransmitData), ctypes.c_uint)
        self._dll.ZCAN_Transmit.restype = ctypes.c_uint
        self._dll.ZCAN_GetReceiveNum.argtypes = (ctypes.c_void_p, ctypes.c_ubyte)
        self._dll.ZCAN_GetReceiveNum.restype = ctypes.c_uint
        self._dll.ZCAN_Receive.argtypes = (
            ctypes.c_void_p, ctypes.POINTER(_ReceiveData), ctypes.c_uint, ctypes.c_int
        )
        self._dll.ZCAN_Receive.restype = ctypes.c_uint
        self._dll.ZCAN_CloseDevice.argtypes = (ctypes.c_void_p,)
        self._dll.ZCAN_CloseDevice.restype = ctypes.c_uint

    @staticmethod
    def _check(result: int, function: str) -> None:
        if int(result) != STATUS_OK:
            raise can.CanError(f"{function} failed with return value {result}")

    def send(self, message: can.Message, timeout: float | None = None) -> None:
        packet = frame_from_message(message)
        result = self._dll.ZCAN_Transmit(self._channel, ctypes.byref(packet), 1)
        if int(result) != 1:
            raise can.CanError(f"ZCAN_Transmit sent {result} of 1 frame")

    def _poll_once(self) -> None:
        count = int(self._dll.ZCAN_GetReceiveNum(self._channel, TYPE_CAN))
        if count <= 0:
            return
        count = min(count, 256)
        packets = (_ReceiveData * count)()
        received = int(self._dll.ZCAN_Receive(self._channel, packets, count, 0))
        for index in range(max(0, received)):
            self._pending.append(message_from_frame(packets[index].frame))

    def recv(self, timeout: float | None = None) -> Optional[can.Message]:
        deadline = None if timeout is None else time.monotonic() + max(0.0, timeout)
        while True:
            if self._pending:
                return self._pending.popleft()
            self._poll_once()
            if self._pending:
                return self._pending.popleft()
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                time.sleep(min(0.002, remaining))
            else:
                time.sleep(0.002)

    def shutdown(self) -> None:
        if self._channel:
            try:
                self._dll.ZCAN_ResetCAN(self._channel)
            except Exception:
                pass
            self._channel = None
        if self._device:
            try:
                self._dll.ZCAN_CloseDevice(self._device)
            except Exception:
                pass
            self._device = None

