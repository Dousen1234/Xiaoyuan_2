"""Command-line demo for commanding the STM32F103/MS4931 three-finger controller."""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
from datetime import datetime, timezone
import sys
import time
import queue
import threading

try:
    import can
except ImportError as exc:  # pragma: no cover - exercised only without dependency
    raise SystemExit("python-can is required: python -m pip install -r requirements.txt") from exc

try:
    from .claw_protocol import (
        CAN_ID_COMMAND,
        CAN_ID_DIAGNOSTIC,
        CAN_ID_FAULT,
        CAN_ID_POSITION,
        CAN_ID_PARAMETER,
        CAN_ID_STATUS,
        Opcode,
        PositionFeedback,
        Result,
        State,
        StatusFeedback,
        decode_feedback,
        encode_command,
        encode_jog,
        encode_run_in,
        encode_seek_open,
        encode_set_can_id,
    )
    from .cxcanfd_bus import CxCanFdBus
except ImportError:
    from claw_protocol import (  # type: ignore
        CAN_ID_COMMAND,
        CAN_ID_DIAGNOSTIC,
        CAN_ID_FAULT,
        CAN_ID_POSITION,
        CAN_ID_PARAMETER,
        CAN_ID_STATUS,
        Opcode,
        PositionFeedback,
        Result,
        State,
        StatusFeedback,
        decode_feedback,
        encode_command,
        encode_jog,
        encode_run_in,
        encode_seek_open,
        encode_set_can_id,
    )
    from cxcanfd_bus import CxCanFdBus  # type: ignore

FEEDBACK_IDS = {CAN_ID_FAULT, CAN_ID_POSITION, CAN_ID_STATUS, CAN_ID_DIAGNOSTIC}
CAN_BITRATE = 1_000_000
HEARTBEAT_PERIOD_SECONDS = 0.2


def configure_can_base(base: int) -> None:
    if not 11 <= base <= 120:
        raise ValueError('CAN ID base must be 11..120')
    global CAN_ID_COMMAND, CAN_ID_POSITION, CAN_ID_STATUS, CAN_ID_DIAGNOSTIC, CAN_ID_PARAMETER, CAN_ID_FAULT, FEEDBACK_IDS
    CAN_ID_COMMAND = base
    CAN_ID_POSITION = base + 1
    CAN_ID_STATUS = base + 2
    CAN_ID_DIAGNOSTIC = base + 3
    CAN_ID_PARAMETER = base + 4
    CAN_ID_FAULT = base + 5
    FEEDBACK_IDS = {CAN_ID_FAULT, CAN_ID_POSITION, CAN_ID_STATUS, CAN_ID_DIAGNOSTIC}


class Sequence:
    def __init__(self) -> None:
        self.value = int(time.monotonic() * 1000) & 0xFF

    def next(self) -> int:
        self.value = (self.value + 1) & 0xFF
        return self.value


def send(bus, payload: bytes) -> None:
    bus.send(can.Message(arbitration_id=CAN_ID_COMMAND, is_extended_id=False, data=payload))


def print_feedback(identifier: int, feedback) -> None:
    if isinstance(feedback, PositionFeedback):
        a1, a2, a3 = feedback.angles_deg
        print(f"position state={feedback.state.name:<6} deg=[{a1:7.2f}, {a2:7.2f}, {a3:7.2f}]")
    elif isinstance(feedback, StatusFeedback):
        prefix = "FAULT" if identifier == CAN_ID_FAULT else "status"
        print(
            f"{prefix} state={feedback.state.name:<6} result={feedback.result.name:<7} "
            f"fault=0x{feedback.fault_mask:02X} sync={feedback.synchronization_error_deg:.2f}deg "
            f"cmd_age={feedback.command_age_ms}ms"
        )
    else:
        print(
            f"diagnostic hall={feedback.hall_codes} duty_permille={feedback.duty_permille} "
            f"counter={feedback.counter}"
        )


def receive_feedback(bus, timeout: float):
    message = bus.recv(timeout)
    if message is None or message.is_extended_id or message.arbitration_id not in FEEDBACK_IDS:
        return None
    try:
        return message.arbitration_id, decode_feedback(message.arbitration_id, bytes(message.data))
    except (ValueError, TypeError):
        return None


def run_motion(bus, sequence: Sequence, target_deg: float, speed_rpm: float,
               relative: bool, timeout: float,
               opcode: Opcode = Opcode.SET_TARGET) -> int:
    action_sequence = sequence.next()
    send(bus, encode_command(action_sequence, opcode,
                             target_deg=target_deg, speed_rpm=speed_rpm,
                             relative=relative))
    deadline = time.monotonic() + timeout
    next_heartbeat = time.monotonic() + HEARTBEAT_PERIOD_SECONDS
    last_position_print = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send(bus, encode_command(action_sequence, Opcode.HEARTBEAT))
            next_heartbeat = now + HEARTBEAT_PERIOD_SECONDS

        received = receive_feedback(bus, min(0.05, max(0.0, next_heartbeat - now)))
        if received is None:
            continue
        identifier, feedback = received
        if isinstance(feedback, PositionFeedback):
            if now - last_position_print >= 0.2:
                print_feedback(identifier, feedback)
                last_position_print = now
            continue
        if not isinstance(feedback, StatusFeedback):
            continue
        # Heartbeats reuse this action sequence, so every status is checked.
        if feedback.sequence != action_sequence:
            continue
        print_feedback(identifier, feedback)
        if feedback.result is not Result.OK:
            return 2
        if feedback.state is State.FAULT or feedback.fault_mask:
            return 3
        if feedback.state is State.HOLD and feedback.target_reached:
            return 0

    print("motion timed out; sending STOP", file=sys.stderr)
    send(bus, encode_command(sequence.next(), Opcode.STOP))
    return 4


def run_prepared_motion(bus, sequence, args) -> int:
    result = run_action(bus, sequence, Opcode.CLEAR_FAULT)
    if result:
        return result
    monitor(bus, 0.4)
    try:
        result = run_motion(bus, sequence, args.angle, args.speed,
                            args.relative, args.timeout, args.motor_opcode)
        if result == 0:
            monitor(bus, 0.5)  # Capture coast after PWM reaches zero.
        return result
    finally:
        send(bus, encode_command(sequence.next(), Opcode.STOP))


def run_action(bus, sequence: Sequence, opcode: Opcode, wait_seconds: float = 0.4) -> int:
    action_sequence = sequence.next()
    send(bus, encode_command(action_sequence, opcode))
    deadline = time.monotonic() + wait_seconds
    while time.monotonic() < deadline:
        received = receive_feedback(bus, 0.1)
        if received is not None and isinstance(received[1], StatusFeedback):
            feedback = received[1]
            if feedback.sequence != action_sequence:
                continue
            print_feedback(*received)
            return 0 if feedback.result is Result.OK else 2
    print("no status feedback received", file=sys.stderr)
    return 4


def hold_input():
    """Read one release command without blocking CAN heartbeats."""
    commands = queue.Queue()
    def reader():
        try:
            while True:
                command = input().strip().lower()
                if command in ('seek', 'seek-open', 'open', 'stop'):
                    commands.put(command)
                    return
                print('Enter seek, open or stop; Ctrl+C also stops.')
        except (EOFError, OSError):
            commands.put('stop')
    threading.Thread(target=reader, daemon=True).start()
    return commands


def run_hold(bus, sequence, action_sequence):
    print('PWM HOLD (not force control). Keep this terminal open. Enter seek/open/stop, or Ctrl+C.')
    commands = hold_input()
    next_heartbeat = 0.0
    feedback_deadline = time.monotonic() + 1.0
    while True:
        now = time.monotonic()
        if now >= next_heartbeat:
            send(bus, encode_command(action_sequence, Opcode.HEARTBEAT))
            next_heartbeat = now + HEARTBEAT_PERIOD_SECONDS
        try:
            command = commands.get_nowait()
        except queue.Empty:
            command = None
        if command:
            # Release first and verify STOP before changing direction.
            result = run_action(bus, sequence, Opcode.STOP)
            if result or command == 'stop':
                return result
            monitor(bus, 0.4)
            if command in ('seek', 'seek-open'):
                return run_jog(bus, sequence, 0, 1, False, seek=True)
            return run_motion(bus, sequence, 0.0, 4.0, False, 20.0)
        received = receive_feedback(bus, 0.05)
        if received is not None and isinstance(received[1], StatusFeedback):
            status = received[1]
            if status.state is State.FAULT or status.fault_mask:
                print_feedback(*received)
                return 3
            if status.sequence == action_sequence:
                if status.result is not Result.OK or status.state is not State.HOLD:
                    print('Hold ended or was rejected; stopping.', file=sys.stderr)
                    return 2
                feedback_deadline = time.monotonic() + 1.0
        if time.monotonic() >= feedback_deadline:
            print('Hold feedback lost; stopping.', file=sys.stderr)
            return 4


def run_jog(bus, sequence: Sequence, motor: int, direction: int, clear: bool, seek: bool = False, grip: bool = False) -> int:
    if clear:
        result = run_action(bus, sequence, Opcode.CLEAR_FAULT)
        if result:
            print("Clear failed; no motion sent. Check CAN status before retrying.", file=sys.stderr)
            return result
    # Observe a quiet interval on the same connection before requesting motion.
    monitor(bus, 0.4)
    action_sequence = sequence.next()
    try:
        send(bus, encode_command(action_sequence, Opcode.GRIP) if grip else
             encode_seek_open(action_sequence, motor) if seek else
             encode_jog(action_sequence, motor, direction))
        deadline = time.monotonic() + (16.0 if seek or grip else 2.0)
        next_heartbeat = time.monotonic()
        while time.monotonic() < deadline:
            if (seek or grip) and time.monotonic() >= next_heartbeat:
                send(bus, encode_command(action_sequence, Opcode.HEARTBEAT))
                next_heartbeat = time.monotonic() + HEARTBEAT_PERIOD_SECONDS
            received = receive_feedback(bus, 0.1)
            if received is None:
                continue
            print_feedback(*received)
            feedback = received[1]
            if not isinstance(feedback, StatusFeedback):
                continue
            if feedback.state is State.FAULT or feedback.fault_mask:
                return 3
            if feedback.sequence != action_sequence:
                continue
            if feedback.result is not Result.OK:
                return 2
            if grip and feedback.state is State.HOLD:
                return run_hold(bus, sequence, action_sequence)
            if feedback.state is State.IDLE:
                if grip:
                    print("No Hall progress: all outputs stopped. This does not confirm grasp or holding force.")
                elif seek:
                    print("No Hall progress: output stopped. Inspect the physical endpoint; zero was NOT set.")
                monitor(bus, 0.3)
                return 0
        print("motion status timed out; no retry", file=sys.stderr)
        return 4
    finally:
        # Also stop on Ctrl+C or a receive error; firmware has its own deadline.
        send(bus, encode_command(sequence.next(), Opcode.STOP))


def run_run_in(bus, sequence: Sequence, motor: int, direction: int,
               duration_minutes: float = 0.0) -> int:
    if not math.isfinite(duration_minutes) or duration_minutes < 0.0:
        raise ValueError('run-in duration must be a finite number >= 0 minutes')
    duration_seconds = duration_minutes * 60.0
    if duration_seconds > 0.0:
        print(f'RUN_IN at 1000 permille for {duration_minutes:g} minutes. Enter stop or press Ctrl+C to stop.')
    else:
        print('RUN_IN at 1000 permille. Enter stop or press Ctrl+C to stop.')
    action_sequence = sequence.next()
    send(bus, encode_run_in(action_sequence, motor, direction))
    next_heartbeat = time.monotonic()
    feedback_deadline = time.monotonic() + 1.0
    run_in_deadline = (time.monotonic() + duration_seconds
                       if duration_seconds > 0.0 else None)
    commands = hold_input()
    try:
        while True:
            now = time.monotonic()
            if run_in_deadline is not None and now >= run_in_deadline:
                print('run-in duration reached; sending STOP')
                return 0
            if now >= next_heartbeat:
                send(bus, encode_command(action_sequence, Opcode.HEARTBEAT))
                next_heartbeat = now + HEARTBEAT_PERIOD_SECONDS
            try:
                command = commands.get_nowait()
            except queue.Empty:
                command = None
            if command:
                return run_action(bus, sequence, Opcode.STOP)
            received = receive_feedback(bus, 0.05)
            if received is not None:
                print_feedback(*received)
                feedback = received[1]
                if isinstance(feedback, StatusFeedback) and feedback.sequence == action_sequence:
                    feedback_deadline = time.monotonic() + 1.0
                    if feedback.result is not Result.OK:
                        return 2
                    if feedback.state is State.FAULT or feedback.fault_mask:
                        return 3
            if time.monotonic() >= feedback_deadline:
                print('run-in feedback lost; sending STOP', file=sys.stderr)
                return run_action(bus, sequence, Opcode.STOP)
    finally:
        send(bus, encode_command(sequence.next(), Opcode.STOP))


def monitor(bus, duration: float) -> int:
    deadline = None if duration <= 0 else time.monotonic() + duration
    while deadline is None or time.monotonic() < deadline:
        received = receive_feedback(bus, 1.0)
        if received is not None:
            print_feedback(*received)
    return 0


def open_bus(args):
    """Open either a python-can backend or the Chuang Xin vendor backend."""

    if args.interface.lower() in {"cxcanfd", "chuangxin", "controlcanfd"}:
        return CxCanFdBus(
            channel=args.channel,
            bitrate=args.bitrate,
            device_index=args.device_index,
            dll_path=args.dll,
        )
    return can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)


class LoggedBus:
    """Record every raw frame before console throttling; no invented alignment."""
    def __init__(self, bus, stream):
        self.bus, self.stream = bus, stream
        self.started = time.monotonic()

    def record(self, direction, message):
        row = {"t_s": time.monotonic() - self.started, "direction": direction,
               "id": message.arbitration_id, "extended": message.is_extended_id,
               "data": bytes(message.data).hex()}
        if direction == "rx" and not message.is_extended_id and message.arbitration_id in FEEDBACK_IDS:
            try:
                row["feedback"] = dataclasses.asdict(decode_feedback(message.arbitration_id, bytes(message.data)))
            except (ValueError, TypeError):
                pass
        self.stream.write(json.dumps(row) + "\n")
        self.stream.flush()

    def send(self, message):
        self.bus.send(message)
        self.record("tx", message)

    def recv(self, timeout):
        message = self.bus.recv(timeout)
        if message is not None:
            self.record("rx", message)
        return message

    def shutdown(self):
        self.bus.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="socketcan",
                        help="python-can interface or cxcanfd for Chuang Xin USBCANFD")
    parser.add_argument("--channel", default="can0",
                        help="CAN channel, e.g. can0, 0 or PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE)
    parser.add_argument("--dll", default=None,
                        help="ControlCANFD.dll/.so path for --interface cxcanfd; or set CXCANFD_DLL")
    parser.add_argument("--device-index", type=int, default=0,
                        help="Chuang Xin USB device index (default: 0)")
    parser.add_argument("--log", help="new JSONL log path (refuses overwrite)")
    parser.add_argument("--claw-group", type=int, choices=(1, 2, 3, 4), default=1,
                        help="log label; params also checks flashed CLAW_PROFILE_GROUP")
    parser.add_argument("--can-base", type=int, default=11,
                        help="claw CAN ID base; IDs are base..base+5")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for name, help_text in (("move", "move to a logical angle"),
                            ("relative", "move relative to the current mean angle")):
        command = subparsers.add_parser(name, help=help_text)
        command.set_defaults(relative=(name == "relative"), motor_opcode=Opcode.SET_TARGET)
        command.add_argument("--clear", action="store_true", help="clear fault before motion on the same connection")
        command.add_argument("angle", type=float, help="final worm-output angle in degrees")
        command.add_argument("--speed", type=float, default=4.0,
                             help="nominal Hall trajectory rate in rpm; not measured speed")
        command.add_argument("--timeout", type=float, default=20.0)

    for name, motor, help_text in (
        ("f1", 1, "move motor 1 only"),
        ("f2", 2, "move motor 2 only"),
        ("f3", 3, "move motor 3 only"),
    ):
        command = subparsers.add_parser(name, help=help_text)
        command.set_defaults(motor_opcode=Opcode(motor + 5))
        command.add_argument("--clear", action="store_true",
                             help="clear fault, observe feedback for 0.4s, then move (no countdown)")
        command.add_argument("angle", type=float,
                             help="absolute logical angle in degrees")
        command.add_argument("--relative", action="store_true",
                             help="treat angle as an increment from this motor's current logical angle")
        command.add_argument("--speed", type=float, default=4.0)
        command.add_argument("--timeout", type=float, default=20.0)

    command = subparsers.add_parser("motor", help="move one selected motor")
    command.add_argument("motor", type=int, choices=(1, 2, 3))
    command.add_argument("angle", type=float,
                         help="absolute logical angle in degrees")
    command.add_argument("--relative", action="store_true",
                         help="treat angle as an increment from the selected motor's current logical angle")
    command.add_argument("--speed", type=float, default=4.0)
    command.add_argument("--timeout", type=float, default=20.0)

    command = subparsers.add_parser(
        "open",
        help="open the claw to the logical 0-degree limit",
    )
    command.set_defaults(angle=0.0, relative=False, motor_opcode=Opcode.SET_TARGET)
    command.add_argument("--clear", action="store_true")
    command.add_argument("--speed", type=float, default=4.0)
    command.add_argument("--timeout", type=float, default=20.0)

    command = subparsers.add_parser("grip", help="close at per-motor PWM until independent no-Hall-progress stops")
    command.add_argument("--clear", action="store_true")
    command = subparsers.add_parser("seek-open", help="open selected motors at configured per-motor PWM; no-progress stops; no auto-zero")
    command.add_argument("motor", nargs="?", type=int, default=None, choices=(1, 2, 3),
                         help="omit for all three; 1/2/3 selects one local motor")
    command.add_argument("--clear", action="store_true")
    subparsers.add_parser("stop")
    subparsers.add_parser("zero")
    subparsers.add_parser("clear")
    command = subparsers.add_parser("jog", help="single fixed 20%% / 200ms commissioning pulse")
    command.add_argument("motor", type=int, choices=(1, 2, 3))
    command.add_argument("--dir", type=int, choices=(0, 1), required=True,
                         help="raw DIR level, not yet calibrated to open/close")
    command.add_argument("--clear", action="store_true", help="explicitly clear fault before pulse")
    command = subparsers.add_parser("run-in", help="continuous 1000 permille run-in until stop or heartbeat loss")
    command.add_argument("motor", nargs="?", default="all", choices=("all", "1", "2", "3"))
    command.add_argument("--dir", type=int, choices=(0, 1), required=True,
                         help="raw DIR level")
    command.add_argument("--clear", action="store_true",
                         help="clear a latched fault on this same CAN connection before starting")
    command.add_argument("--minutes", type=float, default=0.0,
                         help="run duration in minutes; 0 means continuous until stop/Ctrl+C")
    command.add_argument("--confirm", action="store_true", help="required explicit confirmation")
    command = subparsers.add_parser("id-set", help="change and save the claw CAN ID base")
    command.add_argument("base", type=int, choices=range(11, 121))
    command = subparsers.add_parser("monitor")
    command.add_argument("--duration", type=float, default=0.0,
                         help="seconds; 0 means run until interrupted")
    try:
        from .parameters import add_parser
    except ImportError:
        from parameters import add_parser
    add_parser(subparsers)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    sequence = Sequence()
    bus = None
    log_file = None
    try:
        configure_can_base(args.can_base)
        if args.log:
            log_file = open(args.log, "x", encoding="utf-8")
            log_file.write(json.dumps({"event": "session", "utc": datetime.now(timezone.utc).isoformat(),
                                       "arguments": vars(args)}, ensure_ascii=False) + "\n")
        bus = open_bus(args)
        if log_file is not None:
            bus = LoggedBus(bus, log_file)
        if args.command == "params":
            try:
                from .parameters import run
            except ImportError:
                from parameters import run
            return run(bus, sequence, args)
        if args.command in {"move", "relative", "open"} and args.clear:
            return run_prepared_motion(bus, sequence, args)
        if args.command == "move":
            return run_motion(bus, sequence, args.angle, args.speed, False, args.timeout)
        if args.command == "relative":
            return run_motion(bus, sequence, args.angle, args.speed, True, args.timeout)
        if args.command in {"f1", "f2", "f3"}:
            if args.clear:
                return run_prepared_motion(bus, sequence, args)
            return run_motion(bus, sequence, args.angle, args.speed,
                              args.relative, args.timeout, args.motor_opcode)
        if args.command == "motor":
            opcode = (Opcode.SET_TARGET_M1, Opcode.SET_TARGET_M2,
                      Opcode.SET_TARGET_M3)[args.motor - 1]
            return run_motion(bus, sequence, args.angle, args.speed,
                              args.relative, args.timeout, opcode)
        if args.command == "open":
            return run_motion(bus, sequence, 0.0, args.speed, False, args.timeout)
        if args.command == "grip":
            return run_jog(bus, sequence, 0, 0, args.clear, grip=True)
        if args.command == "stop":
            return run_action(bus, sequence, Opcode.STOP)
        if args.command == "zero":
            return run_action(bus, sequence, Opcode.SET_ZERO)
        if args.command == "clear":
            return run_action(bus, sequence, Opcode.CLEAR_FAULT)
        if args.command == "seek-open":
            return run_jog(bus, sequence, 0 if args.motor is None else args.motor,
                           1, args.clear, seek=True)
        if args.command == "jog":
            return run_jog(bus, sequence, args.motor, args.dir, args.clear)
        if args.command == "run-in":
            if not args.confirm:
                print('run-in requires --confirm; no motion sent', file=sys.stderr)
                return 2
            if args.clear:
                result = run_action(bus, sequence, Opcode.CLEAR_FAULT)
                if result:
                    print('Clear failed; no run-in motion sent.', file=sys.stderr)
                    return result
                # Keep the CAN node open while the board emits telemetry. Closing
                # the clear process first can leave those frames unacknowledged and
                # make the firmware latch CLAW_FAULT_COMM_TIMEOUT.
                monitor(bus, 0.4)
            return run_run_in(bus, sequence, 0 if args.motor == 'all' else int(args.motor),
                              args.dir, args.minutes)
        if args.command == "id-set":
            action_sequence = sequence.next()
            send(bus, encode_set_can_id(action_sequence, args.base, True))
            print(f'CAN ID base set request sent: {args.base}..{args.base + 5}; reconnect using the new base.')
            return 0
        return monitor(bus, args.duration)
    except KeyboardInterrupt:
        return 130
    except FileExistsError as exc:
        print(f"Log already exists: {exc.filename}; use a new log name. No CAN connection opened.",
              file=sys.stderr)
        return 1
    except (can.CanError, OSError, ValueError) as exc:
        print(f"CAN error: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus is not None:
            bus.shutdown()
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
