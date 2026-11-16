"""RAM parameter access. This module never sends motion, zero or clear commands."""
import json
import math
import os
import struct
import tempfile
import time
from pathlib import Path
try:
    from .claw_protocol import crc8
except ImportError:
    from claw_protocol import crc8

# name: (wire ID, scale, minimum, maximum)
MOTOR = {
    'kp': (1, 10, 0, 1000), 'ki': (2, 10, 0, 1000), 'kd': (3, 10, 0, 1000),
    'integral_limit': (4, 1, 0, 700), 'min_pwm': (5, 1, 0, 1000),
    'max_pwm': (6, 1, 1, 1000), 'startup_pwm': (7, 1, 1, 1000),
    'seek_pwm': (17, 1, 0, 1000),
    'hold_pwm': (18, 1, 0, 1000),
    'grip_pwm': (14, 1, 0, 1000),
    'startup_counts': (8, 1, 1, 100), 'startup_ms': (9, 1, 10, 800),
}
GLOBAL = {'seek_pwm': (10, 1, 0, 1000), 'seek_stall_ms': (11, 1, 100, 2000),
          'seek_timeout_ms': (12, 1, 2500, 15000),
          'grip_stall_ms': (15, 1, 100, 2000), 'grip_timeout_ms': (16, 1, 2500, 15000)}


def wire_value(spec, value):
    _, scale, low, high = spec
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Parameter must be a finite number')
    raw = round(value * scale)
    if not low <= value <= high or abs(raw - value * scale) > 1e-6:
        raise ValueError(f'Parameter range {low}..{high}, step {1 / scale:g}')
    return raw


def exchange(bus, sequence, channel, param_id, value=None):
    # Lazy import avoids a circular dependency in the CLI.
    try:
        from .claw_demo import send
    except ImportError:
        from claw_demo import send
    seq = sequence.next()
    payload = struct.pack('<BBBBhB', 1, seq, 12 if value is None else 11,
                          param_id, 0 if value is None else value, channel)
    send(bus, payload + bytes([crc8(payload)]))
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        msg = bus.recv(0.05)
        from .claw_demo import CAN_ID_PARAMETER
        if msg is None or msg.arbitration_id != CAN_ID_PARAMETER or msg.is_extended_id or msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        if len(data) != 8 or crc8(data[:7]) != data[7]:
            continue
        if data[:4] != bytes([1, seq, param_id, channel]):
            continue
        if data[6] != 0:
            raise ValueError(f'Parameter rejected: result={data[6]} (1=busy, 3=range, 4=invalid)')
        actual = struct.unpack_from('<h', data, 4)[0]
        if value is not None and actual != value:
            raise ValueError(f'Readback mismatch: requested {value}, received {actual}')
        return actual
    raise ValueError('No parameter reply; check firmware/CAN. No automatic retry or motion sent.')


def snapshot(bus, sequence, group):
    actual_group = exchange(bus, sequence, 0, 13)
    if actual_group != group:
        raise ValueError(f'Firmware group {actual_group} does not match --claw-group {group}')
    return {'format': 'claw-params-v4', 'group': group,
            'motors': {str(ch): {name: exchange(bus, sequence, ch, spec[0]) / spec[1]
                                for name, spec in MOTOR.items()} for ch in (1, 2, 3)},
            'global': {name: exchange(bus, sequence, 0, spec[0]) / spec[1]
                       for name, spec in GLOBAL.items()}}


def validate(document, group):
    if not isinstance(document, dict):
        raise ValueError('Parameter file must be an object')
    if set(document) != {'format', 'group', 'motors', 'global'} or document['format'] != 'claw-params-v4':
        raise ValueError('Unsupported parameter file schema')
    if not isinstance(document['motors'], dict) or not isinstance(document['global'], dict):
        raise ValueError('motors and global must be objects')
    if document['group'] != group or set(document['motors']) != {'1', '2', '3'}:
        raise ValueError('Parameter file group/motors mismatch')
    for values, specs in [(document['global'], GLOBAL)] + [(v, MOTOR) for v in document['motors'].values()]:
        if not isinstance(values, dict) or set(values) != set(specs):
            raise ValueError('Parameter file has missing or unknown fields')
        for name, spec in specs.items():
            wire_value(spec, values[name])
    for p in document['motors'].values():
        if p['min_pwm'] > p['max_pwm'] or p['startup_pwm'] > p['max_pwm']:
            raise ValueError('min_pwm and startup_pwm must not exceed max_pwm')


def run(bus, sequence, args):
    if args.param_action == 'set':
        specs = GLOBAL if args.scope == 0 else MOTOR
        if args.name not in specs:
            raise ValueError('Unknown parameter for this scope; 0=global, 1..3=local motor')
        spec = specs[args.name]
        value = wire_value(spec, args.value)
        if exchange(bus, sequence, 0, 13) != args.claw_group:
            raise ValueError('Firmware group mismatch')
        actual = exchange(bus, sequence, args.scope, spec[0], value)
        print(f'{args.scope}.{args.name} = {actual / spec[1]:g} (RAM, readback verified)')
        return 0
    if args.param_action == 'load':
        document = json.loads(Path(args.file).read_text(encoding='utf-8-sig'))
        # Preserve existing calibrated v3 values, introducing no holding output.
        if isinstance(document, dict) and document.get('format') == 'claw-params-v3':
            motors = document.get('motors')
            if isinstance(motors, dict) and all(isinstance(v, dict) for v in motors.values()):
                for values in motors.values():
                    if 'hold_pwm' in values:
                        raise ValueError('v3 file must not contain hold_pwm')
                    values['hold_pwm'] = 0
                document['format'] = 'claw-params-v4'
        validate(document, args.claw_group)  # Entire file checked before the first write.
        current = snapshot(bus, sequence, args.claw_group)
        try:
            # Broadcast seek first; restore per-motor overrides afterwards.
            for name, spec in GLOBAL.items():
                exchange(bus, sequence, 0, spec[0], wire_value(spec, document["global"][name]))
            for ch in (1, 2, 3):
                before, target = current['motors'][str(ch)], document['motors'][str(ch)]
                # Preserve cross-field validity even when ceiling/startup decrease together.
                exchange(bus, sequence, ch, 6, int(max(before['max_pwm'], target['max_pwm'])))
                exchange(bus, sequence, ch, 5, int(min(before['min_pwm'], target['min_pwm'])))
                for name in ('kp', 'ki', 'kd', 'integral_limit', 'startup_pwm', 'startup_counts',
                             'startup_ms', 'min_pwm', 'max_pwm', 'grip_pwm', 'seek_pwm', 'hold_pwm'):
                    spec = MOTOR[name]
                    exchange(bus, sequence, ch, spec[0], wire_value(spec, target[name]))
            if snapshot(bus, sequence, args.claw_group) != document:
                raise ValueError('Final full readback differs from file')
        except (Exception, KeyboardInterrupt) as exc:
            raise ValueError('Load incomplete: some RAM fields may have changed. Run params show before moving.') from exc
        print('Loaded and verified all parameters in RAM; no motion or zero command sent.')
        return 0
    document = snapshot(bus, sequence, args.claw_group)
    if args.param_action == 'save':
        validate(document, args.claw_group)
        payload = json.dumps(document, ensure_ascii=False, indent=2) + '\n'
        path = Path(args.file)
        if args.force:
            # Stage beside the destination; failed reads/writes retain the old file.
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                                 dir=path.parent, prefix=path.name + '.',
                                                 suffix='.tmp', delete=False) as out:
                    temporary = Path(out.name)
                    out.write(payload)
                    out.flush()
                    os.fsync(out.fileno())
                os.replace(temporary, path)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            print(f'Saved actual board readback to {args.file}')
            return 0
        try:
            with open(args.file, 'x', encoding='utf-8') as out:
                out.write(payload)
        except FileExistsError as exc:
            raise ValueError('Parameter file already exists; use save <file> --force to replace it, or choose a new filename') from exc
        print(f'Saved actual board readback to {args.file}')
    else:
        print(json.dumps(document, indent=2))
    return 0


def add_parser(subparsers):
    parser = subparsers.add_parser('params', help='read/set/save/load RAM parameters; never moves motors')
    actions = parser.add_subparsers(dest='param_action', required=True)
    actions.add_parser('show')
    setter = actions.add_parser('set')
    setter.add_argument('scope', type=int, choices=(0, 1, 2, 3))
    setter.add_argument('name', choices=tuple(MOTOR) + tuple(GLOBAL))
    setter.add_argument('value', type=float)
    for name in ('save', 'load'):
        action = actions.add_parser(name)
        action.add_argument('file')
        if name == 'save':
            action.add_argument('--force', action='store_true',
                                help='replace an existing file after a complete board readback')
