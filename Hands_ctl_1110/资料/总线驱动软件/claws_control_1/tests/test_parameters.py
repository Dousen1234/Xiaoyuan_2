import copy
import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import can
from host_demo import parameters as p
from host_demo.claw_demo import Sequence, build_parser


class Board:
    """Passive protocol fake; no physical bus is opened."""
    def __init__(self):
        self.values = {(0, 10): 300, (0, 11): 500, (0, 12): 5000, (0, 13): 2, (0, 15): 500, (0, 16): 5000}
        for ch in (1, 2, 3):
            for i, v in enumerate((120, 200 if ch == 2 else 0, 0, 150, 100, 1000, 600, 2, 200), 1):
                self.values[ch, i] = v
        for ch in (1, 2, 3): self.values[ch, 14] = 300; self.values[ch, 17] = 300; self.values[ch, 18] = 0
        self.pending, self.writes = [], []
        self.result = 0

    def send(self, message):
        data = bytes(message.data)
        assert message.arbitration_id == 11 and p.crc8(data[:7]) == data[7]
        version, seq, op, param, val, ch = struct.unpack('<BBBBhB', data[:7])
        assert op in (11, 12)  # Never motion, zero or clear.
        if op == 11 and not self.result:
            self.values[ch, param] = val
            if (ch, param) == (0, 10):
                for motor in (1, 2, 3): self.values[motor, 17] = val
            self.writes.append((ch, param, val))
            if ch:
                assert self.values[ch, 5] <= self.values[ch, 6]
                assert self.values[ch, 7] <= self.values[ch, 6]
        reply = struct.pack('<BBBBhB', version, seq, param, ch, self.values[ch, param], self.result)
        self.pending.append(can.Message(arbitration_id=15, is_extended_id=False,
                                        data=reply + bytes([p.crc8(reply)])))

    def recv(self, timeout):
        return self.pending.pop(0) if self.pending else None


class ParameterTests(unittest.TestCase):
    def args(self, *tail):
        return build_parser().parse_args(['--claw-group', '2', 'params', *tail])

    def test_exchange_filters_and_busy(self):
        b = Board()
        b.pending.append(can.Message(arbitration_id=15, data=[0]*8, is_extended_id=False))
        self.assertEqual(p.exchange(b, Sequence(), 2, 2, 105), 105)
        b.result = 1
        with self.assertRaisesRegex(ValueError, 'rejected'):
            p.exchange(b, Sequence(), 2, 2, 200)
        self.assertEqual(b.values[2, 2], 105)

    def test_readback_mismatch_and_timeout(self):
        b = Board()
        with patch.object(b, 'send'):
            packet = struct.pack('<BBBBhB', 1, 1, 2, 2, 1, 0)
            b.pending.append(can.Message(arbitration_id=15, is_extended_id=False,
                                          data=packet + bytes([p.crc8(packet)])))
            seq = Sequence(); seq.value = 0
            with self.assertRaisesRegex(ValueError, 'mismatch'):
                p.exchange(b, seq, 2, 2, 105)
        seq = Sequence()
        with patch.object(b, 'send'), patch.object(p.time, 'monotonic', side_effect=[0, 1]):
            with self.assertRaisesRegex(ValueError, 'No parameter reply'):
                p.exchange(b, seq, 2, 2)

    def test_validation(self):
        good = p.snapshot(Board(), Sequence(), 2)
        for invalid in (None, [], {}, {'format': 'wrong'}):
            with self.assertRaises(ValueError):
                p.validate(invalid, 2)
        for val in (float('nan'), True, -1, 1001, 1.23):
            with self.assertRaises(ValueError):
                p.wire_value(p.MOTOR['kp'], val)
        bad = copy.deepcopy(good); bad['motors']['1']['max_pwm'] = 500
        with self.assertRaises(ValueError):
            p.validate(bad, 2)

    def test_save_load_cross_bounds(self):
        b = Board()
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            path = str(Path(directory) / 'params.json')
            p.run(b, Sequence(), self.args('save', path))
            original = json.loads(Path(path).read_text())
            target = copy.deepcopy(original)
            target['motors']['2'].update(ki=10.5, min_pwm=50, max_pwm=400, startup_pwm=350, grip_pwm=450, seek_pwm=420, hold_pwm=80)
            Path(path).write_text(json.dumps(target))
            p.run(b, Sequence(), self.args('load', path))
            self.assertEqual(p.snapshot(b, Sequence(), 2), target)
            Path(path).write_text(json.dumps(original))
            p.run(b, Sequence(), self.args('load', path))
            self.assertEqual(p.snapshot(b, Sequence(), 2), original)
            with self.assertRaisesRegex(ValueError, 'already exists'):
                p.run(b, Sequence(), self.args('save', path))

    def test_invalid_load_no_writes_and_partial_failure(self):
        b = Board()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'params.json'
            path.write_text('{}')
            with self.assertRaises(ValueError):
                p.run(b, Sequence(), self.args('load', str(path)))
            self.assertEqual(b.writes, [])
            path.write_text(json.dumps(p.snapshot(b, Sequence(), 2)))
            original = p.exchange
            def fail(bus, seq, ch, key, value=None):
                if len(b.writes) >= 2:
                    raise ValueError('connection lost')
                return original(bus, seq, ch, key, value)
            with patch.object(p, 'exchange', side_effect=fail):
                with self.assertRaisesRegex(ValueError, 'Load incomplete'):
                    p.run(b, Sequence(), self.args('load', str(path)))

    def test_force_save_and_load_roundtrip(self):
        b = Board()
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            path = Path(directory) / 'params.json'
            path.write_text('old snapshot')
            expected = p.snapshot(b, Sequence(), 2)
            p.run(b, Sequence(), self.args('save', str(path), '--force'))
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')), expected)
            self.assertEqual(b.writes, [])
            b.values[2, 14] = 500
            p.run(b, Sequence(), self.args('load', str(path)))
            self.assertEqual(p.snapshot(b, Sequence(), 2), expected)
            self.assertEqual(list(Path(directory).glob('*.tmp')), [])

    def test_force_save_failure_preserves_existing_file(self):
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            path = Path(directory) / 'params.json'
            original = b'previous saved parameters'
            path.write_bytes(original)
            for target in ('snapshot', 'os.fsync', 'os.replace'):
                with self.subTest(target=target), patch(
                        'host_demo.parameters.' + target, side_effect=OSError('failure')):
                    with self.assertRaises(OSError):
                        p.run(Board(), Sequence(), self.args('save', str(path), '--force'))
                self.assertEqual(path.read_bytes(), original)
                self.assertEqual(list(Path(directory).glob('*.tmp')), [])

    def test_force_save_wrong_group_preserves_existing_file(self):
        b = Board(); b.values[0, 13] = 1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'params.json'
            path.write_text('previous')
            with self.assertRaisesRegex(ValueError, 'does not match'):
                p.run(b, Sequence(), self.args('save', str(path), '--force'))
            self.assertEqual(path.read_text(), 'previous')
            self.assertEqual(b.writes, [])

    def test_group_mismatch_no_write(self):
        b = Board(); b.values[0, 13] = 1
        with self.assertRaisesRegex(ValueError, 'group mismatch'):
            p.run(b, Sequence(), self.args('set', '2', 'ki', '10'))
        self.assertEqual(b.writes, [])


    def test_v3_load_preserves_values_and_disables_hold(self):
        b=Board()
        doc=p.snapshot(b, Sequence(), 2)
        doc['format']='claw-params-v3'
        for values in doc['motors'].values(): del values['hold_pwm']
        doc['motors']['2']['seek_pwm']=420
        b.values[2,18]=100
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            path=Path(directory)/'old.json';path.write_text(json.dumps(doc))
            p.run(b,Sequence(),self.args('load',str(path)))
        self.assertEqual(b.values[2,18],0)
        self.assertEqual(b.values[2,17],420)


    def test_hold_full_pwm_range(self):
        for value in (0, 301, 1000):
            self.assertEqual(p.wire_value(p.MOTOR['hold_pwm'], value), value)
        for value in (-1, 1001, 1.5):
            with self.assertRaises(ValueError):
                p.wire_value(p.MOTOR['hold_pwm'], value)

