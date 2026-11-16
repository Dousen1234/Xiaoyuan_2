import io
import json
import unittest
from unittest.mock import patch

import can
from host_demo import claw_demo as demo
from host_demo.claw_protocol import CAN_ID_STATUS, State


class MotionTests(unittest.TestCase):
    def test_stale_hold_does_not_finish_and_heartbeat_keeps_action_sequence(self):
        class Bus:
            def __init__(self):
                self.sent = []
                self.reads = 0

            def send(self, message):
                self.sent.append(message)

            def recv(self, timeout):
                self.reads += 1
                seq = self.sent[0].data[1]
                # A stale HOLD follows a valid MOVING, too.
                state = State.MOVING if self.reads == 2 else State.HOLD
                if self.reads in (1, 3):
                    seq = (seq - 1) & 255
                return can.Message(arbitration_id=CAN_ID_STATUS, is_extended_id=False,
                                   data=bytes((1, seq, state, 0, 0, 0, 0,
                                               128 if state == State.HOLD else 0)))

        bus = Bus()
        ticks = iter(i * 0.06 for i in range(100))
        sequence = demo.Sequence()
        with patch.object(demo.time, 'monotonic', side_effect=lambda: next(ticks)):
            self.assertEqual(demo.run_motion(bus, sequence, 5, 4, False, 10), 0)
        self.assertEqual(bus.reads, 4)
        self.assertGreater(len(bus.sent), 1)
        self.assertTrue(all(m.data[1] == bus.sent[0].data[1] for m in bus.sent))

    def test_log_preserves_raw_and_decoded_feedback(self):
        class Bus:
            def recv(self, timeout):
                return can.Message(arbitration_id=CAN_ID_STATUS, is_extended_id=False,
                                   data=bytes((1, 42, 2, 0, 0, 0, 0, 128)))
        stream = io.StringIO()
        demo.LoggedBus(Bus(), stream).recv(0)
        row = json.loads(stream.getvalue())
        self.assertEqual(row['feedback']['sequence'], 42)
        self.assertEqual(row['data'], '012a020000000080')
        self.assertGreaterEqual(row['t_s'], 0)
