import unittest
from unittest.mock import patch

from host_demo import claw_demo as demo
from host_demo.claw_protocol import Opcode, State, Result, StatusFeedback, encode_jog, crc8


class JogTests(unittest.TestCase):
    def test_wire_format(self):
        packet = encode_jog(42, 1, 0)
        self.assertEqual(packet[:7], bytes((1, 42, 9, 0, 0, 0, 1)))
        self.assertEqual(packet[7], crc8(packet[:7]))
        for motor, direction in ((0, 0), (4, 0), (1, 2)):
            with self.assertRaises(ValueError):
                encode_jog(0, motor, direction)

    def test_stale_idle_does_not_finish(self):
        seq = demo.Sequence()
        seq.value = 10
        status = lambda number: (0x182, StatusFeedback(number, State.IDLE, 0, 0, 0, Result.OK, False))
        with patch.object(demo, 'monitor'), patch.object(demo, 'send') as send, \
             patch.object(demo, 'receive_feedback', side_effect=[status(9), status(11)]) as receive:
            self.assertEqual(demo.run_jog(object(), seq, 1, 0, False), 0)
            self.assertEqual(receive.call_count, 2)
            self.assertEqual(send.call_args_list[0].args[1][2], Opcode.JOG)
            self.assertEqual(send.call_args_list[-1].args[1][2], Opcode.STOP)

    def test_interrupt_stops_without_retry(self):
        with patch.object(demo, 'monitor'), patch.object(demo, 'send') as send, \
             patch.object(demo, 'receive_feedback', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                demo.run_jog(object(), demo.Sequence(), 1, 0, False)
            self.assertEqual([c.args[1][2] for c in send.call_args_list], [Opcode.JOG, Opcode.STOP])

    def test_clear_failure_aborts(self):
        with patch.object(demo, 'run_action', return_value=2), patch.object(demo, 'send') as send:
            self.assertEqual(demo.run_jog(object(), demo.Sequence(), 1, 0, True), 2)
            send.assert_not_called()
