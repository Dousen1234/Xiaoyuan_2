import unittest
from unittest.mock import patch
from host_demo import claw_demo as demo
from host_demo.claw_protocol import encode_seek_open, crc8, Opcode, State, Result, StatusFeedback


class SeekTests(unittest.TestCase):
    def test_all_codec_and_cli(self):
        for suffix, expected in [([], None), (['1'], 1), (['2'], 2), (['3'], 3)]:
            args = demo.build_parser().parse_args(['seek-open'] + suffix + ['--clear'])
            self.assertEqual(args.motor, expected)
            self.assertTrue(args.clear)
        with patch('sys.stderr'), self.assertRaises(SystemExit) as rejected:
            demo.build_parser().parse_args(['seek-open', 'all'])
        self.assertEqual(rejected.exception.code, 2)
        packet = encode_seek_open(42, 0)
        self.assertEqual(packet[:7], bytes([1,42,10,0,0,0,0]))
        self.assertEqual(packet[7], crc8(packet[:7]))

    def test_codec(self):
        frame = encode_seek_open(22, 3)
        self.assertEqual(frame[:7], bytes([1,22,10,0,0,0,3]))
        self.assertEqual(frame[7], crc8(frame[:7]))
        with self.assertRaises(ValueError):
            encode_seek_open(0, 4)

    def test_heartbeat_sequence_and_stop(self):
        seq = demo.Sequence()
        seq.value = 10
        status = lambda n: (0x182, StatusFeedback(n, State.IDLE, 0, 0, 0, Result.OK, False))
        with patch.object(demo, 'monitor'), patch.object(demo, 'send') as send, \
             patch.object(demo, 'receive_feedback', side_effect=[status(9),status(11)]) as receive:
            self.assertEqual(demo.run_jog(object(),seq,0,1,False,seek=True),0)
            packets = [c.args[1] for c in send.call_args_list]
            self.assertEqual(receive.call_count,2)
            self.assertEqual(packets[0][2],Opcode.SEEK_OPEN)
            heartbeat = next(p for p in packets if p[2] == Opcode.HEARTBEAT)
            self.assertEqual(heartbeat[1],packets[0][1])
            self.assertEqual(packets[-1][2],Opcode.STOP)
            self.assertNotIn(Opcode.SET_ZERO,[p[2] for p in packets])

    def test_clear_failure(self):
        with patch.object(demo,'run_action',return_value=2), patch.object(demo,'send') as send:
            self.assertEqual(demo.run_jog(object(),demo.Sequence(),1,1,True,seek=True),2)
            send.assert_not_called()

    def test_interrupt(self):
        with patch.object(demo,'monitor'), patch.object(demo,'send') as send, \
             patch.object(demo,'receive_feedback',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                demo.run_jog(object(),demo.Sequence(),1,1,False,seek=True)
            self.assertEqual(send.call_args.args[1][2],Opcode.STOP)


class GripTests(unittest.TestCase):
    def test_cli_and_codec(self):
        args = demo.build_parser().parse_args(['grip', '--clear'])
        self.assertTrue(args.clear)
        for tail in (['10'], ['--speed', '4'], ['--timeout', '8']):
            with patch('sys.stderr'), self.assertRaises(SystemExit):
                demo.build_parser().parse_args(['grip'] + tail)
        frame = demo.encode_command(42, Opcode.GRIP)
        self.assertEqual(frame[:7], bytes([1,42,13,0,0,0,0]))
        self.assertEqual(frame[7], crc8(frame[:7]))
        with self.assertRaises(ValueError):
            demo.encode_command(42, Opcode.GRIP, target_deg=10)

    def test_grip_heartbeat_stop_no_target(self):
        seq = demo.Sequence(); seq.value = 10
        def status(n):
            return (0x182, StatusFeedback(n, State.IDLE, 0, 0, 0, Result.OK, False))
        with patch.object(demo, 'monitor'), patch.object(demo, 'send') as send, \
             patch.object(demo, 'receive_feedback', side_effect=[status(9),status(11)]):
            self.assertEqual(demo.run_jog(object(),seq,0,0,False,grip=True),0)
            packets = [c.args[1] for c in send.call_args_list]
            self.assertEqual(packets[0][2],Opcode.GRIP)
            self.assertTrue(any(p[2]==Opcode.HEARTBEAT and p[1]==11 for p in packets))
            self.assertEqual(packets[-1][2],Opcode.STOP)
            self.assertNotIn(Opcode.SET_TARGET,[p[2] for p in packets])

    def test_grip_interrupt_and_clear_failure(self):
        with patch.object(demo,'run_action',return_value=2), patch.object(demo,'send') as send:
            self.assertEqual(demo.run_jog(object(),demo.Sequence(),0,0,True,grip=True),2)
            send.assert_not_called()
        with patch.object(demo,'monitor'), patch.object(demo,'send') as send, \
             patch.object(demo,'receive_feedback',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                demo.run_jog(object(),demo.Sequence(),0,0,False,grip=True)
            self.assertEqual(send.call_args.args[1][2],Opcode.STOP)
