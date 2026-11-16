import queue
import unittest
from unittest.mock import patch
from host_demo import claw_demo as d


class HoldTests(unittest.TestCase):
    def test_release_paths(self):
        for command in ('stop', 'seek', 'open'):
            commands = queue.Queue(); commands.put(command)
            with patch.object(d, 'hold_input', return_value=commands), \
                 patch.object(d, 'send') as send, patch.object(d, 'monitor'), \
                 patch.object(d, 'run_action', return_value=0) as action, \
                 patch.object(d, 'run_jog', return_value=0) as seek, \
                 patch.object(d, 'run_motion', return_value=0) as motion:
                self.assertEqual(d.run_hold(object(), d.Sequence(), 42), 0)
                self.assertEqual(send.call_args.args[1][2], d.Opcode.HEARTBEAT)
                self.assertEqual(action.call_args.args[2], d.Opcode.STOP)
                self.assertEqual(seek.called, command == 'seek')
                self.assertEqual(motion.called, command == 'open')

    def test_fault_and_missing_feedback(self):
        seq = d.Sequence()
        fault = (0x182, d.StatusFeedback(42,d.State.FAULT,128,0,0,d.Result.FAULT,False))
        with patch.object(d, 'hold_input', return_value=queue.Queue()), patch.object(d,'send'), \
             patch.object(d,'receive_feedback',return_value=fault):
            self.assertEqual(d.run_hold(object(),seq,42),3)
        with patch.object(d,'hold_input',return_value=queue.Queue()), patch.object(d,'send'), \
             patch.object(d,'receive_feedback',return_value=None), \
             patch.object(d.time,'monotonic',side_effect=[0,0,2]):
            self.assertEqual(d.run_hold(object(),seq,42),4)

    def test_hold_does_not_end_grip_until_release(self):
        feedback=(0x182,d.StatusFeedback(11,d.State.HOLD,0,0,0,d.Result.OK,False))
        seq=d.Sequence();seq.value=10
        with patch.object(d,'monitor'), patch.object(d,'send') as send, \
             patch.object(d,'receive_feedback',return_value=feedback), \
             patch.object(d,'run_hold',return_value=0) as hold:
            self.assertEqual(d.run_jog(object(),seq,0,0,False,grip=True),0)
            self.assertEqual(hold.call_args.args[2],11)
            self.assertEqual(send.call_args.args[1][2],d.Opcode.STOP)
