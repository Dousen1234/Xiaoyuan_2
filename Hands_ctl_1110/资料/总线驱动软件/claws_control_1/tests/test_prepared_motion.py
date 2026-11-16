import unittest
from unittest.mock import patch
from host_demo import claw_demo as demo


class PreparedMotionTests(unittest.TestCase):
    def test_clear_failure_does_not_move(self):
        args = demo.build_parser().parse_args(['f1', '3', '--relative', '--clear'])
        with patch.object(demo, 'run_action', return_value=2), \
             patch.object(demo, 'run_motion') as motion:
            self.assertEqual(demo.run_prepared_motion(object(), demo.Sequence(), args), 2)
            motion.assert_not_called()

    def test_interrupt_requests_stop(self):
        args = demo.build_parser().parse_args(['f1', '3', '--relative', '--clear'])
        with patch.object(demo, 'monitor'), \
             patch.object(demo, 'run_action', return_value=0), \
             patch.object(demo, 'run_motion', side_effect=KeyboardInterrupt), \
             patch.object(demo, 'send') as send:
            with self.assertRaises(KeyboardInterrupt):
                demo.run_prepared_motion(object(), demo.Sequence(), args)
            self.assertEqual(send.call_args.args[1][2], demo.Opcode.STOP)
