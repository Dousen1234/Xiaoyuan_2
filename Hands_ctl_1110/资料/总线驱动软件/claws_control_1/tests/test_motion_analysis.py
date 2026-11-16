import unittest
from host_demo.analyze_motion import analyze
from host_demo import claw_demo as demo
from unittest.mock import patch


class MotionAnalysisTests(unittest.TestCase):
    def test_speed_and_last_change(self):
        rows = [{'direction': 'tx', 'id': 0x180, 't_s': 0,
                 'data': '0100010000000000'}]
        for t, angles, state in [(0, [0, 0, 0], 1), (.1, [1, 2, 0], 1),
                                  (.2, [2, 2, 0], 2), (.4, [2, 2, 0], 2)]:
            rows.append({'direction': 'rx', 'id': 0x181, 't_s': t,
                         'feedback': {'angles_deg': angles, 'state': state}})
        metrics, curve = analyze(rows)
        self.assertAlmostEqual(metrics[0]['last_change_s'], .2)
        self.assertAlmostEqual(metrics[1]['mean_deg_s'], 20)
        self.assertIsNone(metrics[2]['last_change_s'])
        self.assertEqual(curve[1][4:7], [10, 20, 0])
        rows[-1]['feedback']['state'] = 3
        with self.assertRaises(ValueError):
            analyze(rows)

    def test_group_clear_and_targets(self):
        for argv, angle, relative in [(['relative', '3'], 3, True),
                                      (['move', '1'], 1, False),
                                      (['open'], 0, False)]:
            args = demo.build_parser().parse_args(argv + ['--clear'])
            with patch.object(demo, 'run_action', return_value=0), \
                 patch.object(demo, 'monitor'), patch.object(demo, 'send'), \
                 patch.object(demo, 'run_motion', return_value=0) as motion:
                demo.run_prepared_motion(object(), demo.Sequence(), args)
                self.assertEqual(motion.call_args.args[2:],
                                 (angle, 4, relative, 20, demo.Opcode.SET_TARGET))
            with patch.object(demo, 'run_action', return_value=2), \
                 patch.object(demo, 'run_motion') as motion:
                self.assertEqual(demo.run_prepared_motion(object(), demo.Sequence(), args), 2)
                motion.assert_not_called()
