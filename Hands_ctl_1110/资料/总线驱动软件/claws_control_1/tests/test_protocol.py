import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host_demo"))

from claw_protocol import (  # noqa: E402
    CAN_ID_DIAGNOSTIC,
    CAN_ID_POSITION,
    CAN_ID_STATUS,
    Opcode,
    PositionFeedback,
    State,
    StatusFeedback,
    crc8,
    decode_feedback,
    encode_command,
    encode_run_in,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc8(bytes.fromhex("01010100282328")), 0x89)

    def test_encode_absolute_target(self):
        frame = encode_command(1, Opcode.SET_TARGET, target_deg=90, speed_rpm=4)
        self.assertEqual(len(frame), 8)
        self.assertEqual(frame[:7], bytes.fromhex("01010100282328"))
        self.assertEqual(frame[7], crc8(frame[:7]))

    def test_encode_single_motor_target(self):
        frame = encode_command(7, Opcode.SET_TARGET_M3,
                               target_deg=12.5, speed_rpm=6)
        self.assertEqual(frame[:7], bytes.fromhex("01070800E2043C"))
        self.assertEqual(frame[7], crc8(frame[:7]))

    def test_rejects_unreliable_low_motion_speed(self):
        with self.assertRaises(ValueError):
            encode_command(1, Opcode.SET_TARGET, target_deg=30, speed_rpm=2)

    def test_accepts_validated_speed_ceiling(self):
        frame = encode_command(1, Opcode.SET_TARGET, target_deg=30, speed_rpm=17)
        self.assertEqual(frame[6], 170)

    def test_decode_position(self):
        frame = bytes.fromhex("0102B80B70172823")
        feedback = decode_feedback(CAN_ID_POSITION, frame)
        self.assertIsInstance(feedback, PositionFeedback)
        self.assertEqual(feedback.state, State.HOLD)
        self.assertEqual(feedback.angles_deg, (30.0, 60.0, 90.0))

    def test_decode_status(self):
        frame = bytes.fromhex("010902000F000A80")
        feedback = decode_feedback(CAN_ID_STATUS, frame)
        self.assertIsInstance(feedback, StatusFeedback)
        self.assertTrue(feedback.target_reached)
        self.assertEqual(feedback.synchronization_error_deg, 0.15)
        self.assertEqual(feedback.command_age_ms, 100)

    def test_decode_diagnostic(self):
        frame = bytes((1, 1, 5, 4, 25, 30, 35, 7))
        feedback = decode_feedback(CAN_ID_DIAGNOSTIC, frame)
        self.assertEqual(feedback.hall_codes, (1, 5, 4))
        self.assertEqual(feedback.duty_permille, (100, 120, 140))

    def test_encode_run_in_all_and_single(self):
        all_frame = encode_run_in(10, 0, 1)
        one_frame = encode_run_in(11, 2, 0)
        self.assertEqual(all_frame[:7], bytes((1, 10, Opcode.RUN_IN, 1, 0, 0, 0)))
        self.assertEqual(one_frame[:7], bytes((1, 11, Opcode.RUN_IN, 0, 0, 0, 2)))
        self.assertEqual(all_frame[7], crc8(all_frame[:7]))
        self.assertEqual(one_frame[7], crc8(one_frame[:7]))


if __name__ == "__main__":
    unittest.main()
