import unittest

import can

from host_demo.cxcanfd_bus import (
    CAN_EFF_FLAG,
    CAN_RTR_FLAG,
    frame_from_message,
    message_from_frame,
    parse_channel,
)


class CxCanFdBusTests(unittest.TestCase):
    def test_parse_channel(self):
        self.assertEqual(parse_channel("0"), 0)
        self.assertEqual(parse_channel("can1"), 1)

    def test_parse_channel_rejects_invalid_value(self):
        with self.assertRaises(ValueError):
            parse_channel("PCAN_USBBUS1")

    def test_classic_frame_round_trip(self):
        original = can.Message(
            arbitration_id=0x180,
            is_extended_id=False,
            data=bytes.fromhex("0102030405060708"),
        )
        packet = frame_from_message(original)
        self.assertEqual(packet.frame.can_id, 0x180)
        self.assertEqual(packet.frame.can_dlc, 8)
        decoded = message_from_frame(packet.frame)
        self.assertEqual(decoded.arbitration_id, 0x180)
        self.assertEqual(bytes(decoded.data), bytes(original.data))

    def test_extended_remote_flags_round_trip(self):
        original = can.Message(
            arbitration_id=0x12345,
            is_extended_id=True,
            is_remote_frame=True,
            data=b"",
        )
        packet = frame_from_message(original)
        self.assertTrue(packet.frame.can_id & CAN_EFF_FLAG)
        self.assertTrue(packet.frame.can_id & CAN_RTR_FLAG)
        decoded = message_from_frame(packet.frame)
        self.assertTrue(decoded.is_extended_id)
        self.assertTrue(decoded.is_remote_frame)
        self.assertEqual(decoded.arbitration_id, original.arbitration_id)


if __name__ == "__main__":
    unittest.main()
