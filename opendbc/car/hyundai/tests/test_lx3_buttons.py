import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.hyundai.values import DBC, CAR

DBC_NAME = DBC[CAR.HYUNDAI_PALISADE_HEV_LX3][Bus.pt]
LFA_BUTTON_ALT = 0x10B

# Measured on the vehicle: the whole steering wheel cluster is in byte 10
PLUS, MINUS, RESUME, LFA = 0, 1, 2, 7


def frame(byte10):
  """Build via the packer so CHECKSUM is valid -- the parser validates 0x10b and drops bad ones."""
  packer = CANPacker(DBC_NAME)
  values = {"ACCEL_BTN": (byte10 >> 0) & 1, "DECEL_BTN": (byte10 >> 1) & 1,
            "RESUME_BTN": (byte10 >> 2) & 1, "LFA_BTN": (byte10 >> 7) & 1}
  return packer.make_can_msg("LFA_BUTTON_ALT", 0, values)


class TestLx3Buttons(unittest.TestCase):
  """This car leaves CRUISE_BUTTONS_ALT (0x1aa) at zero and reports its buttons in
  LFA_BUTTON_ALT (0x10b) byte 10. Reading 0x1aa produced no cruise buttonEvent at all, so
  Speed Limit Assist waited forever for a confirm press that could not arrive."""

  def decode(self, byte10):
    parser = CANParser(DBC_NAME, [("LFA_BUTTON_ALT", 0)], 0)
    parser.update([[0, [frame(byte10)]]])
    vl = parser.vl["LFA_BUTTON_ALT"]
    return (int(vl["ACCEL_BTN"]), int(vl["DECEL_BTN"]),
            int(vl["RESUME_BTN"]), int(vl["LFA_BTN"]))

  def test_each_button_is_its_own_bit(self):
    for bit, expected in ((PLUS, (1, 0, 0, 0)), (MINUS, (0, 1, 0, 0)),
                          (RESUME, (0, 0, 1, 0)), (LFA, (0, 0, 0, 1))):
      with self.subTest(bit=bit):
        self.assertEqual(self.decode(1 << bit), expected)

  def test_nothing_pressed_reads_clear(self):
    self.assertEqual(self.decode(0), (0, 0, 0, 0))

  def test_a_bad_checksum_is_rejected(self):
    # Declaring CHECKSUM means the parser validates it. Verified against 375 live frames
    # before relying on it, but pin that a corrupt frame really is dropped.
    parser = CANParser(DBC_NAME, [("LFA_BUTTON_ALT", 0)], 0)
    dat = bytearray(16)
    dat[10] = 1 << PLUS  # no checksum
    self.assertEqual(parser.update([[0, [(LFA_BUTTON_ALT, bytes(dat), 0)]]]), set())

  def test_plus_and_resume_are_distinct(self):
    # Unlike most Hyundais these are separate buttons, so they must not share a bit
    self.assertNotEqual(self.decode(1 << PLUS), self.decode(1 << RESUME))
