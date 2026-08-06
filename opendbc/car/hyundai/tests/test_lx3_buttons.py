import unittest

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.hyundai.values import DBC, CAR

DBC_NAME = DBC[CAR.HYUNDAI_PALISADE_HEV_LX3][Bus.pt]
LFA_BUTTON_ALT = 0x10B

# Measured on the vehicle: the whole steering wheel cluster is in byte 10
PLUS, MINUS, RESUME, LFA = 0, 1, 2, 7


def frame(byte10):
  dat = bytearray(16)
  dat[10] = byte10
  return (LFA_BUTTON_ALT, bytes(dat), 0)


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

  def test_unrelated_bits_do_not_trigger_a_button(self):
    for bit in (3, 4, 5, 6):
      with self.subTest(bit=bit):
        self.assertEqual(self.decode(1 << bit), (0, 0, 0, 0))

  def test_plus_and_resume_are_distinct(self):
    # Unlike most Hyundais these are separate buttons, so they must not share a bit
    self.assertNotEqual(self.decode(1 << PLUS), self.decode(1 << RESUME))
