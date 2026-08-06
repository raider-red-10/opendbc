import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.hyundai.hyundaicanfd import CanBus, create_buttons_lx3
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.icbm import LX3_COUNTER_STEP, LX3_PRESS_FRAMES

# Measured from a capture of the driver pressing +, resume and - on a 2026 Palisade:
#   +       3 frames    77-89ms   counter steps [2]
#   resume  4-5 frames  120-150ms counter steps [2]
#   -       2-4 frames  42-120ms  counter steps [2]
# and idle counter steps [2] -- the counter runs continuously, pressed or not.
REAL_FRAMES = range(2, 6)
REAL_STEP = 2

# Actual frames from that capture
REAL_IDLE = bytes.fromhex("b39aba00000000000000002000000000")
REAL_PLUS = bytes.fromhex("f5bc4e00000000000000012000000000")


class TestLx3ButtonPressShape(unittest.TestCase):
  """ICBM sent 20 frames all carrying the same counter -- 20 duplicates of one stale frame,
  which no button module produces. A real press is a short burst with the counter still
  advancing."""

  def setUp(self):
    c = CAR.HYUNDAI_PALISADE_HEV_LX3
    fw = [CarParams.CarFw(ecu=e, fwVersion=v[0], address=a, subAddress=s or 0)
          for (e, a, s), v in FW_VERSIONS[c].items()]
    fp = gen_empty_fingerprint()
    fp[2][0xCB] = 24
    self.CP = CarInterface.get_params(c, fp, fw, False, True, False)
    self.packer = CANPacker(DBC[c][Bus.pt])
    self.CAN = CanBus(self.CP)
    self.frame = {"CHECKSUM": 0, "COUNTER_ALT": 100, "ACCEL_BTN": 0,
                  "DECEL_BTN": 0, "RESUME_BTN": 0, "LFA_BTN": 0}

  def burst(self, base=100, accel=True):
    out = []
    for i in range(LX3_PRESS_FRAMES):
      _, dat, _ = create_buttons_lx3(self.packer, self.CP, self.CAN, self.frame,
                                     (base + LX3_COUNTER_STEP * (i + 1)) % 0x100,
                                     accel, not accel)
      out.append(dat)
    return out

  def test_burst_length_matches_a_real_press(self):
    self.assertIn(LX3_PRESS_FRAMES, REAL_FRAMES)

  def test_counter_advances_across_the_burst(self):
    ctrs = [d[2] for d in self.burst()]
    self.assertEqual({b - a for a, b in zip(ctrs, ctrs[1:], strict=False)}, {REAL_STEP},
                     f"counter must advance by {REAL_STEP} per frame, got {ctrs}")

  def test_counter_is_not_frozen(self):
    # The specific defect: every frame carrying the same counter
    ctrs = [d[2] for d in self.burst()]
    self.assertEqual(len(set(ctrs)), len(ctrs), "all frames share one counter value")

  def test_counter_wraps(self):
    ctrs = [d[2] for d in self.burst(base=250)]
    self.assertTrue(all(0 <= c <= 255 for c in ctrs), ctrs)
    self.assertEqual({(b - a) % 256 for a, b in zip(ctrs, ctrs[1:], strict=False)}, {REAL_STEP})

  def test_replaying_a_captured_frame_is_byte_exact(self):
    """We rebuild from parsed signals, so any byte the DBC does not describe packs as zero.
    Byte 11 is 0x20 on every real frame and was being dropped from everything we sent."""
    parser = CANParser(DBC[CAR.HYUNDAI_PALISADE_HEV_LX3][Bus.pt], [("LFA_BUTTON_ALT", 0)], 0)
    parser.update([[0, [(0x10B, REAL_IDLE, 0)]]])
    captured = dict(parser.vl["LFA_BUTTON_ALT"])
    _, rebuilt, _ = self.packer.make_can_msg("LFA_BUTTON_ALT", 0, captured)
    self.assertEqual(bytes(rebuilt), REAL_IDLE)

  def test_our_press_matches_a_real_one(self):
    # Everything but checksum and counter must be identical to a press the car really sent
    parser = CANParser(DBC[CAR.HYUNDAI_PALISADE_HEV_LX3][Bus.pt], [("LFA_BUTTON_ALT", 0)], 0)
    parser.update([[0, [(0x10B, REAL_IDLE, 0)]]])
    self.frame = dict(parser.vl["LFA_BUTTON_ALT"])
    for dat in self.burst(base=int(self.frame["COUNTER_ALT"]), accel=True):
      self.assertEqual(bytes(dat[3:]), REAL_PLUS[3:],
                       "our press differs from a real one outside checksum and counter")

  def test_only_the_requested_button_is_set(self):
    for accel, expected in ((True, 0x01), (False, 0x02)):
      with self.subTest(accel=accel):
        for dat in self.burst(accel=accel):
          self.assertEqual(dat[10], expected, "resume or LFA leaked into the press")


if __name__ == "__main__":
  unittest.main()
