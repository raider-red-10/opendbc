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
  """A driver's press holds the bit across 2-5 consecutive frames of the module's own stream
  and counts once. Our injected frames interleave with the module's continuing idle stream,
  so the SCC sees a release after every one of them and counts EVERY frame as its own press --
  measured on the road: 11 accepted pressed frames moved the set speed by 10. The press is
  therefore exactly one frame; the repeat gap in icbm.py spaces distinct presses."""

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

  def test_press_is_a_single_frame(self):
    # One injected frame = one increment at the SCC (the genuine idle stream provides the
    # release edge). Multi-frame presses each counted multiple times on the road.
    self.assertEqual(LX3_PRESS_FRAMES, 1)

  def test_counter_rides_one_step_ahead(self):
    for base in (100, 250, 254):
      with self.subTest(base=base):
        ctrs = [d[2] for d in self.burst(base=base)]
        self.assertEqual(ctrs, [(base + REAL_STEP) % 256])

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
