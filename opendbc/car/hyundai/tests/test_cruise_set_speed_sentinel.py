import unittest

from opendbc.can import CANParser, CANPacker
from opendbc.car import Bus
from opendbc.car.hyundai.values import DBC, CAR

DBC_NAME = DBC[CAR.HYUNDAI_PALISADE_HEV_LX3][Bus.pt]

MPH_TO_MS = 0.44704
MS_TO_MPH = 1 / MPH_TO_MS
MS_TO_KPH = 3.6
KPH_TO_MS = 1 / MS_TO_KPH
V_CRUISE_MAX_KPH = 145  # openpilot/selfdrive/car/cruise.py


class TestCruiseSetSpeedSentinel(unittest.TestCase):
  """VSetDis = 255 means the SCC has no set speed. Reported as a real speed it becomes 255 mph,
  the planner clamps it to V_CRUISE_MAX, and ICBM presses buttons to drag the car to 90 mph --
  measured on a 2026 Palisade with cruise off."""

  def decode(self, v_set_dis):
    packer = CANPacker(DBC_NAME)
    parser = CANParser(DBC_NAME, [("SCC_CONTROL", 0)], 0)
    addr, dat, bus = packer.make_can_msg("SCC_CONTROL", 0, {"VSetDis": v_set_dis})
    parser.update([[0, [(addr, dat, bus)]]])
    return parser.vl["SCC_CONTROL"]["VSetDis"]

  def normalise(self, v_set_dis):
    """The carstate rule under test."""
    return 0. if v_set_dis >= 255 else v_set_dis * MPH_TO_MS

  def test_sentinel_is_not_a_speed(self):
    self.assertEqual(self.normalise(self.decode(255)), 0.)

  def test_the_sentinel_would_otherwise_exceed_max(self):
    # 255 mph is 410 kph; V_CRUISE_MAX is 145, which is the 90 mph that was observed
    naive_kph = 255 * MPH_TO_MS * MS_TO_KPH
    self.assertGreater(naive_kph, V_CRUISE_MAX_KPH)
    self.assertAlmostEqual(V_CRUISE_MAX_KPH * KPH_TO_MS * MS_TO_MPH, 90, delta=0.5)

  def test_real_set_speeds_pass_through(self):
    for mph in (20, 35, 55, 75, 90, 254):
      with self.subTest(mph=mph):
        got = self.normalise(self.decode(mph)) * MS_TO_MPH
        self.assertAlmostEqual(got, mph, delta=0.6)

  def test_zero_stays_zero(self):
    self.assertEqual(self.normalise(self.decode(0)), 0.)


if __name__ == "__main__":
  unittest.main()
