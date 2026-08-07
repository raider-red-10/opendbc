import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.icbm import (IntelligentCruiseButtonManagementInterface,
                                                 LX3_COUNTER_STEP, LX3_PRESS_FRAMES)

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# 0x10b arrives at 25Hz; the control loop runs at 100Hz
GENUINE_PERIOD = 4
REPEAT_GAP_FRAMES = 25  # BUTTON_REPEAT_S at 100Hz


class TestLx3PressCadence(unittest.TestCase):
  """A real press holds the button bit across 3-5 consecutive 25Hz frames of the module's own
  stream -- 80-200ms of wall time. Queueing all four frames in one 100Hz control cycle puts a
  ~1ms blip on the wire instead, immediately contradicted by the car's continuing idle stream.
  The press must be spread: one injected frame per fresh genuine frame, riding its counter."""

  def setUp(self):
    c = CAR.HYUNDAI_PALISADE_HEV_LX3
    fw = [CarParams.CarFw(ecu=e, fwVersion=v[0], address=a, subAddress=s or 0)
          for (e, a, s), v in FW_VERSIONS[c].items()]
    fp = gen_empty_fingerprint()
    fp[2][0xCB] = 24
    self.CP = CarInterface.get_params(c, fp, fw, False, True, False)
    self.CP_SP = CarInterface.get_params_sp(self.CP, c, fp, fw, False, True, False)
    self.icbm = IntelligentCruiseButtonManagementInterface(self.CP, self.CP_SP)
    self.icbm.last_button_frame = -1000  # no press earlier in the drive
    self.packer = CANPacker(DBC[c][Bus.pt])
    self.CAN = CanBus(self.CP)
    self.counter = 100
    self.CS = SimpleNamespace(
      lfa_btn_info={"CHECKSUM": 0, "COUNTER_ALT": self.counter, "ACCEL_BTN": 0,
                    "DECEL_BTN": 0, "RESUME_BTN": 0, "LFA_BTN": 0},
      lfa_btn_counter=self.counter,
      cruise_btns_alt_info={},  # stock alt-buttons path: no 0x1aa frame captured yet
      is_metric=False)

  def run_ticks(self, ticks, send_button=SendButtonState.increase, stop_after=None):
    """Drive update() like the 100Hz carcontroller does, advancing the genuine 0x10b stream
    every GENUINE_PERIOD ticks. Returns [(tick, CanData)] for every frame queued."""
    sends = []
    cc_sp = structs.CarControlSP()
    for t in range(ticks):
      if t > 0 and t % GENUINE_PERIOD == 0:
        self.counter = (self.counter + LX3_COUNTER_STEP) % 0x100
        self.CS.lfa_btn_counter = self.counter
        if self.CS.lfa_btn_info:
          self.CS.lfa_btn_info["COUNTER_ALT"] = self.counter
      btn = send_button if (stop_after is None or t < stop_after) else SendButtonState.none
      cc_sp.intelligentCruiseButtonManagement.sendButton = btn
      out = self.icbm.update(self.CS, cc_sp, self.packer, t, self.icbm.last_button_frame, self.CAN)
      sends.extend((t, msg) for msg in out)
    return sends

  def test_press_sends_the_measured_number_of_frames(self):
    sends = self.run_ticks(20)  # < repeat gap, so exactly one press
    self.assertEqual(len(sends), LX3_PRESS_FRAMES)

  def test_press_is_spread_like_a_real_one(self):
    # A real 4-frame press spans 3 message periods (~120ms). All frames in one control
    # cycle is the defect this guards against.
    sends = self.run_ticks(20)
    ticks = [t for t, _ in sends]
    self.assertEqual(len(set(ticks)), len(ticks), f"more than one frame in one cycle: {ticks}")
    self.assertGreaterEqual(ticks[-1] - ticks[0], (LX3_PRESS_FRAMES - 1) * GENUINE_PERIOD,
                            f"press collapsed in time: {ticks}")

  def test_counters_ride_the_live_stream(self):
    # Every injected frame sits one step ahead of the genuine frame it follows, across
    # presses -- the genuine counter at tick t is 100 + 2*(t//4)
    sends = self.run_ticks(45)
    self.assertGreater(len(sends), LX3_PRESS_FRAMES)  # spans at least two presses
    for t, msg in sends:
      genuine = (100 + LX3_COUNTER_STEP * (t // GENUINE_PERIOD)) % 0x100
      self.assertEqual(msg[1][2], (genuine + LX3_COUNTER_STEP) % 0x100, f"at tick {t}")

  def test_press_completes_after_request_clears(self):
    # ICBM may stop asking mid-press; a real press does not un-happen halfway through.
    sends = self.run_ticks(20, stop_after=2)
    self.assertEqual(len(sends), LX3_PRESS_FRAMES)

  def test_second_press_waits_for_the_repeat_gap(self):
    # 45 ticks fit exactly two presses; a third would start at ~2x the repeat gap
    sends = self.run_ticks(45)
    ticks = [t for t, _ in sends]
    self.assertEqual(len(sends), 2 * LX3_PRESS_FRAMES)
    self.assertGreaterEqual(ticks[LX3_PRESS_FRAMES], REPEAT_GAP_FRAMES,
                            f"second press started inside the repeat gap: {ticks}")

  def test_nothing_sent_before_a_genuine_frame_is_seen(self):
    self.CS.lfa_btn_info = {}
    self.assertEqual(self.run_ticks(20), [])

  def test_no_press_path_without_the_trait(self):
    from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
    self.icbm.CP_SP.flags &= ~HyundaiFlagsSP.BTN_CLUSTER_0X10B.value
    self.assertEqual(self.run_ticks(20), [])

  def test_frames_carry_the_requested_button_on_the_camera_bus(self):
    for btn, expected in ((SendButtonState.increase, 0x01), (SendButtonState.decrease, 0x02)):
      with self.subTest(btn=btn):
        self.setUp()
        sends = self.run_ticks(20, send_button=btn)
        for _, (_, dat, bus) in sends:
          self.assertEqual(dat[10], expected)
          self.assertEqual(bus, self.CAN.CAM)


if __name__ == "__main__":
  unittest.main()
