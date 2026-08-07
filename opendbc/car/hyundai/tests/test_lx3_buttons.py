import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import DBC, CAR
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

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


class TestLx3ButtonInteraction(unittest.TestCase):
  """openpilot's engagement gate (blockPcmEnable) and panda's controls_allowed latch both key
  off 'the driver recently pressed a cruise button'. On this car the deque behind
  recent_button_interaction() was fed from CRUISE_BUTTONS_ALT (0x1aa) -- always zero -- so
  openpilot never reported itself engaged, pandad's heartbeat said 'unused', and the panda
  cleared controls_allowed 3 heartbeats after every latch. Measured on the road: ICBM worked
  for exactly ~2.5s after each cruise engagement, then every button frame was rejected."""

  def setUp(self):
    c = CAR.HYUNDAI_PALISADE_HEV_LX3
    fw = [CarParams.CarFw(ecu=e, fwVersion=v[0], address=a, subAddress=s or 0)
          for (e, a, s), v in FW_VERSIONS[c].items()]
    fp = gen_empty_fingerprint()
    fp[2][0xCB] = 24
    CP = CarInterface.get_params(c, fp, fw, False, True, False)
    self.CP_SP = CarInterface.get_params_sp(CP, c, fp, fw, False, True, False)
    self.CS = CarState(CP, self.CP_SP)
    self.counter = 0

  def test_trait_gates_on_the_flag_not_the_platform(self):
    self.assertTrue(self.CS.btn_cluster_0x10b)
    self.CP_SP.flags &= ~HyundaiFlagsSP.BTN_CLUSTER_0X10B.value
    self.assertFalse(CarState(self.CS.CP, self.CP_SP).btn_cluster_0x10b)

  def btns(self, byte10=0, fresh=True):
    if fresh:
      self.counter = (self.counter + 2) % 0x100
    return {"CHECKSUM": 0, "COUNTER_ALT": self.counter,
            "ACCEL_BTN": (byte10 >> 0) & 1, "DECEL_BTN": (byte10 >> 1) & 1,
            "RESUME_BTN": (byte10 >> 2) & 1, "BTN_OTHER": (byte10 >> 3) & 0xF,
            "LFA_BTN": (byte10 >> 7) & 1}

  def test_idle_means_no_interaction(self):
    for _ in range(10):
      self.CS._update_lx3_buttons(self.btns(0))
    self.assertFalse(self.CS.recent_button_interaction())

  def test_each_real_button_counts_as_interaction(self):
    for bit in (PLUS, MINUS, RESUME):
      with self.subTest(bit=bit):
        self.setUp()
        self.CS._update_lx3_buttons(self.btns(1 << bit))
        self.assertTrue(self.CS.recent_button_interaction())

  def test_set_toggle_counts_as_interaction(self):
    # byte10 bit 3 is the cancel/set toggle: it sets the cruise speed when there is none and
    # cancels when there is one. The driver engages cruise with it, so it must latch.
    self.CS._update_lx3_buttons(self.btns(0x08))
    self.assertTrue(self.CS.recent_button_interaction())

  def test_interaction_expires_after_idle_frames(self):
    self.CS._update_lx3_buttons(self.btns(1 << MINUS))
    for _ in range(self.CS.cruise_buttons.maxlen):
      self.CS._update_lx3_buttons(self.btns(0))
    self.assertFalse(self.CS.recent_button_interaction())

  def test_stale_frames_do_not_age_the_window(self):
    # The deque window must match panda's (8 samples of the 25Hz message, 320ms), so only a
    # fresh frame -- counter advanced -- may append. carstate runs at 100Hz and would
    # otherwise age the press out in 80ms.
    self.CS._update_lx3_buttons(self.btns(1 << MINUS))
    for _ in range(self.CS.cruise_buttons.maxlen):
      self.CS._update_lx3_buttons(self.btns(0, fresh=False))
    self.assertTrue(self.CS.recent_button_interaction())

  def test_press_produces_exactly_one_event(self):
    self.CS._update_lx3_buttons(self.btns(0))
    events = self.CS._update_lx3_buttons(self.btns(1 << PLUS))
    self.assertEqual([(e.type, e.pressed) for e in events], [(ButtonType.accelCruise, True)])

  def test_set_toggle_produces_no_button_event(self):
    # Feeding it to the interaction deque is enough. A ButtonType.cancel event would read as
    # a user disengage request the moment the driver engages with this button.
    self.CS._update_lx3_buttons(self.btns(0))
    events = self.CS._update_lx3_buttons(self.btns(0x08))
    self.assertEqual(events, [])

  def test_lfa_button_still_reported(self):
    self.CS._update_lx3_buttons(self.btns(0))
    self.CS._update_lx3_buttons(self.btns(1 << LFA))
    self.assertEqual(int(self.CS.lda_button), 1)
