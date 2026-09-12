#!/usr/bin/env python3
from opendbc.testing import parameterized_class, parameterized
import unittest
import numpy as np

from opendbc.car.hyundai.carcontroller import ANGLE_SAFETY_BASELINE_MODEL
from opendbc.car.hyundai.values import HyundaiSafetyFlags, CAR, HyundaiFlags, CarControllerParams
from opendbc.sunnypilot.car.hyundai.values import ANGLE_STEERING_MODEL_BY_CAR, encode_angle_model_id
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed
from opendbc.safety.tests.hyundai_common import Buttons, HyundaiButtonBase, HyundaiLongitudinalBase
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm, AngleSteeringLimitsVM
from opendbc.car.hyundai.interface import CarInterface

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


def round_angle(angle_deg: float, can_offset=0):
  scaled = angle_deg / 0.1
  scaled += can_offset
  return int(scaled) * 0.1


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 0.375 * 0.03125  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 3.6 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)


class TestHyundaiCanfdTorqueSteering(TestHyundaiCanfdBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE = 270

  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2
  MIN_VALID_STEERING_RT_INTERVAL = 810000  # a ~10% buffer, can send steer up to 110Hz

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdTorqueSteering":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, 0)
    self.safety.init_tests()


class TestHyundaiCanfdAngleSteering(TestHyundaiCanfdBase, common.AngleSteeringSafetyTest):
  PLATFORMS = {str(platform): platform for platform in CAR if
               platform.config.flags & HyundaiFlags.CANFD_ANGLE_STEERING and not CarInterface.get_non_essential_params(str(platform)).dashcamOnly}

  # Angle control limits
  BASELINE_PANDA_ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    360,  # degrees (safe upper bound for command, allowing some margin)
    MAX_ANGLE_RATE=5  # comfort rate limit for angle commands, in degrees per frame.
  )

  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10
  ANGLE_SAFETY_THRESHOLD_PCT = -2.0  # Fail if difference is less than -2%

  # Hyundai uses get_max_angle_delta and get_max_angle for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 100  # Hz

  cnt_angle_cmd = 0

  def get_angle_limits(self, car_name):
    limits = CarControllerParams(CarInterface.get_non_essential_params(car_name))
    limits.ANGLE_LIMITS = self.BASELINE_PANDA_ANGLE_LIMITS
    return limits

  def get_baseline_limits(self):
    return self.get_angle_limits(ANGLE_SAFETY_BASELINE_MODEL)

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True, gain: float = 0.0):
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    values = {"ADAS_StrAnglReqVal": angle, "LKAS_ANGLE_ACTIVE": 2 if enabled else 1,
              "ADAS_ACIAnglTqRedcGainVal": gain}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"MDPS_EstStrAnglVal": angle}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _get_steer_cmd_angle_max(self, speed):
    baseline_vm = self.get_vm(ANGLE_SAFETY_BASELINE_MODEL)
    return get_max_angle_vm(max(speed, 1), baseline_vm, self.get_baseline_limits())

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdAngleSteering":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def get_vm(self, car_name):
    return VehicleModel(CarInterface.get_non_essential_params(car_name))

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    # safety_param_sp is process-global: start clean so a model ID (or noise) left by another test
    # doesn't get baked in by init, and restore it in tearDown so it doesn't leak to the next class
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    car_name = ANGLE_SAFETY_BASELINE_MODEL
    for speed in np.linspace(0, 40, 100):
      speed = round_speed(away_round(speed / 0.03125 * 3.6) * 0.03125 / 3.6)
      speed = max(speed, 1)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # at limit (safety tolerance adds 1)
        angl = get_max_angle_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name))
        max_angle = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name)), 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)), f"{angl} -- {max_angle}")

        # above limit (offset 6 to reliably exceed C float tolerance)
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name)), 6) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)), f"should_tx: {should_tx}, max_angle: {max_angle}, speed: {speed}")

  @parameterized("car_name", sorted(PLATFORMS))
  def test_lateral_jerk_limit(self, car_name):
    # car_name = ANGLE_SAFETY_BASELINE_MODEL
    angle_model_id = ANGLE_STEERING_MODEL_BY_CAR.get(car_name, 0)
    self.safety.set_current_safety_param_sp(encode_angle_model_id(angle_model_id))
    self._reset_safety_hooks()

    for speed in np.linspace(0, 40, 100):
      speed = round_speed(away_round(speed / 0.03125 * 3.6) * 0.03125 / 3.6)
      speed = max(speed, 1)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name))) * sign
        sent = self._tx(self._angle_cmd_msg(max_angle_delta, True))
        self.assertTrue(sent)

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        # TODO-SP: Why do I need to set a can_offset so high to pass the tests and why tesla only does +1? and why does it seem to differ based on the baseline?
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name)), 6) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)), vars(self.get_angle_limits(car_name)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_rt_limits(self):
    # TODO: remove and check all safety modes
    if self.LATERAL_FREQUENCY == -1:
      raise unittest.SkipTest("No real time limits")

    # Angle safety enforces real time limits by checking the message send frequency in a 250ms time window
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)  # 1.2x buffer

    for i in range(max_rt_msgs * 2):
      should_tx = i <= max_rt_msgs
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # One under RT interval should do nothing
    self.safety.set_timer(common.RT_INTERVAL - 1)
    for _ in range(5):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # Increment timer and send 1 message to reset RT window
    self.safety.set_timer(common.RT_INTERVAL)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))
    for _ in range(5):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

  def test_torque_reduction_gain(self):
    # Valid gains when enabled
    for gain in [0.0, 0.5, 1.0]:
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, gain=gain)),
                      f"gain={gain} should be allowed when enabled")

    # Reserved values (raw 251+) must fail even when enabled
    for gain in [1.004, 1.008, 1.02]:
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True, gain=gain)),
                       f"gain={gain} (reserved) should be blocked")

    # Non-zero gain when disabled must fail
    for gain in [0.004, 0.5, 1.0]:
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._angle_cmd_msg(0, False, gain=gain)),
                       f"gain={gain} should be blocked when disabled")

    # Zero gain when disabled must pass
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, gain=0.0)))

  @parameterized("car_name", sorted(PLATFORMS))
  def test_max_steering_angle_safety(self, car_name):
    """
    Test that ensures the current car's max steering angles are never more than 2%
    lower than the baseline car across all test speeds.
    """
    baseline_car = ANGLE_SAFETY_BASELINE_MODEL
    baseline_vm = self.get_vm(baseline_car)
    current_vm = self.get_vm(car_name)

    for speed in np.linspace(1, 40, 10):
      baseline_max_angle = get_max_angle_vm(speed, baseline_vm, self.get_baseline_limits())
      current_max_angle = get_max_angle_vm(speed, current_vm, self.get_angle_limits(car_name))

      # Skip if both exceed STEER_ANGLE_MAX (only_relevant_angles logic)
      if current_max_angle > self.STEER_ANGLE_MAX and baseline_max_angle > self.STEER_ANGLE_MAX:
        continue

      # Calculate percentage difference
      if baseline_max_angle != 0:
        angle_diff_pct = ((current_max_angle - baseline_max_angle) / baseline_max_angle) * 100
      else:
        angle_diff_pct = 0

      # Assert that difference is not dangerously low
      self.assertTrue(
        angle_diff_pct >= self.ANGLE_SAFETY_THRESHOLD_PCT,
        f"{car_name} max steering angle at {speed:.1f} m/s [{current_max_angle:.2f}°] is {angle_diff_pct:.2f}% " +
        f"lower than baseline {baseline_car} ({current_max_angle:.2f}° vs {baseline_max_angle:.2f}°). " +
        f"Must be >= {self.ANGLE_SAFETY_THRESHOLD_PCT}% to ensure safety." +
        f"Consider updating the baseline model to be {car_name} (which will lower the threshold for ALL models). " +
        f"Slip Factor: {repr(calc_slip_factor(current_vm))}"
      )

  @parameterized("car_name", sorted(PLATFORMS))
  def test_max_steering_angle_delta_safety(self, car_name):
    """
    Test that ensures the current car's max steering angle deltas are never more than 2%
    lower than the baseline car across all test speeds.
    """
    baseline_car = ANGLE_SAFETY_BASELINE_MODEL
    baseline_vm = self.get_vm(baseline_car)
    baseline_limits = CarControllerParams(CarInterface.get_non_essential_params(baseline_car))
    current_vm = self.get_vm(car_name)
    current_limits = CarControllerParams(CarInterface.get_non_essential_params(car_name))

    for speed in np.linspace(1, 40, 10):
      baseline_max_delta = get_max_angle_delta_vm(speed, baseline_vm, baseline_limits)
      current_max_delta = get_max_angle_delta_vm(speed, current_vm, current_limits)

      # Calculate percentage difference
      if baseline_max_delta != 0:
        delta_diff_pct = ((current_max_delta - baseline_max_delta) / baseline_max_delta) * 100
      else:
        delta_diff_pct = 0

      # Assert that difference is not dangerously low
      self.assertTrue(
        delta_diff_pct >= self.ANGLE_SAFETY_THRESHOLD_PCT,
        f"{car_name} max steering angle delta at {speed:.1f} m/s is {delta_diff_pct:.2f}% " +
        f"lower than {baseline_car} ({current_max_delta:.4f} vs {baseline_max_delta:.4f} deg/frame). " +
        f"Must be >= {self.ANGLE_SAFETY_THRESHOLD_PCT}% to ensure safety." +
        f"Consider updating the baseline model to be {car_name} (which will lower the threshold for ALL models)." +
        f"Slip Factor: {repr(calc_slip_factor(current_vm))}"
      )

  @parameterized("car_name", sorted(PLATFORMS))
  def test_valid_angle_model_extraction(self, car_name):
    """
    Ensures that the safety layer correctly extracts and assigns the expected
    angle steering model ID from the safety_param_sp bitmask for every supported platform.
    """
    angle_model_id = ANGLE_STEERING_MODEL_BY_CAR.get(car_name, 0)
    self.safety.set_current_safety_param_sp(encode_angle_model_id(angle_model_id))
    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_hyundai_angle_model_id(), angle_model_id)

  def test_out_of_bounds_angle_model_fallback(self):
    """
    Verifies that passing an unknown or out of bounds angle model ID
    via safety_param_sp safely defaults to the baseline fallback model (ID 0).
    """
    model_count = len(ANGLE_STEERING_MODEL_BY_CAR)
    self.safety.set_current_safety_param_sp(encode_angle_model_id(model_count + 1))
    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_hyundai_angle_model_id(), 0)

  def test_angle_model_bitmask_isolation(self):
    """
    Tests that bits [0:3] and [8:15] in safety_param_sp are properly ignored.
    Verifies the shift and mask logic works regardless of other flags being set.
    """
    test_id = 2  # HYUNDAI_IONIQ_5_PE

    # 0xFF0F sets every bit to 1 EXCEPT bits 4-7.
    noise_mask = 0xFF0F
    param_with_noise = encode_angle_model_id(test_id) | noise_mask

    self.safety.set_current_safety_param_sp(param_with_noise)
    self._reset_safety_hooks()

    self.assertEqual(self.safety.get_hyundai_angle_model_id(), test_id)


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdTorqueSteering):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdTorqueSteering):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEVBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringAltEVTorque(TestHyundaiCanfdLKASteeringAltEVBase, TestHyundaiCanfdTorqueSteering):

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringAltAngle(TestHyundaiCanfdAngleSteering):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()

  # Angle steering does not use torque — override inherited torque tests
  def test_steer_safety_check(self):
    pass

  def test_non_realtime_limit_up(self):
    pass

  def test_steer_req_bit(self):
    pass

  def test_steer_req_bit_frames(self):
    pass

  def test_steer_req_bit_multi_invalid(self):
    pass

  def test_steer_req_bit_realtime(self):
    pass

  def test_against_torque_driver(self):
    pass

  def test_realtime_limits(self):
    pass


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


class TestHyundaiCanfdCcncAltButtonsTx(unittest.TestCase):
  """CCNC camera-SCC cars are the one configuration allowed to send CRUISE_BUTTONS_ALT (0x1aa),
  which is what lets ICBM adjust the set speed on alt-button cars. Everywhere else 0x1aa must
  stay out of the TX list -- see TestHyundaiCanfdLFASteeringAltButtonsBase.test_button_sends."""

  PT_BUS = 0
  SCC_BUS = 2
  BUTTONS_TX_BUS = 2
  CCNC_PARAM = (HyundaiSafetyFlags.CCNC | HyundaiSafetyFlags.CANFD_ALT_BUTTONS |
                HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_ANGLE_STEERING |
                HyundaiSafetyFlags.HYBRID_GAS)

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    # The button cluster is a declared trait, not implied by CCNC -- see the trait flags design
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.CCNC_PARAM)
    self.safety.init_tests()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _button_msg(self, buttons):
    values = {"CRUISE_BUTTONS": buttons}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.BUTTONS_TX_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def test_set_and_resume_require_controls_allowed(self):
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      for btn in (Buttons.RESUME, Buttons.SET):
        with self.subTest(allowed=allowed, btn=btn):
          self.assertEqual(allowed, self._tx(self._button_msg(btn)))

  def test_cancel_requires_cruise_engaged(self):
    self.safety.set_controls_allowed(0)
    for engaged in (True, False):
      with self.subTest(engaged=engaged):
        self._rx(self._pcm_status_msg(engaged))
        self.assertEqual(engaged, self._tx(self._button_msg(Buttons.CANCEL)))

  def _lfa_btn_msg(self, accel=0, decel=0, resume=0, other=0):
    values = {"ACCEL_BTN": accel, "DECEL_BTN": decel, "RESUME_BTN": resume, "BTN_OTHER": other}
    return self.packer.make_can_msg_safety("LFA_BUTTON_ALT", self.PT_BUS, values)

  def _saturate_interaction(self):
    for _ in range(10):
      self._rx(self._lfa_btn_msg())

  def test_real_buttons_latch_controls_on_engage(self):
    # non-longitudinal latch: cruise-engaged rising edge with a recent button press
    for btn in ("accel", "decel", "resume"):
      with self.subTest(btn=btn):
        self.safety.set_controls_allowed(0)
        self._rx(self._pcm_status_msg(False))
        self._saturate_interaction()
        self._rx(self._lfa_btn_msg(**{btn: 1}))
        self._rx(self._pcm_status_msg(True))
        self.assertTrue(self.safety.get_controls_allowed())

  def test_set_toggle_latches_controls_on_engage(self):
    # byte 10 bit 3 is the cancel/set toggle: it engages cruise from nothing, measured on the
    # vehicle (255 -> current speed). Without counting it as interaction, engaging with it
    # leaves controls_allowed unlatched and every ICBM button frame is rejected.
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._saturate_interaction()
    self._rx(self._lfa_btn_msg(other=1))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_recent_button_no_latch(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._saturate_interaction()
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_no_other_button_value_is_accepted(self):
    # Only the three buttons ICBM and cancel actually use may go out, whatever the state
    self.safety.set_controls_allowed(1)
    self._rx(self._pcm_status_msg(True))
    for btn in range(8):
      if btn in (Buttons.RESUME, Buttons.SET, Buttons.CANCEL):
        continue
      with self.subTest(btn=btn):
        self.assertFalse(self._tx(self._button_msg(btn)))

  def _lfa_button_rx(self, accel=0, decel=0, resume=0):
    """The car's own button message, on the powertrain bus."""
    values = {"ACCEL_BTN": accel, "DECEL_BTN": decel, "RESUME_BTN": resume}
    return self.packer.make_can_msg_safety("LFA_BUTTON_ALT", self.PT_BUS, values)

  def test_controls_allowed_latches_from_the_real_button_message(self):
    """Engagement needs a recent user button press, and this car reports presses in 0x10b.
    Reading 0x1aa fed the tracker a permanent NONE, so controls_allowed never latched and
    every button we tried to send was rejected -- no frame content could have fixed that."""
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))

    self._rx(self._lfa_button_rx(accel=1))   # driver presses +
    self._rx(self._lfa_button_rx())          # and releases
    self._rx(self._pcm_status_msg(True))     # cruise engages on the rising edge
    self.assertTrue(self.safety.get_controls_allowed())

  def test_controls_allowed_does_not_latch_without_a_button_press(self):
    # Saturate the interaction counter with idle frames, then engage
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    for _ in range(16):
      self._rx(self._lfa_button_rx())
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_decel_button_also_counts_as_interaction(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self._lfa_button_rx(decel=1))
    self._rx(self._lfa_button_rx())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def _lfa_button_msg(self, accel=0, decel=0, resume=0, lfa=0):
    values = {"ACCEL_BTN": accel, "DECEL_BTN": decel, "RESUME_BTN": resume, "LFA_BTN": lfa}
    return self.packer.make_can_msg_safety("LFA_BUTTON_ALT", self.BUTTONS_TX_BUS, values)

  def test_lfa_button_msg_requires_controls_allowed(self):
    """0x10b is the only message the LX3 reads presses from, so it is real actuation."""
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      for kwargs in ({"accel": 1}, {"decel": 1}):
        with self.subTest(allowed=allowed, kwargs=kwargs):
          self.assertEqual(allowed, self._tx(self._lfa_button_msg(**kwargs)))

  def test_lfa_button_msg_rejects_resume_and_lfa(self):
    # Resume would pull the car out of standstill; the LFA bit would toggle the driver's
    # lane keeping. Neither is ever ours to send.
    self.safety.set_controls_allowed(1)
    for kwargs in ({"resume": 1}, {"lfa": 1}, {"accel": 1, "resume": 1}, {"decel": 1, "lfa": 1}):
      with self.subTest(kwargs=kwargs):
        self.assertFalse(self._tx(self._lfa_button_msg(**kwargs)))

  def test_lfa_button_msg_rejects_both_or_neither(self):
    # Exactly one of accel/decel -- both at once is not a press a driver could make
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lfa_button_msg(accel=1, decel=1)))
    self.assertFalse(self._tx(self._lfa_button_msg()))

  def test_lfa_button_msg_blocked_without_the_alt_buttons_flag(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.CCNC_PARAM & ~HyundaiSafetyFlags.CANFD_ALT_BUTTONS)
    self.safety.init_tests()
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lfa_button_msg(accel=1)))

  def test_blocked_without_the_alt_buttons_flag(self):
    # A CCNC car that uses the standard 0x1cf message must not be able to send 0x1aa
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.CCNC_PARAM & ~HyundaiSafetyFlags.CANFD_ALT_BUTTONS)
    self.safety.init_tests()
    self.safety.set_controls_allowed(1)
    for btn in (Buttons.RESUME, Buttons.SET):
      with self.subTest(btn=btn):
        self.assertFalse(self._tx(self._button_msg(btn)))


class TestHyundaiCanfdCcncWithoutBtnCluster(unittest.TestCase):
  """CCNC describes the cluster, not the buttons. A CCNC car without the BTN_CLUSTER_0X10B
  safety bit must keep stock alt-buttons behavior: the buttons (and the controls_allowed
  latch) come from 0x1aa, and 0x10b is neither read nor transmittable. When this was gated on
  CCNC alone, every torque CCNC car would have lost its latch."""

  PT_BUS = 0
  SCC_BUS = 2
  CCNC_PARAM = (HyundaiSafetyFlags.CCNC | HyundaiSafetyFlags.CANFD_ALT_BUTTONS |
                HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_ANGLE_STEERING |
                HyundaiSafetyFlags.HYBRID_GAS)

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.CCNC_PARAM)
    self.safety.init_tests()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def test_alt_buttons_latch_controls(self):
    # stock behavior restored: 0x1aa carries the buttons and arms the latch
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS,
                                             {"CRUISE_BUTTONS": Buttons.SET}))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_10b_buttons_do_not_latch(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self.packer.make_can_msg_safety("LFA_BUTTON_ALT", self.PT_BUS, {"DECEL_BTN": 1}))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_10b_tx_rejected_even_with_controls(self):
    self.safety.set_controls_allowed(1)
    msg = self.packer.make_can_msg_safety("LFA_BUTTON_ALT", 2, {"ACCEL_BTN": 1})
    self.assertFalse(self.safety.safety_tx_hook(msg))


if __name__ == "__main__":
  unittest.main()


class TestHyundaiCanfdCcncLongitudinal(unittest.TestCase):
  """Turning on openpilot longitudinal must not drop the CCNC accommodations. The longitudinal
  branch selected the standard RX checks regardless of CCNC, so on a CCNC car ACCELERATOR_ALT's
  +2 counter could never satisfy the +1 check -- is_msg_valid() fails continuously and clears
  controls_allowed -- and 0x10b was not checked at all, so the rx hook never ran for it and the
  button cluster (cruise buttons and the MADS button) went unread."""

  PT_BUS = 0
  PARAM = (HyundaiSafetyFlags.CCNC | HyundaiSafetyFlags.CANFD_ALT_BUTTONS |
           HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_ANGLE_STEERING |
           HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.LONG)

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.PARAM)
    self.safety.init_tests()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def test_accelerator_alt_counter_stepping_by_two_stays_valid(self):
    # Measured on the vehicle: 0x105's counter advances by 2 per frame on CCNC cars
    for i in range(1, 11):
      msg = self.packer.make_can_msg_safety("ACCELERATOR_ALT", self.PT_BUS, {"COUNTER": (2 * i) % 256})
      self.assertTrue(self.safety.safety_rx_hook(msg), f"frame {i} rejected -- counter check not skipped")

  def test_button_cluster_is_rx_checked(self):
    # The rx hook only runs for whitelisted addresses. In longitudinal mode controls_allowed
    # latches on the falling edge of SET, so a press/release proves the 0x10b branch ran.
    self.safety.set_controls_allowed(0)
    for pressed in (1, 0):
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("LFA_BUTTON_ALT", self.PT_BUS,
                                                                 {"DECEL_BTN": pressed}))
    self.assertTrue(self.safety.get_controls_allowed(), "0x10b not checked, so its rx branch never ran")
