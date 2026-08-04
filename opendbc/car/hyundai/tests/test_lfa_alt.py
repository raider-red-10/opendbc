import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.hyundai.hyundaicanfd import CanBus, create_steering_messages
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams

# CAN addresses of the mutually exclusive steering paths
LKAS_ADDR = 0x50
LKAS_ALT_ADDR = 0x110
LFA_ADDR = 0x12A
LFA_ALT_ADDR = 0xCB


def build_params(candidate, cam_msgs):
  """Resolve real CarParams for a platform, emulating which messages the camera sends."""
  car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
            for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]
  fp = gen_empty_fingerprint()
  for m in cam_msgs:
    fp[2][m] = 16
  return CarInterface.get_params(candidate, fp, car_fw, False, True, False)


def steering_addrs(CP):
  """Addresses create_steering_messages() actually emits for this platform."""
  packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
  CAN = CanBus(CP)
  msgs = create_steering_messages(packer, CP, CAN, True, True, 0, 0.0, 0)
  return {m[0] for m in msgs}


class TestLfaAlt(unittest.TestCase):
  """LFA_ALT (0xCB) is the HDA1 angle-steering path. It must be unreachable on
  LKA-steering (HDA2) cars -- create_steering_messages() checks
  CANFD_LKA_STEER_MSG first, so an HDA2 car takes the LKAS branch and exits
  before the LFA_ALT branch is ever evaluated.
  """

  def test_hda2_car_does_not_use_lfa_alt(self):
    # Sorento sends LKAS on the camera bus => LKA steering (HDA2)
    CP = build_params(CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2, [LKAS_ADDR])

    assert CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG, "Sorento must resolve as LKA steering"
    assert not (CP.flags & HyundaiFlags.CCNC), "Sorento must not carry the CCNC flag"

    addrs = steering_addrs(CP)
    assert LKAS_ADDR in addrs, f"expected LKAS on the Sorento, got {[hex(a) for a in addrs]}"
    assert LFA_ALT_ADDR not in addrs, (
      f"LFA_ALT (0xCB) emitted on an HDA2 car -- the LKA branch must win. Got {[hex(a) for a in addrs]}")

  def test_hda2_alt_car_does_not_use_lfa_alt(self):
    # Same platform fingerprinted with LKAS_ALT on the camera bus
    CP = build_params(CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2, [LKAS_ALT_ADDR])

    assert CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT

    addrs = steering_addrs(CP)
    assert LKAS_ALT_ADDR in addrs, f"expected LKAS_ALT, got {[hex(a) for a in addrs]}"
    assert LFA_ALT_ADDR not in addrs, (
      f"LFA_ALT (0xCB) emitted on an HDA2 car. Got {[hex(a) for a in addrs]}")


class TestPalisadeLx3(unittest.TestCase):
  """End-to-end chain for the first HDA1 + angle-steering car on this branch:
  no LKAS on the camera bus => LFA steering; 0xCB present => SEND_LFA;
  SEND_LFA + CANFD_ANGLE_STEERING => LFA_ALT emitted; CCNC + not LKA => CCNC
  safety param.
  """

  def _lx3_params(self):
    # LX3 camera bus carries LFA_ALT (0xCB) and no LKAS; 0x1cf present on ECAN
    # so CANFD_ALT_BUTTONS is not set (matching the source port's final state).
    candidate = CAR.HYUNDAI_PALISADE_HEV_LX3
    car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
              for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]
    fp = gen_empty_fingerprint()
    fp[2][LFA_ALT_ADDR] = 24
    fp[0][0x1cf] = 8
    return CarInterface.get_params(candidate, fp, car_fw, False, True, False)

  def test_resolves_as_hda1_ccnc(self):
    CP = self._lx3_params()
    assert CP.flags & HyundaiFlags.CCNC, "LX3 must carry CCNC"
    assert CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING, "LX3 must be angle steering"
    assert not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG), "LX3 must NOT be LKA steering"

  def test_send_lfa_detected_from_0xcb(self):
    CP = self._lx3_params()
    assert CP.flags & HyundaiFlags.SEND_LFA, "0xCB on the camera bus must set SEND_LFA"

  def test_ccnc_safety_param_set(self):
    from opendbc.car.hyundai.values import HyundaiSafetyFlags
    CP = self._lx3_params()
    param = int(CP.safetyConfigs[-1].safetyParam)
    assert param & HyundaiSafetyFlags.CCNC, "CCNC safety param must be set on an HDA1 CCNC car"
    assert param & HyundaiSafetyFlags.CANFD_ANGLE_STEERING
    assert not (param & HyundaiSafetyFlags.CANFD_LKA_STEER_MSG)

  def test_emits_lfa_alt_not_lkas(self):
    CP = self._lx3_params()
    addrs = steering_addrs(CP)
    assert LFA_ALT_ADDR in addrs, (
      f"LX3 must steer via LFA_ALT (0xCB); got {[hex(a) for a in addrs]}")
    assert LKAS_ADDR not in addrs and LKAS_ALT_ADDR not in addrs, (
      f"LX3 must not emit LKAS; got {[hex(a) for a in addrs]}")

  def test_steer_control_type_is_angle(self):
    from opendbc.car import structs
    CP = self._lx3_params()
    assert CP.steerControlType == structs.CarParams.SteerControlType.angle


if __name__ == "__main__":
  unittest.main()
