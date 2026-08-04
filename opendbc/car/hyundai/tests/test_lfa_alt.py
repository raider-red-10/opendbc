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


if __name__ == "__main__":
  unittest.main()
