import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

FR_CMR_02 = 0x1FA  # camera traffic sign recognition, carries ISLW_SpdCluMainDis

LFA_STEERING = CAR.HYUNDAI_PALISADE_HEV_LX3          # camera talks to the MDPS directly
LKA_STEERING = CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2      # camera goes through an ADAS ECU


def build(candidate, cam_msgs=(), pt_msgs=()):
  car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
            for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]
  fp = gen_empty_fingerprint()
  for m in cam_msgs:
    fp[2][m] = 32
  for m in pt_msgs:
    fp[0][m] = 32
  CP = CarInterface.get_params(candidate, fp, car_fw, False, True, False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fp, car_fw, False, True, False)
  return CP, CP_SP


def available(CP_SP):
  return bool(CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE)


class TestSpeedLimitAvailable(unittest.TestCase):
  """carstate_ext reads FR_CMR_02_100ms from the E-CAN parser on LKA-steering cars and from
  the camera parser on everything else. The availability flag has to be decided on the same
  bus, or an LFA-steering car never gets a car-sourced speed limit -- which silently disables
  Speed Limit Assist."""

  def test_lfa_steering_reads_the_camera_bus(self):
    # 0xCB marks an LFA-steering camera; 0x1fa alongside it is the sign-recognition message
    _, CP_SP = build(LFA_STEERING, cam_msgs=(0xCB, FR_CMR_02))
    self.assertTrue(available(CP_SP))

  def test_lfa_steering_not_available_without_the_message(self):
    _, CP_SP = build(LFA_STEERING, cam_msgs=(0xCB,))
    self.assertFalse(available(CP_SP))

  def test_lfa_steering_ignores_the_message_on_the_wrong_bus(self):
    # The read would come from the camera parser, so E-CAN presence must not set the flag --
    # doing so registers a camera message that is not there and invalidates the parser
    _, CP_SP = build(LFA_STEERING, cam_msgs=(0xCB,), pt_msgs=(FR_CMR_02,))
    self.assertFalse(available(CP_SP))

  def test_lka_steering_still_reads_ecan(self):
    # HDA2 behaviour must not change -- its E-CAN is bus 1 and that is where it reads from
    CP, _ = build(LKA_STEERING, cam_msgs=(0x50,))
    ecan = CanBus(CP).ECAN
    car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
              for (ecu, addr, sub), vers in FW_VERSIONS[LKA_STEERING].items()]
    fp = gen_empty_fingerprint()
    fp[2][0x50] = 16
    fp[ecan][FR_CMR_02] = 32
    CP = CarInterface.get_params(LKA_STEERING, fp, car_fw, False, True, False)
    CP_SP = CarInterface.get_params_sp(CP, LKA_STEERING, fp, car_fw, False, True, False)
    self.assertEqual(ecan, 1)
    self.assertTrue(available(CP_SP))


if __name__ == "__main__":
  unittest.main()
