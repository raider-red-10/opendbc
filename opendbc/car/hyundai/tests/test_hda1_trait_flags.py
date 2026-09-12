import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import (HyundaiFlagsSP, HyundaiSafetyFlagsSP, HyundaiAngleSteeringModel,
                                                   ANGLE_STEERING_MODEL_BY_CAR, ANGLE_MODEL_MASK, ANGLE_MODEL_SHIFT)

ECAN = 0
BLINKERS, BLINKERS_ALT = 0x413, 0x3E3
DOORS, HOD = 0x411, 0x2AF


def resolve(candidate, fp):
  fw = [CarParams.CarFw(ecu=e, fwVersion=v[0], address=a, subAddress=s or 0)
        for (e, a, s), v in FW_VERSIONS[candidate].items()]
  CP = CarInterface.get_params(candidate, fp, fw, False, True, False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fp, fw, False, True, False)
  return CP, CP_SP


def lx3_fp(extra_ecan=()):
  fp = gen_empty_fingerprint()
  fp[2][0xCB] = 24  # LFA_ALT: selects the HDA1 angle path
  for addr in extra_ecan:
    fp[ECAN][addr] = 8
  return fp


class TestHda1TraitFlags(unittest.TestCase):
  """Bus-detectable traits configure themselves from the fingerprint; the content-defined
  button trait is declared per platform and mirrored onto safetyParamSP so panda gates on
  the same truth. See docs/superpowers/specs/2026-08-06-hda1-trait-flags-design.md."""

  def test_lx3_declares_the_button_cluster(self):
    _, CP_SP = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B)
    self.assertTrue(CP_SP.safetyParam & HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)

  def test_button_bit_coexists_with_the_angle_model_id(self):
    # safetyParamSP bits 4-7 carry the panda's per-vehicle angle model; the button trait must
    # live outside that field or the LX3 would be decoded as some other car's physics
    model_field = ANGLE_MODEL_MASK << ANGLE_MODEL_SHIFT
    for name, value in vars(HyundaiSafetyFlagsSP).items():
      if name.isupper():
        self.assertFalse(value & model_field, f"{name} overlaps the angle model ID field")

    _, CP_SP = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertTrue(CP_SP.safetyParam & HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)
    self.assertEqual((CP_SP.safetyParam >> ANGLE_MODEL_SHIFT) & ANGLE_MODEL_MASK, HyundaiAngleSteeringModel.HYUNDAI_PALISADE_HEV_LX3)
    self.assertEqual(ANGLE_STEERING_MODEL_BY_CAR[str(CAR.HYUNDAI_PALISADE_HEV_LX3)], HyundaiAngleSteeringModel.HYUNDAI_PALISADE_HEV_LX3)

  def test_sorento_declares_nothing(self):
    fp = gen_empty_fingerprint()
    fp[2][0x50] = 16  # LKAS: HDA2 probe
    _, CP_SP = resolve(CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2, fp)
    self.assertFalse(CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B)
    self.assertFalse(CP_SP.safetyParam & HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)

  def test_blinkers_alt_detected_from_the_bus(self):
    # 0x413 absent AND 0x3e3 present -> alt; anything else -> standard
    _, with_alt = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp(extra_ecan=(BLINKERS_ALT,)))
    self.assertTrue(with_alt.flags & HyundaiFlagsSP.BLINKERS_ALT)

    _, both = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp(extra_ecan=(BLINKERS, BLINKERS_ALT)))
    self.assertFalse(both.flags & HyundaiFlagsSP.BLINKERS_ALT)

    _, neither = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertFalse(neither.flags & HyundaiFlagsSP.BLINKERS_ALT)

  def test_message_absences_detected_from_the_bus(self):
    _, absent = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertTrue(absent.flags & HyundaiFlagsSP.ABSENT_DOORS_MSG)
    self.assertTrue(absent.flags & HyundaiFlagsSP.ABSENT_HOD_MSG)

    _, present = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp(extra_ecan=(DOORS, HOD)))
    self.assertFalse(present.flags & HyundaiFlagsSP.ABSENT_DOORS_MSG)
    self.assertFalse(present.flags & HyundaiFlagsSP.ABSENT_HOD_MSG)


if __name__ == "__main__":
  unittest.main()
