import json
import pathlib
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams

# Fixture lives alongside this test so the opendbc branch stays self-contained
# and its tests pass when opendbc is checked out on its own.
BASELINE = pathlib.Path(__file__).parent / "sorento_baseline.json"


class TestSorentoRegression(unittest.TestCase):
  """The 2026 Sorento Hybrid works today. The LX3 port must not change it.

  Both camera-bus probes are checked because the real vehicle's LKAS address
  (0x50 vs 0x110) is unknown and they resolve to different flags -- 0x110 adds
  CANFD_LKA_STEER_MSG_ALT. Asserting both makes the gate valid either way.
  """

  def setUp(self):
    self.expected_all = json.loads(BASELINE.read_text())
    self.candidate = CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2
    self.car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
                   for (ecu, addr, sub), vers in FW_VERSIONS[self.candidate].items()]

  def _resolve(self, probe):
    fp = gen_empty_fingerprint()
    fp[2][int(probe, 16)] = 16
    return CarInterface.get_params(self.candidate, fp, self.car_fw, False, True, False)

  def test_resolved_params_unchanged(self):
    for probe, expected in sorted(self.expected_all["variants"].items()):
      with self.subTest(probe=probe):
        CP = self._resolve(probe)

        assert str(CP.carFingerprint) == self.expected_all["carFingerprint"]
        assert int(CP.flags) == expected["flags"], \
          f"[{probe}] Sorento flags changed: {expected['flags']} -> {int(CP.flags)} (baseline decoded as {expected['flagNames']})"
        safety_param = int(CP.safetyConfigs[-1].safetyParam)
        assert safety_param == expected["safetyParam"], \
          f"[{probe}] Sorento safetyParam changed: {expected['safetyParam']} -> {safety_param} (baseline decoded as {expected['safetyFlagNames']})"
        assert bool(CP.alphaLongitudinalAvailable) == expected["alphaLongitudinalAvailable"]

  def test_no_ccnc_contamination(self):
    """None of the CCNC/LFA_ALT machinery may reach the Sorento."""
    for probe in sorted(self.expected_all["variants"]):
      with self.subTest(probe=probe):
        CP = self._resolve(probe)

        assert not (CP.flags & HyundaiFlags.CCNC), "CCNC flag leaked onto the Sorento"
        assert not (CP.flags & HyundaiFlags.SEND_LFA), "SEND_LFA leaked onto the Sorento"
        assert not (int(CP.safetyConfigs[-1].safetyParam) & HyundaiSafetyFlags.CCNC), \
          "CCNC safety param leaked onto the Sorento"
        assert CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG, "Sorento must remain LKA steering"


if __name__ == "__main__":
  unittest.main()
