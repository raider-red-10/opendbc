import os
import unittest
from unittest import mock

from opendbc.car import structs
from opendbc.car.car_helpers import fingerprint

VIN_A = "1" * 17
VIN_B = "2" * 17


def cached_cp(vin=VIN_A):
  cp = structs.CarParams()
  cp.brand = "hyundai"
  cp.carVin = vin
  fw = structs.CarParams.CarFw()
  fw.fwVersion = b"\x00fake"
  cp.carFw = [fw]
  return cp


class TestCachedParamsVinCheck(unittest.TestCase):
  """The CarParams cache has no idea which car the device is plugged into. Trusting it
  blindly means a device moved between cars resolves the previous car's CarParams -- wrong
  steering mode, wrong safety config. A ~1s live VIN read decides whether the cache
  describes this car."""

  def setUp(self):
    self.calls = []
    patches = {
      "get_vin": mock.patch("opendbc.car.car_helpers.get_vin", return_value=(0, 0, VIN_A)),
      "get_present_ecus": mock.patch("opendbc.car.car_helpers.get_present_ecus", return_value=set()),
      "get_fw_versions_ordered": mock.patch("opendbc.car.car_helpers.get_fw_versions_ordered", return_value=[]),
      "can_fingerprint": mock.patch("opendbc.car.car_helpers.can_fingerprint", return_value=(None, {0: {}})),
    }
    self.mocks = {name: p.start() for name, p in patches.items()}
    for p in patches.values():
      self.addCleanup(p.stop)
    self.can_recv = lambda wait_for_one=False: []
    self.can_send = lambda msgs: None
    self.obd = lambda enabled: None

  def run_fingerprint(self, cached_params):
    return fingerprint(self.can_recv, self.can_send, self.obd, cached_params, None)

  def test_cache_used_when_live_vin_matches(self):
    _, _, vin, car_fw, _, _ = self.run_fingerprint(cached_cp(vin=VIN_A))
    self.assertTrue(self.mocks["get_vin"].called, "cache must be validated against the live VIN")
    self.assertEqual(vin, VIN_A)
    self.assertEqual(len(car_fw), 1, "cached FW must be used when the VIN matches")
    self.assertFalse(self.mocks["get_fw_versions_ordered"].called, "no full FW query on a valid cache")

  def test_cache_discarded_when_live_vin_differs(self):
    self.mocks["get_vin"].return_value = (0, 0, VIN_B)
    _, _, vin, car_fw, _, _ = self.run_fingerprint(cached_cp(vin=VIN_A))
    self.assertEqual(vin, VIN_B, "the live car's VIN wins")
    self.assertEqual(car_fw, [], "the stale cache's FW must not be used")
    self.assertTrue(self.mocks["get_fw_versions_ordered"].called, "a full FW query must run for the new car")

  def test_vin_query_failure_discards_the_cache(self):
    # A flaky VIN read must cost a re-fingerprint, never resolve the wrong car
    from opendbc.car.vin import VIN_UNKNOWN
    self.mocks["get_vin"].return_value = (-1, -1, VIN_UNKNOWN)
    _, _, _, car_fw, _, _ = self.run_fingerprint(cached_cp(vin=VIN_A))
    self.assertEqual(car_fw, [], "unverifiable cache must not be trusted")
    self.assertTrue(self.mocks["get_fw_versions_ordered"].called)

  def test_replay_skips_the_vin_check(self):
    self.mocks["get_vin"].side_effect = AssertionError("REPLAY must not query the VIN")
    with mock.patch.dict(os.environ, {"REPLAY": "1"}):
      _, _, vin, car_fw, _, _ = self.run_fingerprint(cached_cp(vin=VIN_A))
    self.assertEqual(vin, VIN_A)
    self.assertEqual(len(car_fw), 1)

  def test_no_cache_queries_once(self):
    self.run_fingerprint(None)
    self.assertEqual(self.mocks["get_vin"].call_count, 1)
    self.assertTrue(self.mocks["get_fw_versions_ordered"].called)


if __name__ == "__main__":
  unittest.main()
