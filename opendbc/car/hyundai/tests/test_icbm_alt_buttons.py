import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.hyundai.hyundaicanfd import CanBus, create_buttons_alt
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC, Buttons, HyundaiFlags
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams

BUTTONS_ALT_ADDR = 0x1AA

# A frame as the car sends it, with the driver holding buttons we must never replay
HELD_FRAME = {
  "CHECKSUM": 0, "COUNTER": 3, "NEW_SIGNAL_1": 0, "SET_ME_1": 1, "DISTANCE_UNIT": 1,
  "NEW_SIGNAL_2": 0, "ADAPTIVE_CRUISE_MAIN_BTN": 1, "NEW_SIGNAL_3": 0, "CRUISE_BUTTONS": 0,
  "LDA_BTN": 1, "NEW_SIGNAL_4": 0, "NORMAL_CRUISE_MAIN_BTN": 1, "NEW_SIGNAL_5": 0,
  "SET_ME_2": 3, "NEW_SIGNAL_6": 0, "BYTE6": 0xAB, "BYTE7": 0xCD, "BYTE8": 0x12,
  "BYTE9": 0x34, "BYTE10": 0x56, "BYTE11": 0x78, "BYTE12": 0x9A, "BYTE13": 0xBC,
  "BYTE14": 0xDE, "BYTE15": 0xF0,
}


def build_params(candidate, ecan_msgs):
  """Resolve real CarParams, emulating which messages are on E-CAN."""
  car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
            for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]
  fp = gen_empty_fingerprint()
  fp[2][0xCB] = 24  # LFA_ALT, marks this as an HDA1 angle-steering camera
  for m in ecan_msgs:
    fp[0][m] = 16
  return CarInterface.get_params(candidate, fp, car_fw, False, True, False)


class TestIcbmAltButtons(unittest.TestCase):
  """ICBM presses SET+/SET- for the driver. On CANFD_ALT_BUTTONS cars that means sending
  CRUISE_BUTTONS_ALT (0x1aa) -- a 16-byte, mostly-unidentified frame we can only build by
  replaying the car's own last message. These tests pin the parts that make that safe."""

  def setUp(self):
    self.CP = build_params(CAR.HYUNDAI_PALISADE_HEV_LX3, [])
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.CAN = CanBus(self.CP)

  def send(self, btn, cnt=7, frame=None):
    addr, dat, bus = create_buttons_alt(self.packer, self.CP, self.CAN,
                                        frame if frame is not None else HELD_FRAME, cnt, btn)
    parser = CANParser(DBC[self.CP.carFingerprint][Bus.pt], [("CRUISE_BUTTONS_ALT", 0)], bus)
    parser.update([[0, [(addr, dat, bus)]]])
    return addr, dat, bus, parser.vl["CRUISE_BUTTONS_ALT"]

  def test_palisade_is_an_alt_button_car(self):
    # The premise of everything below: no 0x1cf on E-CAN means the buttons live in 0x1aa
    self.assertTrue(self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS)

  def test_addresses_the_alt_button_message(self):
    addr, dat, bus, _ = self.send(Buttons.RES_ACCEL)
    self.assertEqual(addr, BUTTONS_ALT_ADDR)
    self.assertEqual(len(dat), 16)
    # LFA-steering cars reach the camera, which is where the SCC lives, on the cam bus
    self.assertEqual(bus, self.CAN.CAM)

  def test_button_round_trips(self):
    for btn in (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL):
      with self.subTest(btn=btn):
        _, _, _, vl = self.send(btn)
        self.assertEqual(vl["CRUISE_BUTTONS"], btn)

  def test_counter_is_ours_not_the_replayed_one(self):
    # We spoof the next counter so the SCC accepts the frame; the copied value must not win
    _, _, _, vl = self.send(Buttons.RES_ACCEL, cnt=42)
    self.assertEqual(vl["COUNTER"], 42)

  def test_checksum_is_recomputed(self):
    # A stale checksum copied from the source frame would be rejected by the car. CANParser
    # drops frames that fail the checksum, so a populated result proves it was recomputed.
    _, _, _, vl = self.send(Buttons.RES_ACCEL)
    self.assertNotEqual(vl["CHECKSUM"], HELD_FRAME["CHECKSUM"])
    self.assertEqual(vl["CRUISE_BUTTONS"], Buttons.RES_ACCEL)

  def test_never_replays_a_held_button(self):
    # 20 copies go out per press. Replaying a main-cruise press would toggle the driver's
    # cruise off, and an LDA press would toggle their lane keeping.
    _, _, _, vl = self.send(Buttons.RES_ACCEL)
    self.assertEqual(vl["ADAPTIVE_CRUISE_MAIN_BTN"], 0)
    self.assertEqual(vl["NORMAL_CRUISE_MAIN_BTN"], 0)
    self.assertEqual(vl["LDA_BTN"], 0)

  def test_preserves_the_rest_of_the_frame(self):
    # Everything we do not deliberately set has to survive, or the SCC sees a malformed frame
    _, _, _, vl = self.send(Buttons.RES_ACCEL)
    for sig in ("SET_ME_1", "SET_ME_2", "DISTANCE_UNIT", "BYTE6", "BYTE9", "BYTE12", "BYTE15"):
      with self.subTest(sig=sig):
        self.assertEqual(vl[sig], HELD_FRAME[sig])


class TestIcbmAvailability(unittest.TestCase):
  """Availability must track the panda TX list. Claiming ICBM on a car whose panda will not
  pass 0x1aa gives the driver a toggle that silently does nothing."""

  def sp(self, candidate, ecan_msgs):
    car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
              for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]
    fp = gen_empty_fingerprint()
    fp[2][0xCB] = 24
    for m in ecan_msgs:
      fp[0][m] = 16
    CP = CarInterface.get_params(candidate, fp, car_fw, False, True, False)
    return CP, CarInterface.get_params_sp(CP, candidate, fp, car_fw, False, True, False)

  def test_available_on_ccnc_alt_button_car(self):
    CP, CP_SP = self.sp(CAR.HYUNDAI_PALISADE_HEV_LX3, [])
    self.assertTrue(CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS)
    self.assertTrue(CP.flags & HyundaiFlags.CCNC)
    self.assertTrue(CP_SP.intelligentCruiseButtonManagementAvailable)

  def test_still_available_when_the_car_uses_the_standard_button_message(self):
    # 0x1cf present -> not an alt-button car -> the original 0x1cf path, unchanged
    CP, CP_SP = self.sp(CAR.HYUNDAI_PALISADE_HEV_LX3, [0x1CF])
    self.assertFalse(CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS)
    self.assertTrue(CP_SP.intelligentCruiseButtonManagementAvailable)


if __name__ == "__main__":
  unittest.main()
