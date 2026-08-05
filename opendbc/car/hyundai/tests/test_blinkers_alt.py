import unittest

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.hyundai.values import DBC, CAR
from opendbc.car.interfaces import CarStateBase

BLINKERS_ALT_ADDR = 0x3E3
DBC_NAME = DBC[CAR.HYUNDAI_PALISADE_HEV_LX3][Bus.pt]

# Measured on a 2026 Palisade Hybrid
FLASH_HZ = 1.32
MSG_HZ = 5.1
HOLD = 100  # frames, must match carstate.py


def frame(byte11):
  dat = bytearray(16)
  dat[11] = byte11
  return (BLINKERS_ALT_ADDR, bytes(dat), 0)


class TestBlinkersAlt(unittest.TestCase):
  """The Palisade does not transmit BLINKERS (0x413). Its lamps are in BLINKERS_ALT (0x3e3)
  byte 11, bit 2 left and bit 4 right -- confirmed on the vehicle by watching each bit flash
  at 1.32Hz for its own stalk while staying static for the other."""

  def decode(self, byte11):
    parser = CANParser(DBC_NAME, [("BLINKERS_ALT", 0)], 0)
    parser.update([[0, [frame(byte11)]]])
    vl = parser.vl["BLINKERS_ALT"]
    return int(vl["LEFT_LAMP"]), int(vl["RIGHT_LAMP"])

  def test_bits_are_independent(self):
    for byte11, expected in ((0, (0, 0)), (1 << 2, (1, 0)), (1 << 4, (0, 1)), (0x14, (1, 1))):
      with self.subTest(byte11=hex(byte11)):
        self.assertEqual(self.decode(byte11), expected)

  def test_no_other_bit_in_the_byte_moves_them(self):
    # If either signal were mis-positioned, some unrelated bit would light it up
    for bit in range(8):
      if bit in (2, 4):
        continue
      with self.subTest(bit=bit):
        self.assertEqual(self.decode(1 << bit), (0, 0))

  def test_no_checksum_or_counter_declared(self):
    # 0x3e3 carries a checksum and counter we have not identified. Declaring either would
    # make the parser validate it, and a mismatch silently drops every frame.
    parser = CANParser(DBC_NAME, [("BLINKERS_ALT", 0)], 0)
    sigs = {s.name for s in parser.dbc.addr_to_msg[BLINKERS_ALT_ADDR].sigs.values()}
    self.assertEqual(sigs, {"LEFT_LAMP", "RIGHT_LAMP"})


class TestBlinkerHold(unittest.TestCase):
  """0x3e3 arrives at 5Hz while the lamp flashes at 1.32Hz, so consecutive frames that catch
  the lamp lit can be far apart. The hold has to outlast that gap or the blinker drops between
  flashes and a lane change cancels itself mid-manoeuvre."""

  def worst_gap_frames(self):
    """Longest run of 100Hz frames between two samples that see the lamp lit."""
    period, sample = 1 / FLASH_HZ, 1 / MSG_HZ
    last_high, gap = None, 0.0
    for i in range(2000):
      t = i * sample
      if (t % period) < period / 2:  # lamp lit
        if last_high is not None:
          gap = max(gap, t - last_high)
        last_high = t
    return gap * 100

  def test_hold_outlasts_the_sampling_gap(self):
    self.assertGreater(HOLD, self.worst_gap_frames())

  def test_the_stock_hold_would_not_have(self):
    # Pins why this car differs -- 50 is correct for the 50Hz BLINKERS message it replaces
    self.assertLess(50, self.worst_gap_frames())

  def test_blinker_survives_a_full_flash_cycle(self):
    class _CS(CarStateBase):
      def update(self, *a, **kw):
        raise NotImplementedError

    cs = _CS.__new__(_CS)
    cs.left_blinker_cnt = cs.right_blinker_cnt = 0
    period, sample = 1 / FLASH_HZ, 1 / MSG_HZ

    dropped = []
    lamp = 0
    for i in range(int(10 / 0.01)):  # 10s at 100Hz
      t = i * 0.01
      # the value carstate sees is whatever the last 5Hz frame carried
      if int(t / sample) != int((t - 0.01) / sample):
        lamp = int((t % period) < period / 2)
      left, _ = cs.update_blinker_from_lamp(HOLD, lamp, 0)
      if t > 1.0 and not left:
        dropped.append(round(t, 2))

    self.assertEqual(dropped, [], f"blinker dropped at {dropped[:5]}")


if __name__ == "__main__":
  unittest.main()
