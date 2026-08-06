"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaican, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags, Buttons, CANFD_CAR
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

try:  # instrumentation only -- opendbc must still import standalone, without openpilot
  from openpilot.sunnypilot.selfdrive.car.lx3_debug import dlog as _dlog
except Exception:  # pragma: no cover
  def _dlog(*a, **k):
    pass

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# Measured on a 2026 Palisade (LX3) from a capture of real presses
LX3_PRESS_FRAMES = 4     # a real press is held 3-5 frames
LX3_COUNTER_STEP = 2     # 0x10b's counter advances by 2 per frame, idle and pressed
BUTTON_REPEAT_S = 0.25   # shortest observed gap between real presses was 121ms

BUTTON_COPIES = 2
BUTTON_COPIES_TIME = 7
BUTTON_COPIES_TIME_IMPERIAL = [BUTTON_COPIES_TIME + 3, 70]
BUTTON_COPIES_TIME_METRIC = [BUTTON_COPIES_TIME, 40]

BUTTONS = {
  SendButtonState.increase: Buttons.RES_ACCEL,
  SendButtonState.decrease: Buttons.SET_DECEL,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    # In-flight LX3 press: how many pressed frames are still to send, which button they hold,
    # and the genuine counter the last one rode on (-1 = send the next frame immediately)
    self.lx3_press_frames_left = 0
    self.lx3_press_accel = False
    self.lx3_last_ridden_counter = -1

  def create_can_mock_button_messages(self, packer, CS, send_button) -> list[CanData]:
    can_sends = []
    copies_xp = BUTTON_COPIES_TIME_METRIC if CS.is_metric else BUTTON_COPIES_TIME_IMPERIAL
    copies = int(np.interp(BUTTON_COPIES_TIME, copies_xp, [1, BUTTON_COPIES]))

    # send resume at a max freq of 10Hz
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
      # send 25 messages at a time to increases the likelihood of resume being accepted
      can_sends.extend([hyundaican.create_clu11(packer, self.frame, CS.clu11, send_button, self.CP)] * copies)
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
        self.last_button_frame = self.frame

    return can_sends

  def create_canfd_mock_button_messages(self, packer, CS, CAN, send_button) -> list[CanData]:
    can_sends = []
    _dlog("icbm.send", dedupe=False, button=send_button, altButtons=bool(self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS),
          haveFrame=bool(getattr(CS, "cruise_btns_alt_info", None)), counter=CS.buttons_counter,
          sinceLast=self.frame - self.last_button_frame)
    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
      # Same spoofed-counter burst as the 0x1cf path below, against CRUISE_BUTTONS_ALT. The
      # frame is a replay of the car's own last button message, so we cannot send anything
      # before the first one arrives. COUNTER is 8 bits here, not 4.
      if CS.cruise_btns_alt_info and (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          for _ in range(20):
            can_sends.append(hyundaicanfd.create_buttons_alt(packer, self.CP, CAN, CS.cruise_btns_alt_info,
                                                             (CS.buttons_counter + button_counter_offset) % 0x100, send_button))
          self.last_button_frame = self.frame
          _dlog("icbm.sent", dedupe=False, addr="0x1aa", button=send_button, frames=20,
                counter=(CS.buttons_counter + button_counter_offset) % 0x100)
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          for _ in range(20):
            can_sends.append(hyundaicanfd.create_buttons(packer, self.CP, CAN, (CS.buttons_counter + button_counter_offset) % 0xF, send_button))
          self.last_button_frame = self.frame

    return can_sends

  def create_lx3_press_messages(self, packer, CS, CAN, send_button) -> list[CanData]:
    """Press a button the way this car's own module does.

    This car ignores CRUISE_BUTTONS_ALT (0x1aa) -- it leaves that field at zero and reads its
    buttons from LFA_BUTTON_ALT (0x10b) byte 10. A real press, measured from a capture of the
    driver pressing +, resume and -, holds the bit across 3-5 consecutive frames of the
    module's continuous 25Hz stream (80-200ms) while the counter keeps its natural +2 step.

    Queueing all the pressed frames in one 100Hz control cycle put a ~1ms blip on the wire,
    contradicted within 40ms by the module's next idle frame. Instead: whenever a fresh
    genuine frame is seen during a press, inject one pressed copy riding one counter step
    ahead of it, so the press spans real time exactly like the module's own. A started press
    always runs to completion -- a real press does not un-happen halfway through.
    """
    can_sends = []
    if not CS.lfa_btn_info:
      return can_sends

    if self.lx3_press_frames_left == 0 and send_button is not None and \
       (self.frame - self.last_button_frame) * DT_CTRL > BUTTON_REPEAT_S:
      self.lx3_press_accel = send_button == Buttons.RES_ACCEL
      self.lx3_press_frames_left = LX3_PRESS_FRAMES
      self.lx3_last_ridden_counter = -1
      self.last_button_frame = self.frame
      _dlog("icbm.send", dedupe=False, addr="0x10b", accel=self.lx3_press_accel, frames=LX3_PRESS_FRAMES)

    if self.lx3_press_frames_left > 0 and int(CS.lfa_btn_counter) != self.lx3_last_ridden_counter:
      cnt = (int(CS.lfa_btn_counter) + LX3_COUNTER_STEP) % 0x100
      can_sends.append(hyundaicanfd.create_buttons_lx3(packer, self.CP, CAN, CS.lfa_btn_info, cnt,
                                                       self.lx3_press_accel, not self.lx3_press_accel))
      self.lx3_last_ridden_counter = int(CS.lfa_btn_counter)
      self.lx3_press_frames_left -= 1
      _dlog("icbm.sent", dedupe=False, addr="0x10b", accel=self.lx3_press_accel,
            counter=cnt, left=self.lx3_press_frames_left)

    return can_sends

  def update(self, CS, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.CP.carFingerprint == CAR.HYUNDAI_PALISADE_HEV_LX3:
      # None while no press is requested; an in-flight press still drains to completion
      send_button = BUTTONS.get(self.ICBM.sendButton)
      can_sends.extend(self.create_lx3_press_messages(packer, CS, CAN, send_button))
    elif self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]

      if self.CP.carFingerprint in CANFD_CAR:
        can_sends.extend(self.create_canfd_mock_button_messages(packer, CS, CAN, send_button))
      else:
        can_sends.extend(self.create_can_mock_button_messages(packer, CS, send_button))

    return can_sends
