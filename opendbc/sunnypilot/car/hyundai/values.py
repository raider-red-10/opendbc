"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class HyundaiSafetyFlagsSP:
  DEFAULT = 0
  ESCC = 1
  LONG_MAIN_CRUISE_TOGGLEABLE = 2
  HAS_LDA_BUTTON = 4
  NON_SCC = 8
  BTN_CLUSTER_0X10B = 16  # mirror of HyundaiFlagsSP.BTN_CLUSTER_0X10B -- keep in sync with hyundai_common.h


class HyundaiFlagsSP(IntFlag):
  """
    Flags for Hyundai specific quirks within sunnypilot.
  """
  ENHANCED_SCC = 1
  HAS_LFA_BUTTON = 2  # Deprecated in favor of HyundaiFlags.HAS_LDA_BUTTON
  LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE = 2 ** 2
  ENABLE_RADAR_TRACKS_DEPRECATED = 2 ** 3
  LONG_TUNING_DYNAMIC = 2 ** 4
  LONG_TUNING_PREDICTIVE = 2 ** 5
  NON_SCC = 2 ** 6
  NON_SCC_RADAR_FCA = 2 ** 7  # most with FCA come from the camera
  NON_SCC_NO_FCA = 2 ** 8  # not all have FCA
  SPEED_LIMIT_AVAILABLE = 2 ** 9  # platforms with speed limit data available
  HAS_LKAS12 = 2 ** 10
  # Declared per platform: the whole steering-wheel cluster is in LFA_BUTTON_ALT (0x10b)
  # byte 10 and CRUISE_BUTTONS_ALT (0x1aa)'s button field is permanently zero. Content-defined
  # (0x1aa exists -- its emptiness is invisible to the fingerprint), so it cannot be
  # auto-detected. Mirrored onto safetyParamSP so panda gates on the same trait.
  BTN_CLUSTER_0X10B = 2 ** 11
  # Auto-detected from the fingerprint in _get_params_sp -- never declare these per platform.
  BLINKERS_ALT = 2 ** 12       # 0x413 absent, lamps in 0x3e3 byte 11 at 5Hz
  ABSENT_DOORS_MSG = 2 ** 13   # DOORS_SEATBELTS (0x411) not transmitted
  ABSENT_HOD_MSG = 2 ** 14     # HOD_FD_01_100ms (0x2af) not transmitted
