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
  BTN_CLUSTER_0X10B = 256  # mirror of HyundaiFlagsSP.BTN_CLUSTER_0X10B -- keep in sync with hyundai_common.h; bits 4-7 are the angle model ID


# Angle steering vehicle model IDs — encoded in safety_param_sp bits [4:7].
# Must stay in sync with the C enum in hyundai_canfd_angle_models.h.
ANGLE_MODEL_SHIFT = 4
ANGLE_MODEL_MASK = 0xF


class HyundaiAngleSteeringModel:
  BASELINE = 0                    # fallback: uses KIA_SPORTAGE_HEV_2026 (most conservative)
  KIA_SPORTAGE_HEV_2026 = 1      # baseline vehicle
  HYUNDAI_IONIQ_5_PE = 2
  KIA_EV6_2025 = 3
  KIA_EV9 = 4
  GENESIS_GV80_2025 = 5
  HYUNDAI_SANTA_FE_HEV_5TH = 6
  HYUNDAI_IONIQ_9 = 7
  KIA_SORENTO_HEV_4TH_LFA2 = 8
  GENESIS_GV70_E_2ND_GEN = 9
  HYUNDAI_AZERA_HEV_7TH = 10
  HYUNDAI_PALISADE_HEV_LX3 = 11


# Mapping from CAR platform name → angle steering model ID.
# Platforms not in this map will use BASELINE (0) on the panda.
ANGLE_STEERING_MODEL_BY_CAR: dict[str, int] = {
  "KIA_SPORTAGE_HEV_2026":              HyundaiAngleSteeringModel.KIA_SPORTAGE_HEV_2026,
  "HYUNDAI_IONIQ_5_PE":                 HyundaiAngleSteeringModel.HYUNDAI_IONIQ_5_PE,
  "KIA_EV6_2025":                       HyundaiAngleSteeringModel.KIA_EV6_2025,
  "KIA_EV9":                            HyundaiAngleSteeringModel.KIA_EV9,
  "GENESIS_GV80_2025":                  HyundaiAngleSteeringModel.GENESIS_GV80_2025,
  "HYUNDAI_SANTA_FE_HEV_5TH_GEN":      HyundaiAngleSteeringModel.HYUNDAI_SANTA_FE_HEV_5TH,
  "HYUNDAI_IONIQ_9":                    HyundaiAngleSteeringModel.HYUNDAI_IONIQ_9,
  "KIA_SORENTO_HEV_4TH_GEN_LFA2":      HyundaiAngleSteeringModel.KIA_SORENTO_HEV_4TH_LFA2,
  "GENESIS_GV70_ELECTRIFIED_2ND_GEN":   HyundaiAngleSteeringModel.GENESIS_GV70_E_2ND_GEN,
  "HYUNDAI_AZERA_HEV_7TH_GEN":         HyundaiAngleSteeringModel.HYUNDAI_AZERA_HEV_7TH,
  "HYUNDAI_PALISADE_HEV_LX3":          HyundaiAngleSteeringModel.HYUNDAI_PALISADE_HEV_LX3,
}


def encode_angle_model_id(model_id: int) -> int:
  """Encode an angle steering model ID into the safety_param_sp bit field."""
  return (model_id & ANGLE_MODEL_MASK) << ANGLE_MODEL_SHIFT


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
