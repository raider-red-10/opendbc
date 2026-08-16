#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x50,  a_can, 16, .check_relay = (a_can) == 0},  /* LKAS */      \
  {0x2A4, a_can, 24, .check_relay = (a_can) == 0},  /* CAM_0x2A4 */ \

#define HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x110, a_can, 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */  \
  {0x362, a_can, 32, .check_relay = (a_can) == 0},  /* CAM_0x362 */ \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x12A, e_can, 16, .check_relay = (e_can) == 0},  /* LFA */            \
  {0x1E0, e_can, 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal) \
  {0x1A0, e_can, 32, .check_relay = (longitudinal)},  /* SCC_CONTROL */ \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                          \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                  \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                 \
           {0x105, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}}},               \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                                                            \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                       \
  {.msg = {{0x1cf, (pt_bus), 8, 50U, .ignore_checksum = true, .max_counter = 0xfU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                                              \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                         \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// CCNC cars transmit ACCELERATOR_ALT (0x105) with a counter that increments by 2 per frame,
// measured on a 2026 Palisade Hybrid (LX3): 299 of 299 consecutive deltas were 2. The safety
// layer's counter check hardcodes a +1 step (update_counter() in safety.h), so it can only
// ever be satisfied by a +1 sequence. max_counter = 0 skips the counter check for this one
// message; checksum, frequency, and quality checks are all still enforced.
#define HYUNDAI_CANFD_COMMON_RX_CHECKS_CCNC(pt_bus)                                                        \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                  \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                 \
           {0x105, (pt_bus), 32, 50U, .max_counter = 0U, .ignore_counter = true, .ignore_quality_flag = true}}},  \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC(pt_bus)                                                                                         \
  HYUNDAI_CANFD_COMMON_RX_CHECKS_CCNC(pt_bus)                                                                                                    \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// LFA_BUTTON_ALT (0x10b) carries the real buttons on cluster-button cars (declared trait) --
// cruise and LFA both. The rx hook only runs for addresses in this list, so without it panda
// never saw a button at all: hyundai_last_button_interaction saturated, controls_allowed
// never latched on the cruise-engaged rising edge, and every button openpilot tried to send
// was rejected. 25Hz measured on the vehicle. Its counter steps by 2, which the +1 counter
// check can never satisfy -- same quirk as 0x105 above, so skip that check. Only declared on
// trait cars: a checked address the car never transmits would invalidate the rx checks and
// clear controls_allowed continuously.
#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC_BTN_CLUSTER(pt_bus)                                                                             \
  HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC(pt_bus)                                                                                               \
  {.msg = {{0x10b, (pt_bus), 16, 25U, .ignore_checksum = true, .max_counter = 0U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                                                            \
  {.msg = {{0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_angle_steering = false;
static bool hyundai_canfd_lka_steer_msg_alt = false;
static bool hyundai_ccnc = false;

static bool get_hyundai_ccnc(void) {
  return hyundai_ccnc;
}

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steer_msg_alt ? 0x110U : 0x50U;
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = msg->data[0] | (msg->data[1] << 8);
  return chksum;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {

  const unsigned pt_bus = hyundai_canfd_lka_steer_msg ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  if (msg->bus == pt_bus) {
    // driver torque
    if (msg->addr == 0xeaU) {
      int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);

      int angle_meas_new = (msg->data[17] << 8) | msg->data[16];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // cruise buttons.
    // Cluster-button cars (declared trait) leave this message's button field at zero and
    // report presses in 0x10B instead, so reading it here would feed
    // hyundai_common_cruise_buttons_check() a permanent NONE. hyundai_last_button_interaction
    // then saturates, controls_allowed never latches on the cruise-engaged rising edge, and
    // every button we try to send is rejected.
    const unsigned int button_addr = hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
    if ((msg->addr == button_addr) && !hyundai_btn_cluster_0x10b) {
      bool main_button = false;
      int cruise_button = 0;
      if (msg->addr == 0x1cfU) {
        cruise_button = msg->data[2] & 0x7U;
        main_button = GET_BIT(msg, 19U);
        mads_button_press = GET_BIT(msg, 23U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      } else {
        cruise_button = (msg->data[4] >> 4) & 0x7U;
        main_button = GET_BIT(msg, 34U);
        // Cluster-button cars report the LFA button in 0x10B instead and never reach this
        // branch (gated above) -- reading it here at 50Hz would continuously overwrite the
        // real button state with NOT_PRESSED.
        mads_button_press = GET_BIT(msg, 39U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // Cluster-button cars carry the LFA button in its own message rather than with the
    // cruise buttons. Measured on a 2026 Palisade Hybrid (LX3): 0x10B byte 10 bit 7, five
    // presses produced five rising edges matching the press cadence, and SET-/RES+/gap
    // produced none. Must match the carstate read of LFA_BUTTON_ALT -- if the two
    // layers disagree, openpilot engages lateral while the panda withholds
    // controls_allowed_lateral and the mismatch check disengages with a takeover alert.
    if (hyundai_btn_cluster_0x10b && (msg->addr == 0x10BU)) {
      mads_button_press = GET_BIT(msg, 87U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;

      // The cruise buttons live here too, one bit each, rather than as an enum: bit 80 is +,
      // bit 81 is -, bit 82 is resume, bit 83 the cancel/set toggle. Measured on a 2026
      // Palisade Hybrid (LX3). + and resume are separate buttons on this car, unlike most
      // Hyundais where RES_ACCEL is both. The toggle engages cruise from nothing (255 ->
      // current speed on the vehicle), so it must count as a button interaction or engaging
      // with it never latches controls_allowed and every ICBM button frame is rejected.
      int cruise_button = HYUNDAI_BTN_NONE;
      if (GET_BIT(msg, 80U) || GET_BIT(msg, 82U)) {
        cruise_button = HYUNDAI_BTN_RESUME;
      } else if (GET_BIT(msg, 81U)) {
        cruise_button = HYUNDAI_BTN_SET;
      } else if (GET_BIT(msg, 83U)) {
        cruise_button = HYUNDAI_BTN_CANCEL;
      } else {
      }
      hyundai_common_cruise_buttons_check(cruise_button, false);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x35U) && hyundai_ev_gas_signal) {
      gas_pressed = msg->data[5] != 0U;
    } else if ((msg->addr == 0x105U) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 103U) || (msg->data[13] != 0U) || GET_BIT(msg, 112U);
    } else if ((msg->addr == 0x100U) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 176U);
    } else {
    }

    // brake press
    if (msg->addr == 0x175U) {
      brake_pressed = GET_BIT(msg, 81U);
    }

    // vehicle moving
    if (msg->addr == 0xa0U) {
      uint32_t fl = (GET_BYTES(msg, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(msg, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(msg, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(msg, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);
    }
  }

  if (msg->bus == scc_bus) {
    // cruise state
    if ((msg->addr == 0x1a0U) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((msg->data[8] >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 66U);
    }
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_CANFD_TORQUE_STEERING_LIMITS = {
    .max_torque = 270,
    .max_rt_delta = 112,
    .max_rate_up = 2,
    .max_rate_down = 3,
    .driver_torque_allowance = 250,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  const AngleSteeringLimits HYUNDAI_CANFD_ANGLE_STEERING_LIMITS = {
    .max_angle = 3600,
    .angle_deg_to_can = 10,
    .frequency = 100U,
  };

  // We need to find a middle ground between all the possible params or find a way to properly fingerprint.
  // HYUNDAI_IONIQ_5_PE: -0.0008688329819908074
  // KIA_EV6_2025: -0.000889804937754786
  // KIA_EV9: -0.0005410588125765342
  // GENESIS_GV80_2025: -0.0005685702046115589
  // HYUNDAI_SANTA_FE_HEV_5TH_GEN: -0.00059689759884299

  // IONIQ 5 PE values.
  // const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
  //   .slip_factor = -0.0008688329819908074,  // calc_slip_factor(VM)
  //   .steer_ratio = 14.26,
  //   .wheelbase = 2.97,
  // };

  // // GENESIS_GV80_2025 values. (values can be found on values.py)
  // const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
  //   .slip_factor = -0.0005685702046115589,  // calc_slip_factor(VM)
  //   .steer_ratio = 14.14,
  //   .wheelbase = 2.95,
  // };

  // HYUNDAI_SANTA_FE_HEV_5TH_GEN values. (values can be found on values.py)
  // const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
  //   .slip_factor = -0.00059689759884299,  // calc_slip_factor(VM)
  //   .steer_ratio = 13.72,
  //   .wheelbase = 2.81,
  // };

  // KIA_SPORTAGE_HEV_2026 values. (most conservative for now) (values can be found on values.py)
  const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
      .slip_factor = -0.0006085930193026732,  // calc_slip_factor(VM)
      .steer_ratio = 13.7,
      .wheelbase = 2.756,
    };


  bool tx = true;

  // HDA1 steering
  if ((msg->addr == 0xCBU) && hyundai_canfd_angle_steering) {
    const int lfa_angle_active = (msg->data[3] >> 4U);
    const bool steer_angle_req = lfa_angle_active == 2;

    int desired_angle = (((uint32_t)(msg->data[5] & 0x3FU)) << 8) | (uint32_t)msg->data[4];
    desired_angle = to_signed(desired_angle, 14);

    if (steer_angle_cmd_checks_vm(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS, HYUNDAI_STEERING_PARAMS)) {
      tx = false;
    }
  }

  // steering
  const unsigned int steer_addr = (hyundai_canfd_lka_steer_msg && !hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (msg->addr == steer_addr) {
    if (hyundai_canfd_angle_steering) {
      const int lkas_angle_active = (msg->data[9] >> 4U) & 0x3U;
      const bool steer_angle_req = lkas_angle_active != 1;

      int desired_angle = (msg->data[11] << 6U) | (msg->data[10] >> 2U);
      desired_angle = to_signed(desired_angle, 14);

      // ADAS_ACIAnglTqRedcGainVal: bit 96, 8 bits, unsigned. Raw 0-250 valid, 251-255 reserved.
      const uint8_t gain_raw = msg->data[12];
      bool gain_violation = gain_raw > 250U;
      if (!steer_angle_req && (gain_raw != 0U)) {
        gain_violation = true;
      }

      if (steer_angle_cmd_checks_vm(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS, HYUNDAI_STEERING_PARAMS) || gain_violation) {
        tx = false;
      }
    } else {
      int desired_torque = (((msg->data[6] & 0xFU) << 7U) | (msg->data[5] >> 1U)) - 1024U;
      bool steer_req = GET_BIT(msg, 52U);

      if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_TORQUE_STEERING_LIMITS)) {
        tx = false;
      }
    }
  }

  // LFA_BUTTON_ALT carries one bit per button in byte 10 rather than a button enum, and is
  // the only message cluster-button cars read presses from. Accel and decel adjust the set
  // speed, so they need controls_allowed; resume and the LFA button are never ours to send.
  if (msg->addr == 0x10BU) {
    bool accel = GET_BIT(msg, 80U);
    bool decel = GET_BIT(msg, 81U);
    bool resume = GET_BIT(msg, 82U);
    bool lfa = GET_BIT(msg, 87U);
    bool one_button = (accel != decel);  // exactly one of the two, never both
    if (!hyundai_canfd_alt_buttons || !hyundai_btn_cluster_0x10b || resume || lfa || !one_button || !controls_allowed) {
      tx = false;
    }
  }

  // cruise buttons check
  if ((msg->addr == 0x1cfU) || (msg->addr == 0x1aaU)) {
    int button = 0;
    bool addr_allowed = true;
    if (msg->addr == 0x1cfU) {
      button = msg->data[2] & 0x7U;
    } else {
      // CRUISE_BUTTONS_ALT, same bit position we read it from in the rx hook. Only cars that
      // actually use the alt button message may send it.
      button = (msg->data[4] >> 4) & 0x7U;
      addr_allowed = hyundai_canfd_alt_buttons;
    }
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);
    bool is_set = (button == HYUNDAI_BTN_SET);

    bool allowed = addr_allowed && ((is_cancel && cruise_engaged_prev) || ((is_resume || is_set) && controls_allowed));
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (((msg->addr == 0x730U) && hyundai_canfd_lka_steer_msg) || ((msg->addr == 0x7D0U) && !hyundai_camera_scc)) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw = (((msg->data[17] & 0x7U) << 8) | msg->data[16]) - 1023U;
    int desired_accel_val = ((msg->data[18] << 4) | (msg->data[17] >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }

    acc_main_on_tx = GET_BIT(msg, 66U);
    hyundai_common_acc_main_on_sync();
  }

  return tx;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const uint16_t HYUNDAI_PARAM_CANFD_ANGLE_STEERING = 1024;
  const uint16_t HYUNDAI_PARAM_CCNC = 2048;

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
  };

  // ADRV_0x160 is checked for radar liveness
  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
    {0x160, 0, 16, .check_relay = true}, // ADRV_0x160
    {0x7D0, 0, 8, .check_relay = false},  // tester present for radar ECU disable
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    {0x1AA, 2, 16, .check_relay = false}, /* CRUISE_BUTTONS_ALT -- ICBM on alt-button cars */ \
    {0x10B, 2, 16, .check_relay = false}, /* LFA_BUTTON_ALT -- buttons the LX3 actually reads */ \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0xCB,  0, 24, .check_relay = true}, /* LFA_ALT -- HDA1 angle steering */ \
    {0x161, 0, 32, .check_relay = true}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = true}, /* CCNC_0x162 */ \
    {0x7C4, 2, 8, .check_relay = true}, /* 0x7C4 */ \
    {0xEA, 2, 24, .check_relay = true}, /* MDPS */ \

  hyundai_common_init(param);

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_angle_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ANGLE_STEERING);
  // TODO: test this restriction
  hyundai_canfd_lka_steer_msg_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT);
  hyundai_ccnc = GET_FLAG(param, HYUNDAI_PARAM_CCNC);

  safety_config ret;
  if (hyundai_longitudinal) {
    if (hyundai_canfd_lka_steer_msg) {
      static RxCheck hyundai_canfd_lka_steer_msg_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
      };

      ret = BUILD_SAFETY_CFG(hyundai_canfd_lka_steer_msg_long_rx_checks, HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS);

    } else {
      // Longitudinal checks for LFA steering
      static RxCheck hyundai_canfd_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
      };

      // The CCNC accommodations are not optional once longitudinal is on: 0x105's counter
      // steps by 2 on these cars, so the standard +1 check can never be satisfied and
      // is_msg_valid() would clear controls_allowed continuously. Keep these in step with the
      // non-longitudinal branch below.
      static RxCheck hyundai_canfd_alt_buttons_ccnc_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_ccnc_btn_cluster_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC_BTN_CLUSTER(0)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(true)
      };

      if (hyundai_canfd_alt_buttons) {
        if (get_hyundai_ccnc() && hyundai_btn_cluster_0x10b) {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_ccnc_btn_cluster_long_rx_checks, ret);
        } else if (get_hyundai_ccnc()) {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_ccnc_long_rx_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
        }
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      if (hyundai_camera_scc) {
        if (get_hyundai_ccnc()) {
          SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs, ret);
        } else {
          SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
        }
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }

  } else {
    if (hyundai_canfd_lka_steer_msg) {
      // *** LKA steering checks ***
      // E-CAN is on bus 1, SCC messages are sent on cars with ADRV ECU.
      // Does not use the alt buttons message
      static RxCheck hyundai_canfd_lka_steer_msg_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_rx_checks, ret);
      if (hyundai_canfd_lka_steer_msg_alt) {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS, ret);
      }

    } else if (!hyundai_camera_scc) {
      // Radar sends SCC messages on these cars instead of camera
      static RxCheck hyundai_canfd_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_radar_scc_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_radar_scc_rx_checks, ret);
      }

    } else {
      // *** LFA steering checks ***
      // Camera sends SCC messages on LFA steering cars.
      // Both button messages exist on some platforms, so we ensure we track the correct one using flag
      static RxCheck hyundai_canfd_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_ccnc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_ccnc_btn_cluster_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC_BTN_CLUSTER(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(false)
      };

      if (get_hyundai_ccnc()) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      }

      if (hyundai_canfd_alt_buttons) {
        if (get_hyundai_ccnc() && hyundai_btn_cluster_0x10b) {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_ccnc_btn_cluster_rx_checks, ret);
        } else if (get_hyundai_ccnc()) {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_ccnc_rx_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_rx_checks, ret);
        }
      } else {
        SET_RX_CHECKS(hyundai_canfd_rx_checks, ret);
      }
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
