#pragma once

#include "opendbc/safety/declarations.h"

static bool byd_longitudinal = false;

const int BYD_STEERING_ANGLE_ADDR = 0x11F;
const int BYD_GAS_BRAKE_ADDR = 0x342;
const int BYD_VEHICLE_SPEED_ADDR = 0x1F0;
const int BYD_PCM_BUTTONS_ADDR = 0x3B0;
const int BYD_ACC_CMD_ADDR = 0x32E;

static void byd_rx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;

  if (bus == 0) {
    // current steering angle, factor -0.1 and little endian
    if (addr == BYD_STEERING_ANGLE_ADDR) {
      int angle_meas_new = (GET_BYTES(msg, 0, 2) & 0xFFFFU);
      // let it be CAN unit degree
      angle_meas_new = to_signed(angle_meas_new, 16);

      update_sample(&angle_meas, angle_meas_new);
    }

    // gas and brakes
    if (addr == BYD_GAS_BRAKE_ADDR) {
      gas_pressed = (GET_BYTES(msg, 0, 1) > 0U);
      brake_pressed = (GET_BYTES(msg, 1, 1) > 0U);
    }

    // vehicle speed
    if (addr == BYD_VEHICLE_SPEED_ADDR) {
      // average of FL and BR
      uint16_t fl_ms = GET_BYTES(msg, 0, 2) & 0x0FFFU;
      uint16_t br_ms = GET_BYTES(msg, 5, 2) & 0x0FFFU;
      vehicle_moving = (fl_ms | br_ms) != 0U;
      UPDATE_VEHICLE_SPEED((fl_ms + br_ms) / 2.0 * 0.1 * KPH_TO_MS);
    }

    // engage logic with buttons
    if (addr == BYD_PCM_BUTTONS_ADDR) {
      // TODO: does it have to be on the rising edge
      bool set_pressed = GET_BIT(msg, 3U);
      bool res_pressed = GET_BIT(msg, 4U);
      bool cancel = GET_BIT(msg, 19U);

      if (set_pressed || res_pressed) {
        controls_allowed = true;
      }

      if (cancel) {
        controls_allowed = false;
      }
    }
  }

  if (bus == 2) {
    // cruise enabled
    if (addr == BYD_ACC_CMD_ADDR) {
      bool engaged_active_low = GET_BIT(msg, 44U);
      pcm_cruise_check(engaged_active_low);
    }
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits BYD_STEERING_LIMITS = {
    .max_angle = 2200,
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {6., 4., 3.}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {8., 6., 4.}
    },
  };

  const LongitudinalLimits BYD_LONG_LIMITS = {
    .max_accel = 130,       // 2.83 m/s^2
    .min_accel = 50,        // -3.2 m/s^2
    .inactive_accel = 100,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;
  int addr = msg->addr;

  // steer violation checks
  if (addr == 482) {

    int desired_angle = (GET_BYTES(msg, 3, 2) & 0xFFFFU);
    bool lka_active = GET_BIT(msg, 8U);

    desired_angle = to_signed(desired_angle, 16);

    if (steer_angle_cmd_checks(desired_angle, lka_active, BYD_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // acc violation checks
  if ((addr == 814) && byd_longitudinal) {
    int desired_accel = GET_BYTES(msg, 0, 1);
    violation |= longitudinal_accel_checks(desired_accel, BYD_LONG_LIMITS);
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static safety_config byd_init(uint16_t param) {

  UNUSED(param);
#ifdef ALLOW_DEBUG
  const int BYD_FLAG_LONGITUDINAL_CONTROL = 1;
  byd_longitudinal = GET_FLAG(param, BYD_FLAG_LONGITUDINAL_CONTROL);
#endif

  static const CanMsg BYD_TX_MSGS[] = {
    {482, 0, 8, .check_relay = true}, // STEERING_MODULE_ADAS
    {790, 0, 8, .check_relay = true}, // LKAS_HUD_ADAS
  };

  static const CanMsg BYD_TX_LONG_MSGS[] = {
    {482, 0, 8, .check_relay = true}, // STEERING_MODULE_ADAS
    {790, 0, 8, .check_relay = true}, // LKAS_HUD_ADAS
    {814, 0, 8, .check_relay = true}  // ACC_CMD
  };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{287, 0, 5, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, // STEER_MODULE_2
    {.msg = {{496, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // WHEEL_SPEED2
    {.msg = {{508, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // STEERING_TORQUE
    {.msg = {{834, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // PEDAL
    {.msg = {{944, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 20U}, { 0 }, { 0 }}},  // PCM_BUTTONS
    {.msg = {{814, 2, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // ACC_CMD
  };

  safety_config ret;
  if (byd_longitudinal) {
    ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_LONG_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }

  return ret;
}


const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};