#pragma once

#include "opendbc/safety/declarations.h"

typedef struct {
  const int ABS_6;
  const int DAS_1;
  const int DAS_2;
  const int EPS_2;
  const int ENGINE_1;
  const int LKAS_COMMAND;
  const int LKA_HUD_2;
  const int BUTTONS_1;
} FiatAddrs;

typedef enum {
  FASTBACK_LIMITED_EDITION,
} FiatPlatform;
static FiatPlatform fiat_platform;
static const FiatAddrs *fiat_addrs;

uint8_t fca_fastback_crc8_lut_j1850[256];  // Static lookup table for CRC8 SAE J1850

static uint32_t fca_fastback_get_checksum(const CANPacket_t *msg) {
  int checksum_pos = 1U;

  if(msg->addr == fiat_addrs->DAS_1) {
    checksum_pos = 2U;
  }

  int checksum_byte = GET_LEN(msg) - checksum_pos;
  return (uint8_t)(msg->data[checksum_byte]);
}

static uint8_t fca_fastback_get_counter(const CANPacket_t *msg) {
  if(msg->addr == fiat_addrs->DAS_1) {
    return msg->data[1U] & 0xF;
  }

  return (msg->data[GET_LEN(msg) - 2U]) & 0xF;
}

static uint32_t fca_fastback_compute_crc(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  // CRC is in the last byte, poly is same as SAE J1850 but uses a different init value and output XOR
  uint8_t crc = 0xFF;
  uint8_t final_xor = 0xFF;

  int crc_pos = 1U;

  if(msg->addr == fiat_addrs->DAS_1) {
    // for DAS_1 only bytes before the checksum are taken in account;
    crc_pos = 2U;
  }

  for (int i = 0; i < (len - crc_pos); i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = fca_fastback_crc8_lut_j1850[crc];
  }

  return (uint8_t)(crc ^ final_xor);
}

static void fiat_rx_hook(const CANPacket_t *msg) {
  const int bus = msg->bus;
  const int addr = msg->addr;

  if (bus == 0 && addr == fiat_addrs->BUTTONS_1) {
    bool lkas_button_pressed = (msg->data[3] & 0x40U) > 0;
    if(lkas_button_pressed && !lateral_controls_allowed_prev) {
      lateral_controls_allowed = true;
    }

    lateral_controls_allowed_prev = lateral_controls_allowed;
  }

  // Measured driver torque - DRIVER_TORQUE: 23|11@0+ (Motorola)
  // MSB at bit 23 (byte 2), spans bytes 2-3
  if ((bus == 0) && (addr == fiat_addrs->EPS_2)) {
    uint16_t torque_driver_new = (msg->data[2] << 3) | (msg->data[3] >> 5);
    update_sample(&torque_driver, torque_driver_new - 1024U);
  }

  // ACC_ENGAGED: 21|1@0+ and ACC_STATE: 7|1@0+ - single bits work with GET_BIT
  if (bus == 1 && addr == fiat_addrs->DAS_2) {
    bool acc_state = GET_BIT(msg, 21U) == 1U;
    pcm_cruise_check(acc_state);

    acc_main_on = GET_BIT(msg, 7U) == 1U;
  }

  if (bus == 0 && addr == fiat_addrs->ABS_6) {
    // VEHICLE_SPEED: 15|11@0+ (Motorola) - MSB at bit 15 (byte 1)
    uint16_t vehicle_speed = (msg->data[1] << 3) | (msg->data[2] >> 5);
    vehicle_moving = vehicle_speed > 0;

    // BRAKE_PRESSURE: 20|11@0+ (Motorola) - MSB at bit 20 (byte 2, bit 4)
    uint16_t brake_pressure = ((msg->data[2] & 0x1FU) << 6) | (msg->data[3] >> 2);
    brake_pressed = brake_pressure > 0;
  }

  // ACCEL_PEDAL_THRESHOLD: 20|8@0+ (Motorola) - MSB at bit 20 (byte 2, bit 4)
  if (bus == 1 && addr == fiat_addrs->ENGINE_1) {
    uint8_t accel_pedal = ((msg->data[2] & 0x1FU) << 3) | (msg->data[1] >> 5);
    gas_pressed = accel_pedal > 0;
  }
}

static bool fiat_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  int addr = msg->addr;

  // STEERING - STEERING_TORQUE: 7|11@0+ (Motorola)
  // MSB at bit 7 (byte 0), spans bytes 0-1
  if (addr == fiat_addrs->LKAS_COMMAND) {
    const TorqueSteeringLimits limits = {
      .max_torque = 360,
      .max_rt_delta = 112,
      .max_rate_up = 4,
      .max_rate_down = 4,
      .driver_torque_allowance = 80,
      .type = TorqueDriverLimited,
    };

    int desired_torque = ((msg->data[0] << 3) | (msg->data[1] >> 5)) - 1024;

    bool steer_req = GET_BIT(msg, 12U);
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (addr == fiat_addrs->DAS_1) {
    const bool is_cancel = GET_BIT(msg, 7U);
    const bool is_acc_set = GET_BIT(msg, 5U);
    const bool allowed = is_cancel || (is_acc_set && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  return tx;
}

static bool fiat_fwd_hook(int bus_num, int addr) {
  // forward to camera
  if (bus_num == 0) {
    return false;
  }

  // forward all messages from camera except LKAS messages
  const bool is_lkas = addr == fiat_addrs->LKAS_COMMAND;
  const bool is_lkas_hud = addr == fiat_addrs->LKA_HUD_2;
  if ((bus_num == 2) && !is_lkas && !is_lkas_hud){
    return true;
  }

  return false;
}

const FiatAddrs FASTBACK_ADDRS = {
  .ABS_6            = 0x101,
  .DAS_1            = 0x2FA,
  .DAS_2            = 0x5A5,
  .EPS_2            = 0x106,
  .ENGINE_1         = 0xFC,
  .LKAS_COMMAND     = 0x1F6,
  .LKA_HUD_2        = 0x547,
  .BUTTONS_1        = 0x384,
};

static safety_config fiat_init(uint16_t param) {
  gen_crc_lookup_table_8(0x1D, fca_fastback_crc8_lut_j1850);

  static RxCheck fastback_rx_checks[] = {
    {.msg = {{FASTBACK_ADDRS.ABS_6,         0, 8, 100U, .max_counter = 15U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.DAS_1,         1, 4, 50U,  .max_counter = 15U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.DAS_2,         1, 8, 1U,   .max_counter = 0U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.EPS_2,         0, 7, 100U, .max_counter = 15U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.ENGINE_1,      1, 8, 100U, .max_counter = 15U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.BUTTONS_1,     0, 8, 4U,   .max_counter = 0U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

    // {.msg = {{FASTBACK_ADDRS.BCM_2,         0, 4, 2U,   .max_counter = 0U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    // {.msg = {{FASTBACK_ADDRS.SEATBELTS,     0, 8, 5U,   .max_counter = 0U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    // {.msg = {{FASTBACK_ADDRS.GEAR,          2, 3, 1U,   .max_counter = 0U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
  };

  static const CanMsg FASTBACK_TX_MSGS[] = {
    {FASTBACK_ADDRS.LKAS_COMMAND, 0, 4, .check_relay = true},
    {FASTBACK_ADDRS.LKA_HUD_2,    0, 8, .check_relay = true},
    {FASTBACK_ADDRS.DAS_1,        1, 4, .check_relay = false},
    {FASTBACK_ADDRS.ENGINE_1,     1, 8, .check_relay = false},
  };

  fiat_platform = FASTBACK_LIMITED_EDITION;
  fiat_addrs = &FASTBACK_ADDRS;

  SAFETY_UNUSED(param);
  return BUILD_SAFETY_CFG(fastback_rx_checks, FASTBACK_TX_MSGS);
}

const safety_hooks fiat_hooks = {
  .init = fiat_init,
  .rx = fiat_rx_hook,
  .tx = fiat_tx_hook,
  .fwd = fiat_fwd_hook,
  .get_counter = fca_fastback_get_counter,
  .get_checksum = fca_fastback_get_checksum,
  .compute_checksum = fca_fastback_compute_crc,
};


