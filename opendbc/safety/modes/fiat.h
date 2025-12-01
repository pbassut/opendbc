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

  // Measured driver torque
  if ((bus == 0) && (addr == fiat_addrs->EPS_2)) {
    uint16_t torque_driver_new = (GET_BYTES(msg, 2, 2) >> 5) & 0x7FFU;

    update_sample(&torque_driver, (torque_driver_new * -1) + 1024U);
  }

  if (bus == 1 && addr == fiat_addrs->DAS_2) {
    bool acc_state = GET_BIT(msg, 21U) == 1U;
    pcm_cruise_check(acc_state);

    acc_main_on = GET_BIT(msg, 7U) == 1U;
  }

  if (bus == 0 && addr == fiat_addrs->ABS_6) {
    vehicle_moving = ((GET_BYTES(msg, 1, 2) >> 5) & 0x7FFU) > 0;
  }

  if (bus == 1 && addr == fiat_addrs->ENGINE_1) {
    gas_pressed = (((GET_BYTES(msg, 2, 2) >> 5) & 0xFFU) * 0.3942) > 0;
  }

  if (bus == 0 && addr == fiat_addrs->ABS_6) {
    brake_pressed = ((GET_BYTES(msg, 2, 2) >> 2) & 0x7FFU) > 0;
  }
}

static bool fiat_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  int addr = msg->addr;

  // STEERING
  if (addr == fiat_addrs->LKAS_COMMAND) {
    const TorqueSteeringLimits limits = {
      .max_torque = 1440,
      .max_rt_delta = 112,
      .max_rate_up = 4,
      .max_rate_down = 3,
      .driver_torque_allowance = 15,
      .type = TorqueDriverLimited,
    };

    int desired_torque = ((GET_BYTES(msg, 0, 2) >> 5) & 0x7FFU) - 1024;

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
};

static safety_config fiat_init(uint16_t param) {
  gen_crc_lookup_table_8(0x1D, fca_fastback_crc8_lut_j1850);

  static RxCheck fastback_rx_checks[] = {
    {.msg = {{FASTBACK_ADDRS.ABS_6,         0, 8, .ignore_quality_flag = true,      .max_counter = 15U }, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.DAS_1,         1, 4, .ignore_quality_flag = true,      .max_counter = 15U },  { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.DAS_2,         1, 8, .ignore_quality_flag = false,     .max_counter = 0U },   { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.EPS_2,         0, 7, .ignore_quality_flag = true,      .max_counter = 15U },  { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.ENGINE_1,      1, 8, .ignore_quality_flag = true,      .max_counter = 15U },  { 0 }, { 0 }}},
  };

  static const CanMsg FASTBACK_TX_MSGS[] = {
    // {CHRYSLER_RAM_DT_ADDRS.CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {FASTBACK_ADDRS.LKAS_COMMAND, 0, 4, .check_relay = true},
    {FASTBACK_ADDRS.LKA_HUD_2,    0, 8, .check_relay = false},
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


