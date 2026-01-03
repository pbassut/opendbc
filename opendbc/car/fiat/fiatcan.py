from opendbc.car.crc import CRC8J1850

PT_BUS = 0
DAS_BUS = 1

def create_lkas_command(packer, frame, apply_steer, enabled):
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_REQUEST_BIT": enabled,
    "COUNTER": frame,
  }
  return packer.make_can_msg("LKAS_COMMAND", PT_BUS, values)

def create_lkas_hud_command(packer, lat_active, mads_enabled=False, eps_faulted=False, beep=False, test=None):
  values = {
    "SOMETHING_HANDS_ON_WHEEL_2": 0,
    "BEEP": 1 if beep else 0,
    "LKAS_LED_STATUS": 1 if eps_faulted or not mads_enabled else 0,
    "HUD_WARNING_TYPE": 15 if eps_faulted else 0,
    "LANE_HUD_INDICATOR": 6 if lat_active else 1,
    "SET_ME_1": 1,
  }

  if eps_faulted:
    values["LANE_HUD_INDICATOR"] = 0

  if test is not None:
    values = {
      "SOMETHING_HANDS_ON_WHEEL_2": test % 4,
      "LKAS_LED_STATUS": test % 2,
      "LKAS_HUD_STATE": (test + 4) % 16,
      "LKAS_FAULTED_2": (test + 3) % 4,
    }

  return packer.make_can_msg("LKA_HUD_2", PT_BUS, values)

def create_cruise_buttons(packer, frame, activate=False):
  button = 32 if activate else 128
  values = {
    "CRUISE_BUTTON_PRESSED": button,
    "COUNTER": frame,
  }
  return packer.make_can_msg("DAS_1", DAS_BUS, values)

def create_gas_command(packer, throttle, frame):
  values = { "ACCEL_PEDAL_THRESHOLD": throttle, "COUNTER": frame }
  return packer.make_can_msg("ENGINE_1", DAS_BUS, values)

def create_friction_brake_command(packer, apply_brake, frame):
  values = { "BRAKE_PRESSURE": apply_brake, "COUNTER": frame }
  return packer.make_can_msg("ABS_6", DAS_BUS, values)

def fiat_fastback_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0xFF
  skip = 1
  if address == 0x2FA:
    # DAS_1 has the checksum on the byte before the last
    skip = 2

  for i in range(len(d) - skip):
    crc ^= d[i]
    crc = CRC8J1850[crc]

  return crc ^ 0xFF