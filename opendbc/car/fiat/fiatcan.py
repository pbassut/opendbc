PT_BUS = 0
DAS_BUS = 1

def create_lkas_command(packer, frame, apply_steer, enabled):
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_REQUEST_BIT": enabled,
    "COUNTER": frame,
  }
  return packer.make_can_msg("LKAS_COMMAND", PT_BUS, values)

def create_lkas_hud_command(packer, lat_active, eps_faulted, test=None):
  values = {
    "SOMETHING_HANDS_ON_WHEEL_2": 0,
    "BEEP": 0,
    "LKAS_LED_STATUS": 1 if eps_faulted else 0,
    "HUD_WARNING_TYPE": 15 if eps_faulted else 0,
    "LANE_HUD_INDICATOR": 6 if lat_active else 1,
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
  return packer.make_can_msg("ENGINE_1", PT_BUS, values)

def create_friction_brake_command(packer, apply_brake, frame):
  values = { "BRAKE_PRESSURE": apply_brake, "COUNTER": frame }
  return packer.make_can_msg("ABS_6", PT_BUS, values)
