from opendbc.car import structs

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def create_lkas_command(packer, frame, apply_steer, control_enabled):
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_WATCH_STATUS": control_enabled,
    "COUNTER": frame,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, activate=False):
  button = 32 if activate else 128
  values = {
    "CRUISE_BUTTON_PRESSED": button,
    "COUNTER": frame,
  }
  return packer.make_can_msg("DAS_1", bus, values)

def create_gas_command(packer, bus, throttle, frame, enabled, at_full_stop):
  values = { "ACCEL_PEDAL_THRESHOLD": throttle, }
  return packer.make_can_msg("ENGINE_1", bus, values)

def create_friction_brake_command(packer, bus, apply_brake, idx, enabled, near_stop, at_full_stop, CP):
  values = {
    "BRAKE_PRESSURE": apply_brake
  }

  return packer.make_can_msg("ABS_6", bus, values)

def create_acc_dashboard_command(packer, bus, enabled, target_speed_kph, hud_control, fcw):
  target_speed = min(target_speed_kph, 255)

  values = {
    "ACC_SET_SPEED": target_speed,
  }

  return packer.make_can_msg("DAS_2", bus, values)
