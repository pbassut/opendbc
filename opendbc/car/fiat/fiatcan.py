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
