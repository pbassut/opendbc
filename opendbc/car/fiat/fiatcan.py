from opendbc.car import structs

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def crc8(data):
  crc = 0xFF
  poly = 0x1D

  for byte in data:
    crc ^= byte

    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc ^ 0xFF

def test_crc(msg, checksum_byte, length=3):
  data_bytes = msg.to_bytes(length)
  computed_checksum = crc8(data_bytes)
  is_valid = computed_checksum == checksum_byte

  print(f"Data Bytes: 0x{data_bytes.hex().upper()} | Provided Checksum: 0x{checksum_byte:02X}")
  print(f"Computed Checksum: 0x{computed_checksum:02X} | Valid: {is_valid}")
  print('-' * 50)

def create_lkas_hud(packer, CP, lkas_active, hud_alert, hud_count, car_model, auto_high_beam):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left cross cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

  color = 2 if lkas_active else 1
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts,
  }

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, frame, apply_steer):
  crc_bytes = crc8((apply_steer + 0x1 + (frame % 0x10)).to_bytes(3))
  print(frame, crc_bytes)
  # LKAS_COMMAND Lane-keeping signal to turn the wheel
  values = {
    "STEERING_TORQUE": apply_steer,
    "COUNTER": frame % 0x10,
    "CHECKSUM": crc_bytes,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, button, bus, activate=False):
  crc_bytes = crc8((button + (frame % 0x10)).to_bytes(2))
  print(frame, button, crc_bytes)
  values = {
    "CRUISE_BUTTON_PRESSED": 8 if activate else 128,
    "COUNTER": frame % 0x10,
    "CHECKSUM": crc_bytes,
  }
  return packer.make_can_msg("DAS_1", bus, values)
