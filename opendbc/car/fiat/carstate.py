from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.fiat.values import DBC, STEER_THRESHOLD
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from openpilot.common.params import Params

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    self.mem_params = Params("/dev/shm/params")

    self.accel_counter = 0

    self.lkas_enabled = False
    self.prev_lkas_enabled = False

    self.prev_high_beam = False
    self.high_beam = False

    # Adaptive cruise control state
    self.adaptive_cruise_enabled = True
    self.prev_cruise_enabled = False
    self.brake_release_timer = 0
    self.last_cruise_speed = 0
    self.speed_adaptation_active = False

  def update(self, can_parsers, *_) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    #ret.seatbeltUnlatched = cp.vl["SEATBELTS"]["SEATBELT_FL"] == 1
    ret.seatbeltUnlatched = False

    # brake pedal
    ret.brake = cp.vl["ABS_6"]['BRAKE_PRESSURE']
    ret.brakePressed = ret.brake > 4

    # gas pedal
    ret.gas = cp_adas.vl["ENGINE_1"]["ACCEL_PEDAL_THRESHOLD"]
    ret.gasPressed = ret.gas > 0

    # car speed
    ret.vEgoRaw = cp_adas.vl["ABS_6"]["VEHICLE_SPEED"] * CV.KPH_TO_MS

    if cp_adas.vl['GEAR']['PARK'] == 1:
      ret.gearShifter = self.parse_gear_shifter('PARK')
    elif cp_adas.vl['GEAR']['DRIVE'] == 1:
      ret.gearShifter = self.parse_gear_shifter('DRIVE')
    elif cp_adas.vl['GEAR']['REVERSE'] == 1:
      ret.gearShifter = self.parse_gear_shifter('REVERSE')
    elif cp_adas.vl['GEAR']['NEUTRAL'] == 1:
      ret.gearShifter = self.parse_gear_shifter('NEUTRAL')

    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["ABS_1"]["WHEEL_SPEED_FL"],
      cp.vl["ABS_1"]["WHEEL_SPEED_FR"],
      cp.vl["ABS_1"]["WHEEL_SPEED_RL"],
      cp.vl["ABS_1"]["WHEEL_SPEED_RR"],
      unit=1,
    )

    # button presses
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
      200,
      cp.vl["BCM_2"]["LEFT_TURN_STALK"] == 1,
      cp.vl["BCM_2"]["RIGHT_TURN_STALK"] == 1
    )

    self.prev_high_beam = self.high_beam
    self.high_beam = cp_adas.vl["BCM_2"]["HIGH_BEAM"] == 1
    ret.genericToggle = cp_adas.vl["BCM_2"]["HIGH_BEAM"] == 1

    self.prev_lkas_enabled = self.lkas_enabled
    self.lkas_enabled = cp.vl["BUTTONS_1"]["LKAS_BUTTON"] == 1

    steer_always_on = self.mem_params.get_bool("SteerAlwaysOn")
    if not self.prev_lkas_enabled and self.lkas_enabled:
      ret.madsEnabled = not steer_always_on
      self.mem_params.put_bool('SteerAlwaysOn', not steer_always_on)

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    # ret.steerFaultTemporary = ret.vEgoRaw > self.CP.minSteerSpeed and cp.vl["EPS_2"]["STEERING_ALLOWED"] == 1
    ret.steerFaultPermanent = cp.vl["EPS_2"]["EPS_FAULT"] == 1
    ret.yawRate = cp.vl["ABS_2"]["YAW_RATE"]

    # cruise state
    ret.cruiseState.available = cp_adas.vl["DAS_2"]["ACC_STATE"] == 1
    ret.cruiseState.enabled = cp_adas.vl["DAS_2"]["ACC_ENGAGED"] == 1
    ret.cruiseState.speed = cp_adas.vl["DAS_2"]["ACC_SET_SPEED"] * CV.KPH_TO_MS

    # Adaptive cruise control logic for Fiat Fastback
    self._update_adaptive_cruise(ret)

    self.accel_counter = cp_adas.vl["ACCEL_1"]["COUNTER"]

    return ret

  def _update_adaptive_cruise(self, ret):
    """
    Adaptive cruise control logic for Fiat Fastback:
    1. If accelerating above cruise speed, update cruise speed to current speed
    2. If braking disables cruise, re-enable at current speed after half second
    """
    if not self.adaptive_cruise_enabled:
      return

    current_speed_kph = ret.vEgo * CV.MS_TO_KPH
    cruise_speed_kph = ret.cruiseState.speed * CV.MS_TO_KPH if ret.cruiseState.speed > 0 else 0

    # Detect cruise state changes
    cruise_was_enabled = self.prev_cruise_enabled
    cruise_now_enabled = ret.cruiseState.enabled
    self.prev_cruise_enabled = cruise_now_enabled

    # Case 1: Cruise is enabled and we're accelerating above set speed
    if cruise_now_enabled and cruise_speed_kph > 0:
      speed_diff = current_speed_kph - cruise_speed_kph

      # If current speed is 5+ km/h above cruise speed, update cruise speed
      if speed_diff >= 2.0 and ret.gasPressed:
        self.speed_adaptation_active = True
        # Signal to update cruise speed to current speed
        ret.cruiseState.speed = ret.vEgo
        self.last_cruise_speed = current_speed_kph

    # Case 2: Cruise was disabled (likely by braking), start timer
    if cruise_was_enabled and not cruise_now_enabled and ret.brakePressed:
      self.brake_release_timer = 0
      self.last_cruise_speed = current_speed_kph

    # Case 3: Brake released after cruise was disabled - start timer
    if not cruise_now_enabled and not ret.brakePressed and self.brake_release_timer >= 0:
      self.brake_release_timer += 1

      # After 0.5 seconds (50 frames at 100Hz), re-enable cruise at current speed
      if self.brake_release_timer >= 50:
        # Create a button event to simulate cruise set button press
        if not hasattr(ret, 'buttonEvents'):
          ret.buttonEvents = []

        # Add setCruise button event to re-enable cruise control
        button_event = structs.CarState.ButtonEvent()
        button_event.type = ButtonType.setCruise
        button_event.pressed = False  # Button release triggers action
        ret.buttonEvents.append(button_event)

        # Set cruise speed to current speed
        ret.cruiseState.speed = ret.vEgo
        ret.cruiseState.enabled = True
        self.brake_release_timer = -1  # Disable timer

  @staticmethod
  def get_cruise_messages():
    cruise_messages = [
      ("DAS_1", 50),
      ("DAS_2", 1),
    ]
    return cruise_messages

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      # sig_address, frequency
      ("BCM_1", 1),
      ("BCM_2", 4),
      ("STEERING", 100),
      ("ABS_1", 100),
      ("ABS_2", 100),
      ("ABS_3", 100),
      ('SEATBELTS', 10),
      ('EPS_2', 100),
      ("ABS_6", 100),
      ("BUTTONS_1", 4),
    ]

    adas_messages = [
      ("BCM_2", 4),
      ("GEAR", 1),
      ('ENGINE_1', 100),
      ("ENGINE_2", 99),
      ("ABS_6", 100),
      ("ACCEL_1", 100),
    ]

    adas_messages += CarState.get_cruise_messages()

    cam_messages = [
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
    }
