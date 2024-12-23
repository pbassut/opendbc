from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.fiat.values import DBC, STEER_THRESHOLD
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP

    self.auto_high_beam = 0
    self.button_counter = 0
    self.lkas_watch_status = -1

    self.distance_button = 0

  def update(self, can_parsers, *_) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()

    self.distance_button = cp_adas.vl["DAS_1"]["CRUISE_BUTTON_PRESSED"]


    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    #ret.seatbeltUnlatched = cp.vl["SEATBELTS"]["SEATBELT_FL"] == 1
    ret.seatbeltUnlatched = False

    # brake pedal
    ret.brake = cp.vl["ABS_6"]['BRAKE_PRESSURE']
    ret.brakePressed = ret.brake > 0

    # gas pedal
    ret.gas = cp.vl["ENGINE_1"]["ACCEL_PEDAL_THRESHOLD"]
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
    # ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
    # 200, cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1, cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2)
    # ret.genericToggle = cp.vl["STEERING_LEVERS"]["HIGH_BEAM_PRESSED"] == 1

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    # ret.steerFaultTemporary = cp.vl["EPS_3"]["DASM_FAULT"] == 1
    ret.yawRate = cp.vl["ABS_2"]["YAW_RATE"]

    # cruise state
    ret.cruiseState.available = cp_adas.vl["DAS_2"]["ACC_STATE"] == 1
    ret.cruiseState.enabled = cp_adas.vl["DAS_2"]["ACC_ENGAGED"] == 1
    # this is wrong
    ret.cruiseState.speed = cp_adas.vl["DAS_2"]["ACC_SET_SPEED"] * CV.KPH_TO_MS

    self.button_counter = cp_adas.vl["DAS_1"]["COUNTER"]

    return ret

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
      ("STEERING", 100),
      ("ABS_1", 80),
      ("ABS_2", 100),
      ("ABS_3", 100),
      ("BCM_1", 1),
      ('ENGINE_1', 99),
      ('SEATBELTS', 10),
      ('EPS_2', 50),
      ("ABS_6", 100),
    ]

    adas_messages = [
      ("GEAR", 1),
      ("ENGINE_2", 99),
      ("ABS_6", 100),
    ]

    adas_messages += CarState.get_cruise_messages()

    cam_messages = [
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
    }
