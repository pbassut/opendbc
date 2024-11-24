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
    # can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.auto_high_beam = 0
    self.button_counter = 0
    #self.lkas_counter = 0
    self.lkas_watch_status = -1

    self.distance_button = 0

  def update(self, can_parsers, *_) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    #cp_cam = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()

    # prev_distance_button = self.distance_button
    self.distance_button = cp_adas.vl["DAS_1"]["CRUISE_BUTTON_PRESSED"]

    # prev_lkas_watch_status = self.lkas_watch_status
    #self.lkas_watch_status = cp_cam.vl["LKAS_COMMAND"]["LKAS_WATCH_STATUS"]

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    #ret.seatbeltUnlatched = cp.vl["SEATBELTS"]["SEATBELT_FL"] == 1
    ret.seatbeltUnlatched = False

    # brake pedal
    ret.brake = cp_adas.vl["ABS_6"]['BRAKE_PRESSURE']
    ret.brakePressed = ret.brake > 0

    # gas pedal
    ret.gas = cp.vl["ENGINE_1"]["ACCEL_PEDAL_THRESHOLD"]
    ret.gasPressed = ret.gas > 0

    # car speed
    ret.vEgoRaw = cp.vl["ABS_6"]["VEHICLE_SPEED"] * CV.KPH_TO_MS
    if cp_adas.vl['GEAR']['PARK'] == 1:
      ret.gearShifter = self.parse_gear_shifter('PARK')
    elif cp_adas.vl['GEAR']['D'] == 1:
      ret.gearShifter = self.parse_gear_shifter('D')
    elif cp_adas.vl['GEAR']['REVERSE'] == 1:
      ret.gearShifter = self.parse_gear_shifter('REVERSE')
    elif cp_adas.vl['GEAR']['N'] == 1:
      ret.gearShifter = self.parse_gear_shifter('N')

    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
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
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"]  #+ cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    #ret.steeringTorque = cp_cam.vl["LKAS_COMMAND"]["STEERING_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # cruise state
    ret.cruiseState.available = cp_adas.vl["DAS_2"]["ACC_STATE"] == 1
    ret.cruiseState.enabled = cp_adas.vl["DAS_2"]["ACC_ENGAGED"] == 1
    ret.cruiseState.speed = cp_adas.vl["DAS_2"]["ACC_SET_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = True
    # ret.cruiseState.standstill = cp_cruise.vl["DAS_3"]["ACC_STANDSTILL"] == 1
    ret.accFaulted = False

    ret.steerFaultTemporary = False
    ret.steerFaultPermanent = False

    # self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    self.button_counter = cp_adas.vl["DAS_1"]["COUNTER"]
    #self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]["COUNTER"]

    #ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

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
      ("ABS_1", 100),
      ("ABS_3", 100),
      ("BCM_1", 2),
      ('ENGINE_1', 99),
      ('SEATBELTS', 10),
      ('EPS_2', 10),
      ("ABS_6", 100),
    ]

    adas_messages = [
      ("GEAR", 1),
      ("ENGINE_2", 99),
      ("ABS_6", 100),
    ]

    adas_messages += CarState.get_cruise_messages()

    cam_messages = [
      #("DAS_3", 10),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
    }
