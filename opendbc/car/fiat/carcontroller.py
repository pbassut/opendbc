from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_meas_steer_torque_limits
from opendbc.car.fiat import fiatcan
from opendbc.car.fiat.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase

DAS_BUS = 1

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_steer_last = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # longitudinal control
    if self.CP.openpilotLongitudinalControl:
      # Gas, brakes, and UI commands - all at 100Hz
      stopping = actuators.longControlState == LongCtrlState.stopping
      # self.apply_gas = int(round(interp(actuators.accel, self.params.GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
      # self.apply_brake = int(round(interp(actuators.accel, self.params.BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))
      self.apply_gas = int(round(interp(actuators.accel)))
      self.apply_brake = int(round(interp(actuators.accel)))

      at_full_stop = at_full_stop and stopping
      near_stop = CC.longActive and (abs(CS.out.vEgo) < self.params.NEAR_STOP_BRAKE_PHASE)

      can_sends.append(fiatcan.create_gas_command(self.packer, DAS_BUS, self.apply_gas, self.frame, CC.enabled, at_full_stop))
      can_sends.append(fiatcan.create_friction_brake_command(self.packer, DAS_BUS, self.apply_brake,
                                                            self.frame, CC.enabled, near_stop, at_full_stop, self.CP))

      # Send dashboard UI commands (ACC status)
      # send_fcw = hud_alert == VisualAlert.fcw
      # can_sends.append(fiatcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, CC.enabled,
      #                                                     hud_v_cruise * CV.MS_TO_KPH, hud_control, send_fcw))

    # cruise buttons
    # ACC cancellation
    if CC.cruiseControl.cancel:
      self.last_button_frame = self.frame
      can_sends.append(fiatcan.create_cruise_buttons(self.packer, CS.button_counter + 1, DAS_BUS, activate=False))

    # ACC resume from standstill
    elif CC.cruiseControl.resume:
      self.last_button_frame = self.frame
      can_sends.append(fiatcan.create_cruise_buttons(self.packer, CS.button_counter + 1, DAS_BUS, activate=True))

    # steering
    if self.frame % self.params.STEER_STEP == 0:
      # steer torque
      new_steer = CC.actuators.steer * self.params.STEER_MAX
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params) * 0.98
      self.apply_steer_last = apply_steer

      can_sends.append(fiatcan.create_lkas_command(self.packer, self.frame, apply_steer, CC.enabled))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake

    return new_actuators, can_sends
