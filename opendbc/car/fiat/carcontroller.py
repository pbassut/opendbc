import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_meas_steer_torque_limits
from opendbc.car.fiat import fiatcan
from opendbc.car.fiat.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

    self.params = CarControllerParams(CP)
    self.apply_steer_last = 0
    self.apply_brake = 0
    self.apply_gas = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.last_button_frame = 0
    self.test_counter = 0

    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.packer_adas = CANPacker(dbc_names[Bus.adas])
    self.params = CarControllerParams(CP)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CS.high_beam and not CS.prev_high_beam:
      self.test_counter += 1

    # cruise buttons
    # ACC cancellation
    if CC.cruiseControl.cancel:
      can_sends.append(fiatcan.create_cruise_buttons(self.packer_pt, CS.button_counter + 1, activate=False))

    # ACC resume from standstill
    elif CC.cruiseControl.resume:
      can_sends.append(fiatcan.create_cruise_buttons(self.packer_pt, CS.button_counter + 1, activate=True))

    # longitudinal control
    if self.CP.openpilotLongitudinalControl and CC.longActive:
      # Gas, brakes, and UI commands - all at 100Hz
      self.apply_gas = int(round(np.interp(actuators.accel)))
      self.apply_brake = int(round(np.interp(actuators.accel)))

      can_sends.append(fiatcan.create_gas_command(self.packer_adas, self.apply_gas, CS.accel_counter + 1))
      can_sends.append(fiatcan.create_friction_brake_command(self.packer_adas, self.apply_brake, CS.accel_counter + 1))

    if self.test_counter % 4 == 0:
      can_sends.append(fiatcan.create_gas_command(self.packer_adas, 12, CS.accel_counter + 1))

    # steering
    # steer torque
    apply_steer = 0
    if CC.latActive:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params) * 0.98

    self.apply_steer_last = apply_steer
    can_sends.append(fiatcan.create_lkas_command(self.packer_pt, self.frame, apply_steer, CS.out.vEgo > self.CP.minSteerSpeed))

    if self.frame % 25 == 0:
      eps_faulted = CS.out.steerFaultPermanent or CS.out.steerFaultTemporary
      can_sends.append(fiatcan.create_lkas_hud_command(self.packer_pt, CC.latActive, eps_faulted))

    self.frame += 1

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake

    return new_actuators, can_sends
