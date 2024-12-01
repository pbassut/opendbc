from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_meas_steer_torque_limits
from opendbc.car.fiat import fiatcan
from opendbc.car.fiat.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase

CAM_BUS = 0
DAS_BUS = 2

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

    # cruise buttons
    das_bus = 1

    # ACC cancellation
    if CC.cruiseControl.cancel:
      self.last_button_frame = self.frame
      can_sends.append(fiatcan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, activate=False))

    # ACC resume from standstill
    elif CC.cruiseControl.resume:
      self.last_button_frame = self.frame
      can_sends.append(fiatcan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, activate=True))

    # steering
    if self.frame % self.params.STEER_STEP == 0:
      # steer torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params) * 0.3
      self.apply_steer_last = apply_steer

      can_sends.append(fiatcan.create_lkas_command(self.packer, self.frame, apply_steer, CC.enabled))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends
