#!/usr/bin/env python3
from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.fiat.values import CAR, FastbackFlags
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "fiat"

    # radar parsing needs some work, see https://github.com/commaai/openpilot/issues/26842
    ret.radarUnavailable = True # DBC[candidate]['radar'] is None
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    #ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.chrysler)]
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]
    #ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.allOutput)]
    #ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Newer FW versions standard on the following platforms, or flashed by a dealer onto older platforms have a higher minimum steering speed.
    new_eps_platform = candidate in (CAR.FASTBACK_LIMITED_EDITION_2024)
    new_eps_firmware = any(fw.ecu == 'eps' and fw.fwVersion[:4] >= b"6841" for fw in car_fw)
    if new_eps_platform or new_eps_firmware:
      ret.flags |= FastbackFlags.HIGHER_MIN_STEERING_SPEED.value

    if ret.flags & FastbackFlags.HIGHER_MIN_STEERING_SPEED:
      # TODO: allow these cars to steer down to 13 m/s if already engaged.
      # TODO: Durango 2020 may be able to steer to zero once above 38 kph
      ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = False

    return ret
