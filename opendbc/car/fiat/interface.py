#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "fiat"

    ret.radarUnavailable = True
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fcaFastback)]

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = False

    return ret
