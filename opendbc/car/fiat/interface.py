#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.common.conversions import Conversions as CV

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "fiat"

    ret.radarUnavailable = True
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fiat)]

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.lateralTuning.torque.kp = 0.11
    ret.lateralTuning.torque.ki = 0.365
    ret.lateralTuning.torque.kf = 0.057

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = False

    ret.experimentalLongitudinalAvailable = True
    ret.pcmCruise = False
    ret.openpilotLongitudinalControl = experimental_long

    ret.minSteerSpeed = 10 * CV.KPH_TO_MS
    ret.minEnableSpeed = 10 * CV.KPH_TO_MS

    # Tuning for experimental long
    ret.longitudinalTuning.kiBP = [5., 35.]
    ret.longitudinalTuning.kiV = [2.0, 1.5]
    ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
    ret.vEgoStopping = 0.25
    ret.vEgoStarting = 0.25

    return ret
