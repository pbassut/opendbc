#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.common.conversions import Conversions as CV


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fiat"

    ret.radarUnavailable = True
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fiat)]

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    # tune.torque.kp = 1.0
    ret.lateralTuning.torque.kpDEPRECATED = 0.11

    # tune.torque.ki = 0.1
    ret.lateralTuning.torque.kiDEPRECATED = 0.37

    # ret.lateralTuning.torque.kf = 0
    ret.lateralTuning.torque.kfDEPRECATED = 0.07

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = False

    # ret.experimentalLongitudinalAvailable = True
    ret.pcmCruise = not alpha_long
    ret.openpilotLongitudinalControl = alpha_long

    ret.minSteerSpeed = 10 * CV.KPH_TO_MS
    ret.minEnableSpeed = 10 * CV.KPH_TO_MS

    # Tuning for alpha long
    ret.longitudinalTuning.kiBP = [5., 35.]
    ret.longitudinalTuning.kiV = [2.0, 1.5]
    ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
    ret.vEgoStopping = 0.25
    ret.vEgoStarting = 0.25

    return ret
