#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fiat.carstate import CarState
from opendbc.car.fiat.carcontroller import CarController
from opendbc.car.fiat.radar_interface import RadarInterface


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fiat"

    ret.radarUnavailable = True

    # safety config
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fiat)]

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4
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
