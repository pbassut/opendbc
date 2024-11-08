from enum import IntFlag
from dataclasses import dataclass, field

from panda import uds
from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class FastbackFlags(IntFlag):
  # Detected flags
  HIGHER_MIN_STEERING_SPEED = 1

@dataclass
class FastbackCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.fca]))


@dataclass
class FastbackPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('chrysler_pacifica_2017_hybrid_generated', 'chrysler_pacifica_2017_hybrid_private_fusion'))


@dataclass(frozen=True)
class FastbackCarSpecs(CarSpecs):
  minSteerSpeed: float = 3.8  # m/s


class CAR(Platforms):
  FASTBACK_LIMITED_EDITION_2024 = FastbackPlatformConfig(
    [FastbackCarDocs("Fastback Limited Edition 2024")],
    FastbackCarSpecs(mass=2242., wheelbase=3.089, steerRatio=16.2),
  )

class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 2  # 50 Hz
    self.STEER_ERROR_MAX = 80

    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 3
    self.STEER_MAX = 261  # higher than this faults the EPS


STEER_THRESHOLD = 120

CHRYSLER_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf132)
CHRYSLER_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf132)

CHRYSLER_SOFTWARE_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)
CHRYSLER_SOFTWARE_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)

CHRYSLER_RX_OFFSET = -0x280

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [CHRYSLER_VERSION_REQUEST],
      [CHRYSLER_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.srs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.combinationMeter],
      rx_offset=CHRYSLER_RX_OFFSET,
      bus=0,
    ),
    Request(
      [CHRYSLER_VERSION_REQUEST],
      [CHRYSLER_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.hybrid, Ecu.engine, Ecu.transmission],
      bus=0,
    ),
    Request(
      [CHRYSLER_SOFTWARE_VERSION_REQUEST],
      [CHRYSLER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.transmission],
      bus=0,
    ),
  ],
  extra_ecus=[
    (Ecu.abs, 0x7e4, None),  # alt address for abs on hybrids, NOTE: not on all hybrid platforms
  ],
)

DBC = CAR.create_dbc_map()
