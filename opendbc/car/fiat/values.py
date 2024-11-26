from enum import IntFlag
from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig

Ecu = CarParams.Ecu


class FastbackFlags(IntFlag):
  # Detected flags
  HIGHER_MIN_STEERING_SPEED = 1

@dataclass
class FastbackCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control(ACC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.fca]))


@dataclass
class FastbackPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: "fca_fastback_limited_edition_2024_generated",
    Bus.adas: "fca_fastback_limited_edition_2024_generated",
    Bus.cam: "fca_fastback_limited_edition_2024_generated",
  })


@dataclass(frozen=True)
class FastbackCarSpecs(CarSpecs):
  minSteerSpeed: float = 8.3  # m/s


class CAR(Platforms):
  FASTBACK_LIMITED_EDITION_2024 = FastbackPlatformConfig(
    [FastbackCarDocs("Fastback Limited Edition 2024")],
    FastbackCarSpecs(mass=2242., wheelbase=3.089, steerRatio=16.2),
  )

class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 1  # 100 Hz

    self.STEER_MAX = 261  # higher than this faults the EPS
    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 3
    self.STEER_ERROR_MAX = 80

    self.STEER_DRIVER_ALLOWANCE = 80
    self.STEER_DRIVER_MULTIPLIER = 3  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1  # from dbc


STEER_THRESHOLD = 120

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
  ],
  extra_ecus=[
  ],
)

DBC = CAR.create_dbc_map()
