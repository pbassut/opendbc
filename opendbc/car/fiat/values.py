from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig

Ecu = CarParams.Ecu

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
  minSteerSpeed: float = 0  # m/s
  tireStiffnessFactor: float = .97  # not optimized yet

class CAR(Platforms):
  FASTBACK_LIMITED_EDITION_2024 = FastbackPlatformConfig(
    [FastbackCarDocs("Fastback Limited Edition 2024")],
    FastbackCarSpecs(mass=1253., wheelbase=2.695, steerRatio=16.89),
  )

class CarControllerParams:
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2

  def __init__(self, CP):
    self.STEER_MAX = 1440
    self.STEER_DELTA_UP = 4
    self.STEER_DELTA_DOWN = 3
    self.STEER_ERROR_MAX = 250

    self.STEER_DRIVER_MULTIPLIER = 4  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1  # from dbc
    self.STEER_DRIVER_ALLOWANCE = 15

    self.NEAR_STOP_BRAKE_PHASE = 0.5  # m/s

    self.ZERO_GAS = 2048  # Coasting
    self.MAX_BRAKE = 400  # ~ -4.0 m/s^2 with regen
    self.MAX_GAS = 3400
    self.MAX_ACC_REGEN = 1514
    self.INACTIVE_REGEN = 1554
    # Camera ACC vehicles have no regen while enabled.
    # Camera transitions to MAX_ACC_REGEN from ZERO_GAS and uses friction brakes instantly
    max_regen_acceleration = 0.

    self.GAS_LOOKUP_BP = [max_regen_acceleration, 0., self.ACCEL_MAX]
    self.GAS_LOOKUP_V = [self.MAX_ACC_REGEN, self.ZERO_GAS, self.MAX_GAS]

    self.BRAKE_LOOKUP_BP = [self.ACCEL_MIN, max_regen_acceleration]
    self.BRAKE_LOOKUP_V = [self.MAX_BRAKE, 0.]


STEER_THRESHOLD = 100

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
  ],
  extra_ecus=[
  ],
)

DBC = CAR.create_dbc_map()
