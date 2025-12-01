from opendbc.car.structs import CarParams
from opendbc.car.fiat.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.FASTBACK_LIMITED_EDITION_2024: {
    (Ecu.combinationMeter, 0x18DA60F1, None): [
      b'\x01\x01\x02\x10\x000',
    ],
    (Ecu.eps, 0x18DA30F1, None): [
      b'\x01\x0105.02.01  \x00\xcf'
    ],
    (Ecu.vsa, 0x18DA28F1, None): [
      b'\x01\x01K9FRAI0020\x00"'
    ],

  },
}
