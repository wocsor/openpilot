from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

# Steer torque limits
class SteerLimitParams:
  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

class CAR:
  CELICA = "TOYOTA CELICA 2003"

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "setCruise": False,
  "resumeCruise": False,
}

# addr: (ecu, cars, bus, 1/freq*100, vl)
STATIC_MSGS = [
  (0x130, Ecu.fwdCamera, (CAR.CELICA), 1, 100, b'\x00\x00\x00\x00\x00\x00\x38'),
  (0x240, Ecu.fwdCamera, (CAR.CELICA), 1,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x241, Ecu.fwdCamera, (CAR.CELICA), 1,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x244, Ecu.fwdCamera, (CAR.CELICA), 1,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x245, Ecu.fwdCamera, (CAR.CELICA), 1,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x248, Ecu.fwdCamera, (CAR.CELICA), 1,   5, b'\x00\x00\x00\x00\x00\x00\x01'),
  (0x367, Ecu.fwdCamera, (CAR.CELICA), 0,  40, b'\x06\x00'),
  (0x414, Ecu.fwdCamera, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x00\x00\x00\x17\x00'),
  (0x466, Ecu.fwdCamera, (CAR.CELICA), 1, 100, b'\x24\x20\xB1'),
  (0x489, Ecu.fwdCamera, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x00\x00\x00\x00'),
  (0x48a, Ecu.fwdCamera, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x00\x00\x00\x00'),
  (0x48b, Ecu.fwdCamera, (CAR.CELICA), 0, 100, b'\x66\x06\x08\x0a\x02\x00\x00\x00'),
  (0x4d3, Ecu.fwdCamera, (CAR.CELICA), 0, 100, b'\x1C\x00\x00\x01\x00\x00\x00\x00'),

  (0x3b1, Ecu.dsu, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x01\x00\x08\x00\x00'),
  (0x1c4, Ecu.dsu, (CAR.CELICA), 0, 2, b'\x05\xea\x1b\x08\x00\x00\xc0\x9f'),
  # (0x1d2, Ecu.dsu, (CAR.CELICA), 0, 3, b'\xf8\x30\xf9\xe8\x00\x54\x80\xb8'),
  # (0x1d3, Ecu.dsu, (CAR.CELICA), 0, 3, b'\x00\xa8\x41\x81\x16\x00\x00\x5c'),
  (0x2c1, Ecu.dsu, (CAR.CELICA), 0, 3, b'\x08\x07\x07\x06\x70\xf8\x00\x4f'),
  (0x3d3, Ecu.dsu, (CAR.CELICA), 0, 50, b'\x00\x00'),
  (0x399, Ecu.dsu, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x00\x00\x00\x00\x00'),
  (0x3bb, Ecu.dsu, (CAR.CELICA), 0, 100, b'\x00\x00\x26\x00'),
  (0x3f9, Ecu.dsu, (CAR.CELICA), 0, 20, b'\x76\x18\x26\x01\x00\x00\x00\xb9'),
  (0x3bc, Ecu.dsu, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x00\x00\x80\x00\x00'),
  #(0x24, Ecu.dsu, (CAR.CELICA), 0, 1, b'\x02\x00\x01\xfd\x42\x03\x80\xf1'),
  (0x3b1, Ecu.dsu, (CAR.CELICA), 0, 100, b'\x00\x00\x00\x01\x00\x08\x00\x00'),
  (0x4ac, Ecu.dsu, (CAR.CELICA), 0, 50, b'\x28\x00\x60\x01\x0a\x00\xa3\xa0'),

  (0x128, Ecu.dsu, (CAR.CELICA), 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, Ecu.dsu, (CAR.CELICA), 1,   2, b'\x00\x00\x00\x46'),
  (0x160, Ecu.dsu, (CAR.CELICA), 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, Ecu.dsu, (CAR.CELICA), 1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x283, Ecu.dsu, (CAR.CELICA), 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),

  (0x344, Ecu.dsu, (CAR.CELICA), 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, Ecu.dsu, (CAR.CELICA), 0,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, Ecu.dsu, (CAR.CELICA), 0,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, Ecu.dsu, (CAR.CELICA), 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]

ECU_FINGERPRINT = {
  Ecu.fwdCamera: [0x2e4],   # steer torque cmd
  Ecu.dsu: [0x283],   # accel cmd
}


FINGERPRINTS = {
  CAR.CELICA: [{
    31: 8, 36: 8, 37: 8, 170: 8, 180: 8, 186: 4, 426: 6, 452: 8, 464: 8, 466: 8, 467: 8, 512: 6, 513: 6, 547: 8, 548: 8, 552: 4, 608: 8, 610: 5, 643: 7, 705: 8, 740: 5, 767:4, 800: 8, 835: 8, 836: 8, 849: 4, 869: 7, 870: 7, 871: 2, 896: 8, 897: 8, 900: 6, 902: 6, 905: 8, 911: 8, 916: 2, 921: 8, 933: 8, 944: 8, 945: 8, 951: 8, 955: 4, 956: 8, 979: 2, 992: 8, 998: 5, 999: 7, 1000: 8, 1001: 8, 1017: 8, 1041: 8, 1042: 8, 1043: 8, 1044: 8, 1056: 8, 1059: 1, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1196: 8, 1227: 8, 1235: 8, 1279: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1561: 8, 1562: 8, 1568: 8, 1569: 8, 1570: 8, 1571: 8, 1572: 8, 1584: 8, 1589: 8, 1592: 8, 1596: 8, 1597: 8, 1600: 8, 1664: 8, 1728: 8, 1779: 8, 1904: 8, 1912: 8, 1990: 8, 1998: 8, 2016: 8, 2017: 8, 2018: 8, 2019: 8, 2020: 8, 2021: 8, 2022: 8, 2023: 8, 2024: 8
  }],
}

# Don't use theses fingerprints for fingerprinting, they are still needed for ECU detection
IGNORED_FINGERPRINTS = [None]

FW_VERSIONS = {
  CAR.CELICA: {
    (Ecu.engine, 0x7e0, None): [
      b'\x02348Q4000\x00\x00\x00\x00\x00\x00\x00\x00A4802000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x02348T1100\x00\x00\x00\x00\x00\x00\x00\x00A4802000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x02348Z3000\x00\x00\x00\x00\x00\x00\x00\x00A4802000\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.esp, 0x7b0, None): [
      b'F152648501\x00\x00\x00\x00\x00\x00',
      b'F152648A30\x00\x00\x00\x00\x00\x00',
      b'F152648361\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.dsu, 0x791, None): [
      b'881514811300\x00\x00\x00\x00',
      b'881514811700\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x7a1, None): [
      b'8965B0E011\x00\x00\x00\x00\x00\x00',
      b'8965B0E012\x00\x00\x00\x00\x00\x00',
      b'8965B48112\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x750, 0xf): [
      b'8821F4701000\x00\x00\x00\x00',
      b'8821F4701300\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'8646F4801200\x00\x00\x00\x00',
      b'8646F4802200\x00\x00\x00\x00',
      b'8646F4809000\x00\x00\x00\x00',
    ],
  },
}

STEER_THRESHOLD = 100

DBC = {
  CAR.CELICA: dbc_dict('toyota_celica_2003_pt', 'toyota_adas'),
}

NO_DSU_CAR = [None]
TSS2_CAR = [None]
