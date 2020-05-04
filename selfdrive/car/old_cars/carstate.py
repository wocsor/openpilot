from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.old_cars.values import CAR, DBC, STEER_THRESHOLD, BUTTON_STATES


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']
    self.buttonStates = BUTTON_STATES.copy()

    # On NO_DSU cars but not TSS2 cars the cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.angle_offset = 0.

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = False #any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
    #                     cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = False #cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED'] != 0

    ret.brakePressed = cp.vl["BRAKE_OUTPUT"]['SENSOR'] > 50
    ret.brakeLights = False #bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["GAS_OUTPUT"]['GAS_PRESSED']
      ret.gasPressed = cp.vl["GAS_OUTPUT"]['GAS_PRESSED'] > 100

    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    invalid_wheelspeeds = [
      cp.vl["WHEEL_SPEEDS"]['SENSOR_ERROR_FL'],
      cp.vl["WHEEL_SPEEDS"]['SENSOR_ERROR_FR'],
      cp.vl["WHEEL_SPEEDS"]['SENSOR_ERROR_RL'],
      cp.vl["WHEEL_SPEEDS"]['SENSOR_ERROR_RR'],
    ]
    valid_wheelspeeds = []
    for e, wheelspeed in zip(invalid_wheelspeeds, [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]):
      if not e:
        valid_wheelspeeds.append(wheelspeed)      
    ret.vEgoRaw = mean(valid_wheelspeeds)
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.001
    ret.steeringAngle = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']

    ret.steeringRate = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']
    can_gear = 0
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["PCM_CRUISE"]['TURN_L'] == 0
    ret.rightBlinker = cp.vl["PCM_CRUISE"]['TURN_R'] == 0

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    # cruise magic
    ret.cruiseState.available = cp.vl["PCM_CRUISE"]['MAIN_ON'] != 0
    ret.cruiseState.enabled = cp.vl["PCM_CRUISE"]['ENGAGED'] != 0 
    if ret.cruiseState.enabled:
      self.buttonStates["accelCruise"] = bool(cp.vl["PCM_CRUISE"]['RES_ACC'])
      self.buttonStates["decelCruise"] = bool(cp.vl["PCM_CRUISE"]['SET_COAST'])
    else:
      self.buttonStates["setCruise"] = bool(cp.vl["PCM_CRUISE"]['SET_COAST'])
      self.buttonStates["resumeCruise"] = bool(cp.vl["PCM_CRUISE"]['RES_ACC'])


    ret.genericToggle = False
    ret.stockAeb = False

    ret.espDisabled = False
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']
    self.steer_warning = cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]

    return ret

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("GEAR", "GEAR_PACKET", 0),
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("SENSOR_ERROR_FL", "WHEEL_SPEEDS", 0),
      ("SENSOR_ERROR_FR", "WHEEL_SPEEDS", 0),
      ("SENSOR_ERROR_RL", "WHEEL_SPEEDS", 0),
      ("SENSOR_ERROR_RR", "WHEEL_SPEEDS", 0),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
      ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
      ("TURN_R", "PCM_CRUISE", 1),
      ("TURN_L", "PCM_CRUISE", 1),
      ("ENGAGED", "PCM_CRUISE", 0),
      ("MAIN_ON", "PCM_CRUISE", 0),
      ("RES_ACC", "PCM_CRUISE", 0),
      ("SET_COAST", "PCM_CRUISE", 0),
      ("SPEED", "PCM_CRUISE", 0),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("LKA_STATE", "EPS_STATUS", 0),
      ("SENSOR", "BRAKE_OUTPUT", 0),
      ("GAS_PRESSED", "GAS_OUTPUT", 0)
    ]

    checks = [
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 50),
      ("STEER_TORQUE_SENSOR", 50),
      ("EPS_STATUS", 25),
      ("GAS_ACTUATOR", 50),
      ("BRAKE_ACTUATOR", 50)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    # use steering message to check if panda is connected to frc
    checks = [("STEERING_LKA", 42)]

    return None
