from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.old_cars.values import CAR, DBC, STEER_THRESHOLD, BUTTON_STATES

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
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

    ret.brakePressed = bool(cp_cam.vl["BRAKE_OUTPUT"]['BRAKE_PRESSED'])
    ret.brakeLights =  ret.brakePressed

    ret.gas = (cp_cam.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp_cam.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
    ret.gasPressed = ret.gas > 15

    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_RR'] * CV.KPH_TO_MS
    invalid_wheelspeeds = [
      cp.vl["WHEEL_SPEEDS"]['SENSOR_FL'],
      cp.vl["WHEEL_SPEEDS"]['SENSOR_FR'],
      cp.vl["WHEEL_SPEEDS"]['SENSOR_RL'],
      cp.vl["WHEEL_SPEEDS"]['SENSOR_RR'],
    ]
    valid_wheelspeeds = []
    for e, wheelspeed in zip(invalid_wheelspeeds, [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]):
      if e:
        valid_wheelspeeds.append(wheelspeed)   
    if valid_wheelspeeds:   
      ret.vEgoRaw = mean(valid_wheelspeeds)
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.001
    ret.steeringAngle = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE']

    ret.steeringRate = cp.vl["VSC"]['STEER_RATE']
    can_gear = 0
    ret.gearShifter = 0
    ret.leftBlinker = False
    ret.rightBlinker = False

    ret.steeringTorque = cp.vl["STEER_TORQUE"]['DRIVER_TORQUE'] * (cp.vl["STEER_TORQUE"]["DIRECTION"] * -1)
    ret.steeringTorqueEps = 0 #cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    # cruise magic
    ret.cruiseState.available = True # cp.vl["PCM_CRUISE"]['MAIN_ON'] != 0
    ret.cruiseState.enabled = True #bool(cp.vl["PCM_CRUISE"]['ENGAGED'])
    cruise_button = bool(cp.vl["CRUISE_BUTTONS"]["CANCEL"])
    ret.cruiseState.enabled = cruise_button
    if ret.cruiseState.enabled:
      self.buttonStates["accelCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['SET_PLUS'])
      self.buttonStates["decelCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['SET_MINUS'])
    else:
      self.buttonStates["setCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['SET_MINUS']) 
      self.buttonStates["resumeCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['SET_PLUS'])

    gear = cp.vl["GEAR_PACKET"]['GEAR']
    if gear == 0:
      ret.gearShifter = GearShifter.park
    elif gear == 1:
      ret.gearShifter = GearShifter.reverse
    elif gear == 2:
      ret.gearShifter = GearShifter.neutral
    elif gear in (3, 4, 5, 6, 7, 8, 9, 10, 11): #6R80 only. Not counting 10R80
      ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = GearShifter.unknown

    ret.genericToggle = False
    ret.stockAeb = False

    ret.espDisabled = False
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = 10. # cp.vl["EPS_STATUS"]['LKA_STATE']
    self.steer_warning = 0 # cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    self.brake_pressure = cp.vl["BRAKE_MODULE"]['BRAKE_PRESSURE']

    return ret

  @staticmethod
  def get_can_parser(CP):
    # bus 0

    signals = [
      # sig_name, sig_address, default
      ("WHEEL_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_RR", "WHEEL_SPEEDS", 0),
      ("SENSOR_FL", "WHEEL_SPEEDS", 0),
      ("SENSOR_FR", "WHEEL_SPEEDS", 0),
      ("SENSOR_RL", "WHEEL_SPEEDS", 0),
      ("SENSOR_RR", "WHEEL_SPEEDS", 0),
      ("DRIVER_TORQUE", "STEER_TORQUE", 0),
      ("DIRECTION", "STEER_TORQUE", 0),
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      ("MAIN_ON","CRUISE_BUTTONS", 0),
      ("SET_MINUS","CRUISE_BUTTONS", 0),
      ("SET_PLUS","CRUISE_BUTTONS", 0),
      ("CANCEL","CRUISE_BUTTONS", 0),
      ("STEER_RATE", "VSC", 0),
      ("FL_BELT", "SEATBELTS", 0),
      ("GEAR", "GEAR_PACKET", 0),
      ("BRAKE_PRESSURE", "BRAKE_MODULE", 0),
    ]

    checks = [
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    # bus 2
    signals = [
      ("SENSOR", "BRAKE_OUTPUT", 0),
      ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
      ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
      ("DRIVER_TORQUE", "STEER_SENSOR", 0),
      ("MOTOR_DUTY", "STEER_SENSOR", 0),
      ("MOTOR_CURRENT", "STEER_SENSOR", 0),
      ("STATE", "STEER_SENSOR", 0),
      ("brake_report_operator_override", "BRAKE_REPORT", 0),
      ("brake_report_dtcs", "BRAKE_REPORT", 0),
      ("brake_report_enabled", "BRAKE_REPORT", 0),
      ("BRAKE_PRESSED", "BRAKE_OUTPUT", 0),
    ]
    checks = [("STEER_SENSOR", 50)]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
