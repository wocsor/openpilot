#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.old_cars.values import CAR, BUTTON_STATES
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.buttonStatesPrev = BUTTON_STATES.copy()

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "old_cars"
    # TODO: old_cars_safety
    ret.safetyModel = car.CarParams.SafetyModel.allOutput

    ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
    ret.steerLimitTimer = 0.4

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.enableCruise = False  # stock cruise control is kept off

    if candidate == CAR.CELICA:
      stop_and_go = False
      ret.safetyParam = 100
      ret.wheelbase = 2.70
      ret.steerRatio = 14.4
      tire_stiffness_factor = 0.444  # not optimized yet
      ret.mass = 2580. * CV.LB_TO_KG + STD_CARGO_KG  # mean between normal and hybrid
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.05]]
      ret.lateralTuning.pid.kf = 0.00003   # full torque for 20 deg at 80mph means 0.00007818594

    ret.steerRateCost = 1.
    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = True
   
    ret.enableDsu = True
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    ret.openpilotLongitudinalControl = True
    cloudlog.warning("ECU Camera Simulated: %r", ret.enableCamera)
    cloudlog.warning("ECU DSU Simulated: %r", ret.enableDsu)
    cloudlog.warning("ECU Gas Interceptor: %r", ret.enableGasInterceptor)

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1.

    # removing the DSU disables AEB and it's considered a community maintained feature
    # intercepting the DSU is a community feature since it requires unofficial hardware
    ret.communityFeature = ret.enableGasInterceptor or ret.enableDsu or smartDsu

    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .15]
    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kiBP = [0., 35.]

    if ret.enableGasInterceptor:
      ret.gasMaxBP = [0., 9., 35]
      ret.gasMaxV = [0.2, 0.5, 0.7]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
    else:
      ret.gasMaxBP = [0., 9., 35]
      ret.gasMaxV = [0.2, 0.5, 0.75]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiV = [0.54, 0.36]

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    #self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, None)

    ret.canValid = True #self.cp.can_valid and self.cp_cam.can_valid
    ret.yawRate = self.VM.yaw_rate(ret.steeringAngle * CV.DEG_TO_RAD, ret.vEgo)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False
    buttonEvents = []

    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    # events
    events = self.create_common_events(ret)

    if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))
      if c.actuators.gas > 0.1:
        # some margin on the actuator to not false trigger cancellation while stopping
        events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      if ret.vEgo < 0.001:
        # while in standstill, send a user alert
        events.append(create_event('manualRestart', [ET.WARNING]))

    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    elif not ret.cruiseState.enabled:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    ret.events = events
    ret.buttonEvents = buttonEvents
    self.buttonStatesPrev = self.buttonStatesPrev = self.CS.buttonStates.copy()
    self.cruise_enabled_prev = ret.cruiseState.enabled
    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel,
                               c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                               c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)

    self.frame += 1
    return can_sends
