from cereal import car
from common.numpy_fast import clip
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg
from selfdrive.car.old_cars.old_cars_can import create_steer_command, create_pedal_command, create_brake_relay, create_brake_actuator_command
from selfdrive.car.old_cars.values import Ecu, CAR, STATIC_MSGS, SteerLimitParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False

    self.last_fault_frame = -200
    self.steer_rate_limited = False

    self.fake_ecus = set()
    if CP.enableCamera: self.fake_ecus.add(Ecu.fwdCamera)
    if CP.enableDsu: self.fake_ecus.add(Ecu.dsu)

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    # *** compute control surfaces ***

    # gas and brake
    if enabled:
      apply_gas = clip(actuators.gas, 0., 1.)
      apply_brake = clip(actuators.brake, 0., 1.)
    else:
      apply_gas = 0.
      apply_brake = 0.

    # steer torque
    # new_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
    # apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams)
    # self.steer_rate_limited = new_steer != apply_steer

    apply_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))

    # only cut torque when steer state is a known fault
    if CS.steer_state in [9, 25]:
      self.last_fault_frame = frame

    # Cut steering for 2s after fault
    if not enabled or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    if not enabled:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = 1

    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    brake_pressure = CS.brake_pressure
    


    print(brake_pressure, apply_brake * 1024)

    can_sends = []

    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)
    # print(enabled)
    if (frame % 2 == 0):
        # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
        # This prevents unexpected pedal range rescaling
        can_sends.append(create_brake_actuator_command(self.packer, brake_pressure, apply_brake, frame//2))
        can_sends.append(create_steer_command(self.packer, apply_steer_req, apply_steer, frame//2))
        can_sends.append(create_pedal_command(self.packer, apply_gas, frame//2))

    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    fcw_alert = hud_alert == VisualAlert.fcw
    steer_alert = hud_alert == VisualAlert.steerRequired

    send_ui = False
    if ((fcw_alert or steer_alert) and not self.alert_active) or \
       (not (fcw_alert or steer_alert) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    elif pcm_cancel_cmd:
      # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
      send_ui = True

    return can_sends
