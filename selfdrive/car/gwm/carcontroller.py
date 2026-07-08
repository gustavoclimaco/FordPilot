import numpy as np
from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.gwm import gwmcan
from openpilot.selfdrive.car.gwm.values import CarControllerParams

LongCtrlState = car.CarControl.Actuators.LongControlState

MAX_USER_TORQUE = 100  # 1.0 Nm

# Stop & Go resume pulse configuration.
# The GWM H6 GT has no native Stop & Go: when the car comes to a full stop
# the ACC ECU deactivates (CRUISE_STATE_2 â†’ 0) and waits for a "resume" input.
# We simulate pressing the AP_ENABLE_COMMAND stalk signal for a short pulse to
# re-engage the ACC ECU automatically whenever openpilot wants to start moving.
#
# RESUME_PULSE_FRAMES: how many 50 Hz frames to hold AP_ENABLE_COMMAND = 1.
#   10 frames = ~200 ms. Increase to 15-20 if the car ignores the first pulse.
# RESUME_ACCEL_THRESHOLD: minimum desired accel (m/sÂ²) to trigger resume.
#   Use a small positive value to avoid spurious triggers from accel noise.
RESUME_PULSE_FRAMES = 10
RESUME_ACCEL_THRESHOLD = 0.05  # m/sÂ²


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_name)
    self.CAN = gwmcan.CanBus(CP)
    self.apply_torque_last = 0
    self.accel = 0.0
    self.frame = 0

    # Stop & Go state
    self.resume_required = False
    self.resume_counter = 0  # frames left to pulse AP_ENABLE_COMMAND

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if CC.cruiseControl.cancel:
      counter = (CS.steer_and_ap_stalk_msg['COUNTER'] + 1) % 16
      can_sends.append(gwmcan.create_buttons_command(
        self.packer,
        self.CAN,
        counter,
        CS.steer_and_ap_stalk_msg,
        cancel_command=True,
      ))

    if self.frame % 2 == 0:  # 50 Hz

      # â”€â”€ Stop & Go Resume Logic â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
      # Trigger a resume pulse when ALL of the following are true:
      #   1. OP owns longitudinal control (longActive)
      #   2. Car is at a full standstill (vEgo â‰ˆ 0)
      #   3. Planner wants to start moving (actuators.accel above threshold)
      #   4. ACC ECU is in standstill-wait state (cruise_state_2 == 0)
      #   5. No ongoing resume pulse already running
      send_resume = False
      if self.CP.openpilotLongitudinalControl:
        accel_desired = actuators.accel if CC.longActive else 0.0
        acc_in_standstill = (CS.cruise_state_2 == 0)

        should_trigger = (
          CC.longActive
          and CS.out.standstill
          and accel_desired > RESUME_ACCEL_THRESHOLD
          and acc_in_standstill
          and not self.resume_required
        )

        if should_trigger:
          self.resume_required = True
          self.resume_counter = RESUME_PULSE_FRAMES

        # Send pulse while counter > 0
        if self.resume_required:
          if self.resume_counter > 0:
            send_resume = True
            self.resume_counter -= 1
          else:
            self.resume_required = False

        # Abort if car moved or ACC re-engaged (no longer needed)
        if not CS.out.standstill or not acc_in_standstill:
          if not send_resume:  # let current pulse finish naturally
            self.resume_required = False
            self.resume_counter = 0
      # â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

      # Steer command
      new_torque = int(round(actuators.steer * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      # Prevent sending the same 'apply_torque = 1' torque repeatedly, as it can cause EPS faults.
      if abs(apply_torque) == 1:
        apply_torque = apply_torque * 2
      if not lat_active:
        apply_torque = 0
      can_sends.append(gwmcan.create_steer_command(
        self.packer,
        self.CAN,
        camera_stock_values=CS.camera_stock_values,
        steer=apply_torque,
        steer_req=lat_active,
      ))
      self.apply_torque_last = apply_torque

      # Satisfy steer nudge requests
      ea_simulated_torque = float(np.clip(apply_torque * 2, -self.params.STEER_MAX, self.params.STEER_MAX))
      if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
        ea_simulated_torque = CS.out.steeringTorque
      can_sends.append(gwmcan.create_wheel_touch(
        self.packer,
        self.CAN,
        eps_stock_values=CS.eps_stock_values,
        ea_simulated_torque=ea_simulated_torque,
      ))

      # Longitudinal control
      if self.CP.openpilotLongitudinalControl:
        standstill = actuators.longControlState == LongCtrlState.stopping
        self.accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if self.accel < 0:
          accel = -abs(self.accel / CarControllerParams.ACCEL_MIN)
        else:
          accel = self.accel / CarControllerParams.ACCEL_MAX
        can_sends.append(gwmcan.create_longitudinal_command(
          self.packer,
          self.CAN,
          longitudinal_stock_values=CS.longitudinal_stock_values,
          accel=accel,
          active=CC.longActive,
          standstill=standstill,
        ))

      # Resume command: inject AP_ENABLE_COMMAND pulse into the stalk message.
      if send_resume:
        resume_counter = (CS.steer_and_ap_stalk_msg['COUNTER'] + 1) % 16
        can_sends.append(gwmcan.create_buttons_command(
          self.packer,
          self.CAN,
          resume_counter,
          CS.steer_and_ap_stalk_msg,
          resume_command=True,
        ))

    if self.frame % 5 == 0:  # 20 Hz
      can_sends.append(gwmcan.create_hud_command(
        self.packer,
        self.CAN,
        hud_stock_values=CS.hud_stock_values,
        steer_required=CC.latActive,
      ))

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
