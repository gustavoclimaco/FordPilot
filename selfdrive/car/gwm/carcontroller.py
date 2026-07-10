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
# the ACC ECU deactivates (CRUISE_STATE_2 drops to the 0-2 "deactivated"
# family) and waits for a "resume" input. We simulate pressing the
# AP_ENABLE_COMMAND stalk signal for a short pulse to re-engage the ACC ECU
# whenever openpilot wants to start moving.
#
# RESUME_PULSE_FRAMES: how many 100 Hz frames to hold AP_ENABLE_COMMAND = 1.
#   20 frames = ~200 ms. Increase to 30-40 if the car ignores the first pulse.
# RESUME_ACCEL_THRESHOLD: minimum desired accel (m/s^2) to trigger resume.
# RESUME_RETRY_FRAMES: cooldown between pulse attempts. Route 0000005e showed
#   the ECU re-engages on the pulse but gives itself up again ~4 s later if
#   the car hasn't moved; a single one-shot pulse then left the real launch
#   with a dead ECU (creep only, gas ignored, driver had to intervene). While
#   openpilot still wants to move and the ECU is still deactivated, retry.
RESUME_PULSE_FRAMES = 20
RESUME_ACCEL_THRESHOLD = 0.05  # m/s^2
RESUME_RETRY_FRAMES = 100      # 1 s at 100 Hz

# Longitudinal command shaping (all values in the ACC_CMD phys scale).
# Maps calibrated from route 0000005d measured cmd -> aEgo medians; see
# gwmcan.create_longitudinal_command for the anchor points.
COAST_ACCEL = -0.2       # m/s^2: above this, engine drag alone is enough
BRAKE_EXIT_ACCEL = -0.05  # m/s^2: hysteresis - leave brake mode only above this
GAS_MAP_BP = [0.0, 0.15, 0.3, 0.6, 1.0, 2.0]
GAS_MAP_V = [0, 700, 1200, 2200, 3300, 4577]
BRAKE_MAP_BP = [-3.5, -2.3, -1.3, -0.6, COAST_ACCEL]
BRAKE_MAP_V = [-107, -95, -75, -55, -44]
# Slew limits per 20 ms frame. Route 0000005e showed 216 brake episodes with
# a median duration of 0.11 s (apply/release chatter around the coast
# boundary) - felt as pumping. Rate-limiting both commands smooths apply and
# release; the hysteresis above stops the mode chatter itself.
GAS_APPLY_SLEW = 120
GAS_RELEASE_SLEW = 240
BRAKE_APPLY_SLEW = 4.0
BRAKE_RELEASE_SLEW = 2.5


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
    self.resume_counter = 0   # frames left to pulse AP_ENABLE_COMMAND
    self.resume_cooldown = 0  # frames until another pulse may fire

    # Longitudinal command state (hysteresis + slew)
    self.braking = False
    self.gas_cmd = 0.0
    self.brake_cmd = 0.0

    # Free-running counter for the intercepted stalk stream. The panda fwd hook
    # blocks the stock STEER_AND_AP_STALK from reaching the camera, so openpilot
    # owns that stream exclusively and must provide a perfectly continuous
    # counter sequence at 100 Hz (independent of stock message phase/jitter).
    self.stalk_counter = 0

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # -- Stop & Go Resume Logic (100 Hz) -----------------------------------
    # Trigger a resume pulse when ALL of the following are true:
    #   1. OP owns longitudinal control (longActive)
    #   2. Car is at a full standstill (vEgo ~ 0)
    #   3. Planner wants to start moving (actuators.accel above threshold)
    #   4. ACC ECU is in standstill-wait state (cruise_state_2 == 0)
    #   5. No ongoing resume pulse already running
    send_resume = False
    if self.CP.openpilotLongitudinalControl:
      accel_desired = actuators.accel if CC.longActive else 0.0
      # CRUISE_STATE_2 values 0/1/2 are all "deactivated" per the DBC. After a
      # stop the ECU sits at 2, and after an expired re-engage it returns to 2
      # - checking == 0 here left the trigger dead exactly when the pulse was
      # needed (route 0000005e launch aborts).
      acc_deactivated = (CS.cruise_state_2 <= 2)

      should_trigger = (
        CC.longActive
        and CS.out.standstill
        and accel_desired > RESUME_ACCEL_THRESHOLD
        and acc_deactivated
        and not self.resume_required
        and self.resume_cooldown == 0
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
          # ECU may drop out again ~4 s after re-engaging if the car hasn't
          # moved; allow another attempt after a short cooldown for as long
          # as the launch conditions persist.
          self.resume_cooldown = RESUME_RETRY_FRAMES
      elif self.resume_cooldown > 0:
        self.resume_cooldown -= 1

      # Abort if car moved or ACC re-engaged (no longer needed)
      if not CS.out.standstill or not acc_deactivated:
        if not send_resume:  # let current pulse finish naturally
          self.resume_required = False
          self.resume_counter = 0
    # ----------------------------------------------------------------------

    # Stalk stream to the camera (100 Hz, every frame). The stock copy is
    # blocked by the panda fwd hook, so this is the only STEER_AND_AP_STALK
    # the camera sees: continuous counter, valid CRC, with AP_ENABLE_COMMAND
    # flipped during the Stop & Go resume pulse and AP_CANCEL_COMMAND on
    # cancel. No interleaving, no duplicate counters.
    self.stalk_counter = (self.stalk_counter + 1) % 16
    can_sends.append(gwmcan.create_buttons_command(
      self.packer,
      self.CAN,
      self.stalk_counter,
      CS.steer_and_ap_stalk_msg,
      cancel_command=CC.cruiseControl.cancel,
      resume_command=send_resume,
    ))

    if self.frame % 2 == 0:  # 50 Hz

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

        if not CC.longActive:
          self.braking = False
          self.gas_cmd = 0.0
          self.brake_cmd = 0.0
        else:
          # Mode hysteresis: enter brake mode below COAST_ACCEL, hand back to
          # gas/coast only once the demand has clearly recovered - and only
          # after the brake pressure has been released gradually.
          if self.braking:
            if self.accel > BRAKE_EXIT_ACCEL:
              brake_target = 0.0
              if self.brake_cmd > -2.0:
                self.braking = False
            else:
              brake_target = float(np.interp(self.accel, BRAKE_MAP_BP, BRAKE_MAP_V))
          else:
            self.braking = self.accel < COAST_ACCEL
            brake_target = float(np.interp(self.accel, BRAKE_MAP_BP, BRAKE_MAP_V)) if self.braking else 0.0

          if self.braking:
            self.gas_cmd = 0.0
            # negative scale: "apply" moves away from 0, "release" toward 0
            if brake_target < self.brake_cmd:
              self.brake_cmd = max(brake_target, self.brake_cmd - BRAKE_APPLY_SLEW)
            else:
              self.brake_cmd = min(brake_target, self.brake_cmd + BRAKE_RELEASE_SLEW)
          else:
            self.brake_cmd = 0.0
            gas_target = float(np.interp(self.accel, GAS_MAP_BP, GAS_MAP_V))
            if gas_target > self.gas_cmd:
              self.gas_cmd = min(gas_target, self.gas_cmd + GAS_APPLY_SLEW)
            else:
              self.gas_cmd = max(gas_target, self.gas_cmd - GAS_RELEASE_SLEW)

        can_sends.append(gwmcan.create_longitudinal_command(
          self.packer,
          self.CAN,
          longitudinal_stock_values=CS.longitudinal_stock_values,
          gas_cmd=self.gas_cmd,
          brake_cmd=self.brake_cmd,
          braking=self.braking,
          active=CC.longActive,
          standstill=standstill,
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
