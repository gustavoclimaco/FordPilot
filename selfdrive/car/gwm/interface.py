from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.gwm.values import GwmFlags


class CarInterface(CarInterfaceBase):

  def __init__(self, CP, FPCP, CarController, CarState):
    super().__init__(CP, FPCP, CarController, CarState)
    self.lat_active = False
    self.isEPSobeying = True
    self.steer_fault_temporary_counter = 0

  def _update(self, c, frogpilot_toggles):
    self.lat_active = c.latActive

    self.isEPSobeying = self.cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) \
                                          if (self.lat_active and not self.isEPSobeying) else 0

    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100

    events = self.create_common_events(ret)
    ret.events = events.to_msg()

    return ret, fp_ret

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    ret.carName = "gwm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gwm)]

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4

    ret.steerControlType = car.CarParams.SteerControlType.torque
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.experimentalLongitudinalAvailable = True
    if experimental_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmFlags.LONG_CONTROL.value

      # Stop & Go: enabled so openpilot holds at standstill and resumes.
      # The GWM H6 GT has no native S&G â€” resume is handled in carcontroller
      # via an AP_ENABLE_COMMAND pulse when the planner wants to start moving.
      ret.autoResumeSng = True

      ret.longitudinalActuatorDelay = 0.25

      # vEgoStopping / vEgoStarting: speed thresholds (m/s) at which OP
      # transitions to/from the stopping state. Keep them tight so the car
      # actually holds position instead of creeping.
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25

      # stopAccel: accel command sent while holding at standstill (m/sÂ²).
      # More negative = firmer hold. -0.75 is conservative; tune down to
      # -1.0 if the car rolls on slopes.
      # ATENÃ‡ÃƒO (rollback 28/06/2026): hÃ¡ um evento registrado de rollback numa
      # ladeira ao retomar do stop. Antes de considerar esse valor validado,
      # revisar a lÃ³gica de retomada em subida (ver haval-h6-port-notes.md, seÃ§Ã£o 5).
      ret.stopAccel = -0.75

      # stoppingDecelRate: how fast OP ramps decel to stopAccel (m/sÂ³).
      ret.stoppingDecelRate = 0.75

      # Pure-integral controller, as in the original sunnypilot-derived port.
      # No kp on purpose: proportional gain couples every speed-measurement
      # jitter straight into the gas/brake command, which on this car felt as
      # constant pulsing (routes 0000005e/60). The integrator changes the
      # command smoothly and converges to whatever bias holds the set speed.
      # The old "stuck 20 km/h under set" failure was the gas map deadzone
      # (fixed in gwmcan with a single through-zero line), not the missing kp.
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.4]

    return ret
