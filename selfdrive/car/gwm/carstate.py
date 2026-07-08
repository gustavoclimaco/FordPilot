import copy

from cereal import car, custom
from opendbc.can.parser import CANParser
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.gwm.values import DBC

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)

    self.steer_and_ap_stalk_msg = {}
    self.eps_stock_values = {}
    self.camera_stock_values = {}
    self.longitudinal_stock_values = {}
    self.hud_stock_values = {}

    self.is_activation_lever_pulled = False
    self.prev_activation_lever_pulled = False
    self.main_on = False
    self.steer_fault_temporary_counter = 0

    # Stop & Go: track cruise state to distinguish standstill from real fault
    self.cruise_state_2 = 0

  def update(self, cp, cp_cam, frogpilot_toggles):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    self.steer_and_ap_stalk_msg = copy.copy(cp.vl["STEER_AND_AP_STALK"])
    self.eps_stock_values = copy.copy(cp.vl["RX_STEER_RELATED"])
    self.camera_stock_values = copy.copy(cp_cam.vl["STEER_CMD"])
    self.longitudinal_stock_values = copy.copy(cp_cam.vl["ACC_CMD"])
    self.hud_stock_values = copy.copy(cp_cam.vl["LATERAL_STATE"])

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Stop & Go fix:
    # The GWM H6 GT has no native Stop & Go â€” the ACC module deactivates
    # (CRUISE_STATE_2 â†’ 0) whenever the car stops. This is expected behaviour,
    # NOT a real ACC fault. We must NOT propagate this as accFaulted, otherwise
    # openpilot drops longitudinal control at every stop and can never resume.
    #
    # Real faults (hardware/comms errors) show up through steerFault paths and
    # other signals. The resume pulse in carcontroller re-engages the ACC ECU.
    self.cruise_state_2 = int(cp_cam.vl["ACC"]["CRUISE_STATE_2"])

    if self.CP.openpilotLongitudinalControl:
      # While OP owns longitudinal: never fault on standstill ACC deactivation
      ret.accFaulted = False
    else:
      # Stock ACC path: honour the original logic
      ret.accFaulted = self.cruise_state_2 == 0

    ret.cruiseState.speed = cp_cam.vl["ACC"]["ACC_SPEED_SELECTION"] * CV.KPH_TO_MS
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.speed = -1

    ret.standstill = abs(ret.vEgoRaw) < 1e-3
    ret.gasPressed = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"] > 0
    ret.brakePressed = cp.vl["BRAKE2"]["PEDAL_BRAKE_PRESSED"] != 0
    ret.parkingBrake = cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"] == 0

    ret.gearShifter = GearShifter.drive    if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 1 else \
                      GearShifter.neutral  if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 2 else \
                      GearShifter.reverse  if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 3 else \
                      GearShifter.park

    ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * \
                           (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
    ret.steeringRateDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_RATE"] * \
                          (-1 if (cp.vl["STEER_AND_AP_STALK"]["RATE_DIRECTION"] > 0) else 1)

    ret.steerFaultTemporary = False
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) \
                                          if (cp.vl["RX_STEER_RELATED"]["EPS_FAULT_PERMANENT"] == 1) else 0
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100
    ret.steerFaultPermanent = False

    ret.steeringTorque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["RX_STEER_RELATED"]["B_RX_EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50,
      cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"],
    )
    ret.leftBlindspot = bool(cp.vl["RADAR_BEHIND"]["BSM_LEFT"] > 0)
    ret.rightBlindspot = bool(cp.vl["RADAR_BEHIND"]["BSM_RIGHT"] > 0)

    # Freio nÃ£o Ã© mais tratado como cancelamento total do ACC (05/07/2026).
    # SÃ³ um cancelamento real (AP_CANCEL_COMMAND) derruba main_on â€” isso Ã© o
    # que permite o volante (MADS/Always On Lateral) continuar ativo com o
    # pÃ© no freio.
    if cp.vl["STEER_AND_AP_STALK"]["AP_CANCEL_COMMAND"]:
      self.main_on = False
    self.is_activation_lever_pulled = bool(cp.vl["STEER_AND_AP_STALK"]["AP_ENABLE_COMMAND"])
    if not self.is_activation_lever_pulled and self.prev_activation_lever_pulled and not self.main_on:
      self.main_on = True
    self.prev_activation_lever_pulled = self.is_activation_lever_pulled

    ret.cruiseState.available = self.main_on
    ret.cruiseState.enabled = self.main_on

    return ret, fp_ret

  @staticmethod
  def get_can_parser(CP, FPCP):
    messages = [
      ("STEER_AND_AP_STALK", 50),
      ("RX_STEER_RELATED", 50),
      ("WHEEL_SPEEDS", 50),
      ("CAR_OVERALL_SIGNALS2", 50),
      ("BRAKE2", 50),
      ("BRAKE", 50),
      ("CAR_OVERALL_SIGNALS", 50),
      ("LIGHTS", 10),
      ("RADAR_BEHIND", 20),
      ("DOOR_DRIVER", 10),
      ("SEATBELT", 10),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    messages = [
      ("STEER_CMD", 50),
      ("ACC_CMD", 50),
      ("LATERAL_STATE", 20),
      ("ACC", 20),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
