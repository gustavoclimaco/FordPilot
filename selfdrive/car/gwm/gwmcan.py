import numpy as np
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import CanBusBase

# Desired accel (m/s^2) above which engine drag alone is enough - no brake.
# Route 0000005d (25 min, 54k engaged frames): gas 0 coasts at only about
# -0.1..-0.2 above 15 km/h (stronger at low speed in gear).
COAST_ACCEL = -0.2


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def create_steer_command(packer: CANPacker, CAN: CanBus, camera_stock_values, steer: float, steer_req: bool):
  steer = int(steer)
  values = {
    "STEER_REQUEST": 1 if steer_req else 0,
    "SET_ME_X01": 1,
    "TORQUE_CMD": steer,
    "TORQUE_REFLECTED": -steer,
    "INVERT_DIRECTION": 1 if (steer > 0 and steer_req) else 0,
    "COUNTER": (camera_stock_values["COUNTER"] + 1) % 16,
    "BYPASS_ME": camera_stock_values["BYPASS_ME"],
  }

  # calculate and insert basic checksum
  dat = packer.make_can_msg("STEER_CMD", 0, values)[2]
  values["BASIC_CHECKSUM"] = gwm_basic_chksum_for_0x12B(dat)
  # calculate and insert CRC
  dat = packer.make_can_msg("STEER_CMD", 0, values)[2]
  values["CRC_X9B"] = checksum(dat[9:16], 0x9B)

  return packer.make_can_msg("STEER_CMD", CAN.main, values)


def create_longitudinal_command(packer: CANPacker, CAN: CanBus, longitudinal_stock_values, accel: float, active: bool, standstill: bool):
  values = {s: longitudinal_stock_values[s] for s in [
    "BYPASSME_1",
    "SPEED_REAL",
    "COUNTER_BRAKE",
    "BYPASSME_2",
    "BYPASS_ACC1",
    "BYPASS_ACC2",
    "COUNTER_ACC",
  ]}

  brake_or_gas = longitudinal_stock_values["BRAKE_OR_GAS_REQ"]
  standstill1 = longitudinal_stock_values["STANDSTILL_1"]
  standstill2 = longitudinal_stock_values["STANDSTILL_2"]
  standstill3 = longitudinal_stock_values["STANDSTILL_3"]
  brake_cmd = 0
  accel_cmd = 0
  # accel is the desired acceleration in m/s^2, already clipped to
  # [ACCEL_MIN, ACCEL_MAX]. Both maps below are calibrated from measured
  # cmd -> aEgo medians on route 0000005d--a8ef4509d7 (25 min, 54k engaged
  # frames). The previous linear maps under-delivered by ~2x across the
  # range: gas 1500 for a 0.67 m/s^2 request produced only ~0.31 (sluggish
  # resumes), and brake -41..-50 produced only ~ -0.06, so light braking
  # did nothing until the planner escalated and the brakes bit deep all at
  # once (felt as harsh late braking behind leads).
  if active and accel < COAST_ACCEL:
    # Measured: -55 ~ -0.57, -65 ~ -1.04, -75 ~ -1.29, -85 ~ -2.0, -95 ~ -2.32
    brake_or_gas = 13
    brake_cmd = np.interp(accel, [-3.5, -2.3, -1.3, -0.6, COAST_ACCEL], [-107, -95, -75, -55, -44])
    accel_cmd = 0
    standstill1 = 1 if standstill else 0
    standstill2 = 3 if standstill else 4  # 3 "active" 4 "inactive"
    standstill3 = 0 if standstill else 1  # 0 "active" 1 "inactive"
  elif active:
    # Gas request, continuous from stock neutral. Desired accels in
    # [COAST_ACCEL, 0) also land here with gas 0: engine drag covers them
    # without touching the brakes.
    # Measured: ~700 ~ +0.15, ~1300 ~ +0.28, ~2000 ~ +0.56, ~2400 ~ +0.66.
    # Above ~0.7 m/s^2 the data thins out (transients only); extrapolate
    # to the previously observed ceiling 4577 and let the PI close the gap.
    brake_or_gas = 12
    brake_cmd = 0
    accel_cmd = np.interp(accel, [0.0, 0.15, 0.3, 0.6, 1.0, 2.0], [0, 700, 1200, 2200, 3300, 4577])
    standstill1 = 0
    standstill2 = 4  # 3 "active" 4 "inactive"
    standstill3 = 1  # 0 "active" 1 "inactive"
  values |= {
    "BRAKE_OR_GAS_REQ": brake_or_gas,
    "BRAKE_CMD": brake_cmd,
    "GAS_CMD": accel_cmd,
    "STANDSTILL_1": standstill1,
    "STANDSTILL_2": standstill2,
    "STANDSTILL_3": standstill3,
  }

  data = packer.make_can_msg("ACC_CMD", 0, values)[2]
  values["CRC_BRAKE_0xEF"] = checksum(data[9:16], 0xEF)
  values["CRC_ACC_0x87"] = checksum(data[25:32], 0x87)

  return packer.make_can_msg("ACC_CMD", CAN.main, values)


def create_wheel_touch(packer: CANPacker, CAN: CanBus, eps_stock_values, ea_simulated_torque: float):
  values = {s: eps_stock_values[s] for s in [
    "A_CRC_X61",
    "A_BYPASSME_2",
    "A_RX_STEER_REQUESTED",
    "A_BYPASSME_1",
    "A_COUNTER",
    "B_CRC_X61",
    "B_RX_DRIVER_TORQUE",
    "B_BYPASSME_1",
    "B_BYPASSME_2",
    "B_RX_EPS_TORQUE",
    "B_COUNTER",
    "B_BYPASSME_3",
  ]}

  values.update({
    "B_RX_DRIVER_TORQUE": ea_simulated_torque,
  })

  # calculate checksum
  dat = packer.make_can_msg("RX_STEER_RELATED", 0, values)[2]
  values["B_CRC_X61"] = checksum(dat[9:16], 0x61)

  return packer.make_can_msg("RX_STEER_RELATED", CAN.camera, values)


def create_buttons_command(packer: CANPacker, CAN: CanBus, counter: int, stock_msg, cancel_command=False, resume_command=False):
  values = {s: stock_msg[s] for s in [
    "STEERING_ANGLE",
    "STEERING_DIRECTION",
    "STEERING_RATE",
    "RATE_DIRECTION",
    "AP_DECREASE_SPEED_COMMAND",
    "AP_INCREASE_SPEED_COMMAND",
    "AP_REDUCE_DISTANCE_COMMAND",
    "AP_INCREASE_DISTANCE_COMMAND",
  ]}

  values |= {
    "AP_CANCEL_COMMAND": stock_msg["AP_CANCEL_COMMAND"] or cancel_command,
    # resume_command simulates pressing the AP stalk rearward (AP_ENABLE_COMMAND)
    # to re-engage the ACC ECU after a standstill â€” Stop & Go resume pulse.
    "AP_ENABLE_COMMAND": 1 if resume_command else stock_msg["AP_ENABLE_COMMAND"],
    "COUNTER": counter,
  }

  data = packer.make_can_msg("STEER_AND_AP_STALK", 0, values)[2]
  values["CRC_X2D"] = checksum(data[1:8], 0x2D)

  return packer.make_can_msg('STEER_AND_AP_STALK', CAN.camera, values)


def create_hud_command(packer: CANPacker, CAN: CanBus, hud_stock_values, steer_required: bool):
  values = {s: hud_stock_values[s] for s in [
    "BYPASSME_1",
    "BYPASSME_2",
    "BY_PASSME",
    "COUNTER",
    "BYPASSME_3",
    "BYPASSME_4",
    "BYPASSME_5",
    "BYPASSME_6",
    "CRUISE_STATE",
  ]}

  values |= {
    "LKAS_STATE": 5 if steer_required else hud_stock_values["LKAS_STATE"],
  }

  data = packer.make_can_msg("LATERAL_STATE", 0, values)[2]
  values["CRC_X66"] = checksum(data[17:24], 0x66)

  return packer.make_can_msg("LATERAL_STATE", CAN.main, values)


def checksum(data, xor_output):
  crc = 0
  poly = 0x1D
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_output


def gwm_basic_chksum_for_0x12B(d: bytearray) -> int:
  invert_direction = d[12] >> 7 & 0x1
  counter = d[15] & 0xF
  steer_requested = d[15] >> 5 & 0x1
  return (28 - (steer_requested * 8) - counter - invert_direction) & 0x1F
