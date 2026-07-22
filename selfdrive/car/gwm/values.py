from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from panda.python import uds
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarDocs, CarHarness, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = car.CarParams.Ecu


# Steer torque / longitudinal limits (mirrors panda/board/safety/safety_gwm.h)
class CarControllerParams:
  STEER_STEP = 2
  # SAFETY: 253, the value validated over the original 700 km trip with graceful
  # degradation (understeer/saturation, never a wheel release). Raw-CAN analysis
  # of a later 700 km trip at 258 (route 0000007d--d44030cafa, seg 174 ~14:30)
  # showed the EPS latch its own EPS_FAULT_PERMANENT flag while openpilot was
  # commanding max torque (258) AND the driver was turning the wheel against it
  # (steerOverride) at 107 km/h in a curve. Once the EPS latches that permanent
  # fault it stops steering entirely (only longitudinal remains) until the
  # ignition is cycled - the "atypical" total lateral loss reported on that trip.
  # The EPS refusal / permanent-fault threshold is the hard limit here; do NOT
  # raise this cap. More curve authority must come from the EPS side (see the
  # port notes), not from commanding more torque than the EPS tolerates.
  STEER_MAX = 253
  STEER_DELTA_UP = 4
  STEER_DELTA_DOWN = 6
  STEER_ERROR_MAX = 80
  ACCEL_MAX = 2
  ACCEL_MIN = -3.5

  def __init__(self, CP):
    pass


class GwmFlags(IntFlag):
  # Set when openpilot has longitudinal control (matches GwmSafetyFlags.LONG_CONTROL in opendbc)
  LONG_CONTROL = 1


@dataclass
class GWMCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  # No dedicated GWM harness exists in FrogPilot's docs yet â€” "custom" reflects
  # the reality that this port currently uses a custom/developer harness.
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass(frozen=True, kw_only=True)
class GWMCarSpecs(CarSpecs):
  pass


@dataclass
class GWMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('gwm_haval_h6_mk3', None))
  flags: int = 0


class CAR(Platforms):
  GWM_HAVAL_H6 = GWMPlatformConfig(
    [GWMCarDocs("GWM H6 GT 2024")],
    GWMCarSpecs(mass=2040, wheelbase=2.738, steerRatio=17.416),
  )


# --- FW fingerprinting (UDS, same scheme as the sunnypilot/opendbc port) ---
GREATWALLMOTORS_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
GREATWALLMOTORS_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

GREATWALLMOTORS_RX_OFFSET = 0x6a

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus, obd_multiplexing in [(1, True), (1, False), (0, False)] for request in [
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=GREATWALLMOTORS_RX_OFFSET,
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
  ]],
)

DBC = CAR.create_dbc_map()
