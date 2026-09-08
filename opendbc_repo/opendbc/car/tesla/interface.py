from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC, FSD_14_FW, Ecu

from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP
from opendbc.car.tesla.preap.sp.carcontroller import PreAPCarController
from opendbc.car.tesla.preap.sp.carstate import PreAPCarState
from opendbc.car.tesla.preap.sp.interface import get_preap_accel_limits, get_preap_params, get_preap_params_sp, is_preap_platform
from opendbc.car.tesla.preap.sp.radar_interface import RadarInterface as PreAPRadarInterface
from opendbc.car.tesla.radar_interface_sp import RadarInterface as ContinentalRadarInterface, RADAR_START_ADDR


def _apply_modern_tesla_v1_capabilities(ret: structs.CarParamsSP) -> structs.CarParamsSP:
  """Version-1 capability overlay for non-Pre-AP Tesla.

  madsFullSettingsAvailable follows the version-0 HAS_VEHICLE_BUS hardware path here;
  the TeslaMadsScreenButton and MadsSteeringMode refinements are applied later
  from the boot parameter snapshot in setup_interfaces.
  """
  ret.madsCapabilityContractVersion = 1
  ret.madsRequired = False
  ret.teslaCoopSteeringAvailable = True
  ret.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.none
  ret.madsMainCruiseAllowed = False
  ret.madsUnifiedEngagementMode = False
  ret.madsFullSettingsAvailable = bool(ret.flags & TeslaFlagsSP.HAS_VEHICLE_BUS)
  ret.madsHandsOnPauseAvailable = False
  ret.preapLateralEngagementMode = structs.CarParamsSP.PreapLateralEngagementMode.independent
  if not ret.madsFullSettingsAvailable:
    ret.madsSteeringMode = structs.CarParamsSP.MadsSteeringMode.disengage
  return ret


def RadarInterface(CP, CP_SP):
  if is_preap_platform(CP):
    return PreAPRadarInterface(CP, CP_SP)
  return ContinentalRadarInterface(CP, CP_SP)


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = staticmethod(RadarInterface)

  def __init__(self, CP, CP_SP=None):
    if CP_SP is None:
      CP_SP = structs.CarParamsSP()
    self._preap_platform = is_preap_platform(CP)
    if self._preap_platform:
      self.CarState = PreAPCarState
      self.CarController = PreAPCarController
      self.RadarInterface = PreAPRadarInterface
    super().__init__(CP, CP_SP)
    if self._preap_platform:
      # Pre-AP resolves cluster speed itself, including a truthful zero on its first DI frame.
      self.v_ego_cluster_seen = True

  def update(self, can_packets):
    if self._preap_platform:
      can_packets = sorted(can_packets, key=lambda packet_group: packet_group[0])
    return super().update(can_packets)

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      return get_preap_accel_limits(current_speed)
    return CarInterfaceBase.get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if is_preap_platform(candidate):
      ret.brand = "tesla"
      return get_preap_params(ret, fingerprint)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    # Model X and HW 2.5 vehicles are missing DAS_settings
    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[candidate]

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    fsd_14 = any(fw.ecu == Ecu.eps and fw.fwVersion in FSD_14_FW.get(candidate, []) for fw in car_fw)
    if fsd_14:
      ret.flags |= TeslaFlags.FSD_14.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.FSD_14.value

    ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X,)  # dashcam only, pending find invalidLkasSetting signal

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    if is_preap_platform(candidate):
      return get_preap_params_sp(stock_cp, ret)

    stock_cp.enableBsm = True

    if candidate == CAR.TESLA_MODEL_X:
      stock_cp.dashcamOnly = False

    if 0x3DF in fingerprint[1]:
      ret.flags |= TeslaFlagsSP.HAS_VEHICLE_BUS.value
      ret.safetyParam |= TeslaSafetyFlagsSP.HAS_VEHICLE_BUS

    return _apply_modern_tesla_v1_capabilities(ret)
