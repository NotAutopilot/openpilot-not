from opendbc.car import structs
from opendbc.car.tesla.preap import interface as nap_interface
from opendbc.car.tesla.preap.interface import get_preap_accel_limits
from opendbc.car.tesla.preap.sp.platform import PREAP_PLATFORM, is_preap_platform
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

PREAP_FLAG_HANDS_ON_PAUSE = 8

__all__ = [
  "PREAP_PLATFORM",
  "get_preap_accel_limits",
  "get_preap_params",
  "get_preap_params_sp",
  "is_preap_platform",
]


# NAP get_preap_params writes vEgoStopping / vEgoStarting / stoppingDecelRate.
# Sunnypilot moved those into CarParams.deprecated; the planner has no consumer
# (known gap, host wave). Swallow only those three names.
_DEPRECATED_LONG_FIELDS = frozenset({"vEgoStopping", "vEgoStarting", "stoppingDecelRate"})


class _CapnpSink:
  def __init__(self, inner):
    object.__setattr__(self, "_inner", inner)

  def __getattr__(self, name):
    return getattr(object.__getattribute__(self, "_inner"), name)

  def __setattr__(self, name, value):
    inner = object.__getattribute__(self, "_inner")
    try:
      setattr(inner, name, value)
    except AttributeError:
      if name not in _DEPRECATED_LONG_FIELDS:
        raise
    except Exception as exc:
      if name not in _DEPRECATED_LONG_FIELDS or "capnp" not in type(exc).__module__:
        raise


def get_preap_params(ret, fingerprint):
  nap_interface.get_preap_params(_CapnpSink(ret), fingerprint)
  pause = False
  try:
    from openpilot.common.params import Params
    pause = bool(Params().get_bool("TeslaPreapHandsOnPause"))
  except Exception:
    pause = False
  if pause and ret.safetyConfigs:
    ret.safetyConfigs[0].safetyParam |= PREAP_FLAG_HANDS_ON_PAUSE
  return ret


def get_preap_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP) -> structs.CarParamsSP:
  """Sunnypilot CarParamsSP overlay. MADS required, momentary stalk, no touchscreen."""
  ret.madsCapabilityContractVersion = 1
  ret.madsFullSettingsAvailable = True
  ret.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
  ret.madsRequired = True
  ret.teslaCoopSteeringAvailable = False
  ret.madsHandsOnPauseAvailable = True
  ret.madsMainCruiseAllowed = False
  ret.madsUnifiedEngagementMode = False
  ret.madsSteeringMode = structs.CarParamsSP.MadsSteeringMode.remainActive
  ret.flags &= ~int(TeslaFlagsSP.HAS_VEHICLE_BUS)
  ret.flags &= ~int(TeslaFlagsSP.COOP_STEERING)
  ret.flags &= ~int(TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER | TeslaFlagsSP.MADS_SCREEN_BUTTON_4_FINGER |
                    TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER)
  ret.flags &= ~int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
  ret.flags &= ~int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
  if not bool(getattr(stock_cp, "radarUnavailable", True)):
    ret.flags |= int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
  safety_param = 0
  if getattr(stock_cp, "safetyConfigs", None):
    safety_param = int(getattr(stock_cp.safetyConfigs[0], "safetyParam", 0) or 0)
  if safety_param & PREAP_FLAG_HANDS_ON_PAUSE:
    ret.flags |= int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
  return ret
