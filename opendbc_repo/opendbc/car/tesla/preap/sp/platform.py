"""Pre-AP platform test with no Pre-AP imports.

Sunnypilot's generic car interface imports this at module load. Keeping it
free of the Pre-AP core means importing opendbc never touches Params or the
Pre-AP config, which matters for build-time codegen and for other brands.
"""

from opendbc.car.tesla.values import CAR
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


PREAP_PLATFORM = "TESLA_MODEL_S_PREAP"


def is_preap_platform(CP_or_candidate) -> bool:
  if isinstance(CP_or_candidate, str):
    return CP_or_candidate == CAR.TESLA_MODEL_S_PREAP
  fingerprint = getattr(CP_or_candidate, "carFingerprint", None)
  if fingerprint is not None:
    return fingerprint == CAR.TESLA_MODEL_S_PREAP
  return CP_or_candidate == CAR.TESLA_MODEL_S_PREAP


def is_preap_ui_platform(bundle_platform: str = "", CP=None) -> bool:
  """Settings visibility. Bundle platform wins when present; never HAS_VEHICLE_BUS."""
  if bundle_platform:
    return bundle_platform == PREAP_PLATFORM
  return bool(CP is not None and is_preap_platform(CP))


def preap_radar_present(CP, CP_SP=None) -> bool:
  """True only on TESLA_MODEL_S_PREAP with PREAP_RADAR_PRESENT set.

  TeslaFlagsSP.PREAP_RADAR_PRESENT is bit 64, the same value as HyundaiFlagsSP.NON_SCC.
  Brand-shared CP_SP.flags therefore require exact Pre-AP platform identity.
  """
  if not is_preap_platform(CP):
    return False
  flags = int(getattr(CP_SP, "flags", 0) or 0) if CP_SP is not None else 0
  return bool(flags & int(TeslaFlagsSP.PREAP_RADAR_PRESENT))
