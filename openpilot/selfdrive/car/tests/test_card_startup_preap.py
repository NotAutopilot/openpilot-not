"""Cold-start vehicle selection for NAPForcePreAP.

PREAP CAN fingerprint is address 551 (0x227) at 4 bytes. A 5-byte frame at
that address is incompatible, so CAN discovery cannot pick TESLA_MODEL_S_PREAP.
Production selection must still construct PREAP when NAPForcePreAP is true,
and must not force it when false/absent.
"""
from opendbc.car.can_definitions import CanData
from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import CAR

from openpilot.selfdrive.car.card import startup_get_car

# Decimal 551 (0x227), 5 bytes: incompatible with PREAP table 551:4.
_INCOMPATIBLE_PREAP = CanData(0x227, b"\x00" * 5, 0)


class _Params:
  def __init__(self, values: dict[str, bool] | None = None):
    self._values = values or {}

  def get_bool(self, key: str) -> bool:
    return bool(self._values[key]) if key in self._values else False


def _can_recv(wait_for_one: bool = False):
  del wait_for_one
  return [[_INCOMPATIBLE_PREAP]]


def _can_send(*args, **kwargs):
  del args, kwargs


def _set_obd(_obd: bool) -> None:
  return None


def _startup(params, monkeypatch, *, fingerprint_env=None, skip_fw_query_env=None):
  monkeypatch.delenv("FINGERPRINT", raising=False)
  monkeypatch.delenv("SKIP_FW_QUERY", raising=False)
  if fingerprint_env is not None:
    monkeypatch.setenv("FINGERPRINT", fingerprint_env)
  if skip_fw_query_env is not None:
    monkeypatch.setenv("SKIP_FW_QUERY", skip_fw_query_env)
  return startup_get_car(params, _can_recv, _can_send, _set_obd, False, False, None, None, False)


def test_nap_force_preap_true_constructs_preap_on_incompatible_can(monkeypatch):
  CI = _startup(_Params({"NAPForcePreAP": True}), monkeypatch)
  assert CI.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP
  assert CI.CP.brand == "tesla"
  assert not CI.CP.dashcamOnly
  assert CI.CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.teslaPreap
  assert CI.CP.fingerprintSource == CarParams.FingerprintSource.fixed


def test_nap_force_preap_false_does_not_force_preap(monkeypatch):
  CI = _startup(_Params({"NAPForcePreAP": False}), monkeypatch, skip_fw_query_env="1")
  assert CI.CP.carFingerprint != CAR.TESLA_MODEL_S_PREAP
  assert CI.CP.carFingerprint == "MOCK"
  assert CI.CP.dashcamOnly


def test_nap_force_preap_absent_does_not_force_preap(monkeypatch):
  CI = _startup(_Params(), monkeypatch, skip_fw_query_env="1")
  assert CI.CP.carFingerprint != CAR.TESLA_MODEL_S_PREAP
  assert CI.CP.carFingerprint == "MOCK"
  assert CI.CP.dashcamOnly


def test_env_fingerprint_wins_over_nap_force_preap(monkeypatch):
  CI = _startup(_Params({"NAPForcePreAP": True}), monkeypatch,
                fingerprint_env="MOCK", skip_fw_query_env="1")
  assert CI.CP.carFingerprint == "MOCK"
  assert CI.CP.dashcamOnly
