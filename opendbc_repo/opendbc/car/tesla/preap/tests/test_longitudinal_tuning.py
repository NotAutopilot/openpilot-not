"""Pre-AP longitudinal feedback ownership contracts."""

from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.tesla.preap import interface


def test_pedal_params_leave_generic_outer_feedback_disabled(monkeypatch):
  pedal_conf = SimpleNamespace(
    use_pedal=True,
    radar_enabled=False,
  )
  monkeypatch.setattr(interface, "nap_conf", pedal_conf)
  params = structs.CarParams.new_message()
  params.wheelbase = 2.96

  configured_params = interface.get_preap_params(params, fingerprint={})

  assert list(configured_params.longitudinalTuning.kpV) == [0.0] * 4
  assert list(configured_params.longitudinalTuning.kiV) == [0.0] * 4
  assert configured_params.longitudinalTuning.kf == 1.0
