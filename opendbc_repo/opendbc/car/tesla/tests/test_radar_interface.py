from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.sp.radar_interface import RadarInterface as PreAPRadarInterface
from opendbc.car.tesla.radar_interface import RadarInterface as NapRadarInterface
from opendbc.car.tesla.radar_interface_sp import RadarInterface as ContinentalRadarInterface
from opendbc.car.tesla.values import CAR


def _params(candidate):
  CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, gen_empty_fingerprint(), [], False, False, False)
  return CP, CP_SP


class TestTeslaRadarDispatch:
  def test_preap_candidate_selects_bosch_implementation(self):
    CP, CP_SP = _params(CAR.TESLA_MODEL_S_PREAP)
    radar = interfaces[CAR.TESLA_MODEL_S_PREAP].RadarInterface(CP, CP_SP)

    assert type(radar) is PreAPRadarInterface
    assert radar.trigger_msg == 0x36E
    assert radar.num_points == 32
    assert radar.bosch_radar
    assert CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP

  def test_preap_dispatch_does_not_use_root_nap_class(self):
    CP, CP_SP = _params(CAR.TESLA_MODEL_S_PREAP)
    radar = interfaces[CAR.TESLA_MODEL_S_PREAP].RadarInterface(CP, CP_SP)

    assert type(radar) is PreAPRadarInterface
    assert type(radar) is not NapRadarInterface
    assert type(radar).__module__.endswith("tesla.preap.sp.radar_interface")

  def test_modern_tesla_does_not_select_preap_adapter(self):
    CP, CP_SP = _params(CAR.TESLA_MODEL_X)
    radar = interfaces[CAR.TESLA_MODEL_X].RadarInterface(CP, CP_SP)

    assert type(radar) is ContinentalRadarInterface
    assert type(radar).__module__.endswith("tesla.radar_interface_sp")
    assert type(radar) is not NapRadarInterface
    assert type(radar) is not PreAPRadarInterface
