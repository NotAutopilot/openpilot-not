from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
import opendbc.car.tesla.preap.nap_conf as nap_conf_mod
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from opendbc.car.tesla.values import CAR
from openpilot.common.params import Params


def test_preap_interface_identifies_car_and_safety_model():
  car_params = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)

  assert car_params.carFingerprint == CAR.TESLA_MODEL_S_PREAP
  assert car_params.brand == "tesla"
  assert car_params.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap


def test_saved_pedal_and_radar_params_produce_safety_param_3():
  params = Params()
  params.put_bool(NAPParamKeys.PEDAL_ENABLED, True, block=True)
  params.put_bool(NAPParamKeys.RADAR_ENABLED, True, block=True)
  nap_conf_mod._PARAMS_AVAILABLE = True
  nap_conf_mod._params = params

  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)

  assert int(ret.safetyConfigs[0].safetyParam) == 3
  assert ret.openpilotLongitudinalControl
  assert not ret.pcmCruise
