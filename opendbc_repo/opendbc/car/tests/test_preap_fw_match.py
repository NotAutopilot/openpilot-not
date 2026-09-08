import unittest

from opendbc.car.fw_versions import match_fw_to_car, match_fw_to_car_exact, build_fw_dict
from opendbc.car.structs import CarParams
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.preap.sp.interface import PREAP_PLATFORM
from opendbc.car.tesla.values import CAR

CarFw = CarParams.CarFw
Ecu = CarParams.Ecu


class TestPreAPFirmwareMatching(unittest.TestCase):
  def test_preap_is_not_a_firmware_candidate(self):
    self.assertNotIn(CAR.TESLA_MODEL_S_PREAP, FW_VERSIONS)
    self.assertNotIn(PREAP_PLATFORM, FW_VERSIONS)

  def test_modern_tesla_firmware_does_not_return_preap(self):
    model3_fw = list(FW_VERSIONS[CAR.TESLA_MODEL_3][(Ecu.eps, 0x730, None)])[0]
    car_fw = [CarFw(ecu=Ecu.eps, fwVersion=model3_fw, brand="tesla", address=0x730, subAddress=0)]
    exact, matches = match_fw_to_car(car_fw, vin="", allow_fuzzy=False)
    self.assertTrue(exact)
    self.assertNotIn(PREAP_PLATFORM, matches)
    self.assertNotIn(CAR.TESLA_MODEL_S_PREAP, matches)
    self.assertIn(CAR.TESLA_MODEL_3, matches)

  def test_unrelated_firmware_does_not_return_preap(self):
    car_fw = [CarFw(ecu=Ecu.eps, fwVersion=b"UNRELATED_FW_NOT_IN_DB", brand="tesla", address=0x730, subAddress=0)]
    live = build_fw_dict(car_fw, filter_brand="tesla")
    matches = match_fw_to_car_exact(live, match_brand="tesla")
    self.assertNotIn(PREAP_PLATFORM, matches)
    self.assertNotIn(CAR.TESLA_MODEL_S_PREAP, matches)

  def test_empty_fw_map_cannot_match(self):
    live = build_fw_dict([], filter_brand="tesla")
    matches = match_fw_to_car_exact(live, match_brand="tesla")
    self.assertNotIn(PREAP_PLATFORM, matches)


if __name__ == "__main__":
  unittest.main()
