import json
import unittest

from opendbc.sunnypilot.car.platform_list import get_car_list, CAR_LIST_JSON_OUT


class TestCarList(unittest.TestCase):
  def test_generator(self):
    generated_car_list = json.dumps(get_car_list(), indent=2, ensure_ascii=False)
    with open(CAR_LIST_JSON_OUT) as f:
      current_car_list = f.read()

    assert generated_car_list == current_car_list, "Run opendbc/sunnypilot/car/platform_list.py to update the car list"

  def test_tesla_preap_selector_entry(self):
    assert get_car_list()["Tesla Model S (Pre-AP) 2012-14"] == {
      "platform": "TESLA_MODEL_S_PREAP",
      "make": "Tesla",
      "brand": "tesla",
      "model": "Model S (Pre-AP)",
      "year": ["2012", "2013", "2014"],
      "package": "All",
    }
