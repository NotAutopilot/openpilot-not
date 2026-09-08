from contextlib import contextmanager

from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.radar_interface import RadarInterface as NapRadarInterface


@contextmanager
def _radar_base_accepts_cp_only(CP_SP: structs.CarParamsSP):
  orig = RadarInterfaceBase.__init__

  def _init_with_sp(self_, CP_, CP_SP_=None):
    orig(self_, CP_, CP_SP if CP_SP_ is None else CP_SP_)

  RadarInterfaceBase.__init__ = _init_with_sp
  try:
    yield
  finally:
    RadarInterfaceBase.__init__ = orig


class RadarInterface(NapRadarInterface):
  def __init__(self, CP, CP_SP):
    with _radar_base_accepts_cp_only(CP_SP):
      super().__init__(CP)
    self.CP_SP = CP_SP
