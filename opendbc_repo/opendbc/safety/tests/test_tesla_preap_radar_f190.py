from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 2

F190 = bytes([0x03, 0x22, 0xF1, 0x90, 0x00, 0x00, 0x00, 0x00])
TESTER = bytes([0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
OTHER_DID = bytes([0x03, 0x22, 0xF0, 0x14, 0x00, 0x00, 0x00, 0x00])
SECURITY = bytes([0x02, 0x27, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00])
ROUTINE = bytes([0x04, 0x31, 0x01, 0x0A, 0x03, 0x00, 0x00, 0x00])


class TestTeslaPreAPRadarF190:
  TX_MSGS = []

  def setup_method(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, PREAP_FLAG_RADAR_EMULATION)
    self.safety.init_tests()

  def test_allows_f190_read_while_disengaged(self):
    self.safety.set_controls_allowed(0)
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, TESTER)) is True
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, F190)) is True

  def test_rejects_f190_when_controls_allowed(self):
    self.safety.set_controls_allowed(1)
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, F190)) is False

  def test_rejects_f190_without_radar(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(0)
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, F190)) is False

  def test_rejects_writes_routines_and_other_dids(self):
    self.safety.set_controls_allowed(0)
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, OTHER_DID)) is False
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, SECURITY)) is False
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 1, ROUTINE)) is False

  def test_rejects_uds_on_chassis_bus(self):
    self.safety.set_controls_allowed(0)
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x641, 0, F190)) is False

  def test_560_still_never_leaves_host(self):
    self.safety.set_controls_allowed(0)
    dat = bytes([0, 1, 1, 0, 0, ord("5"), ord("Y"), ord("J")])
    assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat)) is False
