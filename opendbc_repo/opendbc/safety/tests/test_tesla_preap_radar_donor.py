from opendbc.car.structs import CarParams
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 2

AWD_VIN = "5YJSA1E42FF156789"  # character 8 is '4'
JACK_RWD_VIN = "5YJSA1E25FF106153"  # character 8 is '2' (Tesla dual-motor)
OLD_A_RWD_VIN = "5YJSA1H13EFP20460"  # character 8 is '1' (old A daily lock)


def _payload(safety, getter):
  return bytes(getter(i) for i in range(8))


def _send_donor(safety, vin, position=0, epas_type=1):
  for fragment in range(3):
    _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(fragment, vin, True, position, epas_type)
    allowed = safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    assert allowed is False


class TestTeslaPreAPRadarDonor:
  TX_MSGS = []

  def setup_method(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, PREAP_FLAG_RADAR_EMULATION)
    self.safety.init_tests()

  def test_gtw_silent_until_vin_stream_complete(self):
    source = bytes.fromhex("0281555300000000")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert self.safety.tesla_preap_radar_car_config_captured() is False
    assert self.safety.tesla_preap_radar_ready_debug() is False

    _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(0, "", True, 0, 0)
    self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert self.safety.tesla_preap_radar_car_config_captured() is False
    assert self.safety.tesla_preap_radar_ready_debug() is False

    _send_donor(self.safety, "", position=0, epas_type=0)
    assert self.safety.tesla_preap_radar_ready_debug() is True
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert self.safety.tesla_preap_radar_car_config_captured() is True

  def test_use_radar_clear_keeps_gtw_silent(self):
    for fragment in range(3):
      _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(fragment, AWD_VIN, False, 0, 0)
      self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    assert self.safety.tesla_preap_radar_ready_debug() is False
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert self.safety.tesla_preap_radar_car_config_captured() is False

  def test_empty_vin_keeps_passthrough(self):
    _send_donor(self.safety, "", position=0, epas_type=0)
    source = bytes.fromhex("0281555300000000")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4285555300000010")
    assert self.safety.tesla_preap_radar_donor_active_debug() is False
    assert self.safety.tesla_preap_radar_ready_debug() is True

  def test_empty_vin_still_applies_position(self):
    _send_donor(self.safety, "", position=1, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is False
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4285555310000010")

  def test_awd_donor_vin_sets_4wd_to_match_vin_char(self):
    _send_donor(self.safety, AWD_VIN, position=1, epas_type=3)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True

    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a85555310300010")

  def test_awd_source_config_remains_four_wheel_drive(self):
    _send_donor(self.safety, AWD_VIN, position=0, epas_type=0)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0a90555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a95555300001710")

  def test_this_car_vin_char2_sets_4wd_matching_tinkla(self):
    # 5YJSA1E25FF106153 char 8 is '2' (Tesla dual-motor encoding). Honest
    # chassis 2WD against that VIN is the 1d/11/14/15 freeze (xwdValidity).
    _send_donor(self.safety, JACK_RWD_VIN, position=0, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a95555300001710")

  def test_rwd_single_motor_vin_preserves_chassis_2wd(self):
    # Old A daily lock: char 8 '1', empty-style 2WD declaration.
    _send_donor(self.safety, OLD_A_RWD_VIN, position=0, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4295555300001710")

  def test_donor_vin_replaces_mux_records(self):
    _send_donor(self.safety, AWD_VIN)
    mux = bytes([0x11, 0, 0, 0, 0, 0, 0, 0])
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x405, 0, mux))
    assert self.safety.tesla_preap_radar_vin_feed_captured() is True
    dat = _payload(self.safety, self.safety.tesla_preap_radar_vin_feed_data)
    assert dat[0] == 0x11
    assert dat[1:4] == b"SA1"
    assert dat[4:8] == b"E42F"
