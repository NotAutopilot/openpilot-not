from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.preap.radar_donor_vin import (
  FLOW_CONTROL,
  RADAR_UDS_BUS,
  RADAR_UDS_RX,
  RADAR_UDS_TX,
  RadarDonorVinCommissioner,
  RadarDonorVinFailure,
  RadarDonorVinReader,
  RadarDonorVinState,
  radar_alert_vin_validity,
)


VIN = "5YJSA1E42FF156789"


def _rx(payload: bytes) -> CanData:
  return CanData(RADAR_UDS_RX, (bytes([len(payload)]) + payload).ljust(8, b"\x00"), RADAR_UDS_BUS)


def _first_frame(payload: bytes) -> CanData:
  length = len(payload)
  dat = bytes([0x10 | (length >> 8), length & 0xFF]) + payload[:6]
  return CanData(RADAR_UDS_RX, dat.ljust(8, b"\x00"), RADAR_UDS_BUS)


def _consecutive(sequence: int, data: bytes) -> CanData:
  return CanData(RADAR_UDS_RX, bytes([0x20 | sequence]) + data.ljust(7, b"\x00"), RADAR_UDS_BUS)


def _drive_to_read(reader: RadarDonorVinReader, now: float) -> float:
  replies = {
    RadarDonorVinState.TESTER_PRESENT: b"\x7e\x00",
    RadarDonorVinState.DEFAULT_SESSION: b"\x50\x01",
    RadarDonorVinState.EXTENDED_SESSION: b"\x50\x03",
    RadarDonorVinState.READINESS: b"\x7e\x00",
  }
  while reader.state in replies:
    output = reader.update([], now)
    assert output.can_sends
    now += 0.01
    output = reader.update([_rx(replies[reader.state])], now)
    now += 0.01
    assert output.failure is None
  assert reader.state == RadarDonorVinState.READ_F190
  return now


def test_radar_alert_vin_validity_bit():
  clear = bytes(8)
  set_bit = bytes([0, 0, 0, 0, 0x10, 0, 0, 0])
  assert radar_alert_vin_validity(clear) is False
  assert radar_alert_vin_validity(set_bit) is True
  assert radar_alert_vin_validity(b"\x00") is False


def test_reader_completes_on_multiframe_f190():
  reader = RadarDonorVinReader()
  now = 0.0
  reader.start(now)
  now = _drive_to_read(reader, now)

  output = reader.update([], now)
  assert output.can_sends[0].address == RADAR_UDS_TX
  assert output.can_sends[0].dat[:4] == b"\x03\x22\xf1\x90"

  vin_payload = b"\x62\xf1\x90" + VIN.encode("ascii")
  now += 0.01
  first = reader.update([_first_frame(vin_payload)], now)
  assert first.can_sends == (CanData(RADAR_UDS_TX, FLOW_CONTROL, RADAR_UDS_BUS),)
  now += 0.01
  rest = vin_payload[6:]
  output = reader.update([_consecutive(1, rest[:7]), _consecutive(2, rest[7:])], now)
  assert output.vin == VIN
  assert output.state == RadarDonorVinState.CLEANUP

  now += 0.01
  output = reader.update([], now)
  assert output.can_sends[0].dat[:3] == b"\x02\x3e\x80"
  assert output.can_sends[1].dat[:3] == b"\x02\x10\x01"
  now += 0.01
  output = reader.update([_rx(b"\x50\x01")], now)
  now += 0.01
  output = reader.update([], now)
  now += 0.01
  output = reader.update([_rx(b"\x50\x01")], now)
  assert output.state == RadarDonorVinState.COMPLETE
  assert output.vin == VIN
  assert output.failure is None


def test_reader_rejects_short_vin_and_cleans_up():
  reader = RadarDonorVinReader()
  now = 0.0
  reader.start(now)
  now = _drive_to_read(reader, now)
  reader.update([], now)
  now += 0.01
  output = reader.update([_rx(b"\x62\xf1\x90ABC")], now)
  assert output.failure == RadarDonorVinFailure.INVALID_VIN
  assert output.state == RadarDonorVinState.CLEANUP
  assert output.vin is None


def test_reader_times_out_waiting_for_f190():
  reader = RadarDonorVinReader()
  now = 0.0
  reader.start(now)
  now = _drive_to_read(reader, now)
  reader.update([], now)
  output = reader.update([], now + RadarDonorVinReader.RESPONSE_TIMEOUT)
  assert output.failure == RadarDonorVinFailure.TIMEOUT
  assert output.state == RadarDonorVinState.CLEANUP


def test_reader_does_not_start_from_idle():
  reader = RadarDonorVinReader()
  output = reader.update([], 1.0)
  assert output.state == RadarDonorVinState.IDLE
  assert output.can_sends == ()


def _fault_frame() -> CanData:
  return CanData(0x501, bytes([0, 0, 0, 0, 0x10, 0, 0, 0]), RADAR_UDS_BUS)


def _commissioner_kwargs(**overrides):
  values = {
    "radar_enabled": True,
    "stored_vin": "",
    "controls_allowed": False,
    "enabled": False,
  }
  values.update(overrides)
  return values


def test_commissioner_ignores_happy_radar():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  clear = CanData(0x501, bytes(8), RADAR_UDS_BUS)
  for now in range(10):
    sends = commissioner.update([clear], float(now), **_commissioner_kwargs())
    assert sends == ()
  assert stored == []
  assert commissioner.reader.state == RadarDonorVinState.IDLE


def test_commissioner_starts_after_stable_vin_fault():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  sends = ()
  for now in range(RadarDonorVinCommissioner.STABLE_FRAMES):
    sends = commissioner.update([_fault_frame()], float(now), **_commissioner_kwargs())
  assert sends
  assert sends[0].address == RADAR_UDS_TX
  assert stored == []


def test_commissioner_does_not_touch_existing_donor_vin():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  for now in range(10):
    sends = commissioner.update([_fault_frame()], float(now), **_commissioner_kwargs(stored_vin=VIN))
    assert sends == ()
  assert stored == []


def test_commissioner_force_read_starts_without_vin_fault():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  sends = commissioner.update([], 0.0, **_commissioner_kwargs(stored_vin=VIN), force_read=True)
  assert sends
  assert sends[0].address == RADAR_UDS_TX
  assert stored == []
  assert commissioner.reader.state == RadarDonorVinState.TESTER_PRESENT


def test_commissioner_force_read_does_not_restart_every_tick():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  first = commissioner.update([], 0.0, **_commissioner_kwargs(), force_read=True)
  second = commissioner.update([], 0.01, **_commissioner_kwargs(), force_read=True)
  assert first
  assert second == ()
  assert commissioner.reader.state == RadarDonorVinState.TESTER_PRESENT


def test_commissioner_force_read_can_run_again_after_clear():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  commissioner.update([], 0.0, **_commissioner_kwargs(), force_read=True)
  commissioner.reader.state = RadarDonorVinState.COMPLETE
  commissioner.update([], 0.01, **_commissioner_kwargs(), force_read=False)
  sends = commissioner.update([], 0.02, **_commissioner_kwargs(), force_read=True)
  assert sends
  assert commissioner.reader.state == RadarDonorVinState.TESTER_PRESENT


def test_commissioner_stores_vin_when_cleanup_fails():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  commissioner.reader.vin = VIN
  commissioner.reader.state = RadarDonorVinState.FAILED
  commissioner.reader.failure = RadarDonorVinFailure.TIMEOUT
  commissioner.update([], 1.0, **_commissioner_kwargs())
  commissioner.update([], 1.1, **_commissioner_kwargs())
  assert stored == [VIN]


def test_commissioner_stores_vin_as_soon_as_f190_decodes():
  stored = []
  commissioner = RadarDonorVinCommissioner(stored.append)
  now = 0.0
  commissioner.update([], now, **_commissioner_kwargs(), force_read=True)
  replies = {
    RadarDonorVinState.TESTER_PRESENT: b"\x7e\x00",
    RadarDonorVinState.DEFAULT_SESSION: b"\x50\x01",
    RadarDonorVinState.EXTENDED_SESSION: b"\x50\x03",
    RadarDonorVinState.READINESS: b"\x7e\x00",
  }
  while commissioner.reader.state in replies:
    now += 0.01
    commissioner.update([_rx(replies[commissioner.reader.state])], now, **_commissioner_kwargs())
  assert commissioner.reader.state == RadarDonorVinState.READ_F190
  now += 0.01
  commissioner.update([], now, **_commissioner_kwargs())
  vin_payload = b"\x62\xf1\x90" + VIN.encode("ascii")
  now += 0.01
  commissioner.update([_first_frame(vin_payload)], now, **_commissioner_kwargs())
  now += 0.01
  rest = vin_payload[6:]
  commissioner.update(
    [_consecutive(1, rest[:7]), _consecutive(2, rest[7:])],
    now, **_commissioner_kwargs(),
  )
  assert stored == [VIN]
  assert commissioner.reader.state == RadarDonorVinState.CLEANUP
