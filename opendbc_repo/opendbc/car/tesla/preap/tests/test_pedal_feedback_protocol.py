"""Comma Pedal feedback wire-protocol regression tests.

These tests start from literal CAN payloads instead of CANPacker output. A
packer and parser generated from the same incorrect DBC would otherwise agree
with each other and conceal a wire-contract error.
"""

from opendbc.can import CANParser
from opendbc.car.tesla.preap.pedal_feedback import PEDAL_TIMEOUT_MS, PedalFeedback
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP


PEDAL_DBC = "tesla_preap"
GAS_SENSOR = "GAS_SENSOR"
GAS_COMMAND_ADDR = 0x551
GAS_SENSOR_ADDR = 0x552
PEDAL_BUS = 2


def pedal_checksum(address, data):
  """Checksum used by the pedal firmware for command and feedback payloads."""
  return (sum(data) + (address & 0xFF) + (address >> 8)) & 0xFF


def gas_sensor_frame(state, idx):
  """Build a firmware-format frame: STATE high nibble, counter low nibble."""
  data = bytearray((0x01, 0xD6, 0x00, 0xFA, ((state & 0xF) << 4) | (idx & 0xF), 0))
  data[5] = pedal_checksum(GAS_SENSOR_ADDR, data[:5])
  return GAS_SENSOR_ADDR, bytes(data), PEDAL_BUS


def decode(parser, state, idx, now_ms=0):
  parser.update([now_ms * 1_000_000, [gas_sensor_frame(state, idx)]])
  return parser.vl[GAS_SENSOR]


def make_parser():
  return CANParser(PEDAL_DBC, [(GAS_SENSOR, 0)], PEDAL_BUS)


def test_disabled_commands_are_valid_rolling_watchdog_heartbeats():
  tesla_can = TeslaCANPreAP({})

  for idx in range(16):
    address, data, bus = tesla_can.create_pedal_command(accel_command=99, enable=0)
    assert address == GAS_COMMAND_ADDR
    assert bus == PEDAL_BUS
    assert data[:4] == b"\x00\x00\x00\x00"
    assert data[4] == idx
    assert data[5] == pedal_checksum(address, data[:5])


def test_firmware_known_vector_decodes_state_counter_and_checksum():
  parser = make_parser()
  frame = gas_sensor_frame(state=5, idx=10)

  assert frame[1] == bytes.fromhex("01 d6 00 fa 5a 82")
  parser.update([0, [frame]])

  assert parser.vl[GAS_SENSOR]["STATE"] == 5
  assert parser.vl[GAS_SENSOR]["IDX"] == 10
  assert parser.vl[GAS_SENSOR]["CHECKSUM"] == 0x82


def test_all_wire_state_counter_combinations_decode_without_aliasing():
  parser = make_parser()

  for state in range(16):
    for idx in range(16):
      decoded = decode(parser, state, idx)
      assert decoded["STATE"] == state
      assert decoded["IDX"] == idx


def test_rolling_wire_counter_keeps_feedback_available():
  parser = make_parser()
  feedback = PedalFeedback()

  for sequence, now_ms in enumerate(range(0, 2 * PEDAL_TIMEOUT_MS + 1, 100)):
    decoded = decode(parser, state=0, idx=sequence % 16, now_ms=now_ms)
    assert feedback.update(decoded, now_ms)
    assert feedback.interceptor_state == 0
    assert feedback.idx == sequence % 16
    assert not feedback.timeout
    assert feedback.available


def test_state_changes_cannot_impersonate_a_fresh_counter():
  parser = make_parser()
  feedback = PedalFeedback()

  feedback.update(decode(parser, state=0, idx=7), curr_time_ms=0)
  for state, now_ms in enumerate(range(100, PEDAL_TIMEOUT_MS + 201, 100), start=1):
    feedback.update(decode(parser, state=state, idx=7, now_ms=now_ms), now_ms)

  assert feedback.idx == 7
  assert feedback.timeout


def test_nonzero_wire_state_marks_feedback_unavailable():
  parser = make_parser()
  feedback = PedalFeedback()

  feedback.update(decode(parser, state=5, idx=0), curr_time_ms=0)

  assert feedback.interceptor_state == 5
  assert not feedback.available
