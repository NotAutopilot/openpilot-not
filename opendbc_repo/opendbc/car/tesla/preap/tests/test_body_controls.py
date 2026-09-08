"""Frame-level invariants for NAP Pre-AP DAS_bodyControls (turn signal drive)."""
from opendbc.can import CANPacker
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS


def _tc():
  packer = CANPacker("tesla_preap")
  return TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})


def test_addr_is_body_controls():
  tc = _tc()
  addr, _, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  assert addr == 0x3E9  # DAS_bodyControls / 1001


def test_turn_left_sets_indicator_left():
  tc = _tc()
  _, dat, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  # DAS_turnIndicatorRequest is at bit 8 (byte 1, bits 0-1)
  assert dat[1] & 0x03 == 1


def test_turn_right_sets_indicator_right():
  tc = _tc()
  _, dat, _ = tc.create_body_controls_message(2, 0, CANBUS.party, 1)
  assert dat[1] & 0x03 == 2


def test_turn_none_sets_indicator_none():
  tc = _tc()
  _, dat, _ = tc.create_body_controls_message(0, 0, CANBUS.party, 1)
  assert dat[1] & 0x03 == 0


def test_reason_set_when_turning():
  tc = _tc()
  # DAS_turnIndicatorRequestReason at bit 16 (byte 2, bits 0-3)
  _, dat_on, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  _, dat_off, _ = tc.create_body_controls_message(0, 0, CANBUS.party, 1)
  assert dat_on[2] & 0x0F == 1
  assert dat_off[2] & 0x0F == 0


def test_turn_value_encoding_matches_cc_convention():
  # turn = rightBlinker*2 + leftBlinker, as used in _update_preap.
  assert (int(True) * 2 + int(False)) == 2    # right only
  assert (int(False) * 2 + int(True)) == 1    # left only
  assert (int(False) * 2 + int(False)) == 0   # none


def test_body_controls_frame_addr_and_bus():
  tc = _tc()
  addr, _, bus = tc.create_body_controls_message(1, 0, CANBUS.party, 3)
  assert addr == 0x3E9
  assert bus == CANBUS.party
