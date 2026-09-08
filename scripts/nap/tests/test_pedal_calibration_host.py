"""Native pedal calibration host: admission, CAN, session protocol, persist."""
from __future__ import annotations

from types import SimpleNamespace

import pytest
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID, TeslaCANPreAP

from scripts.nap.calibrate_pedal import (
  BRAKE_APPLIED,
  BRAKE_MESSAGE_ID,
  BRAKE_NOT_APPLIED,
  DI_TORQUE1_ID,
  DI_TORQUE2_ID,
  ESP_B_ID,
  GEAR_NEUTRAL,
  GTW_STATUS_ID,
  PedalCalibrator,
  PedalCalibrationError,
  build_pedal_command,
  esp_b_speed_ms,
  parse_pedal_calibration_args,
  persist_calibration,
)
from scripts.nap.pedal_session import (
  CALIBRATION_PARAM_BUS0,
  CALIBRATION_PARAM_BUS2,
  HEARTBEAT_REQUEST,
  HEARTBEAT_TIMEOUT_MS,
  PREAP_FLAG_ENABLE_PEDAL,
  PREAP_FLAG_PEDAL_CALIBRATION,
  SAFETY_ALLOUTPUT,
  SAFETY_SILENT,
  SAFETY_TESLA_PREAP,
  STANDSTILL_SPEED_MS,
  PedalCalibrationSession,
  TransportError,
  admission_from_submaster,
  calibration_safety_param,
  parse_configured_pedal_bus,
  pedal_calibration_entry_reason,
  processes_released,
  wait_for_manager_release,
)


class RecordingParams:
  def __init__(self):
    self.store = {}
    self.puts = []
    self.fail_on = None

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def put_bool(self, key, value, block=True):
    self.puts.append(key)
    if self.fail_on == key:
      raise OSError("persist failed")
    self.store[key] = bool(value)

  def get(self, key, return_default=False):
    return self.store.get(key)

  def put(self, key, value, block=True):
    self.puts.append(key)
    if self.fail_on == key:
      raise OSError("persist failed")
    self.store[key] = value


class RecvSession:
  def __init__(self, msgs=None):
    self._msgs = list(msgs or [])
    self.sent = []
    self.heartbeats = []
    self.silent = 0

  def can_recv(self):
    return list(self._msgs)

  def can_send(self, addr, dat, bus):
    self.sent.append((int(addr), bytes(dat), int(bus)))

  def send_heartbeat(self, engaged=False):
    self.heartbeats.append(engaged)

  def require_calibration_health(self, mode, param):
    return None

  def set_silent(self):
    self.silent += 1


class FakeSubMaster:
  def __init__(self, *, started=False, engaged=False, v_ego=0.0, car_fresh=True,
               processes=None, device_fresh=True, selfdrive_fresh=True,
               can_msgs=None, can_fresh=False):
    self.seen = {}
    self.alive = {}
    self.valid = {}
    self.data = {}
    self._mark("deviceState", device_fresh)
    self.data["deviceState"] = SimpleNamespace(started=started)
    if started or selfdrive_fresh:
      self._mark("selfdriveState", selfdrive_fresh)
      self.data["selfdriveState"] = SimpleNamespace(enabled=engaged)
    if car_fresh:
      self._mark("carState", True)
      self.data["carState"] = SimpleNamespace(vEgo=v_ego)
    if can_msgs is not None:
      self._mark("can", can_fresh)
      self.data["can"] = can_msgs
    if processes is not None:
      self._mark("managerState", True)
      self.data["managerState"] = SimpleNamespace(processes=processes)

  def _mark(self, name, fresh):
    self.seen[name] = True
    self.alive[name] = fresh
    self.valid[name] = fresh

  def update(self, _timeout=0):
    return None

  def __getitem__(self, name):
    return self.data[name]


def _health_bytes(*, safety_mode=0, safety_param=0, heartbeat_lost=0,
                  ignition_line=0, ignition_can=0):
  from panda import Panda
  n = len(Panda.HEALTH_STRUCT.unpack(bytes(Panda.HEALTH_STRUCT.size)))
  vals = [0] * n
  vals[8] = ignition_line
  vals[9] = ignition_can
  vals[12] = safety_mode
  vals[13] = safety_param
  vals[16] = heartbeat_lost
  return Panda.HEALTH_STRUCT.pack(*vals)


class UsbHandle:
  def __init__(self, *, health=b"", can=b"", fail=None, clock=None):
    self.health = health
    self.can = can
    self.fail = fail
    self.clock = clock
    self.writes = []
    self.reads = []
    self.bulk = []
    self.last_heartbeat_t = None

  def controlWrite(self, request_type, request, value, index, data, timeout=0, **kwargs):
    now = self.clock() if self.clock else 0.0
    self.writes.append((request, value, index, timeout, now))
    if request == HEARTBEAT_REQUEST:
      self.last_heartbeat_t = now
    if self.fail == "write":
      raise RuntimeError("usb timeout")

  def controlRead(self, request_type, request, value, index, length, timeout=0):
    self.reads.append((request, length, timeout))
    if self.fail == "read":
      raise RuntimeError("usb timeout")
    return self.health

  def bulkRead(self, endpoint, length, timeout=0):
    self.bulk.append((endpoint, length, timeout))
    if self.fail == "bulk":
      raise RuntimeError("usb timeout")
    return self.can


def _usb_panda(handle=None, **handle_kw):
  from panda import Panda

  class UsbPanda:
    HEALTH_STRUCT = Panda.HEALTH_STRUCT

    def __init__(self, usb_handle):
      self._handle = usb_handle
      self.can_rx_overflow_buffer = bytearray()
      self.safety_calls = []
      self.tx = []

    def set_safety_mode(self, mode, param=0):
      self.safety_calls.append((mode, param))

    def can_send(self, addr, dat, bus, **kwargs):
      self.tx.append((addr, bytes(dat), bus))

    def close(self):
      return None

  return UsbPanda(handle or UsbHandle(**handle_kw))


def test_parse_configured_pedal_bus_preserves_zero_and_defaults_empty():
  assert parse_configured_pedal_bus(0) == 0
  assert parse_configured_pedal_bus("0") == 0
  assert parse_configured_pedal_bus(b"0") == 0
  assert parse_configured_pedal_bus(None) == 2
  assert parse_configured_pedal_bus("") == 2
  assert parse_configured_pedal_bus(b"") == 2
  assert parse_configured_pedal_bus(2) == 2


def test_parse_pedal_calibration_args_bus_optional():
  confirmed, bus = parse_pedal_calibration_args(["--confirm"])
  assert confirmed is True
  assert bus is None
  confirmed, bus = parse_pedal_calibration_args(["--confirm", "--bus", "0"])
  assert confirmed is True
  assert bus == 0
  confirmed, bus = parse_pedal_calibration_args(["--bus", "2"])
  assert confirmed is False
  assert bus == 2


def test_admission_rejects_nan_negative_and_unknown_zero():
  assert pedal_calibration_entry_reason(
    offroad=False, engaged=False, car_state_fresh=True, v_ego=float("nan"),
  ) == "vehicle must be stationary"
  assert pedal_calibration_entry_reason(
    offroad=False, engaged=False, car_state_fresh=True, v_ego=-0.1,
  ) == "vehicle must be stationary"
  assert pedal_calibration_entry_reason(
    offroad=False, engaged=False, car_state_fresh=True, v_ego=0.51,
  ) == "vehicle must be stationary"
  assert pedal_calibration_entry_reason(
    offroad=False, engaged=False, car_state_fresh=True, v_ego=0.0,
  ) is None
  assert pedal_calibration_entry_reason(
    offroad=False, engaged=False, car_state_fresh=True, v_ego=STANDSTILL_SPEED_MS,
  ) is None
  assert pedal_calibration_entry_reason(
    offroad=True, engaged=False, car_state_fresh=False, v_ego=0.0,
  ) == "waiting for stationary vehicle state"
  assert pedal_calibration_entry_reason(
    offroad=False, engaged=True, car_state_fresh=True, v_ego=0.0,
  ) == "disengage before calibrating the pedal"


def test_admission_from_submaster_does_not_normalize_missing_can_to_zero():
  sm = FakeSubMaster(started=True, engaged=False, car_fresh=False, can_fresh=False)
  assert admission_from_submaster(sm) == "waiting for stationary vehicle state"

  moving = FakeSubMaster(started=True, engaged=False, car_fresh=True, v_ego=float("nan"))
  assert admission_from_submaster(moving) == "vehicle must be stationary"

  parked = FakeSubMaster(started=True, engaged=False, car_fresh=True, v_ego=0.2)
  assert admission_from_submaster(parked) is None


def test_admission_uses_bus0_esp_b_bytes_and_ignores_other_bus():
  moving = bytes(5) + (1000).to_bytes(2, "big") + bytes(1)  # 10.00 kph
  parked = bytes(5) + (10).to_bytes(2, "big") + bytes(1)  # 0.10 kph
  wrong_bus = [SimpleNamespace(address=ESP_B_ID, dat=parked, src=2)]
  sm_wrong = FakeSubMaster(started=True, engaged=False, car_fresh=False,
                           can_msgs=wrong_bus, can_fresh=True)
  assert admission_from_submaster(sm_wrong) == "waiting for stationary vehicle state"

  sm_moving = FakeSubMaster(
    started=True, engaged=False, car_fresh=False,
    can_msgs=[SimpleNamespace(address=ESP_B_ID, dat=moving, src=0)], can_fresh=True,
  )
  assert admission_from_submaster(sm_moving) == "vehicle must be stationary"

  sm_parked = FakeSubMaster(
    started=True, engaged=False, car_fresh=False,
    can_msgs=[SimpleNamespace(address=ESP_B_ID, dat=parked, src=0)], can_fresh=True,
  )
  assert admission_from_submaster(sm_parked) is None


def test_build_pedal_command_packs_protocol_bytes():
  can = TeslaCANPreAP(None)
  addr, dat, bus = build_pedal_command(0.0, enable=1, bus=2, can=can)
  assert addr == GAS_COMMAND_ID
  assert bus == 2
  assert len(dat) == 6
  assert dat[4] & 0x70 == 0
  assert dat[4] & 0x80
  assert (dat[4] & 0x0F) == 0
  expected = TeslaCANPreAP.pedal_checksum(GAS_COMMAND_ID, dat[:5] + b"\x00")
  assert dat[5] == expected

  addr0, dat0, bus0 = build_pedal_command(0.0, enable=0, bus=0, can=can)
  assert bus0 == 0
  assert dat0[4] & 0x80 == 0
  assert dat0[0] == 0 and dat0[1] == 0


def test_esp_b_speed_uses_bytes_5_6():
  dat = bytes([0, 0, 0, 0, 0, 0x00, 0xB4, 0])  # 180 centi-kph = 0.5 m/s
  assert esp_b_speed_ms(dat) == pytest.approx(0.5)
  assert esp_b_speed_ms(bytes(6)) is None


def test_process_can_filters_source_bus_and_invalid_brake():
  session = RecvSession([
    (ESP_B_ID, bytes(5) + (180).to_bytes(2, "big"), 0),
    (ESP_B_ID, bytes(5) + (5000).to_bytes(2, "big"), 2),
    (BRAKE_MESSAGE_ID, bytes([(3 << 2)]), 0),
    (GTW_STATUS_ID, bytes([1]), 0),
    (DI_TORQUE2_ID, bytes([0, GEAR_NEUTRAL]), 0),
    (DI_TORQUE1_ID, bytes(7), 0),
  ])
  cal = PedalCalibrator(RecordingParams(), session, bus=2)
  cal.process_can()
  assert cal.speed_ms == pytest.approx(0.5)
  assert cal.brake_valid is False
  assert cal.brake_pressed is False
  assert cal.car_on is True
  assert cal.gear_neutral is True
  assert cal.check_safety() is False

  session._msgs = [(BRAKE_MESSAGE_ID, bytes([(BRAKE_NOT_APPLIED << 2)]), 0)]
  cal.process_can()
  assert cal.brake_valid is True
  assert cal.brake_pressed is False

  session._msgs = [(BRAKE_MESSAGE_ID, bytes([(BRAKE_APPLIED << 2)]), 0)]
  cal.process_can()
  assert cal.brake_valid is True
  assert cal.brake_pressed is True


def test_stale_speed_and_wrong_pedal_feedback_bus_do_not_open_window():
  session = RecvSession([
    (0x552, bytes([0, 0, 0, 0, 0, 0]), 0),
  ])
  cal = PedalCalibrator(RecordingParams(), session, bus=2)
  cal.process_can()
  assert cal.last_pedal_seen_ms == 0
  assert cal.check_safety() is False


def test_enable_zero_cleanup_sends_without_vehicle_window():
  session = RecvSession()
  can = TeslaCANPreAP(None)
  from scripts.nap.calibrate_pedal import send_safe_release
  send_safe_release(session, bus=2, can=can)
  assert len(session.sent) == 1
  addr, dat, bus = session.sent[0]
  assert addr == GAS_COMMAND_ID
  assert bus == 2
  assert dat[4] & 0x80 == 0


def test_persist_done_is_commit_bit_and_failure_stays_invalid():
  params = RecordingParams()
  persist_calibration(params, min_v=1.0, max_v=50.0, factor=2.0, zero=4.0, bus=0)
  assert params.store["NAPPedalCalibDone"] is True
  assert params.store["NAPPedalEnabled"] is True
  assert params.store["NAPPedalCanBus"] == 0
  assert params.puts[0] == "NAPPedalCalibDone"
  assert params.puts[-1] == "NAPPedalCalibDone"

  failing = RecordingParams()
  failing.fail_on = "NAPPedalCalibFactor"
  with pytest.raises(OSError):
    persist_calibration(failing, min_v=1.0, max_v=50.0, factor=2.0, zero=4.0, bus=2)
  assert failing.store.get("NAPPedalCalibDone") is False
  assert failing.store.get("NAPPedalEnabled") is not True

  with pytest.raises(PedalCalibrationError, match="default calibration"):
    persist_calibration(RecordingParams(), min_v=-3.0, max_v=99.6, factor=1.0, zero=0.0)


def test_session_programs_64_and_96_and_rejects_mixed_flags():
  handle = UsbHandle(health=_health_bytes(safety_mode=SAFETY_TESLA_PREAP, safety_param=64))
  panda = _usb_panda(handle)
  session = PedalCalibrationSession(panda=panda)
  session.set_pedal_calibration_session(2)
  assert panda.safety_calls == [(SAFETY_SILENT, 0), (SAFETY_TESLA_PREAP, CALIBRATION_PARAM_BUS2)]
  assert handle.writes[-1][0] == HEARTBEAT_REQUEST
  assert handle.writes[-1][1:4] == (0, 0, HEARTBEAT_TIMEOUT_MS)

  session.set_pedal_calibration_session(0)
  assert panda.safety_calls[-1] == (SAFETY_TESLA_PREAP, CALIBRATION_PARAM_BUS0)
  with pytest.raises(TransportError, match="invalid pedal bus"):
    session.set_pedal_calibration_session(1)
  with pytest.raises(TransportError, match="bypass"):
    session._set_safety_mode(SAFETY_ALLOUTPUT)
  with pytest.raises(TransportError, match="longitudinal"):
    session._set_safety_mode(SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_ENABLE_PEDAL)
  with pytest.raises(TransportError, match="64 or 96"):
    session._set_safety_mode(SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION | 2)


def test_session_rejects_missing_usb_handle():
  class NoHandle:
    pass

  session = PedalCalibrationSession(panda=NoHandle())
  with pytest.raises(TransportError, match="usb handle missing"):
    session.send_heartbeat(False)


def test_watchdog_protocol_survives_past_five_seconds_then_fails_closed():
  clock = {"t": 0.0}

  def now():
    return clock["t"]

  class WatchdogHandle(UsbHandle):
    def controlRead(self, request_type, request, value, index, length, timeout=0):
      self.reads.append((request, length, timeout))
      lost = 0
      mode = SAFETY_TESLA_PREAP
      param = CALIBRATION_PARAM_BUS2
      if self.last_heartbeat_t is None or (now() - self.last_heartbeat_t) > 5.0:
        lost = 1
        mode = SAFETY_SILENT
        param = 0
      return _health_bytes(safety_mode=mode, safety_param=param, heartbeat_lost=lost)

  handle = WatchdogHandle(clock=now)
  panda = _usb_panda(handle)
  session = PedalCalibrationSession(panda=panda)
  session.set_pedal_calibration_session(2)

  for _ in range(7):
    clock["t"] += 1.0
    session.send_heartbeat(False)
    session.require_calibration_health(SAFETY_TESLA_PREAP, CALIBRATION_PARAM_BUS2)

  heartbeats = [w for w in handle.writes if w[0] == HEARTBEAT_REQUEST]
  assert heartbeats[-1][4] >= 5.0
  assert all(w[1] == 0 and w[2] == 0 for w in heartbeats)

  clock["t"] += 5.1
  with pytest.raises(TransportError, match="watchdog heartbeat lost|unexpected SILENT"):
    session.require_calibration_health(SAFETY_TESLA_PREAP, CALIBRATION_PARAM_BUS2)


def test_wait_for_manager_release_requires_stopped_processes():
  running = SimpleNamespace(name="pandad", running=True, shouldBeRunning=True)
  card = SimpleNamespace(name="card", running=False, shouldBeRunning=False)
  controlsd = SimpleNamespace(name="controlsd", running=False, shouldBeRunning=False)
  sm = FakeSubMaster(processes=[running, card, controlsd])
  with pytest.raises(TransportError, match="timed out"):
    wait_for_manager_release(sm, timeout_s=0.0, sleep=lambda _s: None, now=lambda: 1.0)

  stopped = [
    SimpleNamespace(name="pandad", running=False, shouldBeRunning=False),
    card, controlsd,
  ]
  sm_ok = FakeSubMaster(processes=stopped)
  wait_for_manager_release(sm_ok, timeout_s=1.0, sleep=lambda _s: None, now=lambda: 0.0)
  assert processes_released(sm_ok["managerState"]) is True


def test_calibration_safety_param_matches_native_island():
  assert calibration_safety_param(2) == 64
  assert calibration_safety_param(0) == 96
  assert SAFETY_TESLA_PREAP == 37
