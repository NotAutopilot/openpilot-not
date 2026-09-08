"""Pedal-calibration Panda session and Start admission.

Not a generic diagnostic transport. Calibration is the only caller: teslaPreap
safetyParam 64 (bus 2) or 96 (bus 0), heartbeat engaged=False, bounded USB.
"""
from __future__ import annotations

import signal
import subprocess
import time
from pathlib import Path

from opendbc.car import structs
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID

PREAP_FLAG_ENABLE_PEDAL = 1
PREAP_FLAG_PEDAL_BUS_ZERO = 1 << 5
PREAP_FLAG_PEDAL_CALIBRATION = 1 << 6
CALIBRATION_PARAM_BUS2 = PREAP_FLAG_PEDAL_CALIBRATION
CALIBRATION_PARAM_BUS0 = PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_PEDAL_BUS_ZERO

SAFETY_SILENT = int(structs.CarParams.SafetyModel.silent)
SAFETY_ALLOUTPUT = int(structs.CarParams.SafetyModel.allOutput)
SAFETY_TESLA_PREAP = int(structs.CarParams.SafetyModel.teslaPreap)

PEDAL_CONNECT_RETRIES = 8
PEDAL_CONNECT_DELAY = 0.25
CAN_RECV_TIMEOUT_MS = 100
CAN_RECV_RETRIES = 3
HEALTH_TIMEOUT_MS = 200
HEARTBEAT_TIMEOUT_MS = 200
HEARTBEAT_REQUEST = 0xF3
HEALTH_REQUEST = 0xD2

STANDSTILL_SPEED_MS = 0.5
WAITING_SIGNALS = "waiting for vehicle signals"
ESP_B_ID = 0x155
VEHICLE_CAN_BUS = 0
ESP_SPEED_SCALE = 0.01

TOOL_STOPPED_PROCESSES = ("pandad", "card", "controlsd")
OWNERSHIP_TIMEOUT_S = 8.0

CALIBRATE_PEDAL_MODULE = "scripts.nap.calibrate_pedal"


class TransportError(Exception):
  """Fail-closed USB or safety-mode failure."""


def calibration_safety_param(bus: int) -> int:
  if bus == 2:
    return CALIBRATION_PARAM_BUS2
  if bus == 0:
    return CALIBRATION_PARAM_BUS0
  raise TransportError("invalid pedal bus")


def parse_configured_pedal_bus(value) -> int:
  """Preserve configured bus 0. Absent or empty defaults to 2."""
  if value is None or value == "" or value == b"":
    return 2
  if isinstance(value, (bytes, bytearray)):
    value = value.decode()
  return int(value)


def require_runtime_path(module_file: str | Path | None = None) -> Path:
  from openpilot.common.basedir import BASEDIR
  tools_dir = Path(__file__).resolve().parent
  base = Path(BASEDIR).resolve()
  if base not in tools_dir.parents and tools_dir != base:
    raise TransportError(f"pedal calibration must run from BASEDIR ({base}), not {tools_dir}")
  if module_file is not None:
    path = Path(module_file).resolve()
    if path.parent != tools_dir and tools_dir not in path.parents:
      raise TransportError(f"pedal calibration must run from {tools_dir}")
    return path
  return tools_dir


def mark_script_running(params) -> None:
  params.put_bool("NAPScriptRunning", True, block=True)


def clear_script_running(params) -> None:
  params.put_bool("NAPScriptRunning", False, block=True)


def processes_released(manager_state, names=TOOL_STOPPED_PROCESSES) -> bool:
  by_name = {p.name: p for p in manager_state.processes}
  for name in names:
    proc = by_name.get(name)
    if proc is None:
      return False
    if proc.running:
      return False
    if getattr(proc, "shouldBeRunning", False):
      return False
  return True


def wait_for_manager_release(sm=None, *, timeout_s=OWNERSHIP_TIMEOUT_S,
                             sleep=time.sleep, now=time.monotonic) -> None:
  """Wait until managerState shows pandad/card/controlsd not alive.

  NAPScriptRunning must already be set. Does not invent a manager ack param.
  """
  from cereal import messaging
  sm = sm or messaging.SubMaster(["managerState"])
  deadline = now() + timeout_s
  while now() < deadline:
    sm.update(100)
    seen = getattr(sm, "seen", {}).get("managerState")
    alive = getattr(sm, "alive", {}).get("managerState", True)
    valid = getattr(sm, "valid", {}).get("managerState", True)
    if seen and alive and valid and processes_released(sm["managerState"]):
      return
    sleep(0.05)
  raise TransportError("timed out waiting for driving processes to stop")


def stop_child(process: subprocess.Popen) -> bool:
  """SIGINT (5s) → SIGTERM (2s) → SIGKILL (2s). True if the child is gone."""
  if process.poll() is not None:
    return True
  process.send_signal(signal.SIGINT)
  try:
    process.wait(timeout=5)
    return True
  except subprocess.TimeoutExpired:
    process.terminate()
    try:
      process.wait(timeout=2)
      return True
    except subprocess.TimeoutExpired:
      process.kill()
      try:
        process.wait(timeout=2)
        return True
      except subprocess.TimeoutExpired:
        return False


def pedal_calibration_entry_reason(*, offroad: bool, engaged: bool,
                                   car_state_fresh: bool, v_ego: float) -> str | None:
  """None if wizard entry / Start is allowed.

  Unknown or non-finite motion is never permission. Offroad is not a bypass.
  """
  del offroad
  if engaged:
    return "disengage before calibrating the pedal"
  if not car_state_fresh:
    return "waiting for stationary vehicle state"
  if not 0.0 <= v_ego <= STANDSTILL_SPEED_MS:
    return "vehicle must be stationary"
  return None


def _service_fresh(sm, name: str) -> bool:
  seen = getattr(sm, "seen", {}).get(name, False)
  alive = getattr(sm, "alive", {}).get(name, False)
  valid = getattr(sm, "valid", {}).get(name, False)
  return bool(seen and alive and valid)


def _esp_b_speed_ms(dat: bytes) -> float | None:
  if len(dat) < 7:
    return None
  return ((dat[5] << 8) | dat[6]) * ESP_SPEED_SCALE / 3.6


def _v_ego_from_can(sm) -> float | None:
  if not _service_fresh(sm, "can"):
    return None
  try:
    messages = sm["can"]
  except Exception:
    return None
  speed = None
  for msg in messages:
    addr = int(getattr(msg, "address", getattr(msg, "addr", 0)))
    src = int(getattr(msg, "src", 0)) & 0x7F
    dat = bytes(getattr(msg, "dat", b""))
    if addr == ESP_B_ID and src == VEHICLE_CAN_BUS:
      decoded = _esp_b_speed_ms(dat)
      if decoded is not None:
        speed = decoded
  return speed


def admission_from_submaster(sm) -> str | None:
  """Return a block reason from cereal, or None if Start may proceed.

  Missing SubMaster and stale deviceState fail closed. Started+stale
  selfdriveState fails closed. Offroad still needs fresh carState or
  fresh bus-0 ESP_B. Unknown movement is not admission. Never treat a
  missing speed as 0.
  """
  if sm is None:
    return "waiting for vehicle state"
  if not _service_fresh(sm, "deviceState"):
    return "waiting for device state"

  started = bool(sm["deviceState"].started)
  engaged = False
  if started:
    if not _service_fresh(sm, "selfdriveState"):
      return "waiting for disengage state"
    engaged = bool(sm["selfdriveState"].enabled)

  car_fresh = _service_fresh(sm, "carState")
  if car_fresh:
    v_ego = float(sm["carState"].vEgo)
  else:
    v_ego = _v_ego_from_can(sm)
    car_fresh = v_ego is not None
    if v_ego is None:
      v_ego = float("nan")

  return pedal_calibration_entry_reason(
    offroad=not started, engaged=engaged, car_state_fresh=car_fresh, v_ego=v_ego,
  )


class PedalCalibrationSession:
  """Exclusive Panda access for teslaPreap pedal calibration only."""

  def __init__(self, panda=None):
    self.panda = panda
    self._mode = SAFETY_SILENT
    self._param = 0
    self._mode_set = False

  def connect(self, panda_factory=None, retries=PEDAL_CONNECT_RETRIES, delay=PEDAL_CONNECT_DELAY):
    if self.panda is not None:
      return self.panda
    factory = panda_factory
    if factory is None:
      from panda import Panda
      def factory():
        return Panda(claim=True, disable_checks=False)
    last_exc: Exception | None = None
    for attempt in range(retries):
      try:
        self.panda = factory()
        return self.panda
      except Exception as exc:
        last_exc = exc
        if attempt < retries - 1:
          time.sleep(delay)
    raise TransportError(f"panda connect failed: {last_exc}") from last_exc

  def set_silent(self) -> None:
    self._set_safety_mode(SAFETY_SILENT)

  def set_pedal_calibration_session(self, bus: int = 2) -> None:
    """teslaPreap calibration island. Never allOutput / ENABLE_PEDAL.

    Legal safetyParam is 64 (bus 2) or 96 (bus 0). teslaPreap is a car
    safety mode: heartbeat checks cannot stay disabled.
    """
    param = calibration_safety_param(bus)
    self.set_silent()
    self._set_safety_mode(SAFETY_TESLA_PREAP, param=param)
    self.send_heartbeat(False)

  def _usb_handle(self):
    if self.panda is None:
      raise TransportError("panda not connected")
    handle = getattr(self.panda, "_handle", None)
    buf = getattr(self.panda, "can_rx_overflow_buffer", None)
    has_usb = (
      handle is not None
      and callable(getattr(handle, "controlWrite", None))
      and callable(getattr(handle, "controlRead", None))
      and callable(getattr(handle, "bulkRead", None))
      and isinstance(buf, (bytes, bytearray))
    )
    if not has_usb:
      raise TransportError("panda usb handle missing")
    return handle

  def send_heartbeat(self, engaged: bool = False) -> None:
    """Keep the panda watchdog alive without driving engagement.

    Native Panda.send_heartbeat defaults to engaged=True. Calibration
    encodes 0xf3 with wValue=0, wIndex=0 and a bounded USB timeout.
    """
    handle = self._usb_handle()
    del engaged
    try:
      from panda import Panda
      handle.controlWrite(
        Panda.REQUEST_OUT, HEARTBEAT_REQUEST, 0, 0, b"",
        timeout=HEARTBEAT_TIMEOUT_MS,
      )
    except TransportError:
      raise
    except Exception as exc:
      raise TransportError(f"heartbeat failed: {exc}") from exc

  def read_health(self) -> dict:
    handle = self._usb_handle()
    health_struct = getattr(type(self.panda), "HEALTH_STRUCT", None)
    if health_struct is None or not hasattr(health_struct, "size") or not hasattr(health_struct, "unpack"):
      raise TransportError("panda health failed: missing HEALTH_STRUCT")
    try:
      from panda import Panda
      dat = handle.controlRead(
        Panda.REQUEST_IN, HEALTH_REQUEST, 0, 0, health_struct.size, timeout=HEALTH_TIMEOUT_MS,
      )
      a = health_struct.unpack(dat)
      return {
        "safety_mode": a[12],
        "safety_param": a[13],
        "heartbeat_lost": a[16],
        "ignition_line": a[8],
        "ignition_can": a[9],
      }
    except TransportError:
      raise
    except Exception as exc:
      raise TransportError(f"panda health failed: {exc}") from exc

  def require_calibration_health(self, expected_mode: int, expected_param: int) -> None:
    """Abort on SILENT, watchdog loss, or mode/param mismatch. Never rearm."""
    health = self.read_health()
    if health.get("heartbeat_lost"):
      raise TransportError("watchdog heartbeat lost")
    mode = int(health.get("safety_mode", -1))
    param = int(health.get("safety_param", -1))
    if mode == SAFETY_SILENT:
      raise TransportError("unexpected SILENT safety mode")
    if mode != int(expected_mode) or param != int(expected_param):
      raise TransportError("unexpected safety mode")

  def _set_safety_mode(self, mode: int, param: int = 0) -> None:
    if mode == SAFETY_ALLOUTPUT or mode not in (SAFETY_SILENT, SAFETY_TESLA_PREAP):
      raise TransportError("panda TX bypass is not permitted")
    if mode == SAFETY_TESLA_PREAP:
      if int(param) & PREAP_FLAG_ENABLE_PEDAL:
        raise TransportError("calibration must not grant longitudinal")
      if int(param) not in (CALIBRATION_PARAM_BUS2, CALIBRATION_PARAM_BUS0):
        raise TransportError("calibration safetyParam must be exactly 64 or 96")
    elif int(param) != 0:
      raise TransportError("silent mode cannot carry safetyParam bits")
    if self.panda is None:
      raise TransportError("panda not connected")
    self.panda.set_safety_mode(mode, param)
    self._mode = mode
    self._param = int(param)
    self._mode_set = True

  def can_recv(self):
    handle = self._usb_handle()
    panda = self.panda
    if panda is None:
      raise TransportError("panda not connected")
    last_exc: Exception | None = None
    for _attempt in range(CAN_RECV_RETRIES):
      try:
        from panda import unpack_can_buffer
        dat = handle.bulkRead(1, 16384, timeout=CAN_RECV_TIMEOUT_MS)
        overflow = bytearray(panda.can_rx_overflow_buffer)
        msgs, overflow = unpack_can_buffer(overflow + dat)
        panda.can_rx_overflow_buffer = overflow
        return msgs
      except TransportError:
        raise
      except Exception as exc:
        last_exc = exc
    raise TransportError(f"can_recv failed: {last_exc}") from last_exc

  def can_send(self, addr: int, dat: bytes, bus: int) -> None:
    if self.panda is None:
      raise TransportError("panda not connected")
    if self._mode == SAFETY_ALLOUTPUT:
      raise TransportError("panda TX bypass is not permitted")
    if int(addr) == GAS_COMMAND_ID and self._mode != SAFETY_TESLA_PREAP:
      raise TransportError("pedal frames require teslaPreap calibration safety")
    try:
      self.panda.can_send(addr, dat, bus)
    except Exception as exc:
      raise TransportError(f"can_send failed: {exc}") from exc

  def close(self) -> None:
    if self.panda is None:
      return
    if self._mode_set:
      try:
        self.set_silent()
      except Exception:
        pass
    try:
      self.panda.close()
    except Exception:
      pass
    self.panda = None
    self._mode_set = False
    self._param = 0


def ignition_from_health(health: dict) -> bool | None:
  line = health.get("ignition_line")
  can = health.get("ignition_can")
  if line is None and can is None:
    return None
  return bool(line or can)


def sample_panda_ignition(session_factory=None) -> bool | None:
  """Fresh ignition after the calibrator has released USB. None if unknown.

  Always closes the probe. Does not program teslaPreap.
  """
  factory = session_factory or PedalCalibrationSession
  session = factory()
  try:
    session.connect(retries=3, delay=0.2)
    return ignition_from_health(session.read_health())
  except Exception:
    return None
  finally:
    try:
      session.close()
    except Exception:
      pass
