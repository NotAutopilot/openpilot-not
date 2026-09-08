from __future__ import annotations

import pytest


from scripts.nap.pedal_session import CALIBRATE_PEDAL_MODULE, WAITING_SIGNALS
from scripts.nap.pedal_wizard import (
  MICI_HEIGHT,
  MICI_WIDTH,
  PedalWizardController,
  ScriptState,
  TICI_HEIGHT,
  TICI_WIDTH,
  pedal_safe_exit_decision,
  spawn_pedal_calibrator,
  wizard_layout,
)
from scripts.nap.run_script import main as run_script_main


class FakeParams:
  def __init__(self, offroad=True, bus=2):
    self.store = {"IsOffroad": offroad, "NAPPedalCanBus": bus, "NAPScriptRunning": False}

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def put_bool(self, key, value, block=True):
    self.store[key] = bool(value)

  def get(self, key, return_default=False):
    return self.store.get(key)

  def put(self, key, value, block=True):
    self.store[key] = value


class FakeSubMaster:
  def __init__(self, *, block_reason=None, explode=False):
    self.block_reason = block_reason
    self.explode = explode

  def update(self, _timeout=0):
    if self.explode:
      raise RuntimeError("submaster unavailable")


class FakeProcess:
  def __init__(self, running=True):
    self._running = running
    self._code = None
    self.signals = []
    self.stdout = None

  def poll(self):
    return None if self._running else self._code

  def wait(self, timeout=None):
    if self._running:
      raise Exception("still running")
    return self._code

  def send_signal(self, sig):
    self.signals.append(sig)

  def terminate(self):
    self.signals.append("term")

  def kill(self):
    self.signals.append("kill")

  def finish(self, code):
    self._running = False
    self._code = code


def _controller(*, sm=None, spawn=None, stop=None, ignition=True, params=None,
                hosted=True, admission=None, threads=None):
  popped = []
  closed = []
  rebooted = []
  spawn_calls = []
  pending = []

  def default_spawn(module_name, params_obj=None, *, bus=None):
    params_obj.put_bool("NAPScriptRunning", True, block=True)
    proc = FakeProcess()
    spawn_calls.append({"module": module_name, "bus": bus, "proc": proc})
    return proc

  def default_admission(_sm):
    return None

  def start_thread(fn):
    pending.append(fn)

  ctrl = PedalWizardController(
    title="Pedal Calibration",
    module=CALIBRATE_PEDAL_MODULE,
    instructions="Keep the car ON for calibration.",
    params=params or FakeParams(),
    hosted=hosted,
    admission_fn=admission or default_admission,
    spawn_fn=spawn or default_spawn,
    stop_child_fn=stop or (lambda proc: proc.finish(1) or True),
    sample_ignition_fn=(ignition if callable(ignition) else lambda: ignition),
    clear_fn=lambda p: p.put_bool("NAPScriptRunning", False, block=True),
    submaster_factory=lambda: sm,
    pop_fn=lambda: popped.append("pop"),
    close_fn=lambda: closed.append("close"),
    reboot_fn=lambda: rebooted.append("reboot"),
    start_thread=start_thread if threads is None else threads,
  )
  ctrl._spawn_calls = spawn_calls
  ctrl._popped = popped
  ctrl._closed = closed
  ctrl._rebooted = rebooted
  ctrl._pending = pending
  return ctrl


def test_start_without_submaster_refuses_and_does_not_mark():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=None, params=params)
  assert ctrl.start() == WAITING_SIGNALS
  assert ctrl.state == ScriptState.READY
  assert params.get_bool("NAPScriptRunning") is False
  assert ctrl._spawn_calls == []
  assert ctrl.handoff_requested is False


def test_ready_status_banner_shows_admission_not_instruction_tail():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params)
  assert ctrl.ready_status_banner == "Ready: stationary and disengaged. Start begins calibration."
  assert ctrl.start_enabled is True


def test_start_admission_reason_does_not_mark_or_spawn():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params,
                     admission=lambda _sm: "vehicle must be stationary")
  assert ctrl.start() == "vehicle must be stationary"
  assert ctrl.state == ScriptState.READY
  assert params.get_bool("NAPScriptRunning") is False
  assert ctrl._spawn_calls == []


def test_start_admits_then_spawns_with_local_bus_before_flag_side_effects():
  params = FakeParams(offroad=False, bus=2)
  order = []

  def admission(_sm):
    order.append("admission")
    assert params.get_bool("NAPScriptRunning") is False
    return None

  def spawn(module_name, params_obj=None, *, bus=None):
    order.append(("spawn", bus))
    params_obj.put_bool("NAPScriptRunning", True, block=True)
    return FakeProcess()

  ctrl = _controller(sm=FakeSubMaster(), params=params, admission=admission, spawn=spawn)
  ctrl.select_bus(0)
  assert ctrl.start() is None
  assert order == ["admission", ("spawn", 0)]
  assert params.get_bool("NAPScriptRunning") is True
  assert ctrl.action_label == "Cancel"
  assert ctrl.action_enabled is True


def test_ready_exit_hosted_pops_without_reboot_or_clear():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params)
  assert ctrl.request_exit() == "exited"
  assert ctrl._popped == ["pop"]
  assert ctrl._rebooted == []
  assert params.get_bool("NAPScriptRunning") is False


def test_cancel_stops_child_and_does_not_clear_after_handoff():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params)
  assert ctrl.start() is None
  assert params.get_bool("NAPScriptRunning") is True
  ctrl.cancel()
  proc = ctrl._spawn_calls[0]["proc"]
  proc.finish(1)
  for fn in ctrl._pending:
    fn()
  ctrl.pump()
  assert params.get_bool("NAPScriptRunning") is True
  assert ctrl.state == ScriptState.ERROR
  assert ctrl.probe_inflight is False
  assert ctrl.action_label == "Check ignition / Exit"
  assert ctrl._popped == []


def test_exit_blocked_while_probe_inflight():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params)
  ctrl.start()
  ctrl.cancel()
  ctrl._spawn_calls[0]["proc"].finish(1)
  assert ctrl.request_exit() == "probing"
  assert ctrl.probe_inflight is True
  assert ctrl.request_exit() == "blocked"
  assert params.get_bool("NAPScriptRunning") is True
  assert ctrl._popped == []


def test_safe_exit_after_ignition_off_clears_and_pops():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params, ignition=False)
  ctrl.start()
  ctrl.cancel()
  ctrl._spawn_calls[0]["proc"].finish(1)
  assert ctrl.request_exit() == "probing"
  for fn in ctrl._pending:
    fn()
  ctrl.pump()
  assert pedal_safe_exit_decision(False) == "clear"
  assert params.get_bool("NAPScriptRunning") is False
  assert ctrl._popped == ["pop"]
  assert ctrl._rebooted == []


def test_restart_device_is_explicit_and_never_automatic():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params, ignition=True)
  ctrl.start()
  ctrl.cancel()
  ctrl._spawn_calls[0]["proc"].finish(0)
  ctrl.request_exit()
  for fn in ctrl._pending:
    fn()
  ctrl.pump()
  assert ctrl.restart_visible is True
  assert ctrl.restart_device() == "reboot"
  assert ctrl._rebooted == ["reboot"]
  assert params.get_bool("NAPScriptRunning") is True
  assert ctrl._popped == []


def test_exit_after_car_off_following_on_probe_does_not_reboot():
  params = FakeParams(offroad=False)
  ignition = {"on": True}

  ctrl = _controller(sm=FakeSubMaster(), params=params, ignition=lambda: ignition["on"])
  ctrl.start()
  ctrl.cancel()
  ctrl._spawn_calls[0]["proc"].finish(1)
  ctrl.request_exit()
  for fn in list(ctrl._pending):
    fn()
  ctrl._pending.clear()
  ctrl.pump()
  assert params.get_bool("NAPScriptRunning") is True
  assert ctrl._popped == []
  assert ctrl.restart_required is True
  assert ctrl.action_label == "Check ignition / Exit"

  ignition["on"] = False
  ctrl.request_exit()
  for fn in ctrl._pending:
    fn()
  ctrl.pump()
  assert params.get_bool("NAPScriptRunning") is False
  assert ctrl._popped == ["pop"]
  assert ctrl._rebooted == []


def test_child_fail_after_handoff_does_not_clear():
  params = FakeParams(offroad=False)

  def boom(module_name, params_obj=None, *, bus=None):
    params_obj.put_bool("NAPScriptRunning", True, block=True)
    raise OSError("exec failed")

  ctrl = _controller(sm=FakeSubMaster(), params=params, spawn=boom)
  assert ctrl.start() is None
  assert ctrl.state == ScriptState.ERROR
  assert ctrl.restart_required is True
  assert params.get_bool("NAPScriptRunning") is True
  assert ctrl._popped == []


def test_cancel_ignores_later_child_exit_code():
  params = FakeParams(offroad=False)
  ctrl = _controller(sm=FakeSubMaster(), params=params)
  ctrl.start()
  ctrl.cancel()
  ctrl._events.put(("done", 1))
  ctrl.pump()
  assert ctrl.state == ScriptState.ERROR
  assert "[Script exited with code 1]" not in ctrl.output_lines


def test_spawn_passes_confirm_and_bus(monkeypatch):
  captured = {}
  params = FakeParams()

  class Proc:
    stdout = None

    def poll(self):
      return 0

  def fake_popen(cmd, **kwargs):
    captured["cmd"] = cmd
    return Proc()

  monkeypatch.setattr("scripts.nap.pedal_wizard.subprocess.Popen", fake_popen)
  spawn_pedal_calibrator(CALIBRATE_PEDAL_MODULE, params, bus=0)
  assert "--confirm" in captured["cmd"]
  assert captured["cmd"][-2:] == ["--bus", "0"]
  assert params.get_bool("NAPScriptRunning") is True


def test_standalone_runner_refuses_pedal_without_tmux(monkeypatch):
  ran = []
  monkeypatch.setattr("scripts.nap.run_script.sys.argv", [
    "run_script.py", "Pedal Calibration", "scripts.nap.calibrate_pedal", "hi",
  ])
  monkeypatch.setattr("scripts.nap.run_script.subprocess.run", lambda *a, **k: ran.append("tmux"))
  with pytest.raises(SystemExit) as exc:
    run_script_main()
  assert exc.value.code == 1
  assert ran == []


def _assert_inside(spec, width, height):
  assert spec["x"] >= 0
  assert spec["y"] >= 0
  assert spec["x"] + spec["width"] <= width
  assert spec["y"] + spec["height"] <= height
  assert spec["width"] > 0 and spec["height"] > 0


def _assert_no_overlap(a, b):
  separate = (
    a["x"] + a["width"] <= b["x"]
    or b["x"] + b["width"] <= a["x"]
    or a["y"] + a["height"] <= b["y"]
    or b["y"] + b["height"] <= a["y"]
  )
  assert separate


def test_mici_ready_buttons_fit_536x240():
  layout = wizard_layout(MICI_WIDTH, MICI_HEIGHT, ready=True, show_start=True, banner=True)
  names = ("bus0", "bus2", "start", "action")
  assert set(layout["buttons"]) == set(names)
  for spec in layout["buttons"].values():
    _assert_inside(spec, MICI_WIDTH, MICI_HEIGHT)
  items = list(layout["buttons"].values())
  for i, a in enumerate(items):
    for b in items[i + 1:]:
      _assert_no_overlap(a, b)
  _assert_inside(layout["body"], MICI_WIDTH, MICI_HEIGHT)
  _assert_inside(layout["banner"], MICI_WIDTH, MICI_HEIGHT)


def test_mici_final_actions_stay_on_canvas():
  layout = wizard_layout(MICI_WIDTH, MICI_HEIGHT, ready=False, show_start=True, banner=True)
  assert "bus0" not in layout["buttons"]
  for spec in layout["buttons"].values():
    _assert_inside(spec, MICI_WIDTH, MICI_HEIGHT)
  _assert_no_overlap(layout["buttons"]["start"], layout["buttons"]["action"])


def test_tici_ready_buttons_stay_on_canvas():
  layout = wizard_layout(TICI_WIDTH, TICI_HEIGHT, ready=True, show_start=True, banner=True)
  for spec in layout["buttons"].values():
    _assert_inside(spec, TICI_WIDTH, TICI_HEIGHT)
  items = list(layout["buttons"].values())
  for i, a in enumerate(items):
    for b in items[i + 1:]:
      _assert_no_overlap(a, b)

