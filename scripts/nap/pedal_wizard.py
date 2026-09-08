"""In-process pedal calibration wizard.

Hosted in the existing UI via gui_app.push_widget. Never kills tmux or
opens a second DRM window. Other NAP scripts keep the standalone runner.
"""
from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
from pathlib import Path
from typing import TYPE_CHECKING

from scripts.nap.pedal_session import (
  CALIBRATE_PEDAL_MODULE,
  WAITING_SIGNALS,
  admission_from_submaster,
  clear_script_running,
  mark_script_running,
  parse_configured_pedal_bus,
  sample_panda_ignition,
  stop_child,
)

if TYPE_CHECKING:
  from openpilot.common.params import Params


def waiting_status_text(reason: str) -> str:
  text = (reason or "").strip()
  if not text:
    return ""
  if text.lower().startswith("waiting"):
    return text[0].upper() + text[1:]
  return f"Waiting: {text}"


def _pedal_submaster():
  from cereal import messaging
  return messaging.SubMaster(
    ["deviceState", "pandaStates", "carState", "selfdriveState", "managerState", "can"]
  )


def pedal_ready_lines(sm, params: Params, selected_bus: int | None = None) -> list[str]:
  sm.update(0)
  bus = parse_configured_pedal_bus(selected_bus if selected_bus is not None else params.get("NAPPedalCanBus"))
  lines = [
    f"Pedal CAN bus: {bus}.",
    "Confirm the comma pedal connector is seated. This step talks to the pedal on the selected bus.",
    "First calibration does not require the interceptor toggle.",
  ]
  reason = admission_from_submaster(sm)
  if reason:
    lines.append(waiting_status_text(reason))
  elif params.get_bool("IsOffroad"):
    lines.append("Ready: parked. Ignition on, Neutral, hold brake, then Start.")
  else:
    lines.append("Ready: stationary and disengaged. Start begins calibration.")
  return lines


def pedal_safe_exit_decision(ignition: bool | None) -> str:
  """After the child has released USB: clear only on observed ignition off."""
  if ignition is False:
    return "clear"
  return "restart"


def follow_scroll_offset(line_count: int, line_height: float, bounds_height: float) -> float:
  overflow = line_count * line_height - bounds_height
  return -overflow if overflow > 0 else 0.0

MICI_WIDTH = 536
MICI_HEIGHT = 240
TICI_WIDTH = 2160
TICI_HEIGHT = 1080


def is_compact_canvas(width: float, height: float) -> bool:
  """MICI 536x240 and other small hosted canvases. Do not use desktop scale."""
  return float(width) <= 800 and float(height) <= 500


def wizard_layout(width: float, height: float, *, ready: bool, show_start: bool,
                  banner: bool) -> dict:
  """Pixel layout for the pedal wizard. Tested at 536x240 and 2160x1080."""
  compact = is_compact_canvas(width, height)
  margin = 6 if compact else 50
  title_font_size = 16 if compact else 70
  text_font_size = 13 if compact else 45
  output_font_size = 12 if compact else 35
  line_height = 15 if compact else 45
  button_font_size = 12 if compact else 45
  button_height = 44 if compact else 110
  button_spacing = 6 if compact else 30
  bus_width = 52 if compact else 160
  action_width = 128 if compact else 350
  title_gap = 4 if compact else margin
  title_y = 2 if compact else margin
  button_y = height - margin - button_height
  banner_h = (line_height + 4) if banner else 0
  body_y = title_y + title_font_size + title_gap
  body_bottom = button_y - margin - banner_h
  body_height = max(0.0, body_bottom - body_y)
  body = {
    "x": margin,
    "y": body_y,
    "width": max(0.0, width - margin * 2),
    "height": body_height,
  }
  x_right = width - margin - action_width
  buttons = {
    "action": {"x": x_right, "y": button_y, "width": action_width, "height": button_height},
  }
  start_x = x_right
  if show_start:
    start_x = x_right - button_spacing - action_width
    buttons["start"] = {
      "x": start_x, "y": button_y, "width": action_width, "height": button_height,
    }
  if ready:
    bus2_x = start_x - button_spacing - bus_width
    bus0_x = bus2_x - button_spacing - bus_width
    buttons["bus0"] = {"x": bus0_x, "y": button_y, "width": bus_width, "height": button_height}
    buttons["bus2"] = {"x": bus2_x, "y": button_y, "width": bus_width, "height": button_height}
  banner_rect = None
  if banner:
    banner_rect = {
      "x": margin,
      "y": button_y - banner_h,
      "width": body["width"],
      "height": banner_h,
    }
  return {
    "compact": compact,
    "margin": margin,
    "title_font_size": title_font_size,
    "text_font_size": text_font_size,
    "output_font_size": output_font_size,
    "line_height": line_height,
    "button_font_size": button_font_size,
    "title_y": title_y,
    "body": body,
    "banner": banner_rect,
    "buttons": buttons,
  }



class ScriptState:
  READY = 0
  RUNNING = 1
  COMPLETED = 2
  ERROR = 3


def spawn_pedal_calibrator(module: str, params: Params | None = None, *, bus: int | None = None) -> subprocess.Popen:
  from openpilot.common.basedir import BASEDIR
  from openpilot.common.params import Params
  if module != CALIBRATE_PEDAL_MODULE:
    raise ValueError(f"unapproved pedal module: {module}")
  params = params or Params()
  if bus is not None and bus not in (0, 2):
    raise ValueError("invalid pedal bus")
  env = os.environ.copy()
  env["PYTHONPATH"] = env.get("PYTHONPATH", BASEDIR)
  cmd = [sys.executable, "-m", module, "--confirm"]
  if bus is not None:
    cmd.extend(["--bus", str(int(bus))])
  mark_script_running(params)
  return subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    cwd=str(Path(BASEDIR)),
    start_new_session=True,
    env=env,
    text=True,
    bufsize=1,
  )


def _default_reboot() -> None:
  from openpilot.system.hardware import HARDWARE, PC
  if not PC:
    HARDWARE.reboot()


class PedalWizardController:
  """Start / Cancel / Exit / Restart for in-process pedal calibration.

  Start fail-closes without a live SubMaster. Admission runs before
  NAPScriptRunning. After that flag is set, child failure does not clear it.
  USB ignition probe runs only after the child is confirmed gone.
  """

  def __init__(
    self,
    *,
    title: str,
    module: str,
    instructions: str,
    params: Params | None = None,
    hosted: bool = True,
    admission_fn=admission_from_submaster,
    spawn_fn=spawn_pedal_calibrator,
    stop_child_fn=stop_child,
    sample_ignition_fn=sample_panda_ignition,
    mark_fn=mark_script_running,
    clear_fn=clear_script_running,
    submaster_factory=_pedal_submaster,
    pop_fn=None,
    close_fn=None,
    reboot_fn=None,
    start_thread=None,
  ):
    self.title = title
    self._module = module
    self.instructions = instructions
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self._params = params
    self.hosted = hosted
    self._admission = admission_fn
    self._spawn = spawn_fn
    self._stop_child = stop_child_fn
    self._sample_ignition = sample_ignition_fn
    self._mark = mark_fn
    self._clear = clear_fn
    self._pop_fn = pop_fn
    self._close_fn = close_fn
    self._reboot_fn = reboot_fn or _default_reboot
    self._start_thread = start_thread or (lambda fn: threading.Thread(target=fn, daemon=True).start())

    self.state = ScriptState.READY
    self.output_lines: list[str] = []
    self._events: queue.Queue = queue.Queue()
    self._process = None
    self.selected_bus = parse_configured_pedal_bus(self._params.get("NAPPedalCanBus"))
    if self.selected_bus not in (0, 2):
      self.selected_bus = 2
    self.restart_required = False
    self.probe_inflight = False
    self.probe_done = False
    self.handoff_requested = False
    self._ignore_done = False
    self._last_ignition: bool | None = None
    self._sm = None
    try:
      self._sm = submaster_factory()
    except Exception:
      self._sm = None

  @property
  def is_pedal(self) -> bool:
    return True

  @property
  def start_block_reason(self) -> str | None:
    if self.state != ScriptState.READY:
      return None
    if self._sm is None:
      return WAITING_SIGNALS
    try:
      self._sm.update(0)
      return self._admission(self._sm)
    except Exception:
      return WAITING_SIGNALS

  @property
  def start_enabled(self) -> bool:
    return self.state == ScriptState.READY and self.start_block_reason is None

  @property
  def ready_status_banner(self) -> str:
    if self.state != ScriptState.READY:
      return ""
    reason = self.start_block_reason
    if reason:
      return waiting_status_text(reason)
    if self._params.get_bool("IsOffroad"):
      return "Ready: parked. Ignition on, Neutral, hold brake, then Start."
    return "Ready: stationary and disengaged. Start begins calibration."

  @property
  def status_banner(self) -> str:
    if self.state == ScriptState.READY:
      return self.ready_status_banner
    if self.probe_inflight:
      return "Checking ignition..."
    if self.restart_visible:
      return "Turn the car off, then Check ignition / Exit."
    return ""

  @property
  def action_label(self) -> str:
    if self.probe_inflight:
      return "Exit"
    if self.state == ScriptState.RUNNING:
      return "Cancel"
    if self.state in (ScriptState.COMPLETED, ScriptState.ERROR):
      return "Check ignition / Exit"
    return "Exit"

  @property
  def action_enabled(self) -> bool:
    if self.probe_inflight:
      return False
    return True

  @property
  def restart_visible(self) -> bool:
    if not self.handoff_requested:
      return False
    if self.state not in (ScriptState.COMPLETED, ScriptState.ERROR):
      return False
    if self._process is not None and self._process.poll() is None:
      return self.restart_required
    return True

  @property
  def restart_enabled(self) -> bool:
    return self.restart_visible and not self.probe_inflight

  def ready_lines(self) -> list[str]:
    lines = self.instructions.split("\n")
    lines.append("")
    if self._sm is None:
      lines.extend([
        f"Pedal CAN bus: {self.selected_bus}.",
        "Confirm the comma pedal connector is seated. This step talks to the pedal on the selected bus.",
        "First calibration does not require the interceptor toggle.",
        waiting_status_text(WAITING_SIGNALS),
      ])
      return lines
    lines.extend(pedal_ready_lines(self._sm, self._params, selected_bus=self.selected_bus))
    return lines

  def display_lines(self) -> list[str]:
    if self.state == ScriptState.READY:
      return self.ready_lines()
    return list(self.output_lines)

  def select_bus(self, bus: int) -> None:
    if self.state != ScriptState.READY:
      return
    if bus not in (0, 2):
      return
    self.selected_bus = bus

  def start(self) -> str | None:
    if self.state != ScriptState.READY:
      return "not ready"
    if self._sm is None:
      return WAITING_SIGNALS
    try:
      self._sm.update(0)
      reason = self._admission(self._sm)
    except Exception:
      return WAITING_SIGNALS
    if reason:
      return reason
    self.state = ScriptState.RUNNING
    self.output_lines = ["Starting script...", ""]
    self.handoff_requested = True
    self._ignore_done = False
    try:
      self._process = self._spawn(self._module, self._params, bus=self.selected_bus)
      self._start_thread(self._read_output)
    except Exception as exc:
      self.output_lines.append(f"Error starting script: {exc}")
      self.state = ScriptState.ERROR
      self.restart_required = True
    return None

  def cancel(self) -> None:
    if self.state != ScriptState.RUNNING:
      return
    if self._process is not None and self._process.poll() is None:
      if not self._stop_child(self._process):
        self.output_lines.append("[ERROR] script did not exit; manager will stay paused")
        self.output_lines.append("Use Restart device to recover.")
        self.state = ScriptState.ERROR
        self.restart_required = True
        return
    self._ignore_done = True
    self.state = ScriptState.ERROR
    self.output_lines.append("[Cancelled]")
    self.output_lines.append("Turn the car off, then Check ignition / Exit. Restart device is optional.")

  def request_exit(self) -> str:
    if self.probe_inflight:
      return "blocked"
    if self.state == ScriptState.RUNNING:
      self.cancel()
      return "cancel"
    if self.state == ScriptState.READY:
      self._leave_ready()
      return "exited"
    if self._process is not None and self._process.poll() is None:
      return "blocked"
    self._begin_probe()
    return "probing"

  def restart_device(self) -> str:
    if self.probe_inflight or not self.restart_visible:
      return "blocked"
    self._reboot_fn()
    return "reboot"

  def pump(self) -> bool:
    got_new = False
    while True:
      try:
        kind, payload = self._events.get_nowait()
      except queue.Empty:
        break
      got_new = True
      if kind == "line":
        self.output_lines.append(payload)
      elif kind == "done":
        if self._ignore_done:
          continue
        code = payload
        if code == 0:
          self.output_lines.append("[Script completed successfully]")
          self.state = ScriptState.COMPLETED
        else:
          self.output_lines.append(f"[Script exited with code {code}]")
          self.state = ScriptState.ERROR
        self.output_lines.append("Turn the car off, then Check ignition / Exit. Restart device is optional.")
      elif kind == "probe":
        self.probe_inflight = False
        self.probe_done = True
        self._last_ignition = payload
        if pedal_safe_exit_decision(payload) == "clear":
          self._clear(self._params)
          self._pop_hosted_or_close()
        else:
          self.restart_required = True
          self.output_lines.append("Ignition still on. Turn the car off and Check ignition / Exit, or Restart device.")
    return got_new

  def _begin_probe(self) -> None:
    if self.probe_inflight:
      return
    if self._process is not None and self._process.poll() is None:
      return
    self.probe_inflight = True

    def run():
      ignition = None
      try:
        ignition = self._sample_ignition()
      except Exception:
        ignition = None
      self._events.put(("probe", ignition))

    self._start_thread(run)

  def _read_output(self) -> None:
    try:
      if self._process is not None and self._process.stdout:
        for line in iter(self._process.stdout.readline, ""):
          if line:
            self._events.put(("line", line.rstrip()))
      code = 1
      if self._process is not None:
        code = self._process.wait()
      self._events.put(("done", code))
    except Exception as exc:
      self._events.put(("line", f"[Error reading output: {exc}]"))
      self._events.put(("done", 1))

  def _leave_ready(self) -> None:
    if self.hosted:
      self._pop()
      return
    self._clear(self._params)
    self._close()

  def _pop_hosted_or_close(self) -> None:
    if self.hosted:
      self._pop()
    else:
      self._close()

  def _pop(self) -> None:
    if self._pop_fn is not None:
      self._pop_fn()
      return
    from openpilot.system.ui.lib.application import gui_app
    gui_app.pop_widget()

  def _close(self) -> None:
    if self._close_fn is not None:
      self._close_fn()
      return
    from openpilot.system.ui.lib.application import gui_app
    gui_app.request_close()


def make_pedal_wizard_widget(title: str, module: str, instructions: str, *,
                             hosted: bool = True, controller: PedalWizardController | None = None,
                             **ctrl_kwargs):
  """Widget wrapping PedalWizardController. Imports pyray only when constructed."""
  import pyray as rl
  from openpilot.system.ui.lib.application import gui_app, FontWeight
  from openpilot.system.ui.lib.scroll_panel import GuiScrollPanel
  from openpilot.system.ui.widgets import Widget
  from openpilot.system.ui.widgets.button import Button, ButtonStyle

  ctrl = controller or PedalWizardController(
    title=title, module=module, instructions=instructions, hosted=hosted, **ctrl_kwargs,
  )

  class PedalWizardWidget(Widget):
    def __init__(self):
      super().__init__()
      self.controller = ctrl
      self._scroll_panel = GuiScrollPanel()
      self._font = None
      self._title_font = None
      self.last_layout = None
      compact = not gui_app.big_ui()
      font = 12 if compact else 45
      pad = 2 if compact else 16
      radius = 6 if compact else 10
      self._start_button = self._child(Button("Start", click_callback=self._on_start,
                                              button_style=ButtonStyle.PRIMARY,
                                              font_size=font, text_padding=pad, border_radius=radius))
      self._action_button = self._child(Button("Exit", click_callback=self._on_action,
                                               button_style=ButtonStyle.TRANSPARENT_WHITE_BORDER,
                                               font_size=font, text_padding=pad, border_radius=radius))
      self._bus0_button = self._child(Button("Bus 0", click_callback=lambda: self.controller.select_bus(0),
                                             font_size=font, text_padding=pad, border_radius=radius))
      self._bus2_button = self._child(Button("Bus 2", click_callback=lambda: self.controller.select_bus(2),
                                             font_size=font, text_padding=pad, border_radius=radius))

    def _on_start(self):
      if self.controller.restart_visible:
        self.controller.restart_device()
      else:
        self.controller.start()

    def _on_action(self):
      self.controller.request_exit()

    def _tune(self, layout: dict) -> None:
      font = layout["button_font_size"]
      pad = 2 if layout["compact"] else 16
      radius = 6 if layout["compact"] else 10
      for btn in (self._start_button, self._action_button, self._bus0_button, self._bus2_button):
        btn._label.set_font_size(font)
        btn._label._text_padding = pad
        btn._border_radius = radius

    def _render(self, rect):
      got_new = self.controller.pump()
      ready = self.controller.state == ScriptState.READY
      show_start = ready or self.controller.restart_visible
      banner = self.controller.status_banner
      layout = wizard_layout(rect.width, rect.height, ready=ready, show_start=show_start, banner=bool(banner))
      self.last_layout = layout
      self._tune(layout)
      if self._font is None:
        self._font = gui_app.font(FontWeight.NORMAL)
        self._title_font = gui_app.font(FontWeight.BOLD)

      rl.draw_rectangle_rec(rect, rl.Color(20, 20, 20, 255))
      title_x = rect.x + layout["margin"]
      title_y = rect.y + layout["title_y"]
      rl.draw_text_ex(self._title_font, self.controller.title, rl.Vector2(title_x, title_y),
                      layout["title_font_size"], 0, rl.WHITE)

      body = layout["body"]
      body_rect = rl.Rectangle(rect.x + body["x"], rect.y + body["y"], body["width"], body["height"])
      lines = self.controller.display_lines()
      font_size = layout["text_font_size"] if ready else layout["output_font_size"]
      line_height = layout["line_height"]
      color = rl.Color(200, 200, 200, 255) if ready else rl.WHITE
      content_rect = rl.Rectangle(0, 0, body_rect.width, len(lines) * line_height)
      if got_new and not ready:
        self._scroll_panel.set_offset(follow_scroll_offset(len(lines), line_height, body_rect.height))
      scroll = self._scroll_panel.update(body_rect, content_rect)
      rl.begin_scissor_mode(int(body_rect.x), int(body_rect.y), int(body_rect.width), int(body_rect.height))
      for i, line in enumerate(lines):
        line_y = body_rect.y + scroll + i * line_height
        if line_y + line_height < body_rect.y or line_y > body_rect.y + body_rect.height:
          continue
        rl.draw_text_ex(self._font, line, rl.Vector2(body_rect.x, line_y), font_size, 0, color)
      rl.end_scissor_mode()

      if layout["banner"] is not None:
        waiting = bool(self.controller.start_block_reason) or self.controller.probe_inflight or self.controller.restart_visible
        banner_color = rl.Color(255, 180, 80, 255) if waiting else rl.Color(128, 216, 166, 255)
        b = layout["banner"]
        rl.draw_text_ex(self._font, banner, rl.Vector2(rect.x + b["x"], rect.y + b["y"]),
                        layout["text_font_size"], 0, banner_color)

      def _rect(spec):
        return rl.Rectangle(rect.x + spec["x"], rect.y + spec["y"], spec["width"], spec["height"])

      buttons = layout["buttons"]
      self._action_button.set_text(self.controller.action_label)
      self._action_button.set_button_style(ButtonStyle.TRANSPARENT_WHITE_BORDER)
      self._action_button.set_enabled(self.controller.action_enabled)
      self._action_button.render(_rect(buttons["action"]))

      if "start" in buttons:
        if self.controller.restart_visible:
          self._start_button.set_text("Restart device")
          self._start_button.set_button_style(ButtonStyle.DANGER)
          self._start_button.set_enabled(self.controller.restart_enabled)
        else:
          self._start_button.set_text("Start")
          self._start_button.set_button_style(ButtonStyle.PRIMARY)
          self._start_button.set_enabled(self.controller.start_enabled)
        self._start_button.render(_rect(buttons["start"]))

      if "bus0" in buttons:
        self._bus0_button.set_button_style(
          ButtonStyle.PRIMARY if self.controller.selected_bus == 0 else ButtonStyle.NORMAL)
        self._bus2_button.set_button_style(
          ButtonStyle.PRIMARY if self.controller.selected_bus == 2 else ButtonStyle.NORMAL)
        self._bus0_button.render(_rect(buttons["bus0"]))
        self._bus2_button.render(_rect(buttons["bus2"]))


  return PedalWizardWidget()


def open_pedal_calibration(instructions: str | None = None) -> None:
  """Push the in-process pedal wizard onto the existing UI stack."""
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.layouts.settings.nap_content import CALIBRATE_PEDAL_INSTRUCTIONS
  text = instructions or CALIBRATE_PEDAL_INSTRUCTIONS
  gui_app.push_widget(make_pedal_wizard_widget(
    "Pedal Calibration",
    CALIBRATE_PEDAL_MODULE,
    text,
    hosted=True,
  ))
