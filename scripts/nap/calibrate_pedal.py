#!/usr/bin/env python3
"""Tesla Pre-AP Comma Pedal calibration.

Pedal frames go through teslaPreap calibration safety (param 64 or 96).
ELM327/allOutput are rejected for 0x551. Command bytes come from
TeslaCANPreAP.create_pedal_command. Stages match the existing NAP algorithm.
"""
from __future__ import annotations

import time

from openpilot.common.params import Params
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from opendbc.car.tesla.preap.pedal_feedback import PEDAL_TIMEOUT_MS
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID, PEDAL_D, PEDAL_M1, TeslaCANPreAP
from scripts.nap.pedal_session import (
  SAFETY_TESLA_PREAP,
  STANDSTILL_SPEED_MS,
  PedalCalibrationSession,
  TransportError,
  calibration_safety_param,
  clear_script_running,
  parse_configured_pedal_bus,
  require_runtime_path,
  wait_for_manager_release,
)

GAS_SENSOR_ID = 0x552
GTW_STATUS_ID = 0x348
BRAKE_MESSAGE_ID = 0x20A
DI_TORQUE1_ID = 0x108
DI_TORQUE2_ID = 0x118
ESP_B_ID = 0x155
GEAR_NEUTRAL = 0x30
MAX_PEDAL_ERRORS = 10
SEND_RATE_MS = 20
SOURCE_TIMEOUT_MS = 1000
ESP_SPEED_SCALE = 0.01
VEHICLE_CAN_BUS = 0
BRAKE_APPLIED = 2
BRAKE_NOT_APPLIED = 1
PEDAL_CONNECT_RETRIES = 8
PEDAL_CONNECT_DELAY = 0.25


def current_time_ms() -> int:
  return int(round(time.monotonic() * 1000))


def p(msg=""):
  print(msg, flush=True)


def unpack_can(msg):
  if len(msg) == 4:
    addr, _bus, dat, src = msg
    return int(addr), bytes(dat), int(src)
  if len(msg) == 3:
    addr, dat, src = msg
    return int(addr), bytes(dat), int(src)
  return None


def esp_b_speed_ms(dat: bytes) -> float | None:
  """ESP_vehicleSpeed. Same bytes as tesla_preap.h: data[5]<<8 | data[6]."""
  if len(dat) < 7:
    return None
  raw = (dat[5] << 8) | dat[6]
  return raw * ESP_SPEED_SCALE / 3.6


class PedalCalibrationError(Exception):
  pass


def parse_pedal_calibration_args(argv=None) -> tuple[bool, int | None]:
  """--confirm plus optional --bus 0|2. Absent bus uses configured params."""
  import argparse
  parser = argparse.ArgumentParser(add_help=False)
  parser.add_argument("--confirm", action="store_true", default=False)
  parser.add_argument("--bus", type=int, choices=(0, 2), default=None)
  args, _unknown = parser.parse_known_args(argv)
  return bool(args.confirm), args.bus


def build_pedal_command(accel_command: float, enable: int, bus: int, can: TeslaCANPreAP | None = None):
  can = can or TeslaCANPreAP(None)
  addr, dat, out_bus = can.create_pedal_command(accel_command, enable=enable, pedal_can_bus=bus)
  if enable not in (0, 1):
    raise PedalCalibrationError("invalid enable flag")
  if bus not in (0, 2):
    raise PedalCalibrationError("invalid pedal bus")
  if out_bus != bus:
    raise PedalCalibrationError("command bus mismatch")
  if addr != GAS_COMMAND_ID:
    raise PedalCalibrationError("invalid command address")
  if len(dat) != 6:
    raise PedalCalibrationError("invalid command length")
  return addr, dat, out_bus


def validate_calibration(min_v: float, max_v: float, factor: float, zero: float) -> None:
  if not (max_v > min_v):
    raise PedalCalibrationError("calibration min/max invalid")
  if factor <= 1e-6:
    raise PedalCalibrationError("calibration factor invalid")
  if factor == 1.0 and zero == 0.0:
    raise PedalCalibrationError("default calibration is not a completed calibration")


def persist_calibration(params: Params, *, min_v: float, max_v: float, factor: float,
                        zero: float, bus: int | None = None) -> None:
  """Invalidate Done, write the set, then Done as the commit bit.

  Params are not transactional. A failed or partial write is left invalid
  (Done false). Bus and enable are not published until this commit.
  """
  validate_calibration(min_v, max_v, factor, zero)
  if bus is not None and bus not in (0, 2):
    raise PedalCalibrationError("invalid pedal bus")
  params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, False, block=True)
  try:
    params.put(NAPParamKeys.PEDAL_CALIB_MIN, float(min_v), block=True)
    params.put(NAPParamKeys.PEDAL_CALIB_MAX, float(max_v), block=True)
    params.put(NAPParamKeys.PEDAL_CALIB_FACTOR, float(factor), block=True)
    params.put(NAPParamKeys.PEDAL_CALIB_ZERO, float(zero), block=True)
    if bus is not None:
      params.put(NAPParamKeys.PEDAL_CAN_BUS, int(bus), block=True)
    params.put_bool(NAPParamKeys.PEDAL_ENABLED, True, block=True)
    params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, True, block=True)
  except Exception:
    try:
      params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, False, block=True)
    except Exception:
      pass
    raise


def send_safe_release(session: PedalCalibrationSession, bus: int, can: TeslaCANPreAP | None = None) -> None:
  addr, dat, out_bus = build_pedal_command(0.0, enable=0, bus=bus, can=can)
  session.can_send(addr, dat, out_bus)


def require_pedal_calibration_start(*, confirmed: bool) -> None:
  require_runtime_path()
  if not confirmed:
    raise PedalCalibrationError("destructive tool requires explicit confirmation")


class PedalCalibrator:
  """NAP pedal calibration stages, sent only through teslaPreap calibration safety."""

  STATUS_MESSAGES = [
    "Initializing...",
    "Configuring Panda...",
    "Reading Pedal Zero...",
    "Detecting Pedal Max...",
    "Fine-tuning calibration...",
    "Validating calibration...",
    "Saving values...",
    "Calibration Complete!",
  ]

  def __init__(self, params: Params, session: PedalCalibrationSession, bus: int):
    self.params = params
    self.session = session
    self.pedal_can = bus
    self.can = TeslaCANPreAP(None)
    self.can.pedal_can_bus = bus
    self.expected_mode = SAFETY_TESLA_PREAP
    self.expected_param = calibration_safety_param(bus)
    self._watchdog_active = True

    self.rcv_pedal_idx = -1
    self.last_rcv_pedal_idx = -1
    self.last_pedal_seen_ms = 0
    self.last_pedal_sent_ms = 0
    self.pedal_interceptor_state = 0
    self.pedal_interceptor_value = 1000.0
    self.pedal_enabled = 0
    self.tx_count = 0
    self.pedal_error_count = 0
    self.pedal_timeout = True

    self.car_on = False
    self.brake_pressed = False
    self.brake_valid = False
    self.gear_neutral = False
    self.di_gas = 0.0
    self.speed_ms = float("nan")
    self.speed_seen_ms = 0
    self.brake_seen_ms = 0
    self.gear_seen_ms = 0
    self.car_on_seen_ms = 0
    self.accel_seen_ms = 0

    self.status = 0
    self.prev_status = -1
    self.frame = 0

    self.pedal_zero_count = 0
    self.pedal_zero_values_to_read = 100
    self.pedal_zero_sum = 0.0

    self.pedal_last_value_sent = 0.0
    self.pedal_pressed_value = -1000.0
    self.pedal_max_value = -1000.0
    self.pedal_step = 0

    self.finetuning_stage = 0
    self.finetuning_target = 99.6
    self.finetuning_step = 0.1
    self.finetuning_sum = 0.0
    self.finetuning_count = 0
    self.finetuning_steps = 10
    self.finetuning_best_val = 0.0
    self.finetuning_best_delta = 1000.0
    self.finetuning_start = 0.0

    self.validation_stage = 0
    self.validation_target = 0.0
    self.validation_count = 0
    self.validation_sum = 0.0
    self.validation_steps = 10
    self.validation_value = 0.0

    self.pedal_min = -1000.0
    self.pedal_max = -1000.0
    self.pedal_pressed = -1000.0
    self.pedal_factor = -1000.0

  def cleanup(self) -> None:
    p("")
    p("RESTORING PEDAL / PANDA SAFETY")
    self._watchdog_active = False
    try:
      send_safe_release(self.session, self.pedal_can, can=self.can)
      time.sleep(0.1)
    except Exception as exc:
      p(f"  Warning: safe release failed: {exc}")
    try:
      self.session.set_silent()
    except Exception as exc:
      p(f"  Warning: {exc}")

  def send_pedal_command(self, accel_command, enable=1) -> None:
    addr, dat, out_bus = build_pedal_command(accel_command, enable=enable, bus=self.pedal_can, can=self.can)
    self.session.can_send(addr, dat, out_bus)
    self.last_pedal_sent_ms = current_time_ms()
    self.tx_count += 1

  def _source_fresh(self, seen_ms: int, now_ms: int) -> bool:
    return seen_ms > 0 and (now_ms - seen_ms) <= SOURCE_TIMEOUT_MS

  def _actuating(self) -> bool:
    return self.pedal_enabled == 1 or self.status >= 3

  def _gate(self, ok: bool, *, wait: str, abort: str, disable: bool = False) -> bool:
    if ok:
      return True
    if self._actuating():
      raise PedalCalibrationError(abort)
    if self.frame % 100 == 0:
      p(wait)
    if disable and self.pedal_enabled:
      self.send_pedal_command(0, enable=0)
      self.pedal_enabled = 0
    return False

  def process_can(self) -> None:
    try:
      for msg in self.session.can_recv():
        unpacked = unpack_can(msg)
        if unpacked is None:
          continue
        addr, dat, src = unpacked
        bus = int(src) & 0x7F
        now_ms = current_time_ms()
        if bus == VEHICLE_CAN_BUS:
          if addr == GTW_STATUS_ID and dat:
            self.car_on = (dat[0] & 0x01) == 1
            self.car_on_seen_ms = now_ms
          elif addr == BRAKE_MESSAGE_ID and dat:
            status = (dat[0] >> 2) & 0x03
            if status == BRAKE_APPLIED:
              self.brake_pressed = True
              self.brake_valid = True
            elif status == BRAKE_NOT_APPLIED:
              self.brake_pressed = False
              self.brake_valid = True
            else:
              self.brake_pressed = False
              self.brake_valid = False
            self.brake_seen_ms = now_ms
          elif addr == DI_TORQUE2_ID and len(dat) > 1:
            self.gear_neutral = (dat[1] & 0x70) == GEAR_NEUTRAL
            self.gear_seen_ms = now_ms
          elif addr == DI_TORQUE1_ID and len(dat) > 6:
            self.di_gas = dat[6] * 0.4
            self.accel_seen_ms = now_ms
          elif addr == ESP_B_ID:
            speed = esp_b_speed_ms(dat)
            if speed is not None:
              self.speed_ms = speed
              self.speed_seen_ms = now_ms
        if addr == GAS_SENSOR_ID and bus == self.pedal_can and len(dat) > 4:
          self.pedal_interceptor_state = (dat[4] >> 7) & 0x01
          self.pedal_interceptor_value = ((dat[0] << 8) + dat[1]) * PEDAL_M1 + PEDAL_D
          self.rcv_pedal_idx = dat[4] & 0x0F
          self.last_pedal_seen_ms = now_ms
    except TransportError as exc:
      raise PedalCalibrationError(f"lost CAN feedback: {exc}") from exc

  def check_safety(self) -> bool:
    now_ms = current_time_ms()
    if not self._gate(self._source_fresh(self.speed_seen_ms, now_ms),
                      wait="  Waiting: vehicle speed not fresh. Stay parked.",
                      abort="lost vehicle speed"):
      return False
    if not self._gate(0.0 <= self.speed_ms <= STANDSTILL_SPEED_MS,
                      wait="  Waiting: vehicle must be stationary.",
                      abort="vehicle is moving"):
      return False
    ignition_fresh = self.car_on and self._source_fresh(self.car_on_seen_ms, now_ms)
    if not self._gate(ignition_fresh,
                      wait="  Waiting: Car is not ON! Turn ignition on.",
                      abort="ignition lost"):
      return False
    if not self._gate(self._source_fresh(self.brake_seen_ms, now_ms),
                      wait="  Waiting: Brake not pressed! Press and hold brake.",
                      abort="brake lost", disable=True):
      return False
    if not self._gate(self.brake_valid,
                      wait="  Waiting: Brake state invalid. Press and hold brake.",
                      abort="invalid brake", disable=True):
      return False
    if not self._gate(self.brake_pressed,
                      wait="  Waiting: Brake not pressed! Press and hold brake.",
                      abort="brake lost", disable=True):
      return False
    if not self._gate(self._source_fresh(self.gear_seen_ms, now_ms) and self.gear_neutral,
                      wait="  Waiting: Car is not in NEUTRAL! Shift to N.",
                      abort="left Neutral", disable=True):
      return False
    if not self._gate(self._source_fresh(self.accel_seen_ms, now_ms),
                      wait="  Waiting: Accelerator state not fresh. Release the pedal.",
                      abort="lost accelerator", disable=True):
      return False
    if self.status < 3 and not self._gate(self.di_gas <= 0,
                                          wait="  Waiting: Accelerator pedal is pressed! Release it.",
                                          abort="accelerator pressed", disable=True):
      return False
    return True

  def _advance_on_pedal_message(self) -> None:
    if self.pedal_interceptor_state != 0:
      return
    self.pedal_error_count = 0

    if self.status == 2:
      self.pedal_zero_sum += self.pedal_interceptor_value
      self.pedal_zero_count += 1
      if self.pedal_zero_count >= self.pedal_zero_values_to_read:
        self.pedal_min = self.pedal_zero_sum / self.pedal_zero_count
        p(f"\n  Pedal MIN: {self.pedal_min:.2f}")
        self.pedal_last_value_sent = self.pedal_min
        self.pedal_enabled = 1
        self.status = 3
      elif self.frame % 20 == 0:
        p(f"  Reading zero: {self.pedal_zero_count}/{self.pedal_zero_values_to_read} (val={self.pedal_interceptor_value:.2f})")

    elif self.status == 3:
      if self.di_gas >= 99.6 and self.pedal_max_value == -1000 and self.pedal_step == 1:
        p(f"\n  Pedal MAX: {self.pedal_last_value_sent:.2f}")
        self.pedal_max_value = self.pedal_last_value_sent
        self.pedal_last_value_sent = self.pedal_max_value - 0.5
        self.finetuning_start = self.pedal_max_value - 0.5
        self.pedal_step = 0
        self.status = 4
      if self.di_gas > 0 and self.pedal_pressed_value == -1000 and self.pedal_step == 1:
        p(f"\n  Pedal PRESSED threshold: {self.pedal_last_value_sent:.2f}")
        self.pedal_pressed_value = self.pedal_last_value_sent
      if self.di_gas < 100:
        if self.frame % 50 == 0:
          p(f"  Detecting: sent {self.pedal_last_value_sent:.1f} -> car sees {self.di_gas:.1f}%")
        if self.pedal_step == 1:
          self.pedal_last_value_sent += 1
          self.pedal_step = 0
        else:
          self.pedal_step = 1

    elif self.status == 4:
      if self.finetuning_count < self.finetuning_steps:
        self.finetuning_sum += self.di_gas
        self.finetuning_count += 1
      if self.finetuning_count == self.finetuning_steps:
        avg = self.finetuning_sum / self.finetuning_count
        delta = abs(self.finetuning_target - avg)
        if delta < self.finetuning_best_delta:
          self.finetuning_best_val = self.pedal_last_value_sent
          self.finetuning_best_delta = delta
        self.pedal_last_value_sent += self.finetuning_step
        p(f"  Fine-tuning: target {self.finetuning_target:.1f}%, got {avg:.1f}%")
        self.finetuning_count = 0
        self.finetuning_sum = 0
        if self.pedal_last_value_sent > self.finetuning_start + 0.5:
          if self.finetuning_stage == 0:
            self.pedal_max = self.finetuning_best_val
            self.finetuning_best_val = 0
            self.finetuning_best_delta = 1000.0
            self.finetuning_stage = 1
            self.pedal_last_value_sent = self.pedal_pressed_value - 0.5
            self.finetuning_start = self.pedal_pressed_value - 0.5
            self.finetuning_target = 0.4
            p("\n  Done fine-tuning MAX")
          else:
            self.pedal_pressed = self.finetuning_best_val
            self.pedal_last_value_sent = self.pedal_min
            span = self.pedal_max - self.pedal_pressed
            if span <= 1e-6:
              raise PedalCalibrationError("calibration span invalid")
            self.pedal_factor = 100.0 / span
            self.status = 5
            p("\n  Done fine-tuning ZERO")

    elif self.status == 5:
      if self.validation_stage == 0:
        self.validation_stage = 1
        self.validation_target = self.validation_stage * 10
        self.validation_count = 0
        self.validation_sum = 0
        self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max - self.pedal_pressed) / 100.0
        self.pedal_last_value_sent = self.validation_value
      elif self.validation_count < self.validation_steps:
        self.validation_sum += self.di_gas
        self.validation_count += 1
        if self.validation_count == self.validation_steps:
          avg = self.validation_sum / self.validation_count
          p(f"  Validating {self.validation_target:.0f}%: got {avg:.1f}%")
          self.validation_stage += 1
          if self.validation_stage == 10:
            p("\n  Validation complete")
            self.status = 6
          else:
            self.validation_target = self.validation_stage * 10
            self.validation_count = 0
            self.validation_sum = 0
            self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max - self.pedal_pressed) / 100.0
            self.pedal_last_value_sent = self.validation_value

    elif self.status == 6:
      p("")
      p("CALIBRATION RESULTS")
      p(f"  Pedal Min:    {self.pedal_min:.2f}")
      p(f"  Pedal Max:    {self.pedal_max:.2f}")
      p(f"  Pedal Zero:   {self.pedal_pressed:.2f}")
      p(f"  Pedal Factor: {self.pedal_factor:.4f}")
      self._disable_output_before_persist()
      persist_calibration(
        self.params,
        min_v=self.pedal_min,
        max_v=self.pedal_max,
        factor=self.pedal_factor,
        zero=self.pedal_pressed,
        bus=self.pedal_can,
      )
      p("  Saved to openpilot Params")
      self.status = 7

  def _disable_output_before_persist(self) -> None:
    self._watchdog_active = False
    try:
      self.send_pedal_command(0, enable=0)
    except Exception as exc:
      p(f"  Warning: ENABLE=0 before save failed: {exc}")
    self.pedal_enabled = 0
    try:
      self.session.set_silent()
    except Exception as exc:
      raise PedalCalibrationError(f"could not enter SILENT before save: {exc}") from exc

  def _tick_watchdog(self) -> None:
    self.session.send_heartbeat(False)
    self.session.require_calibration_health(self.expected_mode, self.expected_param)

  def run(self, rate=100) -> int:
    self.status = 2
    p("")
    p("Waiting for safety conditions...")
    p("  - Car ON")
    p("  - Gear in NEUTRAL")
    p("  - Brake PRESSED")
    p("  - Stationary")
    p("  - Accelerator RELEASED")
    p(f"  [TX: Bus {self.pedal_can}, ID 0x{GAS_COMMAND_ID:03X}, teslaPreap calibration]")
    loop_period = 1.0 / rate
    while True:
      loop_start = time.monotonic()
      self.frame += 1
      curr_time_ms = current_time_ms()
      if self._watchdog_active:
        self._tick_watchdog()
      self.process_can()
      if self.status != self.prev_status:
        p(f"\n{self.STATUS_MESSAGES[self.status]}")
        self.prev_status = self.status
      if self.status == 7:
        p("\nCalibration Complete!")
        p("Driving stays paused. Press Restart device to reboot, or turn the car off first.")
        return 0
      if not self.check_safety():
        time.sleep(loop_period)
        continue
      if curr_time_ms - self.last_pedal_sent_ms >= SEND_RATE_MS:
        self.send_pedal_command(self.pedal_last_value_sent, self.pedal_enabled)
        if self.tx_count % 100 == 0:
          p(f"  [TX #{self.tx_count}: val={self.pedal_last_value_sent:.1f}, en={self.pedal_enabled}]")
      if self.rcv_pedal_idx != self.last_rcv_pedal_idx:
        self.last_rcv_pedal_idx = self.rcv_pedal_idx
        self._advance_on_pedal_message()
      self.pedal_timeout = curr_time_ms - self.last_pedal_seen_ms > PEDAL_TIMEOUT_MS
      if self.pedal_timeout:
        self.pedal_error_count += 1
        if self.pedal_error_count > MAX_PEDAL_ERRORS * 10:
          raise PedalCalibrationError("Pedal communication timeout")
      elapsed = time.monotonic() - loop_start
      if elapsed < loop_period:
        time.sleep(loop_period - elapsed)


def run(*, confirmed: bool, params: Params | None = None,
        session: PedalCalibrationSession | None = None, bus: int | None = None) -> int:
  params = params or Params()
  require_pedal_calibration_start(confirmed=confirmed)
  if bus is None:
    bus = parse_configured_pedal_bus(params.get(NAPParamKeys.PEDAL_CAN_BUS))
  else:
    bus = int(bus)
  if bus not in (0, 2):
    raise PedalCalibrationError("invalid pedal bus")

  owned = session is None
  handoff_observed = False
  calibrator = None
  if owned:
    session = PedalCalibrationSession()
  try:
    wait_for_manager_release()
    handoff_observed = True
    if owned:
      session.connect(retries=PEDAL_CONNECT_RETRIES, delay=PEDAL_CONNECT_DELAY)
    else:
      session.connect()
    session.set_pedal_calibration_session(bus)
    calibrator = PedalCalibrator(params, session, bus)
    return calibrator.run()
  except TransportError as exc:
    raise PedalCalibrationError(str(exc)) from exc
  finally:
    if calibrator is not None:
      calibrator.cleanup()
    elif session is not None:
      try:
        send_safe_release(session, bus)
      except Exception:
        pass
      try:
        session.set_silent()
      except Exception:
        pass
    if owned and session is not None:
      session.close()
    if not handoff_observed:
      clear_script_running(params)


def main(argv=None) -> int:
  p("=" * 60)
  p("COMMA PEDAL CALIBRATION")
  p("=" * 60)
  p("Tesla Pre-AP calibration safety only. ELM327/ALLOUTPUT are rejected.")
  try:
    confirmed, bus = parse_pedal_calibration_args(argv)
    return run(confirmed=confirmed, bus=bus)
  except KeyboardInterrupt:
    p("\nInterrupted by user.")
    return 1
  except Exception as exc:
    p(f"ERROR: {exc}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
