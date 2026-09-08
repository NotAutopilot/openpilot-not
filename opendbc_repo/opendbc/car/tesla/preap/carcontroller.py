from enum import IntEnum

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.preap.interface import get_preap_accel_limits
from opendbc.car.tesla.pedal.controller import get_zero_torque, PEDAL_RAMP_RATE_UP
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from opendbc.car.carlog import carlog


def init_preap_can(dbc_names, packers):
  packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
  tesla_can = TeslaCANPreAP(packers)
  tesla_can.pedal_can_bus = nap_conf.pedal_can_bus
  return tesla_can


# Grace period after engage: ramp accel limit from 0 → full over this window.
# Prevents both regen spike (negative) and pedal stab (MPC requesting high
# positive accel on frame 1). Inspired by Tinkla's proportional ramp.
ENGAGE_GRACE_FRAMES = 50  # 0.5s at 100Hz
ENGAGE_GRACE_PEDAL_RAMP_RATE_UP = 0.9  # DI/update at 50Hz

# The pedal controller updates at 50 Hz. Prompt when a sustained deceleration
# request is not being delivered while the command is in the regen range.
# Available regen varies with SOC and battery temperature, so proximity to
# the DI rail is not a reliable "at the limit" signal: on a full battery the
# rail delivers a fraction of nominal regen, and the integrator only reaches
# it near the end of an under-delivery event. Field capture (2026-07-22,
# minutes after supercharging) showed the command within 0.5 DI of the rail
# for just 0.3 s of a multi-second under-delivery, so a rail gate plus a
# consecutive dwell can never complete. Evidence instead accumulates in a
# saturating up/down counter, which also rides through single-update
# shortfall noise. Trigger and clear thresholds are separated so the visible
# prompt cannot chatter.
REGEN_DECEL_PROMPT_DWELL_UPDATES = 40  # 0.8s of net evidence at 50Hz
REGEN_DECEL_PROMPT_MIN_SPEED = 2.0  # m/s; do not prompt for a stopped/settling car
REGEN_DECEL_PROMPT_CLEAR_SPEED = 1.0  # m/s
REGEN_DECEL_SHORTFALL_TRIGGER = 0.35  # m/s²
REGEN_DECEL_SHORTFALL_CLEAR = 0.15  # m/s²
REGEN_COMMAND_TRIGGER_DI = -2.0  # command deep in the regen range
REGEN_COMMAND_CLEAR_DI = -1.0
REGEN_DECEL_REQUEST_TRIGGER = -0.5  # m/s²; a meaningful deceleration request
REGEN_DECEL_REQUEST_CLEAR = -0.2  # m/s²


class RegenDecelMonitor:
  """Detect when the regen path cannot deliver the requested deceleration."""

  def __init__(self):
    self.active = False
    self.evidence_updates = 0

  def reset(self):
    self.active = False
    self.evidence_updates = 0

  def update(self, *, pedal_control_active, in_engage_grace, pedal_di,
             limited_accel, actual_accel, v_ego):
    values_are_finite = all(np.isfinite((pedal_di, limited_accel, actual_accel, v_ego)))
    decel_shortfall = actual_accel - limited_accel
    monitoring_allowed = (
      pedal_control_active
      and not in_engage_grace
      and values_are_finite
      and limited_accel < 0.0
    )

    if self.active:
      keep_prompting = (
        monitoring_allowed
        and v_ego > REGEN_DECEL_PROMPT_CLEAR_SPEED
        and pedal_di <= REGEN_COMMAND_CLEAR_DI
        and limited_accel <= REGEN_DECEL_REQUEST_CLEAR
        and decel_shortfall > REGEN_DECEL_SHORTFALL_CLEAR
      )
      if not keep_prompting:
        self.reset()
      return self.active

    under_delivering = (
      monitoring_allowed
      and v_ego >= REGEN_DECEL_PROMPT_MIN_SPEED
      and pedal_di <= REGEN_COMMAND_TRIGGER_DI
      and limited_accel <= REGEN_DECEL_REQUEST_TRIGGER
      and decel_shortfall >= REGEN_DECEL_SHORTFALL_TRIGGER
    )
    if under_delivering:
      self.evidence_updates = min(self.evidence_updates + 1, REGEN_DECEL_PROMPT_DWELL_UPDATES)
    else:
      self.evidence_updates = max(self.evidence_updates - 1, 0)
    self.active = self.evidence_updates >= REGEN_DECEL_PROMPT_DWELL_UPDATES
    return self.active


class PedalAuthorityState(IntEnum):
  INACTIVE = 0
  ACQUIRING = 1
  ACTIVE = 2
  FAILED = 3


class PedalCommandAction(IntEnum):
  NONE = 0
  RESET = 1
  ACQUIRE = 2
  ENABLE = 3
  RELEASE = 4
  FAILURE = 5


class PedalAuthority:
  """Owns the pedal command-authority lifecycle."""

  MAX_RESET_ATTEMPTS = 4

  def __init__(self):
    self.state = PedalAuthorityState.INACTIVE
    self.reset_feedback_counter = None
    self.reset_attempts = 0

  def _clear_acquisition(self):
    self.reset_feedback_counter = None
    self.reset_attempts = 0

  def _start_acquisition(self, feedback):
    self.state = PedalAuthorityState.ACQUIRING
    self.reset_feedback_counter = feedback.idx
    self.reset_attempts = 1
    return PedalCommandAction.RESET

  def update(self, authority_requested, feedback):
    if not authority_requested:
      action = PedalCommandAction.RELEASE if self.state == PedalAuthorityState.ACTIVE else PedalCommandAction.NONE
      self.state = PedalAuthorityState.INACTIVE
      self._clear_acquisition()
      return action

    if self.state == PedalAuthorityState.FAILED:
      return PedalCommandAction.NONE

    feedback_healthy = feedback.available and feedback.interceptor_state == 0
    if self.state == PedalAuthorityState.ACTIVE:
      if feedback_healthy:
        return PedalCommandAction.ENABLE
      return self._start_acquisition(feedback)

    if self.state == PedalAuthorityState.ACQUIRING:
      feedback_advanced = feedback.idx != self.reset_feedback_counter
      if feedback_healthy and feedback_advanced:
        self.state = PedalAuthorityState.ACTIVE
        self._clear_acquisition()
        return PedalCommandAction.ACQUIRE

      if self.reset_attempts < self.MAX_RESET_ATTEMPTS:
        self.reset_attempts += 1
        return PedalCommandAction.RESET

      self.state = PedalAuthorityState.FAILED
      self._clear_acquisition()
      return PedalCommandAction.FAILURE

    if feedback_healthy:
      self.state = PedalAuthorityState.ACTIVE
      return PedalCommandAction.ACQUIRE

    return self._start_acquisition(feedback)

  def command_failed(self):
    self.state = PedalAuthorityState.FAILED
    self._clear_acquisition()


class PreAPLongController:
  """Pedal-mode longitudinal: VirtualDAS, zero-torque, command authority.

  Stalk-CC spoofs (CANCEL / SET_ACCEL) live in StockCCSpoofer.
  Communicates with the spoofer via CarState flags only.
  """

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_requested_long = False
    self.preap_long_engage_frame = -1000000
    # Snapshot of max-accel-at-engage-speed; used as the deterministic
    # ceiling for the grace-period ramp. Set fresh on each engage rising edge.
    self.engage_a_max = 0.0
    self.preap_long_handoff_slew_active = False
    self.vdas = VirtualDAS(dt=0.02)
    self.pedal_authority = PedalAuthority()
    self.regen_decel_monitor = RegenDecelMonitor()

  @staticmethod
  def _handle_pedal_unavailable(CS):
    engagement = getattr(CS, 'engagement', None)
    if engagement is None:
      carlog.error("Pre-AP pedal authority failed without engagement state")
      return

    engagement.handle_pedal_unavailable()
    # Keep the controller-facing bridge coherent immediately. CarState will
    # refresh these fields from engagement again on its next update.
    CS.cruiseEnabled = engagement.cruiseEnabled
    CS.enableLongControl = engagement.enableLongControl
    CS.enableJustCC = engagement.enableJustCC
    CS.pedal_speed_kph = engagement.pedal_speed_kph
    CS.longCtrlEvent = engagement.longCtrlEvent

  @staticmethod
  def _append_pedal_command(can_sends, CS, command):
    can_sends.append(command)
    CS.pedal_command_counter = command[1][4] & 0x0F

  def update(self, CC, CS, frame, tesla_can, can_bus_party, now_nanos=0):
    can_sends = []
    actuators = CC.actuators

    requested_long = CS.cruiseEnabled and CS.enableLongControl
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    pedal_long_allowed = use_pedal and pedal_transform_valid
    if (not long_active
        or getattr(CS, 'real_brake_pressed', False)
        or getattr(CS.out, 'gasPressed', False)):
      self.regen_decel_monitor.reset()

    requested_long_rising = (not self.prev_requested_long) and requested_long
    if requested_long_rising:
      # Seed the disabled acquisition frames at coast, but do not initialize
      # command dynamics until feedback proves the pedal accepted authority.
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
      CS.pedal_first_enabled_mono_time = 0
    elif not hasattr(CS, 'pedal_first_enabled_mono_time'):
      CS.pedal_first_enabled_mono_time = 0

    # --- Stock CC cancel triggers from pedal mode ---
    # Engage / disengage / button-press in pedal mode all need to drop stock
    # CC if it's running. Publish the request via CarState; StockCCSpoofer
    # consumes it and TXes the CANCEL frame.
    if pedal_long_allowed:
      pedal_button_press = (CS.cruise_buttons != CS.prev_cruise_buttons
                            and CS.cruise_buttons != CruiseButtons.IDLE)
      pedal_long_falling = self.prev_requested_long and not requested_long
      if requested_long_rising or pedal_long_falling or pedal_button_press:
        CS.preap_cc_cancel_needed = True

    self.prev_requested_long = requested_long

    if frame % 2 == 0:
      brake_pressed = getattr(CS, 'real_brake_pressed', False)
      authority_requested = pedal_long_allowed and long_active and not brake_pressed and not CS.out.gasPressed
      pedal_action = self.pedal_authority.update(authority_requested, CS.pedal)
      in_engage_grace = False

      if pedal_action == PedalCommandAction.ACQUIRE:
        self.preap_long_engage_frame = frame
        self.preap_long_handoff_slew_active = True
        zero_torque_di = get_zero_torque().get(CS.out.vEgo)
        self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
        self.vdas.reset(
          measured_accel=CS.out.aEgo,
          commanded_accel=0.0,
          pedal_di_init=self.prev_pedal_di,
          preserve_grade=True,
        )
        _, self.engage_a_max = get_preap_accel_limits(CS.out.vEgo)

      if use_pedal and pedal_action not in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE):
        self.vdas.observe(CS.out.aEgo, list(CC.orientationNED))

      if use_pedal:
        get_zero_torque().update(
          CS.pedal.torque_level,
          self.prev_pedal_di,
          CS.out.vEgo,
          control_active=pedal_action in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE),
          accel_command=self.vdas.jerk_limiter.a_limited,
        )

      if pedal_action == PedalCommandAction.RESET:
        self._append_pedal_command(can_sends, CS, tesla_can.create_pedal_command(0, enable=0))
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      elif pedal_action == PedalCommandAction.RELEASE:
        self._append_pedal_command(can_sends, CS, tesla_can.create_pedal_command(0, enable=0))
        self.prev_pedal_di = 0.0
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      elif pedal_action in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE):
        try:
          engage_elapsed_frames = frame - self.preap_long_engage_frame
          in_engage_grace = engage_elapsed_frames < ENGAGE_GRACE_FRAMES
          accel_request = float(actuators.accel)
          accel_effort_limits = None
          pedal_ramp_rate_up = (
            ENGAGE_GRACE_PEDAL_RAMP_RATE_UP
            if self.preap_long_handoff_slew_active
            else PEDAL_RAMP_RATE_UP
          )
          if in_engage_grace:
            # Cap at grace_progress * engage_a_max so the ceiling is the
            # tuned accel-profile envelope, not the live MPC request.
            # Keeps an MPC outlier on engage from propagating through.
            grace_progress = engage_elapsed_frames / ENGAGE_GRACE_FRAMES
            accel_cap = grace_progress * self.engage_a_max
            accel_request = max(0.0, min(accel_request, accel_cap))
            accel_effort_limits = (0.0, accel_cap)
            pedal_ramp_rate_up = ENGAGE_GRACE_PEDAL_RAMP_RATE_UP

          self.prev_pedal_di = self.vdas.update(
            accel_request, CS.out.vEgo, self.prev_pedal_di,
            a_ego=CS.out.aEgo, freeze_integrator=in_engage_grace,
            orientation_ned=list(CC.orientationNED),
            accel_effort_limits=accel_effort_limits,
            pedal_ramp_rate_up=pedal_ramp_rate_up)
          handoff_slew_complete = (
            self.preap_long_handoff_slew_active
            and not in_engage_grace
            and not self.vdas.pedal_ramp_limited_up
          )
          if handoff_slew_complete:
            self.preap_long_handoff_slew_active = False
          pedal_cmd = nap_conf.di_to_pedal(self.prev_pedal_di)
          command = tesla_can.create_pedal_command(pedal_cmd, enable=1)
          self._append_pedal_command(can_sends, CS, command)
          if pedal_action == PedalCommandAction.ACQUIRE and CS.pedal_first_enabled_mono_time == 0:
            CS.pedal_first_enabled_mono_time = now_nanos
          self.regen_decel_monitor.update(
            pedal_control_active=True,
            in_engage_grace=in_engage_grace,
            pedal_di=self.prev_pedal_di,
            limited_accel=self.vdas.jerk_limiter.a_limited,
            actual_accel=CS.out.aEgo,
            v_ego=CS.out.vEgo,
          )

        except Exception:
          carlog.exception("Pre-AP pedal command failed; sending disabled")
          self.pedal_authority.command_failed()
          self._handle_pedal_unavailable(CS)
          self._append_pedal_command(can_sends, CS, tesla_can.create_pedal_command(0, enable=0))
          self.prev_pedal_di = 0.0
          self.preap_long_handoff_slew_active = False
          self.regen_decel_monitor.reset()
          pedal_action = PedalCommandAction.FAILURE

      elif pedal_action == PedalCommandAction.FAILURE:
        carlog.error("Pre-AP pedal authority acquisition failed")
        self._handle_pedal_unavailable(CS)
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      else:
        self.regen_decel_monitor.reset()

      CS.pedal_authority_requested = authority_requested
      CS.pedal_authority_active = (
        self.pedal_authority.state == PedalAuthorityState.ACTIVE
        and not CS.engagement.pedal_unavailable
      )
      # State and action remain live. The separate engagement-owned failure
      # field is latched so a 10 Hz qlog sample cannot miss the incident.
      CS.pedal_authority_state = int(self.pedal_authority.state)
      CS.pedal_authority_action = int(pedal_action)

    CS.vdas_limited_accel = float(self.vdas.jerk_limiter.a_limited)
    # This is the controller's DI-domain command/seed. RESET preserves the
    # coast seed even though its disabled wire command carries zero.
    CS.pedal_command_di = float(self.prev_pedal_di)
    CS.pedal_brake_required = self.regen_decel_monitor.active
    return can_sends
