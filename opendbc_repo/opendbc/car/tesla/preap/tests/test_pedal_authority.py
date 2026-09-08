import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.carcontroller import (
  REGEN_DECEL_PROMPT_DWELL_UPDATES,
  PedalAuthority,
  PedalAuthorityState,
  PedalCommandAction,
  PreAPLongController,
  RegenDecelMonitor,
)
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.nap_conf import PEDAL_DI_ZERO, PEDAL_MAX_VALUES
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID, PEDAL_D, PEDAL_M1, TeslaCANPreAP
from opendbc.car.tesla.pedal.controller import (
  PEDAL_RAMP_RATE_UP,
  PedalZeroTorque,
)


def _pedal_conf():
  return SimpleNamespace(
    use_pedal=True,
    pedal_factor=1.0,
    di_to_pedal=lambda pedal_di: pedal_di,
    get_pedal_profile_values=lambda: PEDAL_MAX_VALUES,
  )


def _zero_torque():
  return SimpleNamespace(
    get=lambda _v_ego: PEDAL_DI_ZERO,
    update=lambda *_args, **_kwargs: None,
  )


@pytest.fixture
def controller_env(monkeypatch):
  zero_torque = _zero_torque()
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

  feedback = PedalFeedback()
  feedback.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1}, 0)
  engagement = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    enableJustCC=False,
    engagement=engagement,
    real_brake_pressed=False,
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False),
    pedal_interceptor_value=0.0,
    cruise_buttons=0,
    prev_cruise_buttons=0,
    pedal=feedback,
    pedal_timeout=feedback.timeout,
    pccEvent=None,
    preap_cc_cancel_needed=False,
  )
  cc = SimpleNamespace(
    actuators=SimpleNamespace(accel=0.0),
    longActive=False,
    orientationNED=[],
  )
  return PreAPLongController(), cc, cs, TeslaCANPreAP({})


def _decode_pedal_command(command):
  address, data, _bus = command
  assert address == GAS_COMMAND_ID
  raw_command = (data[0] << 8) | data[1]
  return SimpleNamespace(
    enabled=bool(data[4] & 0x80),
    command=raw_command * PEDAL_M1 + PEDAL_D,
    raw_command=raw_command,
    idx=data[4] & 0x0F,
  )


def _activate_longitudinal(cc, cs):
  cs.engagement.cruiseEnabled = True
  cs.engagement.enableLongControl = True
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cc.longActive = True


def _feedback(*, state, idx, available=None):
  return SimpleNamespace(
    available=state == 0 if available is None else available,
    interceptor_state=state,
    idx=idx,
  )


class _FirmwarePedalModel:
  """Minimal command-counter behavior of the pedal firmware."""

  def __init__(self, feedback, initial_command_idx=0):
    self.feedback = feedback
    self.state = 5
    self.feedback_idx = 0
    # A command is accepted only when it follows the last observed counter.
    # Rejected commands are still remembered, allowing the next consecutive
    # counter to restore synchronization from any initial value.
    self.last_command_idx = initial_command_idx
    self.enabled = False
    self._publish(0)

  def apply(self, command, curr_time_ms):
    decoded = _decode_pedal_command(command)
    expected_idx = (self.last_command_idx + 1) % 16
    accepted = decoded.idx == expected_idx
    self.last_command_idx = decoded.idx
    if accepted:
      if decoded.enabled and self.state == 0:
        self.enabled = True
      elif not decoded.enabled:
        self.state = 0
        self.enabled = False

    self.feedback_idx = (self.feedback_idx + 1) % 16
    self._publish(curr_time_ms)
    return decoded, accepted

  def _publish(self, curr_time_ms):
    self.feedback.update({
      "INTERCEPTOR_GAS": 0.0,
      "INTERCEPTOR_GAS2": 0.0,
      "STATE": self.state,
      "IDX": self.feedback_idx,
    }, curr_time_ms)


@pytest.mark.parametrize("initial_command_idx", range(16))
def test_all_initial_firmware_command_counters_recover(controller_env, initial_command_idx):
  controller, cc, cs, tesla_can = controller_env
  firmware = _FirmwarePedalModel(cs.pedal, initial_command_idx=initial_command_idx)
  _activate_longitudinal(cc, cs)

  commands = []
  acceptances = []
  for frame in range(0, 8, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert len(sent) == 1
    decoded, accepted = firmware.apply(sent[0], curr_time_ms=(frame + 2) * 10)
    commands.append(decoded)
    acceptances.append(accepted)
    if firmware.enabled:
      break

  expected_counters = [0, 1] if initial_command_idx == 15 else [0, 1, 2]
  assert [command.idx for command in commands] == expected_counters
  assert [command.enabled for command in commands] == [False] * (len(commands) - 1) + [True]
  assert acceptances[-1]
  assert firmware.enabled
  assert controller.pedal_authority.state == PedalAuthorityState.ACTIVE


def test_controller_wire_counter_wraps_from_fifteen_to_zero(controller_env):
  controller, cc, cs, tesla_can = controller_env
  tesla_can.pedal_idx = 15
  _activate_longitudinal(cc, cs)

  first = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  second = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)

  assert [_decode_pedal_command(command[0]).idx for command in (first, second)] == [15, 0]
  assert all(_decode_pedal_command(command[0]).enabled for command in (first, second))


def test_pedal_authority_reachable_states_obey_transition_invariants():
  feedback_inputs = (
    (False, _feedback(state=0, idx=0)),
    (True, _feedback(state=0, idx=0)),
    (True, _feedback(state=0, idx=1)),
    (True, _feedback(state=0, idx=15)),
    (True, _feedback(state=5, idx=0)),
    (True, _feedback(state=5, idx=1)),
    (True, _feedback(state=5, idx=15)),
    (True, _feedback(state=0, idx=1, available=False)),
  )
  frontier = {(PedalAuthorityState.INACTIVE, None, 0)}

  for _depth in range(7):
    next_frontier = set()
    for state, reset_counter, reset_attempts in frontier:
      for requested, feedback in feedback_inputs:
        authority = PedalAuthority()
        authority.state = state
        authority.reset_feedback_counter = reset_counter
        authority.reset_attempts = reset_attempts
        feedback_healthy = feedback.available and feedback.interceptor_state == 0

        action = authority.update(requested, feedback)

        if not requested:
          expected_action = PedalCommandAction.RELEASE if state == PedalAuthorityState.ACTIVE else PedalCommandAction.NONE
          assert action == expected_action
          assert authority.state == PedalAuthorityState.INACTIVE
        elif state == PedalAuthorityState.FAILED:
          assert action == PedalCommandAction.NONE
          assert authority.state == PedalAuthorityState.FAILED
        elif state == PedalAuthorityState.ACTIVE:
          expected_action = PedalCommandAction.ENABLE if feedback_healthy else PedalCommandAction.RESET
          assert action == expected_action
          assert authority.state == (PedalAuthorityState.ACTIVE if feedback_healthy else PedalAuthorityState.ACQUIRING)
        elif state == PedalAuthorityState.ACQUIRING:
          feedback_advanced = feedback.idx != reset_counter
          if feedback_healthy and feedback_advanced:
            assert action == PedalCommandAction.ACQUIRE
            assert authority.state == PedalAuthorityState.ACTIVE
          elif reset_attempts < PedalAuthority.MAX_RESET_ATTEMPTS:
            assert action == PedalCommandAction.RESET
            assert authority.state == PedalAuthorityState.ACQUIRING
            assert authority.reset_attempts == reset_attempts + 1
          else:
            assert action == PedalCommandAction.FAILURE
            assert authority.state == PedalAuthorityState.FAILED
        elif feedback_healthy:
          assert action == PedalCommandAction.ACQUIRE
          assert authority.state == PedalAuthorityState.ACTIVE
        else:
          assert action == PedalCommandAction.RESET
          assert authority.state == PedalAuthorityState.ACQUIRING

        next_frontier.add((authority.state, authority.reset_feedback_counter, authority.reset_attempts))
    frontier = next_frontier


@pytest.mark.parametrize(
  ("feedback_trace", "expected_actions"),
  (
    pytest.param(
      [(5, 0), (5, 1), (5, 2), (5, 3), (5, 4)],
      [PedalCommandAction.RESET] * 4 + [PedalCommandAction.FAILURE],
      id="advancing-counter-does-not-mask-fault",
    ),
    pytest.param(
      [(5, 7), (0, 7), (0, 8)],
      [PedalCommandAction.RESET, PedalCommandAction.RESET, PedalCommandAction.ACQUIRE],
      id="healthy-feedback-must-advance",
    ),
    pytest.param(
      [(5, 4), (5, 4), (5, 4), (5, 4), (0, 5)],
      [PedalCommandAction.RESET] * 4 + [PedalCommandAction.ACQUIRE],
      id="accepted-feedback-on-deadline-acquires",
    ),
  ),
)
def test_pedal_firmware_feedback_traces(feedback_trace, expected_actions):
  authority = PedalAuthority()

  actions = [
    authority.update(True, _feedback(state=state, idx=idx))
    for state, idx in feedback_trace
  ]

  assert actions == expected_actions


def test_acquisition_sends_four_resets_then_fails_on_next_update():
  authority = PedalAuthority()

  actions = [
    authority.update(True, _feedback(state=5, idx=idx))
    for idx in range(5)
  ]

  assert actions == [
    PedalCommandAction.RESET,
    PedalCommandAction.RESET,
    PedalCommandAction.RESET,
    PedalCommandAction.RESET,
    PedalCommandAction.FAILURE,
  ]
  assert authority.state == PedalAuthorityState.FAILED


def test_failed_request_cannot_late_acquire_and_rearms_only_after_falling_edge():
  authority = PedalAuthority()
  for idx in range(5):
    authority.update(True, _feedback(state=5, idx=idx))

  assert authority.update(True, _feedback(state=0, idx=6)) == PedalCommandAction.NONE
  assert authority.state == PedalAuthorityState.FAILED
  assert authority.update(False, _feedback(state=0, idx=7)) == PedalCommandAction.NONE
  assert authority.state == PedalAuthorityState.INACTIVE
  assert authority.update(True, _feedback(state=0, idx=8)) == PedalCommandAction.ACQUIRE
  assert authority.state == PedalAuthorityState.ACTIVE


def test_healthy_request_acquires_then_releases_once_and_stays_silent():
  authority = PedalAuthority()
  healthy = _feedback(state=0, idx=4)

  assert authority.update(True, healthy) == PedalCommandAction.ACQUIRE
  assert authority.update(True, healthy) == PedalCommandAction.ENABLE
  assert authority.update(False, healthy) == PedalCommandAction.RELEASE
  assert authority.update(False, healthy) == PedalCommandAction.NONE


def test_authority_diagnostic_values_are_stable_integers():
  assert {state.name: int(state) for state in PedalAuthorityState} == {
    "INACTIVE": 0,
    "ACQUIRING": 1,
    "ACTIVE": 2,
    "FAILED": 3,
  }
  assert {action.name: int(action) for action in PedalCommandAction} == {
    "NONE": 0,
    "RESET": 1,
    "ACQUIRE": 2,
    "ENABLE": 3,
    "RELEASE": 4,
    "FAILURE": 5,
  }


def test_feedback_loss_while_active_uses_same_bounded_reacquisition():
  authority = PedalAuthority()
  assert authority.update(True, _feedback(state=0, idx=9)) == PedalCommandAction.ACQUIRE

  actions = [
    authority.update(True, _feedback(state=5, idx=idx))
    for idx in range(10, 15)
  ]
  assert actions == [
    PedalCommandAction.RESET,
    PedalCommandAction.RESET,
    PedalCommandAction.RESET,
    PedalCommandAction.RESET,
    PedalCommandAction.FAILURE,
  ]


def test_reset_frames_preserve_coast_seed(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cs.pedal_interceptor_value = 3.0
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 7}, 20)

  sent = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  assert len(sent) == 1
  assert not _decode_pedal_command(sent[0]).enabled
  assert controller.prev_pedal_di == pytest.approx(3.0)


def test_controller_failure_drops_only_longitudinal_and_latches_unavailable(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 7}, 20)

  sent = []
  for frame in range(0, 10, 2):
    sent.append(controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0))

  assert [len(commands) for commands in sent] == [1, 1, 1, 1, 0]
  assert cs.engagement.cruiseEnabled
  assert not cs.engagement.enableLongControl
  assert cs.engagement.enableJustCC
  assert cs.engagement.pedal_unavailable

  controller.update(cc, cs, frame=10, tesla_can=tesla_can, can_bus_party=0)
  assert cs.pedal_authority_state == int(PedalAuthorityState.INACTIVE)
  assert cs.pedal_authority_action == int(PedalCommandAction.NONE)


def test_pedal_unavailable_condition_latches_until_fresh_request_or_full_disengage():
  engagement = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  engagement.cruiseEnabled = True
  engagement.enableLongControl = True
  engagement.handle_pedal_unavailable()

  assert engagement.pedal_unavailable

  engagement.process_buttons(
    cruise_buttons=0,
    prev_cruise_buttons=0,
    curr_time_ms=1000,
    v_ego=15.0,
    speed_units="KPH",
    use_pedal=True,
    pedal_long_allowed=True,
    long_control_allowed=True,
    real_brake_pressed=False,
  )
  assert engagement.pedal_unavailable

  engagement.process_buttons(
    cruise_buttons=2,
    prev_cruise_buttons=0,
    curr_time_ms=1100,
    v_ego=15.0,
    speed_units="KPH",
    use_pedal=True,
    pedal_long_allowed=True,
    long_control_allowed=True,
    real_brake_pressed=False,
  )
  assert engagement.enableLongControl
  assert not engagement.pedal_unavailable

  engagement.handle_pedal_unavailable()
  engagement.handle_steering_disengage(True)
  assert not engagement.cruiseEnabled
  assert not engagement.pedal_unavailable


def test_fully_disengaged_pedal_is_silent(controller_env):
  controller, cc, cs, tesla_can = controller_env

  assert controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0) == []
  assert controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0) == []


def test_enabled_command_publishes_actual_counter_and_replay_timestamp(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)

  first = controller.update(
    cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0, now_nanos=123456789,
  )

  assert len(first) == 1
  assert _decode_pedal_command(first[0]).enabled
  assert cs.pedal_authority_requested
  assert cs.pedal_authority_active
  assert cs.pedal_authority_state == int(PedalAuthorityState.ACTIVE)
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)
  assert cs.pedal_command_counter == _decode_pedal_command(first[0]).idx
  assert cs.pedal_first_enabled_mono_time == 123456789
  assert cs.vdas_limited_accel == pytest.approx(controller.vdas.jerk_limiter.a_limited)
  assert cs.pedal_command_di == pytest.approx(controller.prev_pedal_di)

  second = controller.update(
    cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0, now_nanos=987654321,
  )
  assert cs.pedal_command_counter == _decode_pedal_command(second[0]).idx
  assert cs.pedal_first_enabled_mono_time == 123456789

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 7}, 40)
  controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0, now_nanos=222222222)
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 7}, 60)
  controller.update(cc, cs, frame=6, tesla_can=tesla_can, can_bus_party=0, now_nanos=333333333)
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 8}, 80)
  reacquired = controller.update(
    cc, cs, frame=8, tesla_can=tesla_can, can_bus_party=0, now_nanos=444444444,
  )
  assert _decode_pedal_command(reacquired[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)
  assert cs.pedal_first_enabled_mono_time == 123456789


@pytest.mark.parametrize("override", ["brake", "gas"])
def test_active_pedal_releases_once_then_stays_silent(controller_env, override):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  active = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert len(active) == 1
  assert _decode_pedal_command(active[0]).enabled

  if override == "brake":
    cs.enableLongControl = False
    cs.real_brake_pressed = True
  else:
    cc.longActive = False
    cs.out.gasPressed = True

  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert len(release) == 1
  assert not _decode_pedal_command(release[0]).enabled
  assert _decode_pedal_command(release[0]).raw_command == 0
  assert controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0) == []


def test_engage_while_brake_already_held_is_lateral_only():
  engagement = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  engagement.preap_brake_pressed_prev = True

  engagement.process_buttons(
    cruise_buttons=2,
    prev_cruise_buttons=0,
    curr_time_ms=1000,
    v_ego=15.0,
    speed_units="KPH",
    use_pedal=True,
    pedal_long_allowed=True,
    long_control_allowed=True,
    real_brake_pressed=True,
  )

  assert engagement.cruiseEnabled
  assert not engagement.enableLongControl
  assert engagement.enableJustCC


def test_timeout_rearm_requires_reset_then_advancing_healthy_feedback(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 7}, 20)
  cs.pedal_timeout = cs.pedal.timeout
  reset = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert len(reset) == 1
  assert not _decode_pedal_command(reset[0]).enabled
  assert _decode_pedal_command(reset[0]).raw_command == 0

  second_reset = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert len(second_reset) == 1
  assert not _decode_pedal_command(second_reset[0]).enabled

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 7}, 40)
  cs.pedal_timeout = cs.pedal.timeout
  third_reset = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert len(third_reset) == 1
  assert not _decode_pedal_command(third_reset[0]).enabled

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 8}, 60)
  cs.pedal_timeout = cs.pedal.timeout
  enabled = controller.update(cc, cs, frame=6, tesla_can=tesla_can, can_bus_party=0)
  assert len(enabled) == 1
  assert _decode_pedal_command(enabled[0]).enabled


def _start_gas_override(cc, cs):
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cs.out.gasPressed = True
  cs.out.aEgo = 1.618
  cs.pedal_interceptor_value = 0.0
  cc.longActive = False
  cc.actuators.accel = 0.34
  cc.orientationNED = [0.0, 0.05, 0.0]


def _hold_gas_override(controller, cc, cs, tesla_can):
  override_commands = []
  for frame in range(0, 460, 2):
    override_commands.extend(controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0))
  return override_commands


def test_requested_engage_during_gas_override_is_silent(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _start_gas_override(cc, cs)

  override_commands = _hold_gas_override(controller, cc, cs, tesla_can)

  assert override_commands == []


def test_engage_grace_starts_on_actual_long_active_rising(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _start_gas_override(cc, cs)
  _hold_gas_override(controller, cc, cs, tesla_can)

  cs.out.gasPressed = False
  cs.out.aEgo = 0.1
  cc.longActive = True
  controller.update(cc, cs, frame=460, tesla_can=tesla_can, can_bus_party=0)

  assert controller.preap_long_engage_frame == 460


@pytest.mark.parametrize("engage_a_max", (0.8, 0.9, 1.0))
def test_non_timeout_gas_override_release_has_no_launch_for_1p5_seconds(
    controller_env, monkeypatch, engage_a_max):
  controller, cc, cs, tesla_can = controller_env
  monkeypatch.setattr(
    'opendbc.car.tesla.preap.carcontroller.get_preap_accel_limits',
    lambda _v_ego: (-1.5, engage_a_max),
  )
  _start_gas_override(cc, cs)
  _hold_gas_override(controller, cc, cs, tesla_can)

  cs.out.gasPressed = False
  cs.out.aEgo = 0.1
  cc.longActive = True
  cc.actuators.accel = 0.67

  release_commands = []
  limited_acceleration_by_frame = {}
  for frame in range(460, 612, 2):
    commands = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    release_commands.extend(_decode_pedal_command(command) for command in commands)
    limited_acceleration_by_frame[frame] = controller.vdas.jerk_limiter.a_limited

  assert release_commands
  assert all(command.enabled for command in release_commands)
  command_steps = [
    current.command - previous.command
    for previous, current in zip(release_commands, release_commands[1:], strict=False)
  ]
  assert release_commands[0].command < 0.1
  assert max(command_steps) < 1.0, command_steps
  assert limited_acceleration_by_frame[508] < 0.4
  assert limited_acceleration_by_frame[610] == pytest.approx(cc.actuators.accel)
  assert not controller.preap_long_handoff_slew_active


def test_gas_override_timeout_rearm_has_no_launch(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _start_gas_override(cc, cs)
  _hold_gas_override(controller, cc, cs, tesla_can)

  cs.out.gasPressed = False
  cs.out.aEgo = 0.1
  cc.longActive = True
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 9}, 4600)
  cs.pedal_timeout = cs.pedal.timeout
  reset = controller.update(cc, cs, frame=460, tesla_can=tesla_can, can_bus_party=0)

  assert controller.preap_long_engage_frame < 0
  assert len(reset) == 1
  assert not _decode_pedal_command(reset[0]).enabled

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 9}, 4620)
  cs.pedal_timeout = cs.pedal.timeout
  second_reset = controller.update(cc, cs, frame=462, tesla_can=tesla_can, can_bus_party=0)
  assert len(second_reset) == 1
  assert not _decode_pedal_command(second_reset[0]).enabled

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 10}, 4640)
  cs.pedal_timeout = cs.pedal.timeout
  first_enabled = controller.update(cc, cs, frame=464, tesla_can=tesla_can, can_bus_party=0)
  assert len(first_enabled) == 1
  first_command = _decode_pedal_command(first_enabled[0])
  assert first_command.enabled
  assert controller.preap_long_engage_frame == 464
  assert first_command.command < PEDAL_RAMP_RATE_UP - 1.0
  assert controller.vdas.jerk_limiter.a_limited < 0.2
  assert controller.vdas.grade_estimator.pitch_lp.x > 0.02


def test_zero_torque_anchor_converges_without_a_command_step():
  zero_torque = PedalZeroTorque()

  # The configured actuator delay exceeds 0.48s, so no candidate may be
  # accepted before this observation window settles.
  for _ in range(24):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  assert zero_torque.value == pytest.approx(PEDAL_DI_ZERO)

  zero_torque.update(
    torque_level=-0.5,
    current_pedal_di=6.0,
    v_ego=17.0,
    control_active=True,
    accel_command=0.0,
  )
  assert zero_torque.value == pytest.approx(0.1)

  for _ in range(100):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  assert zero_torque.value == pytest.approx(6.0)


def test_zero_torque_anchor_ignores_inactive_pedal_feedback():
  zero_torque = PedalZeroTorque()

  for _ in range(100):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=False,
      accel_command=0.0,
    )

  assert zero_torque.value == pytest.approx(PEDAL_DI_ZERO)


@pytest.mark.parametrize(
  ("control_active", "accel_command"),
  ((False, 0.0), (True, 0.3)),
)
def test_zero_torque_anchor_freezes_across_invalid_observations(control_active, accel_command):
  zero_torque = PedalZeroTorque()

  for _ in range(25):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  frozen_value = zero_torque.value
  assert frozen_value == pytest.approx(0.1)

  for _ in range(100):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=control_active,
      accel_command=accel_command,
    )
  assert zero_torque.value == pytest.approx(frozen_value)

  # A newly valid observation must settle again before adaptation resumes.
  for _ in range(24):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  assert zero_torque.value == pytest.approx(frozen_value)

  zero_torque.update(
    torque_level=-0.5,
    current_pedal_di=6.0,
    v_ego=17.0,
    control_active=True,
    accel_command=0.0,
  )
  assert zero_torque.value == pytest.approx(frozen_value + 0.1)


def test_acceleration_trace_cannot_reanchor_zero_torque_or_hit_backstop(controller_env, monkeypatch):
  controller, cc, cs, tesla_can = controller_env
  zero_torque = PedalZeroTorque()
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

  _activate_longitudinal(cc, cs)
  cc.actuators.accel = 0.31
  commands = []
  for update_index, frame in enumerate(range(0, 360, 2)):
    cs.pedal.torque_level = -0.5 if update_index == 150 else -20.0
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    commands.extend(_decode_pedal_command(command).command for command in sent)

  command_steps = [current - previous for previous, current in zip(commands, commands[1:], strict=False)]
  consecutive_backstop_steps = any(
    first >= PEDAL_RAMP_RATE_UP - 0.1 and second >= PEDAL_RAMP_RATE_UP - 0.1
    for first, second in zip(command_steps, command_steps[1:], strict=False)
  )
  assert zero_torque.value == pytest.approx(PEDAL_DI_ZERO)
  assert not consecutive_backstop_steps


def test_max_regen_does_not_prompt_when_requested_decel_is_delivered(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = -1.5

  max_regen_prompted = False
  for frame in range(0, 400, 2):
    # Model delivered deceleration as a one-update response to the controller's
    # previous jerk-limited command, beginning from the vehicle's current zero.
    cs.out.aEgo = controller.vdas.jerk_limiter.a_limited
    # CarController clears edge events before each PreAP controller update.
    cs.pccEvent = None
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    max_regen_prompted |= cs.pccEvent == "pedalMaxRegen"

  assert controller.vdas.jerk_limiter.a_limited == pytest.approx(cc.actuators.accel)
  assert controller.prev_pedal_di < -4.75
  assert not max_regen_prompted


def test_controller_prompts_when_full_regen_under_delivers_while_moving(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = -1.5
  cs.out.aEgo = -0.5

  for frame in range(0, 400, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if cs.pedal_brake_required:
      break

  assert controller.prev_pedal_di <= -4.5
  assert controller.regen_decel_monitor.active
  assert cs.pedal_brake_required


def _update_regen_monitor(monitor, *, actual_accel=-0.5, v_ego=15.0,
                          pedal_di=-5.0, pedal_control_active=True):
  return monitor.update(
    pedal_control_active=pedal_control_active,
    in_engage_grace=False,
    pedal_di=pedal_di,
    limited_accel=-1.5,
    actual_accel=actual_accel,
    v_ego=v_ego,
  )


def test_regen_prompt_requires_sustained_unmet_decel_while_moving():
  monitor = RegenDecelMonitor()

  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES - 1):
    assert not _update_regen_monitor(monitor)

  assert _update_regen_monitor(monitor)


def test_regen_prompt_does_not_fire_at_standstill():
  monitor = RegenDecelMonitor()

  for _ in range(2 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    assert not _update_regen_monitor(monitor, v_ego=0.0)


def test_regen_prompt_uses_hysteresis_and_clears_when_decel_recovers():
  monitor = RegenDecelMonitor()
  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES):
    _update_regen_monitor(monitor)
  assert monitor.active

  # These values have crossed back over the trigger thresholds, but remain
  # inside the clear thresholds so sensor noise cannot chatter the prompt.
  assert _update_regen_monitor(monitor, actual_accel=-1.25, v_ego=1.5, pedal_di=-4.25)

  assert not _update_regen_monitor(monitor, actual_accel=-1.35)
  assert not monitor.active


def test_regen_prompt_clears_as_soon_as_pedal_control_releases():
  monitor = RegenDecelMonitor()
  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES):
    _update_regen_monitor(monitor)
  assert monitor.active

  assert not _update_regen_monitor(monitor, pedal_control_active=False)


def test_regen_prompt_survives_brief_shortfall_dropouts():
  # Acceleration-estimate noise dips the shortfall below the trigger for a
  # single update every so often. Accumulated evidence must survive those
  # dropouts instead of restarting from zero.
  monitor = RegenDecelMonitor()
  fired = False
  for _ in range(3 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    for _ in range(9):
      fired = _update_regen_monitor(monitor) or fired
    # shortfall 0.3 m/s²: below the trigger, above the clear threshold
    fired = _update_regen_monitor(monitor, actual_accel=-1.2) or fired
    if fired:
      break
  assert fired


def test_regen_prompt_requires_deep_regen_command():
  # A command still in the shallow-regen range has authority left; the
  # driver does not need the brake yet.
  monitor = RegenDecelMonitor()
  for _ in range(3 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    assert not _update_regen_monitor(monitor, pedal_di=-1.5)


def _replay_fixture(name):
  with open(Path(__file__).parent / 'data' / name) as f:
    return json.load(f)['samples']


def _replay_monitor(samples):
  # Drive the monitor the way PreAPLongController wires it: driver pedal
  # input or authority loss resets, everything else goes through update().
  monitor = RegenDecelMonitor()
  first_active = None
  for i, (v_ego, a_ego, limited_accel, pedal_di, brake, gas, long_active) in enumerate(samples):
    if not long_active or brake or gas:
      monitor.reset()
      continue
    active = monitor.update(
      pedal_control_active=True,
      in_engage_grace=False,
      pedal_di=pedal_di,
      limited_accel=limited_accel,
      actual_accel=a_ego,
      v_ego=v_ego,
    )
    if active and first_active is None:
      first_active = i
  return first_active


def test_regen_prompt_fires_on_captured_weak_regen_drive():
  # 2026-07-22 field capture, minutes after supercharging: regen delivered
  # only -0.3 m/s² of a ~-1.0 m/s² request for several seconds while the
  # command walked down to the regen rail and the driver never braked.
  samples = _replay_fixture('regen_underdelivery_50hz.json')
  assert _replay_monitor(samples) is not None


def test_regen_prompt_stays_silent_on_captured_grade_decel():
  # Same drive, mountain uphill: deceleration requests are delivered through
  # grade compensation with the pedal still in the propulsion range.
  samples = _replay_fixture('grade_decel_positive_di_50hz.json')
  assert _replay_monitor(samples) is None
