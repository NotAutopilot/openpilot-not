"""Tests for VirtualDAS: JerkLimiter, feedforward, and inner PID."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.tesla.preap.ff_table_default import (
  SPEED_BP as FF_SPEED_BP,
  ACCEL_BP as FF_ACCEL_BP,
  DEFAULT_TABLE as FF_DEFAULT_TABLE,
)
from opendbc.car.tesla.preap.constants import (
  VDAS_EGO_JERK_MAX, VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V,
)
from opendbc.car.tesla.preap.virtual_das import FeedforwardModel, JerkLimiter, VirtualDAS
from opendbc.car.tesla.preap.nap_conf import (
  PEDAL_DI_MIN, PEDAL_DI_ZERO, ACCEL_MAX, REGEN_MAX,
  PEDAL_BP, PEDAL_MAX_VALUES,
)
from opendbc.car.tesla.pedal.controller import (
  PEDAL_RAMP_RATE_UP, PEDAL_RAMP_RATE_DOWN,
)

COMFORT_SNAP_MAX = 4.0  # m/s^4
HIGH_SPEED_FALLBACK_FIELD_SCALE = 0.65


# --- Phase 1: JerkLimiter ---

class TestJerkLimiter:

  def test_default_limits_positive_steps_more_than_negative_steps(self):
    jl = JerkLimiter(dt=0.02)

    positive_step = jl.update(1.0)
    jl.reset()
    negative_step = jl.update(-1.0)

    assert 0.0 < positive_step < 0.02
    assert negative_step == pytest.approx(-0.05)

  def test_positive_acceleration_transition_bounds_jerk_and_snap(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]

    for _ in range(80):
      accelerations.append(jl.update(1.0))

    jerks = np.diff(accelerations) / dt
    snaps = np.diff(np.concatenate(([0.0], jerks))) / dt
    assert np.max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9

  def test_step_response_bounded(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    da_max = 2.5 * 0.02

    prev = 0.0
    for _ in range(100):
      out = jl.update(2.0)
      assert abs(out - prev) <= da_max + 1e-9
      prev = out

  def test_step_response_reaches_target(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    for _ in range(200):
      out = jl.update(1.5)
    assert abs(out - 1.5) < 1e-6

  def test_snap_bounded_ramp_tracking_has_bounded_lag(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    slope = 1.0
    outputs = []
    lags = []

    for i in range(50):
      target = slope * i * 0.02
      out = jl.update(target)
      outputs.append(out)
      lags.append(target - out)

    assert np.all(np.diff(outputs) >= -1e-9)
    assert min(lags) >= -1e-9
    assert max(lags) < 0.13

  def test_ramp_tracking_above_jmax(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    slope = 5.0

    prev = 0.0
    for i in range(50):
      target = slope * i * 0.02
      out = jl.update(target)
      assert abs(out - prev) <= 2.5 * 0.02 + 1e-9
      prev = out

  def test_negative_step(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    jl.a_limited = 1.0

    prev = 1.0
    for _ in range(100):
      out = jl.update(-1.5)
      assert abs(out - prev) <= 2.5 * 0.02 + 1e-9
      prev = out

  def test_negative_target_converges_without_crossing(self):
    dt = 0.02
    target = -1.0
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]

    for _ in range(160):
      accelerations.append(jl.update(target))

    braking_jerks = np.diff(accelerations) / dt
    assert min(accelerations) >= target - 1e-9
    assert accelerations[-1] == pytest.approx(target)
    assert min(braking_jerks) >= -2.5 - 1e-9

  def test_lower_target_during_positive_transition_keeps_immediate_braking_authority(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    for _ in range(10):
      jl.update(2.0)

    before_braking = jl.a_limited
    after_braking = jl.update(before_braking - 1.0)

    assert (after_braking - before_braking) / dt == pytest.approx(-2.5)

  def test_negative_to_positive_reversal_remains_bounded(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    jl.reset(a_init=-0.5)
    accelerations = [-0.5]

    for _ in range(120):
      accelerations.append(jl.update(1.0))

    jerks = np.diff(accelerations) / dt
    snaps = np.diff(np.concatenate(([0.0], jerks))) / dt
    assert max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9
    assert accelerations[-1] == pytest.approx(1.0)

  def test_positive_jerk_restarts_from_applied_hold_state(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    for _ in range(10):
      jl.update(2.0)

    near_target = jl.a_limited + 0.001
    for _ in range(200):
      jl.update(near_target)

    before_hold = jl.a_limited
    held = jl.update(near_target)
    resumed = jl.update(1.0)

    hold_jerk = (held - before_hold) / dt
    resumed_jerk = (resumed - held) / dt
    assert held == pytest.approx(near_target)
    assert hold_jerk == pytest.approx(0.0)
    assert (resumed_jerk - hold_jerk) / dt <= COMFORT_SNAP_MAX + 1e-9

  def test_closer_positive_target_preserves_applied_snap_bound(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]
    for _ in range(10):
      accelerations.append(jl.update(2.0))

    near_target = accelerations[-1] + 0.001
    accelerations.append(jl.update(near_target))

    jerks = np.diff(accelerations) / dt
    clamp_snap = (jerks[-1] - jerks[-2]) / dt
    assert jerks[-2] == pytest.approx(0.8)
    assert jerks[-1] <= 1.0 + 1e-9
    assert abs(clamp_snap) <= COMFORT_SNAP_MAX + 1e-9

  def test_lower_target_after_positive_overshoot_uses_immediate_braking_path(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    for _ in range(10):
      jl.update(2.0)

    near_target = jl.a_limited + 0.001
    overshot_accel = jl.update(near_target)
    lower_target = near_target - 0.001
    after_braking = jl.update(lower_target)
    braking_jerk = (after_braking - overshot_accel) / dt
    expected_immediate_jerk = max((lower_target - overshot_accel) / dt, -2.5)

    assert overshot_accel > near_target
    assert braking_jerk == pytest.approx(expected_immediate_jerk)
    assert braking_jerk >= -2.5 - 1e-9
    assert after_braking >= lower_target - 1e-9

  def test_piecewise_positive_targets_converge_without_runaway_or_oscillation(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]

    for _ in range(10):
      accelerations.append(jl.update(2.0))

    first_close_target = jl.a_limited + 0.001
    first_close_start = len(accelerations) - 1
    for _ in range(200):
      accelerations.append(jl.update(first_close_target))

    for _ in range(13):
      accelerations.append(jl.update(0.8))

    second_close_target = jl.a_limited + 0.003
    second_close_start = len(accelerations) - 1
    for _ in range(200):
      accelerations.append(jl.update(second_close_target))

    jerks = np.diff(accelerations) / dt
    snaps = np.diff(np.concatenate(([0.0], jerks))) / dt
    first_close_end = first_close_start + 201
    first_close_accels = np.array(accelerations[first_close_start:first_close_end])
    second_close_accels = np.array(accelerations[second_close_start:])
    first_overshoot = np.max(first_close_accels) - first_close_target
    second_overshoot = np.max(second_close_accels) - second_close_target
    first_nonzero_errors = first_close_accels[np.abs(first_close_accels - first_close_target) > 1e-12] - first_close_target
    second_nonzero_errors = second_close_accels[np.abs(second_close_accels - second_close_target) > 1e-12] - second_close_target
    first_crossings = np.count_nonzero(np.diff(np.sign(first_nonzero_errors)))
    second_crossings = np.count_nonzero(np.diff(np.sign(second_nonzero_errors)))

    assert np.max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9
    assert first_overshoot < 0.1
    assert second_overshoot < 0.14
    assert first_crossings <= 1
    assert second_crossings <= 1
    assert accelerations[first_close_start + 200] == pytest.approx(first_close_target)
    assert accelerations[-1] == pytest.approx(second_close_target)

  def test_reset(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    jl.update(2.0)
    jl.update(2.0)
    assert jl.a_limited > 0
    jl.reset(a_init=0.5)
    assert jl.a_limited == 0.5

  def test_reset_default(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    for _ in range(10):
      jl.update(1.0)
    jl.reset()
    assert jl.a_limited == 0.0


# --- Shared fixtures for VirtualDAS tests ---

@pytest.fixture()
def mock_nap_conf(monkeypatch):
  mock_conf = SimpleNamespace(get_pedal_profile_values=lambda: PEDAL_MAX_VALUES)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', mock_conf)
  return mock_conf


@pytest.fixture()
def mock_zero_torque(monkeypatch):
  mock_zt = SimpleNamespace(get=lambda _v_ego: PEDAL_DI_ZERO)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: mock_zt)
  return mock_zt


# --- Phase 1: VirtualDAS feedforward + jerk limiter ---

class TestVirtualDAS:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_steady_state_zero_accel(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(0.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_ZERO) < 1e-3

  def test_steady_state_max_accel(self):
    vdas = VirtualDAS(dt=0.02)
    physical_max = float(np.interp(15.0, PEDAL_BP, PEDAL_MAX_VALUES))
    for _ in range(500):
      di = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di == pytest.approx(physical_max * HIGH_SPEED_FALLBACK_FIELD_SCALE, abs=0.01)
    assert di < physical_max

  def test_steady_state_max_regen(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(500):
      di = vdas.update(REGEN_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_MIN) < 0.5

  def test_jerk_limiting_active_on_step(self):
    vdas = VirtualDAS(dt=0.02)
    di_first = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=0.0)
    expected_max = float(np.interp(15.0, PEDAL_BP, PEDAL_MAX_VALUES))
    assert di_first < expected_max * 0.5

  def test_rate_limit_backstop(self):
    vdas = VirtualDAS(dt=0.02)
    prev = 0.0
    for _ in range(100):
      di = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=prev)
      assert di - prev <= PEDAL_RAMP_RATE_UP + 1e-9
      assert prev - di <= PEDAL_RAMP_RATE_DOWN + 1e-9
      prev = di

  def test_lead_expiry_negative_to_positive_target_is_s_curve_shaped_before_di_backstop(self):
    dt = 0.02
    vdas = VirtualDAS(dt=dt)
    v_ego = 15.0
    regen_target = -0.4
    departure_target = 0.34
    previous_di = vdas._feedforward(0.0, v_ego)
    vdas.reset(measured_accel=0.0, commanded_accel=0.0, pedal_di_init=previous_di)
    braking_accelerations = [vdas.jerk_limiter.a_limited]

    # Model a lead disappearing while regen is still ramping in, rather than
    # after the requested deceleration has already settled to zero jerk.
    for _ in range(7):
      previous_di = vdas.update(
        regen_target,
        v_ego=v_ego,
        prev_pedal_di=previous_di,
        a_ego=0.0,
        freeze_integrator=True,
      )
      braking_accelerations.append(vdas.jerk_limiter.a_limited)

    pre_departure_jerk = (braking_accelerations[-1] - braking_accelerations[-2]) / dt
    assert pre_departure_jerk == pytest.approx(-2.5)

    limited_accelerations = [vdas.jerk_limiter.a_limited]
    pedal_commands = [previous_di]
    for _ in range(200):
      previous_di = vdas.update(
        departure_target,
        v_ego=v_ego,
        prev_pedal_di=previous_di,
        a_ego=limited_accelerations[0],
        freeze_integrator=True,
      )
      limited_accelerations.append(vdas.jerk_limiter.a_limited)
      pedal_commands.append(previous_di)

    jerks = np.diff(limited_accelerations) / dt
    snaps = np.diff(np.concatenate(([pre_departure_jerk], jerks))) / dt
    pedal_steps = np.diff(pedal_commands)

    assert np.max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9
    assert np.max(pedal_steps) < PEDAL_RAMP_RATE_UP - 1e-6
    assert limited_accelerations[-1] == pytest.approx(departure_target)

  def test_reset_clears_state(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(50):
      vdas.update(2.0, v_ego=20.0, prev_pedal_di=vdas.prev_pedal_di)

    vdas.reset(measured_accel=0.0, commanded_accel=0.0, pedal_di_init=5.0)
    assert vdas.jerk_limiter.a_limited == 0.0
    assert vdas.prev_pedal_di == 5.0
    assert vdas.inner_pid.i == 0.0

  @pytest.mark.parametrize("measured_accel", [-0.176, 2.041])
  def test_reset_neutral_command_does_not_replay_measured_acceleration(self, measured_accel):
    vdas = VirtualDAS(dt=0.02)

    vdas.reset(measured_accel=measured_accel, commanded_accel=0.0)
    first_limited_accel = vdas.jerk_limiter.update(0.0)

    assert vdas.a_ego_filter.x == pytest.approx(measured_accel)
    assert vdas.prev_a_ego_filtered == pytest.approx(measured_accel)
    assert first_limited_accel == pytest.approx(0.0)

  def test_observe_does_not_mutate_commanded_jerk_state(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(10):
      vdas.jerk_limiter.update(1.0)
    state_before_observation = (
      vdas.jerk_limiter.a_limited,
      vdas.jerk_limiter.j_limited,
      vdas.jerk_limiter.state,
      vdas.jerk_limiter.target_accel,
    )

    vdas.observe(-0.8)

    assert (
      vdas.jerk_limiter.a_limited,
      vdas.jerk_limiter.j_limited,
      vdas.jerk_limiter.state,
      vdas.jerk_limiter.target_accel,
    ) == state_before_observation

  def test_positive_measurement_does_not_make_first_negative_command_positive(self):
    vdas = VirtualDAS(dt=0.02)
    vdas.reset(measured_accel=2.041, commanded_accel=0.0)

    vdas.update(
      -0.487,
      v_ego=15.0,
      prev_pedal_di=vdas.prev_pedal_di,
      a_ego=2.041,
      freeze_integrator=True,
    )

    assert vdas.jerk_limiter.a_limited < 0.0

  def test_reset_preserves_pedal_coast_seed(self):
    vdas = VirtualDAS(dt=0.02)
    coast_pedal_di = 3.25

    vdas.reset(measured_accel=-0.176, commanded_accel=0.0, pedal_di_init=coast_pedal_di)

    assert vdas.prev_pedal_di == pytest.approx(coast_pedal_di)

  def test_small_accel_near_zero(self):
    """Small accel produces a small positive DI near zero-torque (smooth interp, no cliff)."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(0.05, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di > PEDAL_DI_ZERO - 1.0
    assert di < PEDAL_DI_ZERO + 3.0

  def test_negative_accel_produces_regen(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(-1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di < PEDAL_DI_ZERO

  def test_field_correction_is_continuous_from_five_to_thirty_mps(self):
    vdas_slow = VirtualDAS(dt=0.02)
    vdas_fast = VirtualDAS(dt=0.02)

    for _ in range(500):
      di_slow = vdas_slow.update(ACCEL_MAX, v_ego=5.0, prev_pedal_di=vdas_slow.prev_pedal_di)
      di_fast = vdas_fast.update(ACCEL_MAX, v_ego=30.0, prev_pedal_di=vdas_fast.prev_pedal_di)

    low_speed_physical_max = float(np.interp(5.0, PEDAL_BP, PEDAL_MAX_VALUES))
    high_speed_physical_max = float(np.interp(30.0, PEDAL_BP, PEDAL_MAX_VALUES))
    assert di_slow == pytest.approx(
      low_speed_physical_max * HIGH_SPEED_FALLBACK_FIELD_SCALE,
      abs=0.01,
    )
    assert di_fast == pytest.approx(
      high_speed_physical_max * HIGH_SPEED_FALLBACK_FIELD_SCALE,
      abs=0.01,
    )


# --- Phase 2: Inner PID + delay compensation ---

def _simulate_plant(vdas, a_cmd, v_ego, dt, n_steps, plant_delay_steps=15, plant_tau=0.2):
  """Simulate VirtualDAS driving a first-order plant with delay.

  Plant model: a_actual follows pedal_di through a first-order lag (tau)
  with a pure transport delay. This is a simplified model of the
  pedal → inverter → motor → acceleration chain.
  """
  delay_buffer = [0.0] * plant_delay_steps
  a_actual = 0.0
  alpha = dt / (plant_tau + dt)

  max_pedal = float(np.interp(v_ego, PEDAL_BP, PEDAL_MAX_VALUES))
  di_to_accel = ACCEL_MAX / max(max_pedal, 1.0)

  history = []
  for _ in range(n_steps):
    pedal_di = vdas.update(
      a_cmd, v_ego, vdas.prev_pedal_di,
      a_ego=a_actual, freeze_integrator=False)

    delayed_di = delay_buffer.pop(0)
    delay_buffer.append(pedal_di)

    target_accel = delayed_di * di_to_accel
    a_actual += alpha * (target_accel - a_actual)

    history.append({'pedal_di': pedal_di, 'a_actual': a_actual, 'a_cmd': a_cmd})

  return history


class TestInnerPID:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_pid_correction_reduces_steady_state_error(self):
    """With feedback from the plant, system should settle near the target."""
    vdas = VirtualDAS(dt=0.02)
    hist = _simulate_plant(vdas, a_cmd=1.0, v_ego=15.0, dt=0.02, n_steps=500)
    final_error = abs(hist[-1]['a_actual'] - 1.0)
    assert final_error < 0.5, f"Steady-state error too large: {final_error}"

  def test_settling_time(self):
    """System should settle within 3 seconds for a 1 m/s² step."""
    vdas = VirtualDAS(dt=0.02)
    hist = _simulate_plant(vdas, a_cmd=1.0, v_ego=15.0, dt=0.02, n_steps=300)

    settled = False
    for i in range(len(hist) - 10):
      window = hist[i:i+10]
      if all(abs(h['a_actual'] - 1.0) < 0.3 for h in window):
        settle_time = i * 0.02
        settled = True
        break

    assert settled, "System did not settle within 6 seconds"
    assert settle_time < 3.0, f"Settled at {settle_time:.2f}s, expected < 3.0s"

  def test_inner_feedback_holds_cruise_against_sustained_road_load(self, monkeypatch):
    coast_pedal_di = 3.0
    accel_per_di_mps2 = 0.063
    road_load_accel_mps2 = -0.40
    steady_error_bound_mps2 = 0.105
    settling_time_bound_s = 7.0
    simulation_dt_s = 0.02
    simulation_steps = round(30.0 / simulation_dt_s)
    steady_window_steps = round(2.0 / simulation_dt_s)
    delay_steps = round(0.40 / simulation_dt_s)
    plant_tau_s = 0.25
    plant_alpha = simulation_dt_s / (plant_tau_s + simulation_dt_s)
    zero_torque = SimpleNamespace(get=lambda _v_ego: coast_pedal_di)
    monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

    vdas = VirtualDAS(dt=simulation_dt_s)
    vdas.reset(measured_accel=0.0, commanded_accel=0.0, pedal_di_init=coast_pedal_di)
    real_feedforward = vdas._feedforward
    acceleration_efforts_mps2 = []

    def record_feedforward(acceleration_effort_mps2, v_ego):
      acceleration_efforts_mps2.append(acceleration_effort_mps2)
      return real_feedforward(acceleration_effort_mps2, v_ego)

    monkeypatch.setattr(vdas, '_feedforward', record_feedforward)
    delayed_pedal_di = [coast_pedal_di] * delay_steps
    initial_speed_mps = 30.0
    v_ego = initial_speed_mps
    a_ego = 0.0
    acceleration_errors_mps2 = []
    pedal_commands_di = []
    integrated_speed_change_mps = 0.0

    for _ in range(simulation_steps):
      pedal_di = vdas.update(
        0.0,
        v_ego=v_ego,
        prev_pedal_di=vdas.prev_pedal_di,
        a_ego=a_ego,
        freeze_integrator=False,
      )
      applied_pedal_di = delayed_pedal_di.pop(0)
      delayed_pedal_di.append(pedal_di)
      plant_target_accel_mps2 = (
        (applied_pedal_di - coast_pedal_di) * accel_per_di_mps2
        + road_load_accel_mps2
      )
      a_ego += plant_alpha * (plant_target_accel_mps2 - a_ego)
      integrated_speed_change_mps += a_ego * simulation_dt_s
      v_ego += a_ego * simulation_dt_s
      acceleration_errors_mps2.append(abs(a_ego))
      pedal_commands_di.append(pedal_di)

    steady_errors_mps2 = acceleration_errors_mps2[-steady_window_steps:]
    post_settling_errors_mps2 = acceleration_errors_mps2[
      round(settling_time_bound_s / simulation_dt_s):
    ]
    steady_efforts_mps2 = acceleration_efforts_mps2[-steady_window_steps:]
    steady_pedals_di = pedal_commands_di[-steady_window_steps:]
    max_pedal_di = float(np.interp(v_ego, PEDAL_BP, PEDAL_MAX_VALUES))
    assert v_ego > 0.0
    assert v_ego == pytest.approx(initial_speed_mps + integrated_speed_change_mps)
    assert min(steady_efforts_mps2) > REGEN_MAX
    assert max(steady_efforts_mps2) < ACCEL_MAX
    assert min(steady_pedals_di) > PEDAL_DI_MIN
    assert max(steady_pedals_di) < max_pedal_di
    assert max(steady_errors_mps2) <= steady_error_bound_mps2, (
      f"cruise-hold error reached {max(steady_errors_mps2):.3f} m/s^2 under "
      + f"{abs(road_load_accel_mps2):.2f} m/s^2 sustained road load; "
      + f"speed fell to {v_ego:.2f} m/s with {vdas.inner_pid.i:.3f} m/s^2 integral correction"
    )
    assert max(post_settling_errors_mps2) <= steady_error_bound_mps2, (
      f"cruise-hold error had not settled below {steady_error_bound_mps2:.2f} m/s^2 "
      + f"within {settling_time_bound_s:.1f}s; max later error was "
      + f"{max(post_settling_errors_mps2):.3f} m/s^2"
    )

  def test_integrator_freeze_during_grace(self):
    """Integrator should not accumulate during engage grace period."""
    vdas = VirtualDAS(dt=0.02)

    for _ in range(50):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.0, freeze_integrator=True)

    assert abs(vdas.inner_pid.i) < 1e-9

  def test_integrator_accumulates_after_grace(self):
    """After grace period ends, integrator should start correcting."""
    vdas = VirtualDAS(dt=0.02)

    # Grace period: frozen
    for _ in range(50):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.0, freeze_integrator=True)
    assert abs(vdas.inner_pid.i) < 1e-9

    # After grace: should accumulate
    for _ in range(100):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.0, freeze_integrator=False)
    assert abs(vdas.inner_pid.i) > 0.01

  @pytest.mark.parametrize("persistent_error_mps2", [-0.09, -0.0196, 0.0196, 0.09])
  def test_persistent_sub_deadband_error_earns_residual_authority(self, persistent_error_mps2):
    vdas = VirtualDAS(dt=0.02)

    for _ in range(round(5.0 / vdas.dt)):
      vdas.update(
        persistent_error_mps2,
        v_ego=25.0,
        prev_pedal_di=vdas.prev_pedal_di,
        a_ego=0.0,
        freeze_integrator=False,
      )

    assert abs(vdas.inner_pid.i) > 0.005
    assert vdas.inner_pid.i * persistent_error_mps2 > 0.0

  def test_sub_deadband_sign_changing_noise_does_not_accumulate_residual_authority(self):
    vdas = VirtualDAS(dt=0.02)
    maximum_integral_mps2 = 0.0

    for step in range(round(10.0 / vdas.dt)):
      noisy_command_mps2 = 0.09 if (step // 25) % 2 == 0 else -0.09
      vdas.update(
        noisy_command_mps2,
        v_ego=25.0,
        prev_pedal_di=vdas.prev_pedal_di,
        a_ego=0.0,
        freeze_integrator=False,
      )
      maximum_integral_mps2 = max(maximum_integral_mps2, abs(vdas.inner_pid.i))

    assert maximum_integral_mps2 < 1e-9

  def test_sign_changing_sub_deadband_error_never_completes_dwell(self):
    vdas = VirtualDAS(dt=0.02)

    selected_errors_mps2 = [
      vdas._gate_pid_error_noise(
        0.09 if (step // 5) % 2 == 0 else -0.09,
        freeze_integrator=False,
      )
      for step in range(round(10.0 / vdas.dt))
    ]

    assert selected_errors_mps2 == pytest.approx(np.zeros(len(selected_errors_mps2)))

  def test_persistent_error_dwell_restarts_after_freeze_observe_and_reset(self):
    vdas = VirtualDAS(dt=0.02)

    def hold_small_error(duration_s, freeze_integrator=False):
      for _ in range(round(duration_s / vdas.dt)):
        vdas.update(
          0.09,
          v_ego=25.0,
          prev_pedal_di=vdas.prev_pedal_di,
          a_ego=0.0,
          freeze_integrator=freeze_integrator,
        )

    hold_small_error(0.8)
    hold_small_error(vdas.dt, freeze_integrator=True)
    hold_small_error(0.4)
    assert vdas.inner_pid.i == 0.0

    hold_small_error(1.0)
    assert vdas.inner_pid.i > 0.0
    vdas.observe(a_ego=0.0)
    hold_small_error(0.4)
    assert vdas.inner_pid.i == 0.0

    hold_small_error(1.0)
    assert vdas.inner_pid.i > 0.0
    vdas.reset(measured_accel=0.0, commanded_accel=0.09)
    hold_small_error(0.4)
    assert vdas.inner_pid.i == 0.0

  def test_anti_windup(self):
    """Integrator should be bounded by acceleration-map authority."""
    vdas = VirtualDAS(dt=0.02)

    for _ in range(2000):
      vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=-1.0, freeze_integrator=False)

    assert vdas.inner_pid.i <= ACCEL_MAX + 1e-9
    assert vdas.inner_pid.i >= REGEN_MAX - 1e-9

  def test_reset_clears_pid_state(self):
    """Reset should zero out the inner PID and filter state."""
    vdas = VirtualDAS(dt=0.02)

    for _ in range(100):
      vdas.update(2.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.5)

    assert abs(vdas.inner_pid.i) > 0.01
    assert abs(vdas.a_ego_filter.x) > 0

    vdas.reset()
    assert vdas.inner_pid.i == 0.0
    assert vdas.inner_pid.p == 0.0
    assert vdas.a_ego_filter.x == 0.0
    assert vdas.prev_a_ego_filtered == 0.0

  def test_no_feedback_graceful(self):
    """With a_ego=0 (no sensor), VirtualDAS still produces valid output."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                       a_ego=0.0)
    assert di > PEDAL_DI_ZERO
    assert np.isfinite(di)

  def test_matched_feedback_no_correction(self):
    """When a_ego matches a_cmd, PID correction should be near zero."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=1.0)
    assert abs(vdas.inner_pid.i) < 0.5

  def test_backward_compat_no_a_ego_arg(self):
    """Calling update() without a_ego still works (defaults to 0.0)."""
    vdas = VirtualDAS(dt=0.02)
    di = vdas.update(1.0, v_ego=15.0, prev_pedal_di=0.0)
    assert np.isfinite(di)

  def test_prediction_clamps_one_frame_acceleration_spike(self, monkeypatch):
    vdas = VirtualDAS(dt=0.02)
    feedforward_inputs_mps2 = []
    real_feedforward = vdas._feedforward

    def record_feedforward(acceleration_effort_mps2, v_ego):
      feedforward_inputs_mps2.append(acceleration_effort_mps2)
      return real_feedforward(acceleration_effort_mps2, v_ego)

    monkeypatch.setattr(vdas, '_feedforward', record_feedforward)

    vdas.update(
      0.0, v_ego=15.0, prev_pedal_di=0.0,
      a_ego=100.0, freeze_integrator=False,
    )

    future_time_s = float(np.interp(15.0, VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V))
    expected_future_accel_mps2 = vdas.a_ego_filter.x + VDAS_EGO_JERK_MAX * future_time_s
    expected_trim_mps2 = -expected_future_accel_mps2 * vdas.inner_pid.k_i * vdas.dt
    assert vdas.inner_pid.i == pytest.approx(expected_trim_mps2)
    assert feedforward_inputs_mps2 == pytest.approx([expected_trim_mps2])


# --- Phase 3: FeedforwardModel ---

class TestFeedforwardModel:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_default_table_only_scales_positive_branch_above_zero_speed(self):
    """The field correction must not alter regen or the 0 m/s fallback."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel
    from opendbc.car.tesla.preap.ff_table_default import SPEED_BP, ACCEL_BP

    ff = FeedforwardModel(table_path="/nonexistent")

    for speed in SPEED_BP:
      max_pedal = float(np.interp(speed, PEDAL_BP, PEDAL_MAX_VALUES))
      for accel in ACCEL_BP:
        expected = float(np.interp(accel,
                                   [REGEN_MAX, 0.0, ACCEL_MAX],
                                   [PEDAL_DI_MIN, 0.0, max_pedal]))
        if speed >= 5.0 and accel > 0.0:
          expected *= HIGH_SPEED_FALLBACK_FIELD_SCALE
        got = ff.get(accel, speed, zero_torque_di=0.0)
        assert got == pytest.approx(expected, abs=0.01), (
          f"Mismatch at speed={speed}, accel={accel}: got={got:.2f}, expected={expected:.2f}"
        )

  def test_zero_torque_shift_positive_accel(self):
    """Positive accel zt offset fades: full at accel=0, zero at ACCEL_MAX."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    ff = FeedforwardModel(table_path="/nonexistent")
    # At accel=1.0, blend = 1 - 1.0/2.5 = 0.6, so offset = 2.0 * 0.6 = 1.2
    di_zero_zt = ff.get(1.0, 15.0, zero_torque_di=0.0)
    di_with_zt = ff.get(1.0, 15.0, zero_torque_di=2.0)
    assert abs((di_with_zt - di_zero_zt) - 1.2) < 0.2
    # At ACCEL_MAX, offset should be zero
    di_max_zero = ff.get(ACCEL_MAX, 15.0, zero_torque_di=0.0)
    di_max_zt = ff.get(ACCEL_MAX, 15.0, zero_torque_di=2.0)
    assert abs(di_max_zt - di_max_zero) < 0.1

  def test_zero_torque_shift_at_max_regen(self):
    """At max regen, zero-torque offset should blend to zero."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    ff = FeedforwardModel(table_path="/nonexistent")
    di_zero_zt = ff.get(REGEN_MAX, 15.0, zero_torque_di=0.0)
    di_with_zt = ff.get(REGEN_MAX, 15.0, zero_torque_di=2.0)
    assert abs(di_with_zt - di_zero_zt) < 0.1

  def test_small_accel_smooth_near_zero_torque(self):
    """Small accel produces a value near zero-torque via smooth interp (no cliff)."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    ff = FeedforwardModel(table_path="/nonexistent")
    di = ff.get(0.05, 15.0, zero_torque_di=3.0)
    assert abs(di - 3.0) < 2.0  # near zero-torque, not a big jump

  def test_json_override_loads(self, tmp_path):
    """Custom JSON table overrides the default."""
    import json
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    custom = {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [
        [-5.0, 0.0, 80.0],
        [-5.0, 0.0, 80.0],
      ],
    }
    path = tmp_path / "ff_table.json"
    path.write_text(json.dumps(custom))

    ff = FeedforwardModel(table_path=str(path))
    assert ff.speed_bp == [0.0, 40.0]
    di = ff.get(2.5, 20.0, zero_torque_di=0.0)
    assert abs(di - 80.0) < 0.5

  def test_invalid_json_falls_back_to_default(self, tmp_path):
    """Corrupted JSON file should fall back to defaults."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel
    from opendbc.car.tesla.preap.ff_table_default import SPEED_BP

    path = tmp_path / "bad.json"
    path.write_text("{invalid json")

    ff = FeedforwardModel(table_path=str(path))
    assert ff.speed_bp == list(SPEED_BP)

  @pytest.mark.parametrize("invalid_override", [
    {
      'speed_bp': [0.0, float('nan')],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 10 ** 400],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 0.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 2.5, 0.0],
      'table': [[-5.0, 80.0, 0.0], [-5.0, 80.0, 0.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, float('nan'), 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 1.0, 0.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.1, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 90.1], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [10.0, 30.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 70.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-0.5, 0.0, 0.5],
      'table': [[-5.0, 0.0, 70.0], [-5.0, 0.0, 80.0]],
    },
  ], ids=[
    "nonfinite-breakpoint",
    "overflowing-breakpoint",
    "duplicate-breakpoint",
    "unordered-breakpoint",
    "missing-row",
    "ragged-row",
    "nonfinite-row",
    "nonmonotonic-row",
    "below-minimum-di",
    "above-maximum-di",
    "incomplete-speed-coverage",
    "incomplete-acceleration-coverage",
  ])
  def test_invalid_override_tables_fall_back_to_defaults(self, tmp_path, invalid_override):
    path = tmp_path / "invalid_table.json"
    path.write_text(json.dumps(invalid_override))

    ff = FeedforwardModel(table_path=str(path))

    assert ff.speed_bp == list(FF_SPEED_BP)
    assert ff.accel_bp == list(FF_ACCEL_BP)
    assert ff.table == [list(row) for row in FF_DEFAULT_TABLE]

  def test_zero_torque_transition_is_monotonic_and_c1_across_speeds(self):
    ff = FeedforwardModel(table_path="/nonexistent")
    step = 1e-4

    for speed in np.linspace(0.0, 40.0, 9):
      for zero_torque_di in (0.0, 3.0, 5.0):
        samples = [ff.get(accel, speed, zero_torque_di)
                   for accel in np.linspace(-0.3, 0.3, 121)]
        assert all(right >= left for left, right in zip(samples, samples[1:], strict=False))

        for accel in (-0.25, 0.0, 0.25):
          center = ff.get(accel, speed, zero_torque_di)
          slope_left = (center - ff.get(accel - step, speed, zero_torque_di)) / step
          slope_right = (ff.get(accel + step, speed, zero_torque_di) - center) / step
          assert slope_left == pytest.approx(slope_right, abs=0.05), (
            f"speed={speed}, zero_torque_di={zero_torque_di}, accel={accel}"
          )

  def test_override_with_unsafe_zero_torque_transition_falls_back(self, tmp_path):
    custom = {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, -0.251, -0.25, 0.0, 0.25, 0.251, 2.5],
      'table': [
        [-5.0, -5.0, 0.0, 0.1, 0.2, 80.0, 80.0],
        [-5.0, -5.0, 0.0, 0.1, 0.2, 80.0, 80.0],
      ],
    }
    path = tmp_path / "steep_monotonic_table.json"
    path.write_text(json.dumps(custom))
    ff = FeedforwardModel(table_path=str(path))

    samples = [ff.get(accel, 20.0, zero_torque_di=0.0)
               for accel in np.linspace(-0.25, 0.25, 501)]

    assert ff.accel_bp == list(FF_ACCEL_BP)
    assert all(right >= left for left, right in zip(samples, samples[1:], strict=False))

  def test_vdas_uses_ff_model(self):
    """VirtualDAS._feedforward should use the FeedforwardModel."""
    vdas = VirtualDAS(dt=0.02)
    assert hasattr(vdas, 'ff_model')

    for _ in range(200):
      di = vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di > PEDAL_DI_ZERO


# --- Phase 4: Grade Estimation ---

class TestGradeEstimator:

  def test_flat_road_zero_compensation(self):
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    for _ in range(100):
      grade, pitch_comp = ge.update([0.0, 0.0, 0.0])
    assert abs(grade) < 0.01
    assert abs(pitch_comp) < 0.01

  def test_uphill_positive_grade(self):
    """Uphill (positive pitch) should report positive grade_accel
    (gravity decelerates the car, so we need more pedal)."""
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    pitch = math.radians(3.0)  # ~5% grade
    for _ in range(200):
      grade, _ = ge.update([0.0, pitch, 0.0])
    assert grade > 0.4  # sin(3°) * 9.81 ≈ 0.51

  def test_downhill_negative_grade(self):
    """Downhill (negative pitch) should report negative grade_accel."""
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    pitch = math.radians(-3.0)
    for _ in range(200):
      grade, _ = ge.update([0.0, pitch, 0.0])
    assert grade < -0.4

  def test_empty_orientation_graceful(self):
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    grade, pitch_comp = ge.update([])
    assert grade == 0.0
    assert pitch_comp == 0.0

  def test_none_orientation_graceful(self):
    """VirtualDAS with orientation_ned=None should not crash."""
    vdas = VirtualDAS(dt=0.02)
    di = vdas.update(0.5, v_ego=15.0, prev_pedal_di=0.0, orientation_ned=None)
    assert np.isfinite(di)

  def test_pitch_compensation_clamped(self):
    """Transient pitch compensation should be clamped."""
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator, MAX_PITCH_COMPENSATION
    ge = GradeEstimator(dt=0.02)
    # Sudden large pitch change
    ge.update([0.0, 0.0, 0.0])
    _, pitch_comp = ge.update([0.0, math.radians(20.0), 0.0])
    assert abs(pitch_comp) <= MAX_PITCH_COMPENSATION + 0.01

  def test_sustained_pitch_outlier_cannot_exceed_steady_grade_limit(self):
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator, MAX_STEADY_GRADE_COMPENSATION
    ge = GradeEstimator(dt=0.02)

    for _ in range(500):
      grade, _ = ge.update([0.0, math.radians(20.0), 0.0])

    assert abs(grade) <= MAX_STEADY_GRADE_COMPENSATION
    for _ in range(50):
      recovered_grade, _ = ge.update([0.0, 0.0, 0.0])
    assert abs(recovered_grade) < 0.3

  def test_orientation_dropout_holds_filtered_steady_grade(self):
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)

    for _ in range(200):
      steady_grade, _ = ge.update([0.0, math.radians(3.0), 0.0])

    dropout_grade, dropout_transient = ge.update([])

    assert dropout_grade == pytest.approx(steady_grade)
    assert dropout_transient == 0.0

  @pytest.mark.parametrize(("dropout_duration_s", "expected_grade_scale"), [
    (0.10, 1.0),
    (0.50, 1.0),
    (1.25, 0.5),
    (2.00, 0.0),
    (4.64, 0.0),
  ])
  def test_orientation_dropout_holds_then_decays_steady_grade(
    self, dropout_duration_s, expected_grade_scale,
  ):
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    dt = 0.02
    settled_pitch = math.radians(4.45)
    ge = GradeEstimator(dt=dt)

    for _ in range(500):
      ge.update([0.0, settled_pitch, 0.0])

    for _ in range(round(dropout_duration_s / dt)):
      dropout_grade, dropout_transient = ge.update([])
      assert dropout_transient == 0.0

    expected_grade = math.sin(settled_pitch) * 9.81 * expected_grade_scale
    assert dropout_grade == pytest.approx(expected_grade, abs=0.02)

  def test_crest_reacquisition_after_long_dropout_uses_new_grade_sign(self):
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    dt = 0.02
    ge = GradeEstimator(dt=dt)

    for _ in range(500):
      ge.update([0.0, math.radians(4.45), 0.0])
    for _ in range(round(4.64 / dt)):
      dropout_grade, dropout_transient = ge.update([])

    assert abs(dropout_grade + dropout_transient) < 0.02

    grade, transient = ge.update([0.0, math.radians(-3.0), 0.0])
    first_reacquired_compensation = grade + transient
    assert -0.25 <= first_reacquired_compensation < 0.0

    for _ in range(round(1.50 / dt) - 1):
      grade, transient = ge.update([0.0, math.radians(-3.0), 0.0])

    assert grade == pytest.approx(math.sin(math.radians(-3.0)) * 9.81, abs=0.05)
    assert abs(transient) < 0.15

  def test_grade_does_not_change_net_acceleration_feedback(self, mock_nap_conf, mock_zero_torque):
    """Wheel-speed acceleration and planner targets remain in the net domain."""
    import math
    vdas = VirtualDAS(dt=0.02)
    pitch = math.radians(-3.0)  # downhill

    for _ in range(100):
      vdas.update(0.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.5, orientation_ned=[0.0, pitch, 0.0])

    assert vdas.a_ego_filter.x == pytest.approx(0.5, abs=0.01)

  def test_reset_clears_grade(self):
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    for _ in range(100):
      ge.update([0.0, math.radians(5.0), 0.0])
    assert abs(ge.pitch_lp.x) > 0.01
    ge.update([])

    ge.reset()

    assert ge.update([]) == (0.0, 0.0)
    assert ge.update([0.0, 0.0, 0.0]) == (0.0, 0.0)


class TestVDASDomainBoundaries:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_residual_feedback_enters_feedforward_in_acceleration_domain(self, monkeypatch):
    desired_acceleration_mps2 = 0.4
    feedforward_sentinel_di = 10.0
    feedforward_inputs_mps2 = []
    pedal_outputs_di = []
    vdas = VirtualDAS(dt=0.02)
    vdas.reset(
      measured_accel=0.0,
      commanded_accel=desired_acceleration_mps2,
      pedal_di_init=feedforward_sentinel_di,
    )

    def record_feedforward(acceleration_effort_mps2, _v_ego):
      feedforward_inputs_mps2.append(acceleration_effort_mps2)
      return feedforward_sentinel_di

    monkeypatch.setattr(vdas, '_feedforward', record_feedforward)

    for _ in range(100):
      pedal_outputs_di.append(vdas.update(
        desired_acceleration_mps2,
        v_ego=20.0,
        prev_pedal_di=feedforward_sentinel_di,
        a_ego=0.0,
        freeze_integrator=False,
      ))

    assert len(feedforward_inputs_mps2) == 100
    assert feedforward_inputs_mps2[-1] > desired_acceleration_mps2 + 0.05, (
      "sustained acceleration error did not move the feedforward input: "
      + f"got {feedforward_inputs_mps2[-1]:.3f} m/s^2"
    )
    assert pedal_outputs_di == [feedforward_sentinel_di] * 100

  def test_engage_effort_limits_include_grade_compensation(self, monkeypatch):
    import math

    feedforward_inputs_mps2 = []
    vdas = VirtualDAS(dt=0.02)
    orientation_ned = [0.0, math.radians(3.0), 0.0]
    for _ in range(200):
      vdas.observe(0.0, orientation_ned)
    vdas.reset(
      measured_accel=0.0,
      commanded_accel=0.0,
      pedal_di_init=0.0,
      preserve_grade=True,
    )

    def record_feedforward(acceleration_effort_mps2, _v_ego):
      feedforward_inputs_mps2.append(acceleration_effort_mps2)
      return 0.0

    monkeypatch.setattr(vdas, '_feedforward', record_feedforward)
    pedal_di = vdas.update(
      0.0,
      v_ego=15.0,
      prev_pedal_di=0.0,
      a_ego=0.0,
      freeze_integrator=True,
      orientation_ned=orientation_ned,
      accel_effort_limits=(0.0, 0.0),
    )

    assert feedforward_inputs_mps2 == [0.0]
    assert pedal_di == pytest.approx(0.0)

  def test_engage_pedal_ramp_limit_applies_after_feedforward(self, monkeypatch):
    vdas = VirtualDAS(dt=0.02)
    monkeypatch.setattr(vdas, '_feedforward', lambda _acceleration_effort_mps2, _v_ego: 20.0)

    pedal_di = vdas.update(
      0.0,
      v_ego=15.0,
      prev_pedal_di=0.0,
      a_ego=0.0,
      freeze_integrator=True,
      pedal_ramp_rate_up=0.9,
    )

    assert pedal_di == pytest.approx(0.9)

  def test_pid_starts_with_acceleration_domain_limits(self):
    vdas = VirtualDAS(dt=0.02)

    assert vdas.inner_pid.pos_limit == ACCEL_MAX
    assert vdas.inner_pid.neg_limit == REGEN_MAX

  @pytest.mark.parametrize(("base_effort_mps2", "seeded_integral_mps2", "authority_rail_mps2"), [
    (ACCEL_MAX, 0.5, ACCEL_MAX),
    (REGEN_MAX, -0.5, REGEN_MAX),
  ])
  def test_retained_integral_is_clipped_to_remaining_acceleration_authority(
      self, base_effort_mps2, seeded_integral_mps2, authority_rail_mps2):
    vdas = VirtualDAS(dt=0.02)
    initial_pedal_di = vdas._feedforward(base_effort_mps2, v_ego=15.0)
    vdas.reset(
      measured_accel=base_effort_mps2,
      commanded_accel=base_effort_mps2,
      pedal_di_init=initial_pedal_di,
    )
    vdas.inner_pid.i = seeded_integral_mps2

    vdas.update(
      base_effort_mps2,
      v_ego=15.0,
      prev_pedal_di=initial_pedal_di,
      a_ego=base_effort_mps2,
      freeze_integrator=True,
    )

    expected_retained_integral_mps2 = authority_rail_mps2 - base_effort_mps2
    assert vdas.inner_pid.i == pytest.approx(expected_retained_integral_mps2)

  def test_physical_pedal_rail_freezes_and_unwinds_acceleration_integral(self, tmp_path):
    high_table = {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [REGEN_MAX, 0.0, ACCEL_MAX],
      'table': [
        [PEDAL_DI_MIN, 0.0, max(PEDAL_MAX_VALUES)],
        [PEDAL_DI_MIN, 0.0, max(PEDAL_MAX_VALUES)],
      ],
    }
    table_path = tmp_path / "valid_high_table.json"
    table_path.write_text(json.dumps(high_table))
    vdas = VirtualDAS(dt=0.02)
    vdas.ff_model = FeedforwardModel(table_path=str(table_path))

    v_ego = 0.0
    desired_acceleration_mps2 = 1.5
    max_profile_pedal_di = float(np.interp(v_ego, PEDAL_BP, PEDAL_MAX_VALUES))
    raw_table_request_di = vdas.ff_model.get(desired_acceleration_mps2, v_ego, zero_torque_di=0.0)
    assert vdas.ff_model.table == high_table['table']
    assert raw_table_request_di > max_profile_pedal_di

    initial_integral_mps2 = 0.2
    vdas.reset(
      measured_accel=0.0,
      commanded_accel=desired_acceleration_mps2,
      pedal_di_init=max_profile_pedal_di,
    )
    vdas.inner_pid.i = initial_integral_mps2

    pedal_outputs_di = []
    for _ in range(20):
      pedal_outputs_di.append(vdas.update(
        desired_acceleration_mps2,
        v_ego=v_ego,
        prev_pedal_di=max_profile_pedal_di,
        a_ego=0.0,
        freeze_integrator=False,
      ))

    assert pedal_outputs_di == [max_profile_pedal_di] * 20
    assert vdas.inner_pid.i == pytest.approx(initial_integral_mps2), (
      f"integral grew against the physical pedal rail: {initial_integral_mps2:.3f} -> {vdas.inner_pid.i:.3f}"
    )

    reversed_measurement_mps2 = 2.0
    vdas.a_ego_filter.x = reversed_measurement_mps2
    vdas.prev_a_ego_filtered = reversed_measurement_mps2
    vdas.update(
      desired_acceleration_mps2,
      v_ego=v_ego,
      prev_pedal_di=max_profile_pedal_di,
      a_ego=reversed_measurement_mps2,
      freeze_integrator=False,
    )

    assert vdas.inner_pid.i < initial_integral_mps2

  @pytest.mark.parametrize(("target_acceleration_mps2", "measured_acceleration_mps2"), [
    (ACCEL_MAX, -1.0),
    (REGEN_MAX, 1.0),
  ])
  def test_feedback_respects_remaining_acceleration_authority(
      self, monkeypatch, target_acceleration_mps2, measured_acceleration_mps2):
    feedforward_inputs_mps2 = []
    vdas = VirtualDAS(dt=0.02)
    real_feedforward = vdas._feedforward
    initial_pedal_di = real_feedforward(target_acceleration_mps2, v_ego=15.0)
    vdas.reset(
      measured_accel=measured_acceleration_mps2,
      commanded_accel=target_acceleration_mps2,
      pedal_di_init=initial_pedal_di,
    )

    def record_feedforward(acceleration_effort_mps2, v_ego):
      feedforward_inputs_mps2.append(acceleration_effort_mps2)
      return real_feedforward(acceleration_effort_mps2, v_ego)

    monkeypatch.setattr(vdas, '_feedforward', record_feedforward)

    for _ in range(300):
      vdas.update(
        target_acceleration_mps2,
        v_ego=15.0,
        prev_pedal_di=vdas.prev_pedal_di,
        a_ego=measured_acceleration_mps2,
        freeze_integrator=False,
      )

    assert min(feedforward_inputs_mps2) >= REGEN_MAX
    assert max(feedforward_inputs_mps2) <= ACCEL_MAX
    if target_acceleration_mps2 == ACCEL_MAX:
      assert vdas.inner_pid.i <= 1e-12, (
        f"positive integral {vdas.inner_pid.i:.3f} exceeded zero remaining acceleration authority"
      )
    else:
      assert vdas.inner_pid.i >= -1e-12, (
        f"negative integral {vdas.inner_pid.i:.3f} exceeded zero remaining regen authority"
      )

  @pytest.mark.parametrize((
    "target_acceleration_mps2",
    "initial_integral_mps2",
    "forced_feedforward_di",
    "previous_pedal_di",
    "expected_pedal_di",
    "reversed_measurement_mps2",
    "blocked_direction",
  ), [
    (0.5, 0.2, 60.0, 0.0, PEDAL_RAMP_RATE_UP, 1.5, 1.0),
    (-0.5, -0.2, PEDAL_DI_MIN, 50.0, 50.0 - PEDAL_RAMP_RATE_DOWN, -1.5, -1.0),
  ])
  def test_final_di_slew_backstop_freezes_and_unwinds_acceleration_integral(
      self, monkeypatch, target_acceleration_mps2, initial_integral_mps2,
      forced_feedforward_di, previous_pedal_di, expected_pedal_di,
      reversed_measurement_mps2, blocked_direction):
    vdas = VirtualDAS(dt=0.02)
    vdas.reset(
      measured_accel=0.0,
      commanded_accel=target_acceleration_mps2,
      pedal_di_init=previous_pedal_di,
    )
    vdas.inner_pid.i = initial_integral_mps2
    monkeypatch.setattr(vdas, '_feedforward', lambda _acceleration, _v_ego: forced_feedforward_di)

    pedal_di = vdas.update(
      target_acceleration_mps2,
      v_ego=20.0,
      prev_pedal_di=previous_pedal_di,
      a_ego=0.0,
      freeze_integrator=False,
    )

    assert pedal_di == pytest.approx(expected_pedal_di)
    assert vdas.inner_pid.i == pytest.approx(initial_integral_mps2), (
      f"integral grew into a blocked DI request: {initial_integral_mps2:.3f} -> {vdas.inner_pid.i:.3f}"
    )

    vdas.a_ego_filter.x = reversed_measurement_mps2
    vdas.prev_a_ego_filtered = reversed_measurement_mps2
    vdas.update(
      target_acceleration_mps2,
      v_ego=20.0,
      prev_pedal_di=previous_pedal_di,
      a_ego=reversed_measurement_mps2,
      freeze_integrator=False,
    )

    assert (vdas.inner_pid.i - initial_integral_mps2) * blocked_direction < 0.0

  def test_transient_grade_compensation_enters_feedforward_in_acceleration_domain(self):
    class FixedGradeEstimator:
      def update(self, _orientation_ned):
        return 0.0, 0.2

    vdas = VirtualDAS(dt=0.02)
    vdas.grade_estimator = FixedGradeEstimator()
    vdas.jerk_limiter.reset(a_init=0.4)
    vdas.a_ego_filter.x = 0.4
    vdas.prev_a_ego_filtered = 0.4
    expected_di = vdas._feedforward(0.6, v_ego=20.0)

    pedal_di = vdas.update(
      0.4, v_ego=20.0, prev_pedal_di=expected_di,
      a_ego=0.4, freeze_integrator=False, orientation_ned=[0.0, 0.0, 0.0],
    )

    assert pedal_di == pytest.approx(expected_di)
    assert vdas.inner_pid.i == pytest.approx(0.0)

  def test_engage_reset_starts_estimator_from_measured_acceleration(self, monkeypatch):
    from opendbc.car.tesla.preap.carcontroller import PreAPLongController
    from opendbc.car.tesla.preap.engagement import PreAPEngagement
    from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP

    controller = PreAPLongController()
    measured_acceleration = 0.7
    cc = SimpleNamespace(
      actuators=SimpleNamespace(accel=0.0),
      longActive=False,
      orientationNED=[],
    )
    cs = SimpleNamespace(
      cruiseEnabled=True,
      enableLongControl=True,
      enableJustCC=False,
      engagement=PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750),
      real_brake_pressed=False,
      out=SimpleNamespace(vEgo=15.0, aEgo=measured_acceleration, gasPressed=False),
      pedal_interceptor_value=0.0,
      cruise_buttons=0,
      prev_cruise_buttons=0,
      pedal=SimpleNamespace(available=True, interceptor_state=0, idx=1, torque_level=0.0),
      pedal_timeout=False,
      preap_cc_cancel_needed=False,
    )

    zero_torque = SimpleNamespace(get=lambda _v_ego: PEDAL_DI_ZERO, update=lambda *_args, **_kwargs: None)
    controller_conf = SimpleNamespace(
      use_pedal=True,
      pedal_factor=1.0,
      di_to_pedal=lambda pedal_di: pedal_di,
    )
    monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', controller_conf)
    monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
    controller.update(cc, cs, frame=1, tesla_can=None, can_bus_party=0)

    assert controller.vdas.a_ego_filter.x == pytest.approx(0.0)
    assert controller.vdas.prev_a_ego_filtered == pytest.approx(0.0)

    cc.longActive = True
    controller.update(cc, cs, frame=2, tesla_can=TeslaCANPreAP({}), can_bus_party=0)

    assert controller.vdas.a_ego_filter.x == pytest.approx(measured_acceleration)
    assert controller.vdas.prev_a_ego_filtered == pytest.approx(measured_acceleration)
    assert controller.vdas.jerk_limiter.a_limited == pytest.approx(0.0)

  def test_preserved_grade_reset_keeps_acceleration_filter_in_net_domain(self):
    import math

    vdas = VirtualDAS(dt=0.02)
    measured_acceleration = 0.1
    orientation_ned = [0.0, math.radians(5.0), 0.0]
    for _ in range(300):
      vdas.observe(measured_acceleration, orientation_ned)

    assert vdas.a_ego_filter.x == pytest.approx(measured_acceleration, abs=0.01)

    vdas.reset(measured_accel=measured_acceleration, commanded_accel=0.0, preserve_grade=True)

    assert vdas.a_ego_filter.x == pytest.approx(measured_acceleration, abs=0.01)
    assert vdas.prev_a_ego_filtered == pytest.approx(measured_acceleration, abs=0.01)
    assert vdas.jerk_limiter.a_limited == pytest.approx(0.0)
