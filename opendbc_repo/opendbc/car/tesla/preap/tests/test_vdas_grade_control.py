"""Closed-loop grade contracts for the Pre-AP acceleration controller."""

import math
from dataclasses import dataclass, replace
from types import SimpleNamespace

import pytest
from numpy import interp

from opendbc.car.tesla.preap import virtual_das
from opendbc.car.tesla.preap.constants import VDAS_ACCEL_JERK_MAX, VDAS_DECEL_JERK_MAX
from opendbc.car.tesla.preap.virtual_das import FeedforwardModel, GRAVITY, VirtualDAS


CONTROL_DT_S = 0.02
PLANT_DELAY_S = 0.40
PLANT_TAU_S = 0.25
GRADE_ESTIMATOR_SETTLING_S = 6.0
SHORT_HORIZON_S = 1.5
STEADY_HORIZON_S = 15.0
STEADY_WINDOW_S = 2.0
GRADE_ACCEL_MPS2 = 0.50
NET_ACCEL_TOLERANCE_MPS2 = 0.12
SHORT_HORIZON_SPEED_DRIFT_MPS = 0.10
EFFORT_TOLERANCE_MPS2 = 0.12
COAST_PEDAL_DI = 3.0
STEP_GRADE_ACCEL_MPS2 = 0.40
MATRIX_GRADE_ACCEL_MPS2 = 0.30
STEP_RESPONSE_TOLERANCE_MPS2 = 0.30
STEP_RESPONSE_UPHILL_OVERSHOOT_LIMIT_MPS2 = 0.10
STEP_RESPONSE_MIN_DRIFT_IMPROVEMENT_MPS = 0.07
STEP_RESPONSE_MIN_PEDAL_DELTA_DI = 2.0
STEADY_MEAN_TOLERANCE_MPS2 = 0.12
STEADY_PEAK_TOLERANCE_MPS2 = 0.18
STEADY_SPEED_DRIFT_MPS = 2.5
PHYSICAL_RAIL_MARGIN_DI = 0.25
ROUTE_PLANT_PEDAL_DI_BP = [-5.0, -2.0, 0.0, 3.0, 8.0, 15.0, 22.0, 27.0, 35.0, 50.0]
ROUTE_PLANT_NET_ACCEL_BP = [-1.05, -0.62, -0.50, -0.36, 0.0, 0.50, 1.15, 1.65, 2.10, 2.45]
ROUTE_INITIAL_TARGET_MPS2 = -0.57
ROUTE_PEAK_TARGET_MPS2 = 0.75
ROUTE_TARGET_RAMP_UP_S = 6.25
ROUTE_TARGET_HOLD_S = 2.50
ROUTE_TARGET_RAMP_DOWN_S = 2.25
ROUTE_INITIAL_PEDAL_DI = -1.85
ROUTE_SPEED_MPS = 20.0
ROUTE_PEAK_ACCEL_LIMIT_MPS2 = 1.00
ROUTE_UNCERTAIN_PEAK_ACCEL_LIMIT_MPS2 = 1.10
ROUTE_POSITIVE_EXCESS_LIMIT_MPS = 0.75
ROUTE_POSITIVE_JERK_LIMIT_MPS3 = 1.00
ROUTE_FINAL_TRACKING_ERROR_LIMIT_MPS2 = 0.15
RESIDUAL_BUILD_LOAD_MPS2 = 0.443
NEGATIVE_HANDOFF_LOAD_MPS2 = 0.20
NEGATIVE_HANDOFF_TARGET_MPS2 = -0.20
RESIDUAL_BUILD_RAMP_S = 0.50
RESIDUAL_BUILD_HOLD_S = 12.0
NEGATIVE_HANDOFF_RAMP_S = 0.80
NEGATIVE_HANDOFF_HOLD_S = 1.0
NEGATIVE_HANDOFF_SETTLED_WINDOW_S = 1.0
NEGATIVE_HANDOFF_PEDAL_MARGIN_DI = 0.05
PEDAL_PLANT_ACCEL_PER_DI_MPS2 = 0.063
NEGATIVE_HANDOFF_MAX_PEDAL_STEP_DI = 0.50
NEGATIVE_HANDOFF_MAX_EFFORT_JERK_MPS3 = 0.75
NEGATIVE_HANDOFF_MAX_UNDERSHOOT_MPS2 = 0.12
NEGATIVE_HANDOFF_GRADE_SEPARATION_TOLERANCE_MPS2 = 5e-5
NEAR_ZERO_TARGET_MPS2 = 0.02
NEAR_ZERO_HALF_CYCLE_S = 0.20
NEAR_ZERO_CYCLE_COUNT = 6
NEAR_ZERO_MAX_TRIM_LOSS_MPS2 = 0.05
FULL_BRAKE_TARGET_MPS2 = -0.30
STILL_NEGATIVE_RECOVERY_TARGET_MPS2 = -0.04
FULL_BRAKE_HOLD_S = 0.60
STILL_NEGATIVE_RECOVERY_HOLD_S = 2.0
SPEED_RAMP_TARGET_ACCELERATIONS_MPS2 = (0.50, 0.75)
SPEED_RAMP_WARMUP_S = 10.0
SPEED_RAMP_TIMEOUT_S = 45.0
SPEED_RAMP_INITIAL_MPS = 5.0
SPEED_RAMP_FINAL_MPS = 20.0
SPEED_RAMP_ROLLING_WINDOW_S = 0.75
SPEED_RAMP_DROOP_LIMIT_MPS2 = 0.075
SPEED_RAMP_TRACKING_ERROR_LIMIT_MPS = 0.75
SPEED_RAMP_JERK_LIMIT_MPS3 = 1.00
SPEED_RAMP_PEAK_ACCEL_LIMIT_MPS2 = 1.00


@dataclass(frozen=True)
class GradePlantSample:
  net_acceleration_mps2: float
  speed_mps: float
  acceleration_effort_mps2: float
  integral_trim_mps2: float


@dataclass(frozen=True)
class DelayedPedalPlantCase:
  delay_s: float
  tau_s: float
  acceleration_per_di_mps2: float
  grade_sensor_scale: float


NOMINAL_PLANT = DelayedPedalPlantCase(0.40, 0.25, 0.063, 1.0)
UNCERTAIN_PLANTS = (
  NOMINAL_PLANT,
  DelayedPedalPlantCase(0.50, 0.35, 0.0567, 0.90),
  DelayedPedalPlantCase(0.30, 0.20, 0.0693, 1.10),
)


@dataclass(frozen=True)
class PedalPlantSample:
  elapsed_s: float
  net_acceleration_mps2: float
  speed_delta_mps: float
  pedal_di: float


@dataclass(frozen=True)
class RoutePlantCase:
  delay_s: float
  tau_s: float
  output_scale: float


@dataclass(frozen=True)
class RoutePlantSample:
  elapsed_s: float
  target_acceleration_mps2: float
  net_acceleration_mps2: float
  pedal_di: float


@dataclass(frozen=True)
class SpeedRampPlantSample:
  speed_mps: float
  net_acceleration_mps2: float
  pedal_di: float


@dataclass(frozen=True)
class ResidualHandoffSample:
  target_acceleration_mps2: float
  net_acceleration_mps2: float
  output: float


NOMINAL_ROUTE_PLANT = RoutePlantCase(0.40, 0.25, 1.0)
UNCERTAIN_ROUTE_PLANTS = (
  RoutePlantCase(0.30, 0.20, 0.9),
  NOMINAL_ROUTE_PLANT,
  RoutePlantCase(0.50, 0.35, 1.1),
)


def route_target_acceleration(elapsed_s: float) -> float:
  if elapsed_s <= ROUTE_TARGET_RAMP_UP_S:
    ramp_fraction = elapsed_s / ROUTE_TARGET_RAMP_UP_S
    return ROUTE_INITIAL_TARGET_MPS2 + ramp_fraction * (
      ROUTE_PEAK_TARGET_MPS2 - ROUTE_INITIAL_TARGET_MPS2
    )

  hold_end_s = ROUTE_TARGET_RAMP_UP_S + ROUTE_TARGET_HOLD_S
  if elapsed_s <= hold_end_s:
    return ROUTE_PEAK_TARGET_MPS2

  ramp_down_fraction = (elapsed_s - hold_end_s) / ROUTE_TARGET_RAMP_DOWN_S
  return ROUTE_PEAK_TARGET_MPS2 * (1.0 - ramp_down_fraction)


def run_route_shaped_acceleration(
    plant: RoutePlantCase,
    monkeypatch,
) -> list[RoutePlantSample]:
  """Exercise the fallback controller against fixed, field-observed plant anchors."""
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: 0.0),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  controller.ff_model = FeedforwardModel(table_path="/nonexistent")
  controller.reset(
    measured_accel=ROUTE_INITIAL_TARGET_MPS2,
    commanded_accel=ROUTE_INITIAL_TARGET_MPS2,
    pedal_di_init=ROUTE_INITIAL_PEDAL_DI,
  )

  pedal_di = ROUTE_INITIAL_PEDAL_DI
  net_acceleration_mps2 = ROUTE_INITIAL_TARGET_MPS2
  delayed_pedals_di = [pedal_di] * round(plant.delay_s / CONTROL_DT_S)
  plant_alpha = CONTROL_DT_S / (plant.tau_s + CONTROL_DT_S)
  duration_s = ROUTE_TARGET_RAMP_UP_S + ROUTE_TARGET_HOLD_S + ROUTE_TARGET_RAMP_DOWN_S
  samples = []

  for step in range(round(duration_s / CONTROL_DT_S)):
    elapsed_s = (step + 1) * CONTROL_DT_S
    target_acceleration_mps2 = route_target_acceleration(elapsed_s)
    pedal_di = controller.update(
      target_acceleration_mps2,
      v_ego=ROUTE_SPEED_MPS,
      prev_pedal_di=pedal_di,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )
    applied_pedal_di = delayed_pedals_di.pop(0)
    delayed_pedals_di.append(pedal_di)
    plant_target_acceleration_mps2 = float(interp(
      applied_pedal_di,
      ROUTE_PLANT_PEDAL_DI_BP,
      ROUTE_PLANT_NET_ACCEL_BP,
    )) * plant.output_scale
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )
    samples.append(RoutePlantSample(
      elapsed_s=elapsed_s,
      target_acceleration_mps2=target_acceleration_mps2,
      net_acceleration_mps2=net_acceleration_mps2,
      pedal_di=pedal_di,
    ))

  return samples


def peak_delivered_acceleration(samples: list[RoutePlantSample]) -> float:
  return max(sample.net_acceleration_mps2 for sample in samples)


def run_residual_trim_handoff(
    *,
    grade_acceleration_mps2: float,
    identity_feedforward: bool,
    monkeypatch,
) -> list[ResidualHandoffSample]:
  """Build residual road-load trim, then cross into a finite-jerk decel request."""
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: COAST_PEDAL_DI),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  orientation_ned = [0.0, math.asin(grade_acceleration_mps2 / GRAVITY), 0.0]
  for _ in range(round(GRADE_ESTIMATOR_SETTLING_S / CONTROL_DT_S)):
    controller.observe(a_ego=0.0, orientation_ned=orientation_ned)

  initial_output = grade_acceleration_mps2 if identity_feedforward else COAST_PEDAL_DI
  controller.reset(
    measured_accel=0.0,
    commanded_accel=0.0,
    pedal_di_init=initial_output,
    preserve_grade=True,
  )
  if identity_feedforward:
    monkeypatch.setattr(
      controller,
      "_feedforward",
      lambda acceleration_effort_mps2, _speed_mps: acceleration_effort_mps2,
    )

  output = initial_output
  net_acceleration_mps2 = 0.0
  delayed_outputs = [output] * round(PLANT_DELAY_S / CONTROL_DT_S)
  plant_alpha = CONTROL_DT_S / (PLANT_TAU_S + CONTROL_DT_S)
  phases = (
    (RESIDUAL_BUILD_RAMP_S, 0.0, RESIDUAL_BUILD_LOAD_MPS2),
    (RESIDUAL_BUILD_HOLD_S, 0.0, RESIDUAL_BUILD_LOAD_MPS2),
    (NEGATIVE_HANDOFF_RAMP_S, NEGATIVE_HANDOFF_TARGET_MPS2, NEGATIVE_HANDOFF_LOAD_MPS2),
    (NEGATIVE_HANDOFF_HOLD_S, NEGATIVE_HANDOFF_TARGET_MPS2, NEGATIVE_HANDOFF_LOAD_MPS2),
  )
  target_acceleration_mps2 = 0.0
  residual_road_load_mps2 = 0.0
  samples = []

  for duration_s, target_end_mps2, load_end_mps2 in phases:
    frame_count = round(duration_s / CONTROL_DT_S)
    target_start_mps2 = target_acceleration_mps2
    load_start_mps2 = residual_road_load_mps2
    for frame_index in range(1, frame_count + 1):
      phase_fraction = frame_index / frame_count
      target_acceleration_mps2 = target_start_mps2 + (
        target_end_mps2 - target_start_mps2
      ) * phase_fraction
      residual_road_load_mps2 = load_start_mps2 + (
        load_end_mps2 - load_start_mps2
      ) * phase_fraction
      output = controller.update(
        target_acceleration_mps2,
        v_ego=25.0,
        prev_pedal_di=output,
        a_ego=net_acceleration_mps2,
        freeze_integrator=False,
        orientation_ned=orientation_ned,
      )
      applied_output = delayed_outputs.pop(0)
      delayed_outputs.append(output)
      if identity_feedforward:
        plant_target_acceleration_mps2 = (
          applied_output - grade_acceleration_mps2 - residual_road_load_mps2
        )
      else:
        plant_target_acceleration_mps2 = (
          (applied_output - COAST_PEDAL_DI) * PEDAL_PLANT_ACCEL_PER_DI_MPS2
          - grade_acceleration_mps2
          - residual_road_load_mps2
        )
      net_acceleration_mps2 += plant_alpha * (
        plant_target_acceleration_mps2 - net_acceleration_mps2
      )
      samples.append(ResidualHandoffSample(
        target_acceleration_mps2=target_acceleration_mps2,
        net_acceleration_mps2=net_acceleration_mps2,
        output=output,
      ))

  return samples


def test_residual_trim_handoff_tracks_negative_command_without_harsh_pedal_step(monkeypatch):
  samples = run_residual_trim_handoff(
    grade_acceleration_mps2=0.0,
    identity_feedforward=False,
    monkeypatch=monkeypatch,
  )
  settled_sample_count = round(NEGATIVE_HANDOFF_SETTLED_WINDOW_S / CONTROL_DT_S)
  settled_samples = samples[-settled_sample_count:]
  pedal_steps_di = [
    current.output - previous.output
    for previous, current in zip(samples, samples[1:], strict=False)
  ]

  assert all(sample.target_acceleration_mps2 <= -0.15 for sample in settled_samples)
  assert max(abs(step_di) for step_di in pedal_steps_di) <= NEGATIVE_HANDOFF_MAX_PEDAL_STEP_DI
  # This fixture ends with road load equal to the negative target magnitude,
  # making coast its equilibrium. The regen-side margin is not a universal
  # rule for negative commands under other road loads.
  assert max(sample.output for sample in settled_samples) <= (
    COAST_PEDAL_DI - NEGATIVE_HANDOFF_PEDAL_MARGIN_DI
  )
  assert min(
    sample.net_acceleration_mps2 - sample.target_acceleration_mps2
    for sample in settled_samples
  ) >= -NEGATIVE_HANDOFF_MAX_UNDERSHOOT_MPS2


def test_negative_command_handoff_keeps_grade_effort_separate(monkeypatch):
  flat_samples = run_residual_trim_handoff(
    grade_acceleration_mps2=0.0,
    identity_feedforward=True,
    monkeypatch=monkeypatch,
  )
  grade_acceleration_mps2 = 0.35
  uphill_samples = run_residual_trim_handoff(
    grade_acceleration_mps2=grade_acceleration_mps2,
    identity_feedforward=True,
    monkeypatch=monkeypatch,
  )
  settled_sample_count = round(NEGATIVE_HANDOFF_SETTLED_WINDOW_S / CONTROL_DT_S)
  flat_settled_outputs = [sample.output for sample in flat_samples[-settled_sample_count:]]
  uphill_settled_outputs = [sample.output for sample in uphill_samples[-settled_sample_count:]]
  flat_effort_jerks_mps3 = [
    (current.output - previous.output) / CONTROL_DT_S
    for previous, current in zip(flat_samples, flat_samples[1:], strict=False)
  ]

  assert max(abs(jerk_mps3) for jerk_mps3 in flat_effort_jerks_mps3) <= NEGATIVE_HANDOFF_MAX_EFFORT_JERK_MPS3
  assert uphill_settled_outputs == pytest.approx([
    flat_output + grade_acceleration_mps2
    for flat_output in flat_settled_outputs
  ], abs=NEGATIVE_HANDOFF_GRADE_SEPARATION_TOLERANCE_MPS2)


def test_repeated_near_zero_crossings_preserve_learned_disturbance_trim(monkeypatch):
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  controller = VirtualDAS(dt=CONTROL_DT_S)
  monkeypatch.setattr(
    controller,
    "_feedforward",
    lambda acceleration_effort_mps2, _speed_mps: acceleration_effort_mps2,
  )
  controller.reset(measured_accel=0.0, commanded_accel=0.0, pedal_di_init=0.0)
  output = 0.0
  net_acceleration_mps2 = 0.0
  plant_alpha = CONTROL_DT_S / (PLANT_TAU_S + CONTROL_DT_S)
  delayed_outputs = [output] * round(PLANT_DELAY_S / CONTROL_DT_S)

  for _ in range(round(RESIDUAL_BUILD_HOLD_S / CONTROL_DT_S)):
    output = controller.update(
      0.0,
      v_ego=25.0,
      prev_pedal_di=output,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )
    applied_output = delayed_outputs.pop(0)
    delayed_outputs.append(output)
    plant_target_acceleration_mps2 = applied_output - RESIDUAL_BUILD_LOAD_MPS2
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )

  learned_trim_output = output
  crossing_outputs = []
  target_acceleration_mps2 = 0.0
  half_cycle_steps = round(NEAR_ZERO_HALF_CYCLE_S / CONTROL_DT_S)
  for half_cycle_index in range(NEAR_ZERO_CYCLE_COUNT * 2):
    target_end_mps2 = NEAR_ZERO_TARGET_MPS2 * (-1.0 if half_cycle_index % 2 == 0 else 1.0)
    target_start_mps2 = target_acceleration_mps2
    for step_index in range(1, half_cycle_steps + 1):
      target_acceleration_mps2 = target_start_mps2 + (
        target_end_mps2 - target_start_mps2
      ) * step_index / half_cycle_steps
      output = controller.update(
        target_acceleration_mps2,
        v_ego=25.0,
        prev_pedal_di=output,
        a_ego=net_acceleration_mps2,
        freeze_integrator=False,
        orientation_ned=[0.0, 0.0, 0.0],
      )
      applied_output = delayed_outputs.pop(0)
      delayed_outputs.append(output)
      plant_target_acceleration_mps2 = applied_output - RESIDUAL_BUILD_LOAD_MPS2
      net_acceleration_mps2 += plant_alpha * (
        plant_target_acceleration_mps2 - net_acceleration_mps2
      )
      crossing_outputs.append(output)

  assert min(crossing_outputs) >= learned_trim_output - NEAR_ZERO_MAX_TRIM_LOSS_MPS2
  assert max(
    abs(current - previous)
    for previous, current in zip(crossing_outputs, crossing_outputs[1:], strict=False)
  ) <= NEGATIVE_HANDOFF_MAX_EFFORT_JERK_MPS3 * CONTROL_DT_S


def run_full_brake_and_still_negative_recovery(
    *,
    identity_feedforward: bool,
    monkeypatch,
) -> tuple[float, list[float], list[float]]:
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: COAST_PEDAL_DI),
  )
  controller = VirtualDAS(dt=CONTROL_DT_S)
  output = 0.0 if identity_feedforward else COAST_PEDAL_DI
  if identity_feedforward:
    monkeypatch.setattr(
      controller,
      "_feedforward",
      lambda acceleration_effort_mps2, _speed_mps: acceleration_effort_mps2,
    )
  controller.reset(measured_accel=0.0, commanded_accel=0.0, pedal_di_init=output)

  net_acceleration_mps2 = 0.0
  plant_alpha = CONTROL_DT_S / (PLANT_TAU_S + CONTROL_DT_S)
  delayed_outputs = [output] * round(PLANT_DELAY_S / CONTROL_DT_S)
  for _ in range(round(RESIDUAL_BUILD_HOLD_S / CONTROL_DT_S)):
    output = controller.update(
      0.0,
      v_ego=25.0,
      prev_pedal_di=output,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )
    applied_output = delayed_outputs.pop(0)
    delayed_outputs.append(output)
    acceleration_effort_mps2 = (
      applied_output
      if identity_feedforward
      else (applied_output - COAST_PEDAL_DI) * PEDAL_PLANT_ACCEL_PER_DI_MPS2
    )
    plant_target_acceleration_mps2 = acceleration_effort_mps2 - RESIDUAL_BUILD_LOAD_MPS2
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )

  learned_trim_output = output
  braking_outputs = []
  for _ in range(round(FULL_BRAKE_HOLD_S / CONTROL_DT_S)):
    output = controller.update(
      FULL_BRAKE_TARGET_MPS2,
      v_ego=25.0,
      prev_pedal_di=output,
      a_ego=0.0,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )
    braking_outputs.append(output)

  recovery_outputs = []
  for _ in range(round(STILL_NEGATIVE_RECOVERY_HOLD_S / CONTROL_DT_S)):
    output = controller.update(
      STILL_NEGATIVE_RECOVERY_TARGET_MPS2,
      v_ego=25.0,
      prev_pedal_di=output,
      a_ego=0.0,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )
    recovery_outputs.append(output)

  return learned_trim_output, braking_outputs, recovery_outputs


def test_handoff_shares_command_jerk_budget_in_both_directions(monkeypatch):
  learned_trim_effort, braking_efforts, recovery_efforts = run_full_brake_and_still_negative_recovery(
    identity_feedforward=True,
    monkeypatch=monkeypatch,
  )
  braking_jerks_mps3 = [
    (current - previous) / CONTROL_DT_S
    for previous, current in zip([learned_trim_effort] + braking_efforts[:-1], braking_efforts, strict=True)
  ]
  recovery_jerks_mps3 = [
    (current - previous) / CONTROL_DT_S
    for previous, current in zip([braking_efforts[-1]] + recovery_efforts[:-1], recovery_efforts, strict=True)
  ]

  minimum_braking_jerk_mps3 = min(braking_jerks_mps3)
  maximum_recovery_jerk_mps3 = max(recovery_jerks_mps3)
  assert minimum_braking_jerk_mps3 >= -VDAS_DECEL_JERK_MAX or math.isclose(
    minimum_braking_jerk_mps3,
    -VDAS_DECEL_JERK_MAX,
    abs_tol=1e-12,
  )
  assert maximum_recovery_jerk_mps3 <= VDAS_ACCEL_JERK_MAX or math.isclose(
    maximum_recovery_jerk_mps3,
    VDAS_ACCEL_JERK_MAX,
    abs_tol=1e-12,
  )


def test_handoff_full_brake_and_recovery_stay_inside_comfort_pedal_step(monkeypatch):
  learned_trim_pedal_di, braking_pedals_di, recovery_pedals_di = run_full_brake_and_still_negative_recovery(
    identity_feedforward=False,
    monkeypatch=monkeypatch,
  )
  braking_steps_di = [
    current - previous
    for previous, current in zip([learned_trim_pedal_di] + braking_pedals_di[:-1], braking_pedals_di, strict=True)
  ]
  recovery_steps_di = [
    current - previous
    for previous, current in zip([braking_pedals_di[-1]] + recovery_pedals_di[:-1], recovery_pedals_di, strict=True)
  ]

  assert min(braking_steps_di) >= -NEGATIVE_HANDOFF_MAX_PEDAL_STEP_DI
  assert max(recovery_steps_di) <= NEGATIVE_HANDOFF_MAX_PEDAL_STEP_DI


def run_speed_ramp_acceleration(
    target_acceleration_mps2: float,
    monkeypatch,
) -> tuple[float, list[SpeedRampPlantSample]]:
  """Integrate speed through the fallback transition using observed plant anchors."""
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: 0.0),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  controller.ff_model = FeedforwardModel(table_path="/nonexistent")
  pedal_di = float(interp(
    target_acceleration_mps2,
    ROUTE_PLANT_NET_ACCEL_BP,
    ROUTE_PLANT_PEDAL_DI_BP,
  ))
  net_acceleration_mps2 = target_acceleration_mps2
  speed_mps = SPEED_RAMP_INITIAL_MPS
  controller.reset(
    measured_accel=net_acceleration_mps2,
    commanded_accel=target_acceleration_mps2,
    pedal_di_init=pedal_di,
  )

  delayed_pedals_di = [pedal_di] * round(NOMINAL_ROUTE_PLANT.delay_s / CONTROL_DT_S)
  plant_alpha = CONTROL_DT_S / (NOMINAL_ROUTE_PLANT.tau_s + CONTROL_DT_S)
  warmup_steps = round(SPEED_RAMP_WARMUP_S / CONTROL_DT_S)
  timeout_steps = round(SPEED_RAMP_TIMEOUT_S / CONTROL_DT_S)
  warmup_accelerations_mps2 = []
  samples = []

  for step in range(warmup_steps + timeout_steps):
    pedal_di = controller.update(
      target_acceleration_mps2,
      v_ego=speed_mps,
      prev_pedal_di=pedal_di,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )
    applied_pedal_di = delayed_pedals_di.pop(0)
    delayed_pedals_di.append(pedal_di)
    plant_target_acceleration_mps2 = float(interp(
      applied_pedal_di,
      ROUTE_PLANT_PEDAL_DI_BP,
      ROUTE_PLANT_NET_ACCEL_BP,
    ))
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )

    if step < warmup_steps:
      warmup_accelerations_mps2.append(net_acceleration_mps2)
    else:
      speed_mps += net_acceleration_mps2 * CONTROL_DT_S
      samples.append(SpeedRampPlantSample(
        speed_mps=speed_mps,
        net_acceleration_mps2=net_acceleration_mps2,
        pedal_di=pedal_di,
      ))
      if speed_mps >= SPEED_RAMP_FINAL_MPS:
        break

  rolling_window_steps = round(SPEED_RAMP_ROLLING_WINDOW_S / CONTROL_DT_S)
  warmup_mean_acceleration_mps2 = sum(
    warmup_accelerations_mps2[-rolling_window_steps:]
  ) / rolling_window_steps
  return warmup_mean_acceleration_mps2, samples


def assert_speed_ramp_transition_is_smooth(
    target_acceleration_mps2: float,
    monkeypatch,
) -> None:
  warmup_mean_acceleration_mps2, samples = run_speed_ramp_acceleration(
    target_acceleration_mps2,
    monkeypatch,
  )
  jerks_mps3 = [
    (current.net_acceleration_mps2 - previous.net_acceleration_mps2) / CONTROL_DT_S
    for previous, current in zip(samples, samples[1:], strict=False)
  ]
  rolling_window_steps = round(SPEED_RAMP_ROLLING_WINDOW_S / CONTROL_DT_S)
  rolling_mean_accelerations_mps2 = [
    sum(sample.net_acceleration_mps2 for sample in samples[start:start + rolling_window_steps]) / rolling_window_steps
    for start in range(len(samples) - rolling_window_steps + 1)
  ]
  rolling_mean_droop_mps2 = warmup_mean_acceleration_mps2 - min(rolling_mean_accelerations_mps2)
  accumulated_tracking_error_mps = sum(
    max(target_acceleration_mps2 - sample.net_acceleration_mps2, 0.0) * CONTROL_DT_S
    for sample in samples
  )
  peak_absolute_jerk_mps3 = max(abs(jerk_mps3) for jerk_mps3 in jerks_mps3)
  peak_acceleration_mps2 = max(sample.net_acceleration_mps2 for sample in samples)

  assert samples[-1].speed_mps >= SPEED_RAMP_FINAL_MPS
  assert rolling_mean_droop_mps2 <= SPEED_RAMP_DROOP_LIMIT_MPS2, (
    f"{SPEED_RAMP_ROLLING_WINDOW_S:.2f} s mean drooped {rolling_mean_droop_mps2:.3f} m/s²; "
    + f"warmup mean {warmup_mean_acceleration_mps2:.3f} m/s²; "
    + f"accumulated tracking error {accumulated_tracking_error_mps:.3f} m/s"
  )
  assert accumulated_tracking_error_mps <= SPEED_RAMP_TRACKING_ERROR_LIMIT_MPS
  assert peak_absolute_jerk_mps3 <= SPEED_RAMP_JERK_LIMIT_MPS3
  assert peak_acceleration_mps2 <= SPEED_RAMP_PEAK_ACCEL_LIMIT_MPS2
  assert min(sample.pedal_di for sample in samples) > virtual_das.PEDAL_DI_MIN
  assert max(sample.pedal_di for sample in samples) < 50.0


def test_speed_ramp_through_fallback_transition_is_smooth(monkeypatch):
  assert_speed_ramp_transition_is_smooth(SPEED_RAMP_TARGET_ACCELERATIONS_MPS2[0], monkeypatch)


def test_speed_ramp_at_route_peak_remains_bounded(monkeypatch):
  assert_speed_ramp_transition_is_smooth(SPEED_RAMP_TARGET_ACCELERATIONS_MPS2[1], monkeypatch)


def test_route_shaped_positive_transition_bounds_delivered_acceleration(monkeypatch):
  samples = run_route_shaped_acceleration(NOMINAL_ROUTE_PLANT, monkeypatch)
  positive_tracking_excess_mps = sum(
    max(sample.net_acceleration_mps2 - sample.target_acceleration_mps2, 0.0) * CONTROL_DT_S
    for sample in samples
  )
  peak_positive_jerk_mps3 = max(
    (current.net_acceleration_mps2 - previous.net_acceleration_mps2) / CONTROL_DT_S
    for previous, current in zip(samples, samples[1:], strict=False)
  )
  final_samples = samples[-round(1.0 / CONTROL_DT_S):]
  final_mean_tracking_error_mps2 = sum(
    sample.net_acceleration_mps2 - sample.target_acceleration_mps2
    for sample in final_samples
  ) / len(final_samples)

  peak_acceleration_mps2 = peak_delivered_acceleration(samples)
  assert peak_acceleration_mps2 <= ROUTE_PEAK_ACCEL_LIMIT_MPS2, (
    f"delivered peak {peak_acceleration_mps2:.3f} m/s²; "
    + f"positive tracking excess {positive_tracking_excess_mps:.3f} m/s"
  )
  assert positive_tracking_excess_mps <= ROUTE_POSITIVE_EXCESS_LIMIT_MPS
  assert peak_positive_jerk_mps3 <= ROUTE_POSITIVE_JERK_LIMIT_MPS3
  assert abs(final_mean_tracking_error_mps2) <= ROUTE_FINAL_TRACKING_ERROR_LIMIT_MPS2
  assert min(sample.pedal_di for sample in samples) > virtual_das.PEDAL_DI_MIN
  assert max(sample.pedal_di for sample in samples) < 50.0


@pytest.mark.parametrize("plant", UNCERTAIN_ROUTE_PLANTS)
def test_route_shaped_positive_transition_survives_plant_uncertainty(monkeypatch, plant):
  samples = run_route_shaped_acceleration(plant, monkeypatch)

  assert peak_delivered_acceleration(samples) <= ROUTE_UNCERTAIN_PEAK_ACCEL_LIMIT_MPS2


def run_grade_hold(*, speed_mps: float, grade_acceleration_mps2: float,
                   duration_s: float, monkeypatch) -> list[GradePlantSample]:
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  acceleration_effort_mps2 = grade_acceleration_mps2
  monkeypatch.setattr(
    controller,
    "_feedforward",
    lambda requested_effort_mps2, _speed_mps: requested_effort_mps2,
  )

  orientation_ned = [0.0, math.asin(grade_acceleration_mps2 / GRAVITY), 0.0]
  for _ in range(round(GRADE_ESTIMATOR_SETTLING_S / CONTROL_DT_S)):
    controller.observe(a_ego=0.0, orientation_ned=orientation_ned)

  controller.reset(
    measured_accel=0.0,
    commanded_accel=0.0,
    pedal_di_init=acceleration_effort_mps2,
    preserve_grade=True,
  )

  delayed_efforts_mps2 = [acceleration_effort_mps2] * round(PLANT_DELAY_S / CONTROL_DT_S)
  plant_alpha = CONTROL_DT_S / (PLANT_TAU_S + CONTROL_DT_S)
  net_acceleration_mps2 = 0.0
  current_speed_mps = speed_mps
  samples = []

  for _ in range(round(duration_s / CONTROL_DT_S)):
    acceleration_effort_mps2 = controller.update(
      0.0,
      v_ego=current_speed_mps,
      prev_pedal_di=acceleration_effort_mps2,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=orientation_ned,
    )
    applied_effort_mps2 = delayed_efforts_mps2.pop(0)
    delayed_efforts_mps2.append(acceleration_effort_mps2)
    plant_target_acceleration_mps2 = applied_effort_mps2 - grade_acceleration_mps2
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )
    current_speed_mps += net_acceleration_mps2 * CONTROL_DT_S
    samples.append(GradePlantSample(
      net_acceleration_mps2,
      current_speed_mps,
      acceleration_effort_mps2,
      controller.inner_pid.i,
    ))

  return samples


@pytest.mark.parametrize("speed_mps", [0.0, 5.0, 15.0, 30.0])
@pytest.mark.parametrize("grade_acceleration_mps2", [GRADE_ACCEL_MPS2, -GRADE_ACCEL_MPS2])
def test_preserved_grade_handoff_holds_net_acceleration_for_1p5_seconds(
    monkeypatch, speed_mps, grade_acceleration_mps2):
  samples = run_grade_hold(
    speed_mps=speed_mps,
    grade_acceleration_mps2=grade_acceleration_mps2,
    duration_s=SHORT_HORIZON_S,
    monkeypatch=monkeypatch,
  )

  maximum_net_acceleration_mps2 = max(abs(sample.net_acceleration_mps2) for sample in samples)
  speed_drift_mps = samples[-1].speed_mps - speed_mps
  final_effort_mps2 = samples[-1].acceleration_effort_mps2

  assert maximum_net_acceleration_mps2 <= NET_ACCEL_TOLERANCE_MPS2
  assert abs(speed_drift_mps) <= SHORT_HORIZON_SPEED_DRIFT_MPS
  assert final_effort_mps2 == pytest.approx(
    grade_acceleration_mps2,
    abs=EFFORT_TOLERANCE_MPS2,
  )


@pytest.mark.parametrize("speed_mps", [0.0, 5.0, 15.0, 30.0])
@pytest.mark.parametrize("grade_acceleration_mps2", [GRADE_ACCEL_MPS2, -GRADE_ACCEL_MPS2])
def test_steady_grade_hold_does_not_double_compensate(
    monkeypatch, speed_mps, grade_acceleration_mps2):
  samples = run_grade_hold(
    speed_mps=speed_mps,
    grade_acceleration_mps2=grade_acceleration_mps2,
    duration_s=STEADY_HORIZON_S,
    monkeypatch=monkeypatch,
  )
  steady_sample_count = round(STEADY_WINDOW_S / CONTROL_DT_S)
  steady_samples = samples[-steady_sample_count:]

  assert max(abs(sample.net_acceleration_mps2) for sample in steady_samples) <= NET_ACCEL_TOLERANCE_MPS2
  assert max(
    abs(sample.acceleration_effort_mps2 - grade_acceleration_mps2)
    for sample in steady_samples
  ) <= EFFORT_TOLERANCE_MPS2
  assert max(abs(sample.integral_trim_mps2) for sample in steady_samples) <= 0.02


def run_grade_step(*, speed_mps: float, uphill_load_mps2: float,
                   duration_s: float, plant: DelayedPedalPlantCase,
                   monkeypatch) -> list[PedalPlantSample]:
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: COAST_PEDAL_DI),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  controller.reset(
    measured_accel=0.0,
    commanded_accel=0.0,
    pedal_di_init=COAST_PEDAL_DI,
  )
  pedal_di = COAST_PEDAL_DI
  net_acceleration_mps2 = 0.0

  for _ in range(round(2.0 / CONTROL_DT_S)):
    pedal_di = controller.update(
      0.0,
      v_ego=speed_mps,
      prev_pedal_di=pedal_di,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )

  delay_steps = round(plant.delay_s / CONTROL_DT_S)
  delayed_pedals_di = [pedal_di] * delay_steps
  plant_alpha = CONTROL_DT_S / (plant.tau_s + CONTROL_DT_S)
  sensed_grade_mps2 = uphill_load_mps2 * plant.grade_sensor_scale
  orientation_ned = [0.0, math.asin(sensed_grade_mps2 / GRAVITY), 0.0]
  current_speed_mps = speed_mps
  speed_delta_mps = 0.0
  samples = []

  for step in range(round(duration_s / CONTROL_DT_S)):
    pedal_di = controller.update(
      0.0,
      v_ego=max(current_speed_mps, 0.0),
      prev_pedal_di=pedal_di,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=orientation_ned,
    )
    applied_pedal_di = delayed_pedals_di.pop(0)
    delayed_pedals_di.append(pedal_di)
    plant_target_acceleration_mps2 = (
      (applied_pedal_di - COAST_PEDAL_DI) * plant.acceleration_per_di_mps2
      - uphill_load_mps2
    )
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )
    speed_delta_mps += net_acceleration_mps2 * CONTROL_DT_S
    current_speed_mps = speed_mps + speed_delta_mps
    samples.append(PedalPlantSample(
      elapsed_s=(step + 1) * CONTROL_DT_S,
      net_acceleration_mps2=net_acceleration_mps2,
      speed_delta_mps=speed_delta_mps,
      pedal_di=pedal_di,
    ))

  return samples


@pytest.mark.parametrize("uphill_load_mps2", [STEP_GRADE_ACCEL_MPS2, -STEP_GRADE_ACCEL_MPS2])
def test_flat_to_grade_step_improves_on_no_pitch_baseline_at_1p5_seconds(
    monkeypatch, uphill_load_mps2):
  samples = run_grade_step(
    speed_mps=15.0,
    uphill_load_mps2=uphill_load_mps2,
    duration_s=SHORT_HORIZON_S,
    plant=NOMINAL_PLANT,
    monkeypatch=monkeypatch,
  )
  no_pitch_samples = run_grade_step(
    speed_mps=15.0,
    uphill_load_mps2=uphill_load_mps2,
    duration_s=SHORT_HORIZON_S,
    plant=replace(NOMINAL_PLANT, grade_sensor_scale=0.0),
    monkeypatch=monkeypatch,
  )
  checkpoint_samples = [sample for sample in samples if sample.elapsed_s >= 1.4]
  mean_net_acceleration_mps2 = sum(
    sample.net_acceleration_mps2 for sample in checkpoint_samples
  ) / len(checkpoint_samples)

  assert abs(mean_net_acceleration_mps2) <= STEP_RESPONSE_TOLERANCE_MPS2
  if uphill_load_mps2 > 0.0:
    assert mean_net_acceleration_mps2 <= STEP_RESPONSE_UPHILL_OVERSHOOT_LIMIT_MPS2
  assert (
    abs(samples[-1].speed_delta_mps) + STEP_RESPONSE_MIN_DRIFT_IMPROVEMENT_MPS
    <= abs(no_pitch_samples[-1].speed_delta_mps)
  )
  assert abs(samples[-1].pedal_di - COAST_PEDAL_DI) >= STEP_RESPONSE_MIN_PEDAL_DELTA_DI
  assert math.copysign(1.0, samples[-1].pedal_di - COAST_PEDAL_DI) == math.copysign(
    1.0,
    uphill_load_mps2,
  )


@pytest.mark.parametrize("speed_mps", [0.0, 5.0, 15.0, 30.0])
@pytest.mark.parametrize("uphill_load_mps2", [MATRIX_GRADE_ACCEL_MPS2, -MATRIX_GRADE_ACCEL_MPS2])
@pytest.mark.parametrize("plant", UNCERTAIN_PLANTS)
def test_grade_hold_survives_speed_and_plant_uncertainty(
    monkeypatch, speed_mps, uphill_load_mps2, plant):
  samples = run_grade_step(
    speed_mps=speed_mps,
    uphill_load_mps2=uphill_load_mps2,
    duration_s=STEADY_HORIZON_S,
    plant=plant,
    monkeypatch=monkeypatch,
  )
  steady_samples = samples[-round(STEADY_WINDOW_S / CONTROL_DT_S):]
  mean_net_acceleration_mps2 = sum(
    sample.net_acceleration_mps2 for sample in steady_samples
  ) / len(steady_samples)

  assert abs(mean_net_acceleration_mps2) <= STEADY_MEAN_TOLERANCE_MPS2
  assert max(abs(sample.net_acceleration_mps2) for sample in steady_samples) <= STEADY_PEAK_TOLERANCE_MPS2
  assert abs(samples[-1].speed_delta_mps) <= STEADY_SPEED_DRIFT_MPS
  assert min(sample.pedal_di for sample in steady_samples) >= virtual_das.PEDAL_DI_MIN + PHYSICAL_RAIL_MARGIN_DI
  assert max(sample.pedal_di for sample in steady_samples) <= 50.0 - PHYSICAL_RAIL_MARGIN_DI
  assert math.copysign(1.0, steady_samples[-1].pedal_di - COAST_PEDAL_DI) == math.copysign(
    1.0,
    uphill_load_mps2,
  )
