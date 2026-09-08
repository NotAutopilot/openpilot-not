"""Tests for Pre-AP longitudinal acceleration limits."""

from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap import interface
from opendbc.car.tesla.preap import virtual_das
from opendbc.car.tesla.preap.nap_conf import PEDAL_MAX_VALUES


CAPTURE_SPEED = 31.394  # m/s, mean over 4.7 seconds before the speed step
CAPTURE_STEADY_OUTER_ACCEL = 0.37296  # m/s², mean over the same window
CAPTURE_STEADY_PEDAL_DI = 16.651  # median decoded enabled 0x551 command
# Solving the deployed feedforward mapping at the values above gives 4.997 DI.
# This is inferred controller zero-torque state, not pedal calibration voltage.
CAPTURE_INFERRED_ZERO_TORQUE_DI = 5.0
HIGH_SPEED_FALLBACK_FIELD_SCALE = 0.65


class CapturedZeroTorque:
  def get(self, _v_ego):
    return CAPTURE_INFERRED_ZERO_TORQUE_DI


@pytest.mark.parametrize(
  ("personality", "maximum_accel"),
  (
    (0, 0.70),
    (1, 0.60),
    (2, 0.50),
  ),
)
def test_highway_set_speed_step_uses_comfort_ceiling(monkeypatch, personality, maximum_accel):
  """Keep a speed increase from reproducing the measured highway pedal surge."""
  params = SimpleNamespace(get=lambda _key, return_default: str(personality).encode())
  monkeypatch.setattr(interface, "_params", params)

  _, accel_max = interface.get_preap_accel_limits(31.35)

  assert accel_max <= maximum_accel


def test_highway_speed_step_bounds_pedal_command_rise(monkeypatch):
  """Bound the 0x551 rise that produced 1.22 m/s² in the highway capture."""
  params = SimpleNamespace(get=lambda _key, return_default: b"1")
  monkeypatch.setattr(interface, "_params", params)
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: PEDAL_MAX_VALUES),
  )
  monkeypatch.setattr(virtual_das, "get_zero_torque", lambda: CapturedZeroTorque())

  _, accel_max = interface.get_preap_accel_limits(CAPTURE_SPEED)
  controller = virtual_das.VirtualDAS(dt=0.02)
  pedal_di = 0.0

  for _ in range(200):
    pedal_di = controller.update(
      CAPTURE_STEADY_OUTER_ACCEL,
      v_ego=CAPTURE_SPEED,
      prev_pedal_di=pedal_di,
      a_ego=0.0,
      freeze_integrator=True,
    )
  steady_pedal_di = pedal_di
  calibrated_capture_bound_di = CAPTURE_INFERRED_ZERO_TORQUE_DI + HIGH_SPEED_FALLBACK_FIELD_SCALE * (
    CAPTURE_STEADY_PEDAL_DI - CAPTURE_INFERRED_ZERO_TORQUE_DI
  )
  assert CAPTURE_INFERRED_ZERO_TORQUE_DI < steady_pedal_di <= calibrated_capture_bound_di

  pedal_commands = []
  for _ in range(50):
    pedal_di = controller.update(
      accel_max,
      v_ego=CAPTURE_SPEED,
      prev_pedal_di=pedal_di,
      a_ego=0.0,
      freeze_integrator=True,
    )
    pedal_commands.append(pedal_di)

  pedal_rise_di = max(pedal_commands) - steady_pedal_di
  assert 1.0 <= pedal_rise_di <= 7.0


@pytest.mark.parametrize(
  ("personality", "expected_accel"),
  (
    (0, 1.0),
    (1, 0.9),
    (2, 0.8),
  ),
)
def test_city_accel_limits_are_unchanged(monkeypatch, personality, expected_accel):
  params = SimpleNamespace(get=lambda _key, return_default: str(personality).encode())
  monkeypatch.setattr(interface, "_params", params)

  _, accel_max = interface.get_preap_accel_limits(15.0)

  assert accel_max == pytest.approx(expected_accel)
