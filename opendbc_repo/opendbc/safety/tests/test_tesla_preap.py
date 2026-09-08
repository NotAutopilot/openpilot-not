#!/usr/bin/env python3
import unittest

from opendbc.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.values import CAR, CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

# Safety param flags matching tesla_preap.h (LONG_CONTROL removed — dead code)
PREAP_FLAG_ENABLE_PEDAL = 1
PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4
PREAP_FLAG_HANDS_ON_PAUSE = 8
PREAP_FLAG_PEDAL_BUS_ZERO = 1 << 5
PREAP_FLAG_PEDAL_CALIBRATION = 1 << 6

# Stalk lever positions from tesla_preap.h / tesla_preap.dbc
STALK_IDLE = 0
STALK_FWD_CANCEL = 1
STALK_RWD_ENGAGE = 2


def _fix_epas_checksum(msg):
  """Compute Tesla byte-sum checksum for EPAS_sysStatus (checksum at byte 7)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 7:
      chk += data[i]
  data[7] = chk & 0xFF
  return addr, bytes(data), bus


def _fix_das_checksum(msg):
  """Compute Tesla byte-sum checksum for DAS_steeringControl (checksum at byte 3)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 3:
      chk += data[i]
  data[3] = chk & 0xFF
  return addr, bytes(data), bus


def _get_preap_vm():
  """VehicleModel matching PREAP_STEERING_PARAMS in tesla_preap.h.

  Built from Pre-AP CarSpecs directly so this file does not import
  tesla.interface (sunnypilot's CarInterfaceBase pulls Pre-AP boot).
  """
  specs = CAR.TESLA_MODEL_S_PREAP.config.specs
  CP = CarParams()
  CP.mass = specs.mass + STD_CARGO_KG
  CP.wheelbase = specs.wheelbase
  CP.steerRatio = specs.steerRatio
  CP.centerToFront = specs.wheelbase * specs.centerToFrontRatio
  CP.rotationalInertia = scale_rot_inertia(CP.mass, CP.wheelbase)
  CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(
    CP.mass, CP.wheelbase, CP.centerToFront, specs.tireStiffnessFactor)
  return VehicleModel(CP)


class TeslaPreAPTestMixin(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  # Abstract base class — concrete subclasses (SteeringOnly, WithPedal) do the work.
  # __test__ = False prevents pytest from collecting this class directly (it still
  # gets collected via MRO without this, because CarSafetyTest is a TestCase).
  __test__ = False
  # Pre-AP has no relay and no bus 2 forwarding
  RELAY_MALFUNCTION_ADDRS = {}
  FWD_BUS_LOOKUP = {}
  FWD_BLACKLISTED_ADDRS = {}

  TX_MSGS = [
    [0x488, 0],  # DAS_steeringControl
    [0x2B9, 0],  # DAS_control
    [0x214, 0],  # EPB_epasControl
    [0x551, 0],  # Pedal bus 0
    [0x551, 2],  # Pedal bus 2
    [0x45,  0],  # STW_ACTN_RQ (stalk spoof)
    [0x3E9, 0],  # DAS_bodyControls (turn signal)
  ]

  STANDSTILL_THRESHOLD = 0.5 / 3.6  # 0.5 kph in m/s

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10
  LATERAL_FREQUENCY = 50  # Hz

  # Tesla uses VM-based limits, not breakpoint tables
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  GAS_PRESSED_THRESHOLD = 0  # DI_torque1 byte 6 != 0

  cnt_epas = 0
  cnt_angle_cmd = 0

  packer: CANPackerSafety

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = _get_preap_vm()
    self.packer = CANPackerSafety("tesla_preap")
    self.safety = libsafety_py.libsafety

  def _angle_cmd_msg(self, angle, state, increment_timer=True, bus=0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values,
                                           fix_checksum=_fix_das_checksum)

  def _angle_meas_msg(self, angle, hands_on_level=0, eac_status=1, eac_error_code=0):
    values = {
      "EPAS_internalSAS": angle,
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code,
      "EPAS_sysStatusCounter": self.cnt_epas % 16,
    }
    self.__class__.cnt_epas += 1
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values,
                                           fix_checksum=_fix_epas_checksum)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_safety("BrakeMessage", 0, values)

  def _speed_msg(self, speed):
    values = {"ESP_vehicleSpeed": speed * 3.6}  # m/s to kph
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _speed_msg_2(self, speed):
    return None  # Pre-AP has no second speed source

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_safety("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    lever = STALK_RWD_ENGAGE if enable else STALK_FWD_CANCEL
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever})

  def _lkas_button_msg(self, enabled):
    raise unittest.SkipTest("Pre-AP has no LKAS/MADS button")

  def _acc_state_msg(self, enabled):
    raise unittest.SkipTest("Pre-AP arms MADS from stalk, not ACC main")

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("DI_torque2", 0, {"DI_gear": gear})

  def _di_brake_msg(self, brake):
    values = {"DI_gear": 4, "DI_brakePedal": 1 if brake else 0}
    return self.packer.make_can_msg_safety("DI_torque2", 0, values)

  def _door_msg(self, door_fl=0, door_fr=0, door_rl=0, door_rr=0):
    values = {
      "DOOR_STATE_FL": door_fl,
      "DOOR_STATE_FR": door_fr,
      "DOOR_STATE_RL": door_rl,
      "DOOR_STATE_RR": door_rr,
    }
    return self.packer.make_can_msg_safety("GTW_carState", 0, values)

  def _engage_and_advance_timer(self):
    """Engage via stalk and advance timer past the 600ms echo filter window."""
    self._rx(self._pcm_status_msg(True))
    self.safety.set_timer(700000)

  def _stalk_lever(self, lever):
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever})

  def _model_board_heartbeat_long_timeout(self):
    # panda/board/main.c 1Hz: 3 ticks of controls_allowed && !heartbeat_engaged
    # sets controls_allowed=false and does not clear cruise_engaged_prev.
    # libsafety does not run that countdown; apply the resulting state.
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    self.safety.set_controls_allowed(False)

  # =====================================================================
  # Base class overrides for Pre-AP differences
  # =====================================================================

  def test_angle_cmd_when_enabled(self):
    # Tesla uses VM-based limits — test_lateral_accel_limit covers this
    pass

  def test_angle_cmd_when_disabled(self):
    self._rx(self._angle_meas_msg(0))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, 0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(100, 0)))

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_vehicle_moving(self):
    # Pre-AP uses: vehicle_moving = speed > (0.5f * KPH_TO_MS)
    # Due to float32 precision in the DBC factor (0.00999999978 vs 0.01),
    # exactly 0.5 kph may register as slightly above threshold. Use values
    # that are unambiguously below/above regardless of float precision.
    self.assertFalse(self.safety.get_vehicle_moving())
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_vehicle_moving())
    # 0.3 kph → clearly below 0.5 kph threshold
    self._rx(self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 0.3}))
    self.assertFalse(self.safety.get_vehicle_moving())
    # 1.0 kph → clearly above 0.5 kph threshold
    self._rx(self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 1.0}))
    self.assertTrue(self.safety.get_vehicle_moving())

  def test_prev_user_brake(self):
    # PRE-AP BRAKE ARCHITECTURE:
    # The panda keeps the framework's brake_pressed=false so generic brake
    # disengagement never kills lateral control. Separate raw-brake latches
    # still block enabled pedal commands at the TX hook.
    #
    # Python ORs DI_torque2.DI_brakePedal and BrakeMessage.driverBrakeStatus,
    # drops longitudinal while keeping cruise/lateral enabled, and suppresses
    # public CarState.brakePressed. Panda independently ORs the same two raw
    # sources so it closes the interval before Python observes BrakeMessage.
    #
    # Framework invariant: brake_pressed remains false; pedal authority does not.
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())

  def test_allow_user_brake_at_zero_speed(self):
    # Brake does not clear controls_allowed because lateral remains available;
    # the separate pedal TX interlock is covered below.
    self._rx(self._speed_msg(0))
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_not_allow_user_brake_when_moving(self):
    # Brake while moving still leaves controls_allowed set for lateral. Python
    # drops longitudinal, while panda independently blocks enabled pedal TX.
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_disengage_on_brake(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_brake_disengage_with_acc_main_off(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_brake_disengage_with_control_request(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_heartbeat_agreement_disengage_on_brake(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_heartbeat_agreement_pause_on_brake(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_mads_brake_disengage_lateral_only_mode(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_pause_lateral_on_brake(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_pause_lateral_on_brake_persistent_control_allowed_off(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_pause_lateral_on_brake_with_pressed_and_released(self):
    raise unittest.SkipTest("Pre-AP keeps brake_pressed false so MADS brake exit does not run")

  def test_cruise_engaged_prev(self):
    # Pre-AP uses a 600ms echo filter on stalk cancel. Advancing the timer
    # past the window is required for cancel to take effect.
    for engaged in [True, False]:
      self._rx(self._pcm_status_msg(engaged))
      if not engaged:
        self.safety.set_timer(700000)
        self._rx(self._pcm_status_msg(False))
      self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())

  def test_disable_control_allowed_from_cruise(self):
    self._engage_and_advance_timer()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_body_controls_turn_indicator_allowed(self):
    # Valid turn-indicator requests (0-3) are allowed when controls_allowed.
    self.safety.set_controls_allowed(True)
    for turn in range(4):
      msg = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
              {"DAS_turnIndicatorRequest": turn})
      self.assertTrue(self._tx(msg), f"turn={turn} should be allowed")

  def test_body_controls_blocked_when_not_allowed(self):
    # Like other actuation, blocked when controls are not allowed.
    self.safety.set_controls_allowed(False)
    msg = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
            {"DAS_turnIndicatorRequest": 1})
    self.assertFalse(self._tx(msg))

  def test_epas_and_body_tx_allowed_with_lateral_only(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    epas = self.packer.make_can_msg_safety("EPB_epasControl", 0, {"EPB_epasEACAllow": 1})
    body = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
            {"DAS_turnIndicatorRequest": 1})
    self.assertTrue(self._tx(epas))
    self.assertTrue(self._tx(body))

  def test_body_tx_blocked_when_both_off(self):
    # NAP does not gate 0x214 on controls_allowed; only 0x3E9 is gated.
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    body = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
            {"DAS_turnIndicatorRequest": 1})
    self.assertFalse(self._tx(body))

  def test_stalk_cancel_clears_lateral(self):
    self._engage_and_advance_timer()
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_gear_clears_lateral(self):
    self._rx(self._pcm_status_msg(True))
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._gear_msg(0))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_door_clears_lateral(self):
    self._rx(self._pcm_status_msg(True))
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._door_msg(door_fl=1))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  # =====================================================================
  # Pre-AP specific safety tests
  # =====================================================================

  def test_gear_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(4))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_door_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._door_msg(door_fl=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_steering_disengage_hands_on(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_steering_disengage_epas_error_codes(self):
    # EPAS error codes 6-9 with EAC_INHIBITED (status=0) should disengage.
    # Must reinit safety between iterations to clear stale state.
    for error_code in [6, 7, 8, 9]:
      self.setUp()
      self._setup_safety_hooks()
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed(), f"Setup failed for error code {error_code}")
      self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=error_code))
      self.assertFalse(self.safety.get_controls_allowed(), f"Error code {error_code} should disengage")

  def test_steering_no_disengage_on_other_error_codes(self):
    for error_code in [0, 1, 2, 3, 4, 5, 10, 11, 12]:
      self.setUp()
      self._setup_safety_hooks()
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed())
      self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=error_code))
      self.assertTrue(self.safety.get_controls_allowed(), f"Error code {error_code} should NOT disengage")

  def test_stalk_cancel_echo_filter(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # Cancel within echo window should be filtered
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    # Cancel after echo window should work
    self.safety.set_timer(700000)
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stalk_rearm_after_steering_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self.assertFalse(self.safety.get_controls_allowed())
    # Held MAIN after hands recovery must not rearm.
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._stalk_lever(STALK_IDLE))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_fresh_pull_rearms_after_heartbeat_stale_prev(self):
    self._rx(self._pcm_status_msg(True))
    self._model_board_heartbeat_long_timeout()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    # Held lever==2 must not rearm.
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    self._rx(self._stalk_lever(STALK_IDLE))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_steering_control_type(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._angle_meas_msg(0))
    for control_type in range(4):
      should_tx = control_type in (0, 1)
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, control_type)),
                       f"Control type {control_type} should {'pass' if should_tx else 'block'}")

  def test_epas_control_type(self):
    self.safety.set_controls_allowed(True)
    for mode in range(8):
      msg = self.packer.make_can_msg_safety("EPB_epasControl", 0, {"EPB_epasEACAllow": mode})
      should_tx = mode <= 1
      self.assertEqual(should_tx, self._tx(msg),
                       f"EPB mode {mode} should {'pass' if should_tx else 'block'}")

  def test_no_aeb(self):
    self.safety.set_controls_allowed(True)
    for aeb_event in range(4):
      msg = self.packer.make_can_msg_safety("DAS_control", 0, {"DAS_aebEvent": aeb_event})
      should_tx = aeb_event == 0
      self.assertEqual(should_tx, self._tx(msg),
                       f"AEB event {aeb_event} should {'pass' if should_tx else 'block'}")

  def test_lateral_accel_limit(self):
    # Verify VM-based lateral accel limits constrain steering at speed.
    # Ramp steering angle up in max_delta increments at a fixed speed, find the
    # angle at which the panda blocks further increases, and assert it's within
    # a tight tolerance of Python's VehicleModel computation.
    #
    # Float precision note: panda uses float32 for the curvature_factor
    # computation while Python uses float64. The difference is typically < 2°
    # at highway speeds. We allow 25% tolerance to absorb this while still
    # catching any bug that would make the limit off by a factor of 2 or more
    # (e.g. wrong slip_factor, wrong MAX_LATERAL_ACCEL, wrong wheelbase).
    #
    # TODO: for bit-exact boundary testing, port the approach from
    # test_tesla_hw1.py (round_angle + _reset_speed_measurement +
    # set_desired_angle_last) which does precise +1/+2 CAN-unit tests by
    # matching the panda's float32 arithmetic in Python.
    for speed in [20.0, 30.0]:
      self.setUp()
      self._setup_safety_hooks()
      self.safety.set_controls_allowed(True)
      # Must fill the vehicle_speed sample buffer (6 slots) so min converges
      for _ in range(10):
        self._rx(self._speed_msg(speed))
      self._rx(self._angle_meas_msg(0))
      self._tx(self._angle_cmd_msg(0, 1))

      expected_max = get_max_angle_vm(speed, self.VM, CarControllerParams)
      max_delta = get_max_angle_delta_vm(max(speed, 1), self.VM, CarControllerParams)
      angle = 0.0
      blocked_at = None
      for _ in range(5000):
        next_angle = angle + max_delta
        if next_angle > self.STEER_ANGLE_MAX:
          break
        if not self._tx(self._angle_cmd_msg(next_angle, 1)):
          blocked_at = next_angle
          break
        angle = next_angle

      self.assertIsNotNone(
        blocked_at,
        f"Speed {speed}: VM limit never blocked — reached {angle:.1f} deg (Python expected max {expected_max:.1f} deg)",
      )
      # Tight bound: blocked angle must be within ±25% of Python's computation.
      # Absorbs float32/float64 drift but catches order-of-magnitude bugs.
      lower_bound = expected_max * 0.75
      upper_bound = expected_max * 1.25
      self.assertGreaterEqual(
        blocked_at,
        lower_bound,
        f"Speed {speed}: blocked at {blocked_at:.1f} deg — too LOW (expected ~{expected_max:.1f}, bound {lower_bound:.1f})",
      )
      self.assertLessEqual(
        blocked_at,
        upper_bound,
        f"Speed {speed}: blocked at {blocked_at:.1f} deg — too HIGH (expected ~{expected_max:.1f}, bound {upper_bound:.1f})",
      )

  def _setup_safety_hooks(self):
    """Subclasses call this to set up the correct safety hooks."""
    raise NotImplementedError


class TestTeslaPreAPSteeringOnly(TeslaPreAPTestMixin, unittest.TestCase):
  """Pre-AP with no pedal — lateral only."""
  __test__ = True  # re-enable collection (mixin sets __test__=False)

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()

  def test_pedal_blocked_without_flag(self):
    self.safety.set_controls_allowed(True)
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_no_pedal_does_not_invalidate_rx_checks(self):
    # Regression: route 02ae0a637825acd6|196fda2496 — tester with no Comma Pedal
    # hit "Controls Mismatch" because the 0x552 rx_check used frequency=0,
    # which divides by zero in safety_tick (safety.h:330) and marks it lagging.
    # When ENABLE_PEDAL is unset, 0x552 must not be in rx_checks at all.
    di_state = self.packer.make_can_msg_safety("DI_state", 0, {"DI_state": 1})
    for msg in (self._angle_meas_msg(0), self._pcm_status_msg(False), self._speed_msg(0),
                self._user_brake_msg(False), self._user_gas_msg(0), self._gear_msg(4),
                self._door_msg(), di_state):
      self._rx(msg)
    # Tick 500ms after msgs — under the 1s min lag window for real checks but long
    # enough that a freq=0 divide-by-zero would have already tripped the pedal row.
    self.safety.set_timer(int(5e5))
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid(),
                    "rx_checks must stay valid without a pedal installed")


class TestTeslaPreAPWithPedal(TeslaPreAPTestMixin, unittest.TestCase):
  """Pre-AP with Comma Pedal enabled."""
  __test__ = True  # re-enable collection (mixin sets __test__=False)

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, PREAP_FLAG_ENABLE_PEDAL)
    self.safety.init_tests()

  # Pedal interceptor (0x552) values are raw 16-bit integers read by the panda as
  # `(data[0] << 8) | data[1]`. The DBC scales them to physical:
  #   physical = raw * 0.0507968128 - 22.85856576
  # Panda threshold: raw > 650 → gas_pressed (chosen from real drive data; see
  # comments in tesla_preap.h rx_hook). Helper values below:
  PEDAL_RAW_AT_REST_MAX = 633      # max observed at rest in real drive data
  PEDAL_RAW_NOISE_THRESHOLD = 650  # panda threshold
  PEDAL_RAW_CLEAR_PRESS = 800      # clearly pressed

  @staticmethod
  def _raw_to_physical(raw):
    return raw * 0.0507968128 - 22.85856576

  def _pedal_msg(self, raw_value, bus=0):
    """Craft a 0x552 message with the given raw value by converting to physical."""
    return self.packer.make_can_msg_safety("GAS_SENSOR", bus,
                                           {"INTERCEPTOR_GAS": self._raw_to_physical(raw_value)})

  def _user_gas_msg(self, gas):
    # With pedal enabled, gas is detected from pedal interceptor (0x552),
    # not DI_torque1. The C code ignores DI_torque1 gas when pedal is active.
    # Use clearly pressed value when gas=True; clearly not pressed when gas=False.
    raw = self.PEDAL_RAW_CLEAR_PRESS if gas else 400
    return self._pedal_msg(raw)

  def test_pedal_allowed_with_flag(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertTrue(self._tx(msg))

  def test_pedal_blocked_without_controls(self):
    self.assertFalse(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_pedal_not_allowed_with_lateral_only(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_pedal_tx_after_heartbeat_stale_prev_fresh_pull(self):
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(enable_msg))
    self._model_board_heartbeat_long_timeout()
    self.assertFalse(self._tx(enable_msg))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self._tx(enable_msg))
    self._rx(self._stalk_lever(STALK_IDLE))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(enable_msg))

  def test_pedal_gas_detection_bus_0(self):
    # Verify pedal gas detection works on bus 0 (first wiring config).
    self.assertFalse(self.safety.get_gas_pressed_prev())
    # Clearly pressed: raw 800 → > 650 → gas_pressed=True
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=0))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    "Pedal gas on bus 0 must set gas_pressed")
    # Clearly not pressed: raw 400 → < 650 → gas_pressed=False
    self._rx(self._pedal_msg(400, bus=0))
    self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_pedal_gas_detection_bus_2(self):
    # Verify pedal gas detection works on bus 2 (second wiring config).
    # Regression test: earlier version had `if (msg->bus != 0U) return;` at the
    # top of rx_hook that broke bus-2-wired pedals.
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    "Pedal gas on bus 2 must set gas_pressed (wiring config variant)")

  def test_pedal_gas_blocks_longitudinal_tx(self):
    # Full-chain test: pedal press → gas_pressed → !get_longitudinal_allowed() → pedal TX blocked.
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_longitudinal_allowed())
    # Press pedal (clearly pressed)
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=0))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    # Pedal TX must be blocked
    tx_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(tx_msg), "Pedal TX must be blocked during gas press")

  def test_pedal_rest_noise_does_not_trigger_gas(self):
    # Regression test for the pedal-engagement bug found in drive d0cdc986c5d023f5.
    # The pedal interceptor's resting voltage oscillates with noise; real Pre-AP
    # drive data showed raw values 424-633 while the driver was NOT pressing gas.
    # The original threshold of 450 was inside this noise range, causing false
    # gas_pressed readings that blocked pedal TX and prevented engagement.
    #
    # Verify that values across the entire observed rest-noise range do NOT
    # trigger gas_pressed.
    for raw in [424, 450, 475, 500, 550, 600, self.PEDAL_RAW_AT_REST_MAX]:
      for bus in [0, 2]:
        self.setUp()
        self._setup_safety_hooks()
        self._rx(self._pedal_msg(raw, bus=bus))
        self.assertFalse(self.safety.get_gas_pressed_prev(),
                         f"Raw {raw} on bus {bus} must NOT trigger gas_pressed (in rest noise range)")

  def test_pedal_rest_noise_does_not_block_longitudinal(self):
    # End-to-end regression test: after engaging, pedal rest noise must not cause
    # longitudinal TX to be blocked. Before this fix, noise-level raw values
    # (450-633) were stuck setting gas_pressed=True, blocking all pedal TX.
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    tx_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    # Pump pedal messages across the at-rest noise range; TX must remain allowed
    for raw in [424, 450, 500, 550, 600, 633]:
      self._rx(self._pedal_msg(raw, bus=2))  # real drive had pedal on bus 2
      self.assertFalse(self.safety.get_gas_pressed_prev(),
                       f"Raw {raw} (noise) must not set gas_pressed")
      self.assertTrue(self._tx(tx_msg),
                      f"Pedal TX must be allowed at raw {raw} (noise range)")

  def test_pedal_release_enable_0_always_allowed(self):
    # A disabled GAS_COMMAND relinquishes authority and lets Comma Pedal pass
    # the driver's OEM pedal voltage through. The controller sends it once on
    # authority loss; panda must allow that release regardless of engagement.
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})

    # Case 1: not engaged, no gas — still allowed (benign)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(disable_msg),
                    "enable=0 must be allowed when not engaged")

    # Case 2: driver gas must not prevent the one-shot authority release.
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertTrue(self._tx(disable_msg),
                    "enable=0 must be allowed during gas override")

  def test_pedal_enable_1_blocked_on_gas_press(self):
    # Conversely, enable=1 (authoritative accel command) MUST be blocked
    # when driver is pressing gas, preventing openpilot from overriding the driver.
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(enable_msg), "enable=1 allowed before gas press")

    # Driver presses gas
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertFalse(self._tx(enable_msg),
                     "enable=1 must be blocked during driver gas press")

  def test_each_raw_brake_source_blocks_only_enabled_pedal_commands(self):
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})

    for source, brake_msg in (("DI_torque2", self._di_brake_msg(True)),
                              ("BrakeMessage", self._user_brake_msg(True))):
      with self.subTest(source=source):
        self.setUp()
        self._rx(self._pcm_status_msg(True))
        self.assertTrue(self._tx(enable_msg))

        self._rx(brake_msg)

        self.assertTrue(self.safety.get_controls_allowed())
        self.assertFalse(self._tx(enable_msg))
        self.assertTrue(self._tx(disable_msg))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))

  def test_di_brake_closes_twenty_ms_seam_and_sources_clear_independently(self):
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    self._rx(self._pcm_status_msg(True))
    self._rx(self._di_brake_msg(False))
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self._tx(enable_msg))

    self._rx(self._di_brake_msg(True))
    self.assertFalse(self._tx(enable_msg))

    self.safety.set_timer(20000)
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self._tx(enable_msg))

    self._rx(self._di_brake_msg(False))
    self.assertTrue(self._tx(enable_msg))

    self._rx(self._user_brake_msg(True))
    self.assertFalse(self._tx(enable_msg))
    self._rx(self._di_brake_msg(False))
    self.assertFalse(self._tx(enable_msg))

    self._rx(self._user_brake_msg(False))
    self.assertTrue(self._tx(enable_msg))

  def test_pedal_enable_0_blocked_without_flag(self):
    # If PREAP_FLAG_ENABLE_PEDAL is not set, NO 0x551 TX is allowed
    # (not even enable=0). This is the "pedal feature disabled" gate.
    # Override setUp to init without the pedal flag.
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})
    self.assertFalse(self._tx(disable_msg),
                     "enable=0 must still be blocked without PREAP_FLAG_ENABLE_PEDAL")

  def test_pedal_enable_0_with_high_gas_blocked(self):
    # Defense-in-depth: ENABLE=0 + non-zero GAS_COMMAND must be blocked.
    # Legitimate passthrough sends GAS_COMMAND=0 (physical) which is raw ~450.
    # Any ENABLE=0 message with a raw value above 500 is suspicious (possible
    # bug or attack attempting to exploit a hypothetical Comma Pedal firmware
    # flaw where ENABLE=0 is not honored).
    # Verify: legitimate passthrough (physical 0) allowed, high-value blocked.
    self._rx(self._pcm_status_msg(True))  # engage
    # Legitimate: physical 0 = raw 450 → <=500 → allowed
    ok_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                             {"GAS_COMMAND": 0, "ENABLE": 0})
    self.assertTrue(self._tx(ok_msg))
    # Attack: physical 100 = raw 2419 → >500 → blocked
    attack_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 100, "ENABLE": 0})
    self.assertFalse(self._tx(attack_msg),
                     "ENABLE=0 with high GAS_COMMAND must be blocked (defense-in-depth)")


def _fix_gas_checksum(msg):
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(min(5, len(data))):
    chk += data[i]
  if len(data) > 5:
    data[5] = chk & 0xFF
  return addr, bytes(data), bus


class TestTeslaPreAPPedalCalibration(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("tesla_preap")
    self.safety = libsafety_py.libsafety
    self.idx = 0
    self._init(PREAP_FLAG_PEDAL_CALIBRATION)

  def _init(self, flags):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, flags)
    self.safety.init_tests()
    self.idx = 0

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _gas(self, bus, enable, raw=0, raw2=None):
    if raw2 is None:
      raw2 = raw
    values = {
      "GAS_COMMAND": raw / 0.0507968128 + 22.85856576 if raw else 0,
      "GAS_COMMAND2": raw2 / 0.1015936256 + 22.85856576 if raw2 else 0,
      "ENABLE": enable,
      "IDX": self.idx,
    }
    self.idx = (self.idx + 1) % 16
    return self.packer.make_can_msg_safety("GAS_COMMAND", bus, values, fix_checksum=_fix_gas_checksum)

  def _prime(self, gear=3, di_brake=True, brake_msg=True):
    self._rx(self.packer.make_can_msg_safety("DI_torque2", 0, {
      "DI_gear": gear, "DI_brakePedal": 1 if di_brake else 0,
    }))
    self._rx(self.packer.make_can_msg_safety("BrakeMessage", 0, {
      "driverBrakeStatus": 2 if brake_msg else 1,
    }))

  def test_enable0_without_window(self):
    self.assertTrue(self._tx(self._gas(2, 0, 0)))

  def test_enable0_high_raw_blocked(self):
    self.assertFalse(self._tx(self._gas(2, 0, 501)))

  def test_enable1_unprimed_blocked(self):
    self.assertFalse(self._tx(self._gas(2, 1, 0)))

  def test_enable1_drive_blocked(self):
    self._prime(gear=4, di_brake=True)
    self.assertFalse(self._tx(self._gas(2, 1, 0)))

  def test_enable1_neutral_no_brake_blocked(self):
    self._prime(gear=3, di_brake=False, brake_msg=False)
    self.assertFalse(self._tx(self._gas(2, 1, 0)))

  def test_enable1_window_open(self):
    self._prime(gear=3, di_brake=True, brake_msg=True)
    self.assertTrue(self._tx(self._gas(2, 1, 0)))

  def test_wrong_bus_blocked(self):
    self._prime()
    self.assertFalse(self._tx(self._gas(0, 1, 0)))

  def test_bus0_param_96(self):
    self._init(PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_PEDAL_BUS_ZERO)
    self._prime()
    self.assertTrue(self._tx(self._gas(0, 1, 0)))
    self.assertFalse(self._tx(self._gas(2, 1, 0)))

  def test_steering_isolated(self):
    self._prime()
    msg = self.packer.make_can_msg_safety("DAS_steeringControl", 0, {
      "DAS_steeringAngleRequest": 0, "DAS_steeringControlType": 1,
    }, fix_checksum=_fix_das_checksum)
    self.assertFalse(self._tx(msg))

  def test_mixed_flags_fail_closed(self):
    self._init(PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_ENABLE_PEDAL)
    self._prime()
    self.assertFalse(self._tx(self._gas(2, 1, 0)))
    self._init(PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_HANDS_ON_PAUSE)
    self._prime()
    self.assertFalse(self._tx(self._gas(2, 1, 0)))

  def test_stale_window(self):
    self._prime()
    self.safety.set_timer(1000001)
    self.assertFalse(self._tx(self._gas(2, 1, 0)))


class TestTeslaPreAPHandsOnPause(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("tesla_preap")
    self.safety = libsafety_py.libsafety
    self.cnt_epas = 0
    self.cnt_angle = 0
    self._init(PREAP_FLAG_HANDS_ON_PAUSE)

  def _init(self, flags):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, flags)
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)
    self.cnt_epas = 0
    self.cnt_angle = 0

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _epas(self, hands=0, eac_status=1, eac_error=0):
    values = {
      "EPAS_internalSAS": 0,
      "EPAS_handsOnLevel": hands,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error,
      "EPAS_sysStatusCounter": self.cnt_epas % 16,
    }
    self.cnt_epas += 1
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values, fix_checksum=_fix_epas_checksum)

  def _steer(self, enabled=True):
    values = {"DAS_steeringAngleRequest": 0, "DAS_steeringControlType": 1 if enabled else 0}
    return self.packer.make_can_msg_safety("DAS_steeringControl", 0, values, fix_checksum=_fix_das_checksum)

  def _stalk(self, enable=True):
    lever = STALK_RWD_ENGAGE if enable else STALK_FWD_CANCEL
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever})

  def _idle(self):
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": STALK_IDLE})

  def _gas_enable(self):
    return self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})

  def test_default_off_hands_on_exits(self):
    self._init(0)
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_pause_inhibits_and_blocks_enabled_steer(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer(True)))

  def test_epas_reject_6_9_exits(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2, eac_status=0, eac_error=6))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_disabled_held_stalk_does_not_arm(self):
    self._rx(self._epas(hands=2))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self._rx(self._stalk(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_door_while_paused_exits(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self._rx(self.packer.make_can_msg_safety("GTW_carState", 0, {"DOOR_STATE_FL": 1}))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_steering_control_inhibited())

  def test_gear_while_paused_exits(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self._rx(self.packer.make_can_msg_safety("DI_torque2", 0, {"DI_gear": 3}))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_resume_1s_boundary(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.safety.set_timer(0)
    self._rx(self._epas(hands=0))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.safety.set_timer(999999)
    self._rx(self._epas(hands=0))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.safety.set_timer(1000000)
    self._rx(self._epas(hands=0))
    self.assertFalse(self.safety.get_steering_control_inhibited())

  def test_frozen_config_mixed_calib_not_pause(self):
    self._init(PREAP_FLAG_HANDS_ON_PAUSE | PREAP_FLAG_PEDAL_CALIBRATION)
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertFalse(self.safety.get_steering_control_inhibited())

  def test_fresh_pull_rearms_after_release(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.safety.set_timer(0)
    self._rx(self._epas(hands=0))
    self.safety.set_timer(1000000)
    self._rx(self._epas(hands=0))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self._rx(self._stalk(False))
    self._rx(self._stalk(True))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_door_clears_inhibit_and_fresh_pull_rearms(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self._rx(self.packer.make_can_msg_safety("GTW_carState", 0, {"DOOR_STATE_FL": 1}))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self.packer.make_can_msg_safety("GTW_carState", 0, {"DOOR_STATE_FL": 0}))
    self._rx(self._epas(hands=0))
    self._rx(self._stalk(False))
    self._rx(self._stalk(True))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_paused_then_epas6_exits_inhibit(self):
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self._rx(self._epas(hands=2, eac_status=0, eac_error=6))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_pause_keeps_active_pedal_tx_and_blocks_enabled_steer(self):
    self._init(PREAP_FLAG_HANDS_ON_PAUSE | PREAP_FLAG_ENABLE_PEDAL)
    self._rx(self._stalk(True))
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(self._gas_enable()))
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.assertTrue(self._tx(self._gas_enable()))
    self.assertFalse(self._tx(self._steer(True)))
    self.assertTrue(self._tx(self._steer(False)))

  def test_pause_does_not_start_inactive_long_or_pedal_tx(self):
    self._init(PREAP_FLAG_HANDS_ON_PAUSE | PREAP_FLAG_ENABLE_PEDAL)
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._gas_enable()))
    self.assertFalse(self._tx(self._steer(True)))
    self._rx(self._idle())
    self._rx(self._stalk(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._gas_enable()))
    self.safety.set_timer(0)
    self._rx(self._epas(hands=0))
    self.safety.set_timer(1000000)
    self._rx(self._epas(hands=0))
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._gas_enable()))

  def test_default_off_hands_on_drops_pedal_tx(self):
    self._init(PREAP_FLAG_ENABLE_PEDAL)
    self._rx(self._stalk(True))
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(self._gas_enable()))
    self._rx(self._epas(hands=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._gas_enable()))
    self.assertFalse(self._tx(self._steer(True)))

  def test_pause_epas6_drops_active_pedal_tx(self):
    self._init(PREAP_FLAG_HANDS_ON_PAUSE | PREAP_FLAG_ENABLE_PEDAL)
    self._rx(self._stalk(True))
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._epas(hands=2))
    self.assertTrue(self._tx(self._gas_enable()))
    self._rx(self._epas(hands=2, eac_status=0, eac_error=6))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._gas_enable()))
    self.assertFalse(self._tx(self._steer(True)))

  def test_pause_fresh_pull_rearms_after_heartbeat_stale_prev(self):
    self._init(PREAP_FLAG_HANDS_ON_PAUSE | PREAP_FLAG_ENABLE_PEDAL)
    self._rx(self._stalk(True))
    self.assertTrue(self._tx(self._gas_enable()))
    # Board heartbeat withdrawal: libsafety does not run the 1Hz countdown.
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    self.assertFalse(self._tx(self._gas_enable()))
    self._rx(self._stalk(True))
    self.assertFalse(self._tx(self._gas_enable()))
    self._rx(self._idle())
    self._rx(self._stalk(True))
    self.assertTrue(self._tx(self._gas_enable()))


if __name__ == "__main__":
  unittest.main()
