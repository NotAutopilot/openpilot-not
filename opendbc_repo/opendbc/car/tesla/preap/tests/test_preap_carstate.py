#!/usr/bin/env python3
"""Pre-AP carstate update tests.

Regression coverage for the class of bug where update_preap writes a field on
`ret` (the CarState capnp struct) without a matching schema entry in car.capnp.
These writes look like ordinary Python assignment but silently require the
schema to agree; the first update call crashes card with AttributeError, which
leaves the panda in elm327 safe mode and surfaces as 'Unknown Vehicle Variant'
(canError) in the UI.
"""
import unittest
from unittest.mock import patch, PropertyMock

from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.nap_conf import nap_conf


class TestPreAPCarStateUpdate(unittest.TestCase):

  @staticmethod
  def _can_packet(message, values):
    address, dat, bus = CANPacker("tesla_preap").make_can_msg(message, 0, values)
    return [(1, [CanData(address, dat, bus)])]

  def _make_interface(self):
    CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
    CP = CarInterface.get_params("TESLA_MODEL_S_PREAP",
                                 {i: {} for i in range(8)},
                                 [],
                                 alpha_long=False, is_release=False, docs=False)
    return CarInterface(CP)

  def test_update_runs_without_crashing(self):
    """update() with empty CAN must not raise — exercises every ret.X write path."""
    CI = self._make_interface()
    # Ten iterations; mirrors upstream test_car_interfaces pattern and catches
    # issues that only appear after state has accumulated.
    for _ in range(10):
      CI.update([])

  def test_nap_specific_fields_on_carstate(self):
    """NAP-specific booleans written by update_preap must exist on the schema."""
    CI = self._make_interface()
    CS = CI.update([])
    for field in ("teslaCCEngaged", "teslaCCDisengaged", "teslaCCNotArmed",
                  "pedalMaxRegen", "pedalLongActive", "enableLongControl",
                  "pedalAuthorityRequested",
                  "pedalAuthorityState", "pedalAuthorityAction", "pedalCommandCounter",
                  "pedalFeedbackState", "pedalFeedbackCounter", "pedalFirstEnabledMonoTime",
                  "vdasLimitedAccel", "pedalCommandDi", "pedalAuthorityFailed"):
      self.assertTrue(hasattr(CS, field), f"CarState schema missing {field}")

  def test_regen_brake_prompt_uses_controller_level_state(self):
    CI = self._make_interface()
    CI.CS.pedal_brake_required = True
    self.assertTrue(CI.update([]).pedalMaxRegen)

    CI.CS.pedal_brake_required = False
    CI.CS.pccEvent = "pedalMaxRegen"
    self.assertFalse(CI.update([]).pedalMaxRegen)

  def test_pedal_authority_diagnostics_publish_owned_state(self):
    CI = self._make_interface()
    CI.CS.pedal_authority_requested = True
    CI.CS.pedal_authority_state = 2
    CI.CS.pedal_authority_action = 3
    CI.CS.pedal_command_counter = 14
    CI.CS.pedal_first_enabled_mono_time = 123456789
    CI.CS.vdas_limited_accel = -0.25
    CI.CS.pedal_command_di = 3.5
    CI.CS.pedal.interceptor_state = 5
    CI.CS.pedal.idx = 11
    CI.CS.engagement.pedal_unavailable = True

    with patch.object(CI.CS.pedal, "update"):
      CS = CI.update([])

    self.assertTrue(CS.pedalAuthorityRequested)
    self.assertEqual(CS.pedalAuthorityState, 2)
    self.assertEqual(CS.pedalAuthorityAction, 3)
    self.assertEqual(CS.pedalCommandCounter, 14)
    self.assertEqual(CS.pedalFeedbackState, 5)
    self.assertEqual(CS.pedalFeedbackCounter, 11)
    self.assertEqual(CS.pedalFirstEnabledMonoTime, 123456789)
    self.assertAlmostEqual(CS.vdasLimitedAccel, -0.25)
    self.assertAlmostEqual(CS.pedalCommandDi, 3.5)
    self.assertTrue(CS.pedalAuthorityFailed)

  def test_pedal_long_active_reports_accepted_authority_not_request_intent(self):
    CI = self._make_interface()
    CI.CS.engagement.cruiseEnabled = True
    CI.CS.engagement.enableLongControl = True
    CI.CS.pedal_authority_active = False

    with patch.object(type(nap_conf), "use_pedal", new_callable=PropertyMock, return_value=True):
      self.assertFalse(CI.update([]).pedalLongActive)

      CI.CS.pedal_authority_active = True
      self.assertTrue(CI.update([]).pedalLongActive)

  def test_enable_long_control_publishes_fsm_intent_not_authority(self):
    CI = self._make_interface()
    CI.CS.engagement.enableLongControl = True
    CI.CS.pedal_authority_active = False

    with patch.object(type(nap_conf), "use_pedal", new_callable=PropertyMock, return_value=True):
      published = CI.update([])
      self.assertTrue(published.enableLongControl)
      self.assertFalse(published.pedalLongActive)

      CI.CS.engagement.enableLongControl = False
      published = CI.update([])
      self.assertFalse(published.enableLongControl)

  def test_hands_on_level_two_disengages(self):
    for hands_on_level, should_disengage in ((1, False), (2, True), (3, True)):
      with self.subTest(hands_on_level=hands_on_level):
        CI = self._make_interface()
        packets = self._can_packet("EPAS_sysStatus", {
          "EPAS_handsOnLevel": hands_on_level,
          "EPAS_eacStatus": 1,
          "EPAS_eacErrorCode": 0,
        })
        CS = CI.update(packets)
        self.assertEqual(CS.steeringDisengage, should_disengage)

  def test_cluster_speed_uses_dash_signal(self):
    digital_speed = 42
    for speed_units, conversion in ((0, CV.MPH_TO_MS), (1, CV.KPH_TO_MS)):
      with self.subTest(speed_units=speed_units):
        CI = self._make_interface()
        packets = self._can_packet("DI_state", {
          "DI_speedUnits": speed_units,
          "DI_digitalSpeed": digital_speed,
        })
        CS = CI.update(packets)
        expected_speed = digital_speed * conversion
        self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, expected_speed, places=5)

  def test_turn_signal_stalk_state_uses_lever_level(self):
    for lever, expected in ((0, 0), (1, 1), (2, 2), (3, 0)):
      with self.subTest(lever=lever):
        CI = self._make_interface()
        packets = self._can_packet("STW_ACTN_RQ", {"TurnIndLvr_Stat": lever})
        CS = CI.update(packets)
        self.assertEqual(CS.turnSignalStalkState, expected)

  def test_internal_brake_signal_ors_both_raw_sources_while_public_signal_stays_suppressed(self):
    CI = self._make_interface()

    CS = CI.update(self._can_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 1}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS = CI.update(self._can_packet("BrakeMessage", {"driverBrakeStatus": 1}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS = CI.update(self._can_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0}))
    self.assertFalse(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS = CI.update(self._can_packet("BrakeMessage", {"driverBrakeStatus": 2}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS = CI.update(self._can_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS = CI.update(self._can_packet("BrakeMessage", {"driverBrakeStatus": 1}))
    self.assertFalse(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)


if __name__ == "__main__":
  unittest.main()
