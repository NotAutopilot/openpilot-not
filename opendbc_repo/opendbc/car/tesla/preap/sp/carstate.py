import os
from opendbc.can import CANDefine
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.preap.carstate import get_preap_can_parsers, update_preap
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.carstate import HANDS_ON_DISENGAGE_LEVEL
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from opendbc.car.tesla.values import CANBUS, DBC, CruiseButtons

ButtonType = structs.CarState.ButtonEvent.Type

# Match panda PREAP_HANDS_ON_RESUME_US. Host pull admission stays closed
# for this hold after hands clear; a held MAIN is not an edge on expiry.
PREAP_HANDS_ON_RESUME_MS = 1000

# NAP update_preap writes ret.brake = 0. Sunnypilot grouped brake @5 under
# CarState.deprecated (comma #3338); Honda already writes ret.deprecated.brake.
# Forward that one name. Anything else must raise.
_DEPRECATED_CARSTATE_FIELDS = frozenset({"brake"})
_REAL_CAR_STATE = structs.CarState


class _DeprecatedCarStateForwarder:
  GearShifter = _REAL_CAR_STATE.GearShifter
  ButtonEvent = _REAL_CAR_STATE.ButtonEvent
  CruiseState = _REAL_CAR_STATE.CruiseState
  WheelSpeeds = _REAL_CAR_STATE.WheelSpeeds

  def __init__(self):
    object.__setattr__(self, "_inner", _REAL_CAR_STATE())

  def __getattr__(self, name):
    inner = object.__getattribute__(self, "_inner")
    if name in _DEPRECATED_CARSTATE_FIELDS:
      return getattr(inner.deprecated, name)
    return getattr(inner, name)

  def __setattr__(self, name, value):
    inner = object.__getattribute__(self, "_inner")
    if name in _DEPRECATED_CARSTATE_FIELDS:
      setattr(inner.deprecated, name, value)
      return
    setattr(inner, name, value)

  def unwrap(self):
    return object.__getattribute__(self, "_inner")


class PreAPCarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    CANBUS.powertrain = CANBUS.party
    CANBUS.autopilot_powertrain = CANBUS.autopilot_party

    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
    self.can_defines = {
      **self.can_define_party.dv,
      **self.can_define_pt.dv,
      **self.can_define_chassis.dv,
    }
    self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False
    self.hands_on_level = 0
    self.das_control = None
    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0
    self.msg_stw_actn_req = None
    self.prev_stalk_follow = 0
    self.speed_units = "MPH"

    self.engagement = PreAPEngagement(
      double_pull_enabled=nap_conf.double_pull_enabled,
      double_pull_window_ms=nap_conf.double_pull_window_ms,
    )
    self.cruiseEnabled = False
    self.enableLongControl = False
    self.enableJustCC = False
    self.pedal_speed_kph = 0.0
    self.longCtrlEvent = None
    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False

    self.pedal = PedalFeedback()
    self.pedal_interceptor_value = 0.0
    self.pedal_timeout = True
    self.pccEvent = None
    self.pedal_authority_requested = False
    self.pedal_authority_active = False
    self.pedal_authority_state = 0
    self.pedal_authority_action = 0
    self.pedal_command_counter = 0
    self.pedal_first_enabled_mono_time = 0
    self.vdas_limited_accel = 0.0
    self.pedal_command_di = 0.0

    self._prev_cruise_enabled = False
    self._prev_enable_long = False
    self._intent_sequence = 0
    self._intent_epoch = int.from_bytes(os.urandom(8), "little") or 1
    self._latched_lateral = structs.CarStateSP.PreapLateralIntent.none
    self._latched_longitudinal = structs.CarStateSP.PreapLongitudinalIntent.none
    self._orig_handle_steering_disengage = self.engagement.handle_steering_disengage
    self.engagement.handle_steering_disengage = self._handle_steering_disengage
    self._orig_process_buttons = self.engagement.process_buttons
    self.engagement.process_buttons = self._process_buttons
    self._epas_hands = 0
    self._epas_rejecting = False
    self._epas_fault = False
    self._pull_inhibit = False
    self._pull_inhibit_clear_timing = False
    self._pull_inhibit_clear_ts_ms = 0

  def update_button_enable(self, buttonEvents):
    return False

  def _pause_effective(self) -> bool:
    return bool(int(getattr(self.CP_SP, "flags", 0) or 0) & int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE))

  def _hands_on_pause_gate(self) -> bool:
    return (
      self._pause_effective()
      and self._epas_hands >= HANDS_ON_DISENGAGE_LEVEL
      and not self._epas_rejecting
      and not self._epas_fault
    )

  def _clear_unadmitted_pull_state(self) -> None:
    self.engagement.pending_enable = False
    self.engagement.stalk_pull_time_ms = 0
    self.engagement.prev_stalk_pull_time_ms = -1000
    self.engagement.pending_cancel_at_ms = 0

  def _reset_pull_inhibit(self) -> None:
    self._pull_inhibit = False
    self._pull_inhibit_clear_timing = False
    self._pull_inhibit_clear_ts_ms = 0

  def _update_pull_inhibit(self, now_ms: int) -> None:
    if not self._pause_effective() or self._epas_rejecting or self._epas_fault:
      self._reset_pull_inhibit()
      return

    if self._epas_hands >= HANDS_ON_DISENGAGE_LEVEL:
      self._pull_inhibit = True
      self._pull_inhibit_clear_timing = False
      return

    if not self._pull_inhibit:
      return

    if not self._pull_inhibit_clear_timing:
      self._pull_inhibit_clear_timing = True
      self._pull_inhibit_clear_ts_ms = now_ms
      return

    if now_ms - self._pull_inhibit_clear_ts_ms >= PREAP_HANDS_ON_RESUME_MS:
      self._reset_pull_inhibit()

  def _process_buttons(self, cruise_buttons, prev_cruise_buttons, curr_time_ms=0, *args, **kwargs):
    self._update_pull_inhibit(int(curr_time_ms or 0))
    rising_main = (
      cruise_buttons == CruiseButtons.MAIN
      and prev_cruise_buttons != CruiseButtons.MAIN
    )
    # Panda rejects lever==2 while inhibited, including the 1s resume hold.
    # Swallow the host edge, drop stale pending/double-pull state, and leave
    # an already-active pedal target alone. Held MAIN must not become an
    # edge when the hold expires.
    if self._pull_inhibit or self._hands_on_pause_gate():
      if rising_main:
        cruise_buttons = prev_cruise_buttons
      self._clear_unadmitted_pull_state()
    return self._orig_process_buttons(cruise_buttons, prev_cruise_buttons, curr_time_ms, *args, **kwargs)

  def _handle_steering_disengage(self, steering_disengage):
    if self._epas_rejecting or self._epas_fault:
      self._reset_pull_inhibit()
      if steering_disengage:
        self.engagement.prev_steering_disengage = False
      self._orig_handle_steering_disengage(steering_disengage)
      return

    if self._hands_on_pause_gate():
      if steering_disengage and not self.engagement.prev_steering_disengage:
        # Pause lateral only. Keep already-active healthy pedal cruise.
        self._clear_unadmitted_pull_state()
      self.engagement.prev_steering_disengage = steering_disengage
      return
    if not self._pause_effective():
      self._reset_pull_inhibit()
    self._orig_handle_steering_disengage(steering_disengage)

  def _revoke_unadmitted_held_hands(self, ret: structs.CarState) -> None:
    self.engagement.cruiseEnabled = False
    self.engagement.enableLongControl = False
    self.engagement.enableJustCC = False
    self.engagement.preap_cc_cancel_needed = False
    self.engagement.preap_cc_engage_needed = False
    self.engagement.pedal_speed_kph = 0.0
    self._clear_unadmitted_pull_state()
    self.cruiseEnabled = False
    self.enableLongControl = False
    self.enableJustCC = False
    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False
    self.pedal_speed_kph = 0.0
    ret.cruiseState.enabled = False
    ret.enableLongControl = False

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    epas = can_parsers[Bus.chassis].vl["EPAS_sysStatus"]
    self._epas_hands = int(epas["EPAS_handsOnLevel"])
    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas["EPAS_eacStatus"]), None)
    eac_error = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas["EPAS_eacErrorCode"]), None)
    self._epas_fault = eac_status == "EAC_FAULT"
    self._epas_rejecting = eac_status == "EAC_INHIBITED" and eac_error in (
      "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_REQ",
      "EAC_ERROR_HIGH_ANGLE_SAFETY", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY",
    )
    cruise_before = bool(self.engagement.cruiseEnabled)
    structs.CarState = _DeprecatedCarStateForwarder
    try:
      ret = update_preap(self, can_parsers)
    finally:
      structs.CarState = _REAL_CAR_STATE
    if isinstance(ret, _DeprecatedCarStateForwarder):
      ret = ret.unwrap()
    unadmitted_held = (not cruise_before) and self._hands_on_pause_gate()
    if unadmitted_held:
      self._revoke_unadmitted_held_hands(ret)
    if self._hands_on_pause_gate():
      # Match panda: hands-only pause is not a host steeringDisengage.
      ret.steeringDisengage = False
    ret.brakePressed = bool(self.real_brake_pressed)
    ret.handsOnLevel = int(self.hands_on_level)
    ret_sp = structs.CarStateSP()
    self._publish_mads_intent(ret_sp, ret)
    return ret, ret_sp

  def _fresh_driver_set_cruise(self, ret: "structs.CarState | None") -> bool:
    if ret is None:
      return False
    for be in getattr(ret, "buttonEvents", None) or []:
      if bool(getattr(be, "pressed", False)) and be.type == ButtonType.setCruise:
        return True
    return False

  def _publish_mads_intent(self, ret_sp: structs.CarStateSP, ret: "structs.CarState | None" = None) -> None:
    Lateral = structs.CarStateSP.PreapLateralIntent
    Longitudinal = structs.CarStateSP.PreapLongitudinalIntent

    cruise = bool(self.engagement.cruiseEnabled)
    enable_long = bool(self.engagement.enableLongControl)
    healthy = (
      self._epas_hands < HANDS_ON_DISENGAGE_LEVEL
      and not self._epas_rejecting
      and not self._epas_fault
    )
    fresh_set_cruise = self._fresh_driver_set_cruise(ret)

    if cruise and not self._prev_cruise_enabled:
      if self._pause_effective() and self._epas_hands >= HANDS_ON_DISENGAGE_LEVEL:
        lateral = None
      else:
        lateral = Lateral.mainCruiseRequest
    elif not cruise and self._prev_cruise_enabled:
      lateral = Lateral.forceDisable
    elif cruise and fresh_set_cruise and healthy:
      lateral = Lateral.mainCruiseRequest
    else:
      lateral = None

    if enable_long and not self._prev_enable_long:
      longitudinal = Longitudinal.enable
    elif not enable_long and self._prev_enable_long:
      longitudinal = Longitudinal.disable
    else:
      longitudinal = None

    if lateral is not None or longitudinal is not None:
      # New sequence identity per emission. Long-only does not keep a stale
      # mainCruiseRequest that would re-enable MADS after host disable.
      self._intent_sequence = (self._intent_sequence + 1) & 0xFFFFFFFF
      self._latched_lateral = lateral if lateral is not None else Lateral.none
      self._latched_longitudinal = longitudinal if longitudinal is not None else Longitudinal.none

    ret_sp.preapLateralIntent = self._latched_lateral
    ret_sp.preapLongitudinalIntent = self._latched_longitudinal
    ret_sp.preapIntentSequence = self._intent_sequence
    ret_sp.preapIntentEpoch = self._intent_epoch

    self._prev_cruise_enabled = cruise
    self._prev_enable_long = enable_long

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return get_preap_can_parsers(CP)
