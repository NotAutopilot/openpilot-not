import struct
import crcmod
from ctypes import create_string_buffer
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CANBUS

# Comma Pedal protocol constants
PEDAL_M1 = 0.050796813    # Primary scaling factor
PEDAL_M2 = 0.101593626    # Secondary scaling (2x M1 for redundancy)
PEDAL_D = -22.85856576    # Offset
GAS_COMMAND_ID = 0x551

# Default STW_ACTN_RQ signal values (all signals except counter, CRC, and button).
# Each key here is copied from the live stalk frame (create_action_request uses
# msg_stw.get(key, default), so the default is only a fallback when the signal is
# absent) — so these mirror the driver's real stalk state.
#
# VSL_Enbl_Rq=1 matches what the driver's stalk module emits — the DI rejects /
# anomalously interprets frames with bit 6 of byte 0 cleared. See drive-3
# segment 2: NAP's spoofed UP_1ST at 47 kph went STANDBY→OFF instead of
# STANDBY→ENABLED because of this single bit. Was previously a typo
# ("VSL_Enbl_Stat", which is not a DBC signal) — silently dropped by the
# packer, so the bit always read 0.
#
# WprSw6Posn (wiper 6-position switch) MUST be listed so it is preserved from the
# live frame. When it was omitted, the packer wrote it as 0 in every spoofed
# frame; at engage that one-frame jump from the driver's real wiper position to 0
# was read by the body controller as a wiper actuation, causing a single phantom
# wipe on every engagement. We do NOT add the other unpreserved switch bits
# (StW_Sw07..15, SpdCtrlLvrStat_Inv) on purpose — keep this change minimal so we
# don't risk spoofing a horn/high-beam/other stalk press.
_STW_DEFAULTS = {
  "VSL_Enbl_Rq": 1, "DTR_Dist_Rq": 0, "TurnIndLvr_Stat": 0,
  "HiBmLvr_Stat": 0, "WprWashSw_Psd": 0, "WprWash_R_Sw_Posn_V2": 0,
  "WprSw6Posn": 0,
  "StW_Lvr_Stat": 0, "StW_Cond_Flt": 0, "StW_Cond_Psd": 0,
  "HrnSw_Psd": 0, "StW_Sw00_Psd": 0, "StW_Sw01_Psd": 0,
  "StW_Sw02_Psd": 0, "StW_Sw03_Psd": 0, "StW_Sw04_Psd": 0,
  "StW_Sw05_Psd": 0, "StW_Sw06_Psd": 0,
}


class TeslaCANPreAP(TeslaCANRaven):
  def __init__(self, packers):
    super().__init__(packers)
    self.pedal_can_bus = 2
    # Pedal firmware watchdog requires consecutive counter values
    self.pedal_idx = 0
    # STW_ACTN_RQ uses CRC-8 (poly 0x1D), not the byte-sum checksum
    self.stw_crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

  @staticmethod
  def pedal_checksum(msg_id, dat):
    """Comma Pedal CAN checksum: addr bytes + data bytes, truncated to 8 bits."""
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_pedal_command(self, accel_command, enable=1, pedal_can_bus=None):
    """Build GAS_COMMAND (0x551) using raw struct packing for firmware byte-compatibility."""
    if pedal_can_bus is None:
      pedal_can_bus = self.pedal_can_bus

    idx = self.pedal_idx
    self.pedal_idx = (self.pedal_idx + 1) % 16

    if enable == 1:
      int_cmd1 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M1)))
      int_cmd2 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M2)))
    else:
      int_cmd1 = 0
      int_cmd2 = 0

    msg = create_string_buffer(6)
    struct.pack_into("BBBBB", msg, 0,
                     (int_cmd1 >> 8) & 0xFF, int_cmd1 & 0xFF,
                     (int_cmd2 >> 8) & 0xFF, int_cmd2 & 0xFF,
                     ((enable << 7) + idx) & 0xFF)
    struct.pack_into("B", msg, 5, self.pedal_checksum(GAS_COMMAND_ID, msg.raw))

    return (GAS_COMMAND_ID, bytes(msg.raw), pedal_can_bus)

  @staticmethod
  def create_radar_vin_msg(fragment, vin, use_radar, position, epas_type):
    """Host→panda 0x560 fragment. Panda consumes this; it does not go on the car.

    Layout is the Tinkla 0.6.6 create_radar_VIN_msg packing.
    """
    padded = (vin + (" " * 17))[:17]
    if fragment == 0:
      flags = (1 if use_radar else 0) + ((position & 0x03) << 1) + ((epas_type & 0x07) << 3)
      dat = bytes((0, 1, flags, 0, 0, ord(padded[0]), ord(padded[1]), ord(padded[2])))
    elif fragment == 1:
      dat = bytes((1, *(ord(padded[i]) for i in range(3, 10))))
    else:
      dat = bytes((2, *(ord(padded[i]) for i in range(10, 17))))
    return (0x560, dat, 0)

  def create_epas_control(self, counter, mode):
    values = {
      "EPB_epasEACAllow": mode,
      "EPB_epasControlCounter": counter,
      "EPB_epasControlChecksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)[1]
    values["EPB_epasControlChecksum"] = self.checksum(0x214, data)
    return self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)

  def create_action_request(self, button_to_press, bus, counter, msg_stw=None):
    """Build STW_ACTN_RQ to simulate cruise stalk button press."""
    values = {"MC_STW_ACTN_RQ": counter, "CRC_STW_ACTN_RQ": 0, "SpdCtrlLvr_Stat": button_to_press}
    if msg_stw is not None:
      for key, default in _STW_DEFAULTS.items():
        values[key] = msg_stw.get(key, default)
    else:
      values.update(_STW_DEFAULTS)

    data = self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)[1]
    values["CRC_STW_ACTN_RQ"] = self.stw_crc(data[:7])
    return self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)

  def create_body_controls_message(self, turn, hazard, bus, counter):
    """Build DAS_bodyControls (0x3E9) to drive the turn indicator.

    turn: 0=none, 1=left, 2=right (matches CC.rightBlinker*2 + CC.leftBlinker).
    Reason=1 (DAS_ACTIVE_NAV_LANE_CHANGE) when turning, 0 otherwise. This is the
    proven Tinkla/Tesla-Unity Pre-AP blinker mechanism — Pre-AP has no AP ECU,
    so openpilot supplies DAS_bodyControls directly.
    """
    values = {
      "DAS_headlightRequest": 0,
      "DAS_hazardLightRequest": hazard,
      "DAS_wiperSpeed": 0,
      "DAS_turnIndicatorRequest": turn,
      "DAS_highLowBeamDecision": 0,
      "DAS_highLowBeamOffReason": 0,
      "DAS_turnIndicatorRequestReason": 1 if turn > 0 else 0,
      "DAS_bodyControlsCounter": counter,
      "DAS_bodyControlsChecksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)[1]
    values["DAS_bodyControlsChecksum"] = self.checksum(0x3E9, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)
