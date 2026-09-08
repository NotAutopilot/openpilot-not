"""Read the VIN already stored in a Tesla Bosch radar.

This is a read-only UDS conversation on the radar bus. It never writes
F190, never runs routine 0x0A03, and never flashes firmware. The host
uses the returned VIN to impersonate the donor on the live GTW feeds.
"""

from dataclasses import dataclass
from enum import Enum, auto

from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.preap.nap_conf import normalize_radar_donor_vin


RADAR_UDS_TX = 0x641
RADAR_UDS_RX = 0x651
RADAR_UDS_BUS = 1
F190 = 0xF190

TESTER_PRESENT = b"\x3e\x00"
DEFAULT_SESSION = b"\x10\x01"
EXTENDED_SESSION = b"\x10\x03"
READ_F190 = b"\x22\xf1\x90"
CLEANUP_MARKER = b"\x3e\x80"
FLOW_CONTROL = b"\x30\x00\x00\x00\x00\x00\x00\x00"

# TeslaRadarAlertMatrix 0x501, Intel 1-bit signals.
VIN_VALIDITY_BYTE = 4
VIN_VALIDITY_MASK = 0x10


class RadarDonorVinState(Enum):
  IDLE = auto()
  TESTER_PRESENT = auto()
  DEFAULT_SESSION = auto()
  EXTENDED_SESSION = auto()
  READINESS = auto()
  READ_F190 = auto()
  CLEANUP = auto()
  COMPLETE = auto()
  FAILED = auto()


class RadarDonorVinFailure(Enum):
  ABORTED = auto()
  TIMEOUT = auto()
  MALFORMED_RESPONSE = auto()
  UNEXPECTED_RESPONSE = auto()
  NEGATIVE_RESPONSE = auto()
  INVALID_VIN = auto()


class _IsoTpState(Enum):
  INCOMPLETE = auto()
  COMPLETE = auto()
  MALFORMED = auto()


@dataclass(frozen=True)
class _IsoTpResult:
  state: _IsoTpState
  payload: bytes | None = None
  can_sends: tuple[CanData, ...] = ()


class _IsoTpAssembler:
  def __init__(self):
    self.reset()

  def consume(self, data: bytes, expected_prefix: bytes) -> _IsoTpResult:
    if len(data) != 8:
      return self._malformed()
    frame_type = data[0] >> 4
    if frame_type == 0:
      length = data[0] & 0x0F
      if self._length is not None or not 1 <= length <= 7:
        return self._malformed()
      payload = data[1:1 + length]
      if not payload.startswith(expected_prefix) and not payload.startswith(b"\x7f"):
        return self._malformed()
      return _IsoTpResult(_IsoTpState.COMPLETE, payload)
    if frame_type == 1:
      length = ((data[0] & 0x0F) << 8) | data[1]
      if self._length is not None or length <= 7:
        return self._malformed()
      prefix = data[2:]
      if not prefix.startswith(expected_prefix) and not prefix.startswith(b"\x7f"):
        return self._malformed()
      self._payload = bytearray(prefix)
      self._length = length
      self._sequence = 1
      return _IsoTpResult(
        _IsoTpState.INCOMPLETE,
        can_sends=(CanData(RADAR_UDS_TX, FLOW_CONTROL, RADAR_UDS_BUS),),
      )
    if frame_type == 2:
      sequence = data[0] & 0x0F
      if self._length is None or sequence != self._sequence:
        return self._malformed()
      self._sequence = (self._sequence + 1) & 0x0F
      remaining = self._length - len(self._payload)
      self._payload.extend(data[1:1 + remaining])
      if len(self._payload) < self._length:
        return _IsoTpResult(_IsoTpState.INCOMPLETE)
      payload = bytes(self._payload)
      self.reset()
      return _IsoTpResult(_IsoTpState.COMPLETE, payload)
    return self._malformed()

  def reset(self) -> None:
    self._payload = bytearray()
    self._length: int | None = None
    self._sequence = 1

  def _malformed(self) -> _IsoTpResult:
    self.reset()
    return _IsoTpResult(_IsoTpState.MALFORMED)


@dataclass(frozen=True)
class RadarDonorVinOutput:
  state: RadarDonorVinState
  can_sends: tuple[CanData, ...]
  vin: str | None = None
  failure: RadarDonorVinFailure | None = None


@dataclass
class _Request:
  payload: bytes
  positive_prefix: bytes
  sent_at: float
  timeout: float


def radar_alert_vin_validity(data: bytes) -> bool:
  """True when TeslaRadarAlertMatrix asserts RADC_a037_vinValidity."""
  return len(data) == 8 and (data[VIN_VALIDITY_BYTE] & VIN_VALIDITY_MASK) != 0


def _uds_frame(payload: bytes) -> CanData:
  return CanData(RADAR_UDS_TX, (bytes([len(payload)]) + payload).ljust(8, b"\x00"), RADAR_UDS_BUS)


class RadarDonorVinCommissioner:
  """Read F190 when vinValidity sticks, or when the UI asks once."""

  STABLE_FRAMES = 8

  def __init__(self, store_vin):
    self.store_vin = store_vin
    self.reader = RadarDonorVinReader()
    self._vin_fault_frames = 0
    self._attempted = False
    self._force_used = False
    self._persisted_vin: str | None = None

  @property
  def read_finished(self) -> bool:
    return self.reader.state in (RadarDonorVinState.COMPLETE, RadarDonorVinState.FAILED)

  def _persist_vin(self, vin: str | None) -> None:
    # F190 is the VIN. Cleanup is courtesy. A cleanup timeout used to
    # drop a completed read, which left the spare radar on this-car
    # passthrough for the rest of the drive.
    if not vin or vin == self._persisted_vin:
      return
    self.store_vin(vin)
    self._persisted_vin = vin

  def _start_read(self, now: float) -> None:
    self._attempted = True
    self._vin_fault_frames = 0
    self._persisted_vin = None
    self.reader.start(now)

  def update(self, can_packets: list[CanData], now: float, *, radar_enabled: bool,
             stored_vin: str, controls_allowed: bool, enabled: bool,
             force_read: bool = False) -> tuple[CanData, ...]:
    if not radar_enabled:
      return ()

    running = self.reader.state not in (
      RadarDonorVinState.IDLE, RadarDonorVinState.COMPLETE, RadarDonorVinState.FAILED,
    )
    if controls_allowed or enabled:
      if running:
        return self.reader.abort(now).can_sends
      return ()

    if not force_read:
      self._force_used = False
    elif force_read and not self._force_used and not running:
      self._force_used = True
      self._start_read(now)
      running = True

    if self.reader.state in (RadarDonorVinState.COMPLETE, RadarDonorVinState.FAILED):
      self._persist_vin(self.reader.vin)
      return ()
    if running:
      output = self.reader.update(can_packets, now)
      self._persist_vin(output.vin)
      return output.can_sends

    if stored_vin or self._attempted:
      return ()
    for packet in can_packets:
      if packet.address == 0x501 and packet.src == RADAR_UDS_BUS and radar_alert_vin_validity(packet.dat):
        self._vin_fault_frames += 1
    if self._vin_fault_frames < self.STABLE_FRAMES:
      return ()
    self._start_read(now)
    output = self.reader.update(can_packets, now)
    self._persist_vin(output.vin)
    return output.can_sends


class RadarDonorVinReader:
  """One-shot F190 read. Caller owns Params, sockets, and when to start."""

  RESPONSE_TIMEOUT = 3.0
  RESPONSE_PENDING_TIMEOUT = 5.0
  CLEANUP_TIMEOUT = 3.0
  OVERALL_TIMEOUT = 20.0

  def __init__(self):
    self.state = RadarDonorVinState.IDLE
    self.failure: RadarDonorVinFailure | None = None
    self.vin: str | None = None
    self._assembler = _IsoTpAssembler()
    self._request: _Request | None = None
    self._started_at = 0.0
    self._cleanup_deadline: float | None = None
    self._cleanup_acks_remaining = 2

  def start(self, now: float) -> None:
    self.state = RadarDonorVinState.TESTER_PRESENT
    self.failure = None
    self.vin = None
    self._assembler.reset()
    self._request = None
    self._started_at = now
    self._cleanup_deadline = None
    self._cleanup_acks_remaining = 2

  def abort(self, now: float) -> RadarDonorVinOutput:
    if self.state not in (RadarDonorVinState.IDLE, RadarDonorVinState.COMPLETE, RadarDonorVinState.FAILED):
      self._fail(RadarDonorVinFailure.ABORTED, now)
    return self._output(())

  def update(self, can_packets: list[CanData], now: float) -> RadarDonorVinOutput:
    if self.state in (RadarDonorVinState.IDLE, RadarDonorVinState.COMPLETE, RadarDonorVinState.FAILED):
      return self._output(())
    if self.state != RadarDonorVinState.CLEANUP and now - self._started_at >= self.OVERALL_TIMEOUT:
      self._fail(RadarDonorVinFailure.TIMEOUT, now)
      return self._output(())
    if self.state == RadarDonorVinState.CLEANUP and self._cleanup_deadline is not None and now >= self._cleanup_deadline:
      self._fail(RadarDonorVinFailure.TIMEOUT, now)
      return self._output(())

    can_sends: list[CanData] = []
    if self._request is not None:
      if now - self._request.sent_at >= self._request.timeout:
        self._fail(RadarDonorVinFailure.TIMEOUT, now)
        return self._output(())
      for packet in can_packets:
        if packet.address != RADAR_UDS_RX or packet.src != RADAR_UDS_BUS:
          continue
        result = self._assembler.consume(packet.dat, self._request.positive_prefix)
        can_sends.extend(result.can_sends)
        if result.state == _IsoTpState.MALFORMED:
          self._fail(RadarDonorVinFailure.MALFORMED_RESPONSE, now)
          return self._output(can_sends)
        if result.state == _IsoTpState.COMPLETE:
          self._handle_payload(result.payload or b"", now)
          return self._output(can_sends)
      return self._output(can_sends)

    request = self._request_for_state()
    if request is not None:
      payload, prefix = request
      timeout = self.CLEANUP_TIMEOUT if self.state == RadarDonorVinState.CLEANUP else self.RESPONSE_TIMEOUT
      self._assembler = _IsoTpAssembler()
      self._request = _Request(payload, prefix, now, timeout)
      if self.state == RadarDonorVinState.CLEANUP:
        if self._cleanup_deadline is None:
          # Marker first, then the two default-session ACKs share one deadline.
          can_sends.append(_uds_frame(CLEANUP_MARKER))
          self._cleanup_deadline = now + self.CLEANUP_TIMEOUT
      can_sends.append(_uds_frame(payload))
    return self._output(can_sends)

  def _request_for_state(self) -> tuple[bytes, bytes] | None:
    if self.state in (RadarDonorVinState.TESTER_PRESENT, RadarDonorVinState.READINESS):
      return TESTER_PRESENT, b"\x7e\x00"
    if self.state in (RadarDonorVinState.DEFAULT_SESSION, RadarDonorVinState.CLEANUP):
      return DEFAULT_SESSION, b"\x50\x01"
    if self.state == RadarDonorVinState.EXTENDED_SESSION:
      return EXTENDED_SESSION, b"\x50\x03"
    if self.state == RadarDonorVinState.READ_F190:
      return READ_F190, b"\x62\xf1\x90"
    return None

  def _handle_payload(self, payload: bytes, now: float) -> None:
    assert self._request is not None
    request = self._request
    if payload.startswith(b"\x7f"):
      self._handle_negative(payload, request.payload[0], now)
      return
    if not payload.startswith(request.positive_prefix):
      self._fail(RadarDonorVinFailure.UNEXPECTED_RESPONSE, now)
      return
    self._request = None
    if self.state == RadarDonorVinState.TESTER_PRESENT:
      self.state = RadarDonorVinState.DEFAULT_SESSION
    elif self.state == RadarDonorVinState.DEFAULT_SESSION:
      self.state = RadarDonorVinState.EXTENDED_SESSION
    elif self.state == RadarDonorVinState.EXTENDED_SESSION:
      self.state = RadarDonorVinState.READINESS
    elif self.state == RadarDonorVinState.READINESS:
      self.state = RadarDonorVinState.READ_F190
    elif self.state == RadarDonorVinState.READ_F190:
      vin = normalize_radar_donor_vin(payload[3:])
      if not vin:
        self._fail(RadarDonorVinFailure.INVALID_VIN, now)
        return
      self.vin = vin
      self.state = RadarDonorVinState.CLEANUP
      self._cleanup_acks_remaining = 2
    elif self.state == RadarDonorVinState.CLEANUP:
      self._cleanup_acks_remaining -= 1
      if self._cleanup_acks_remaining <= 0:
        self.state = RadarDonorVinState.FAILED if self.failure else RadarDonorVinState.COMPLETE

  def _handle_negative(self, payload: bytes, request_sid: int, now: float) -> None:
    if len(payload) != 3 or payload[1] != request_sid:
      self._fail(RadarDonorVinFailure.MALFORMED_RESPONSE, now)
      return
    if payload[2] == 0x78:
      assert self._request is not None
      self._request.sent_at = now
      self._request.timeout = self.RESPONSE_PENDING_TIMEOUT
      return
    self._fail(RadarDonorVinFailure.NEGATIVE_RESPONSE, now)

  def _fail(self, failure: RadarDonorVinFailure, now: float) -> None:
    if self.failure is None:
      self.failure = failure
    self._request = None
    self._assembler.reset()
    if self.state == RadarDonorVinState.CLEANUP:
      self.state = RadarDonorVinState.FAILED
    else:
      self._cleanup_acks_remaining = 2
      self._cleanup_deadline = None
      self.state = RadarDonorVinState.CLEANUP
      _ = now

  def _output(self, can_sends: list[CanData] | tuple[CanData, ...]) -> RadarDonorVinOutput:
    return RadarDonorVinOutput(self.state, tuple(can_sends), self.vin, self.failure)
