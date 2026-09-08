"""Detect a Bosch object table that has stopped changing.

Tinkla's five-second rule: a salvage radar that rejects VIN / xWD /
position keeps publishing the same targets. Calibration SGUFail with
live tracks must not trip this. A static table of four or more points
for ~5 s at 8 Hz, together with SGUFail, is the lockout.
"""

from __future__ import annotations


class BoschTableFreezeWatch:
  STABLE_CYCLES = 40  # ~5 s at the Bosch trigger rate
  MIN_POINTS = 4

  def __init__(self, stable_cycles: int = STABLE_CYCLES, min_points: int = MIN_POINTS):
    self.stable_cycles = stable_cycles
    self.min_points = min_points
    self._signature: tuple | None = None
    self._stuck_cycles = 0

  def reset(self) -> None:
    self._signature = None
    self._stuck_cycles = 0

  def update(self, points) -> bool:
    signature = _point_signature(points)
    if signature is None or len(signature) < self.min_points:
      self.reset()
      return False
    if signature == self._signature:
      self._stuck_cycles += 1
    else:
      self._signature = signature
      self._stuck_cycles = 1
    return self._stuck_cycles >= self.stable_cycles


def _point_signature(points) -> tuple | None:
  rows = []
  for point in points:
    try:
      rows.append((round(float(point.dRel), 2), round(float(point.yRel), 2),
                   round(float(point.vRel), 2)))
    except (AttributeError, TypeError, ValueError):
      return None
  return tuple(sorted(rows)) if rows else None
