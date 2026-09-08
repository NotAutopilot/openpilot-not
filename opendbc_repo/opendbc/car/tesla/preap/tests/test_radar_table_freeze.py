from types import SimpleNamespace

from opendbc.car.tesla.preap.radar_table_freeze import BoschTableFreezeWatch


def _pts(*rows):
  return [SimpleNamespace(dRel=d, yRel=y, vRel=v) for d, y, v in rows]


def test_live_table_does_not_freeze():
  watch = BoschTableFreezeWatch(stable_cycles=3, min_points=4)
  points = _pts((10, 0, -1), (20, 1, 0), (30, -1, 0), (40, 0.5, 0))
  assert watch.update(points) is False
  points[0].dRel = 9.5
  assert watch.update(points) is False
  points[0].dRel = 9.0
  assert watch.update(points) is False


def test_stuck_table_freezes_after_stable_cycles():
  watch = BoschTableFreezeWatch(stable_cycles=3, min_points=4)
  points = _pts((10, 0, 0), (20, 1, 0), (30, -1, 0), (40, 0.5, 0))
  assert watch.update(points) is False
  assert watch.update(points) is False
  assert watch.update(points) is True


def test_too_few_points_never_freezes():
  watch = BoschTableFreezeWatch(stable_cycles=2, min_points=4)
  points = _pts((10, 0, 0), (20, 1, 0))
  assert watch.update(points) is False
  assert watch.update(points) is False
