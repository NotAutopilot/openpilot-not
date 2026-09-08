#!/usr/bin/env python3
"""In-memory generate() adapter for the pinned Tesla radar DBC scripts.

Does not modify tesla_radar_bosch.py / tesla_radar_continental.py. Registry
discovery in generator.py imports this module and calls generate().
"""
from __future__ import annotations

import io
import os
import runpy
from contextlib import contextmanager
from pathlib import Path

_DIR = Path(__file__).resolve().parent
_SCRIPTS = (
  "tesla_radar_bosch.py",
  "tesla_radar_continental.py",
)


def generate() -> dict[str, str]:
  outputs: dict[str, str] = {}
  expected = frozenset(script.replace(".py", ".dbc") for script in _SCRIPTS)

  @contextmanager
  def local_open(file, mode="r", *args, **kwargs):
    name = os.path.basename(os.fspath(file))
    if "w" in mode and name in expected:
      with io.StringIO() as buf:
        yield buf
        outputs[name] = buf.getvalue()
      return
    raise OSError(f"nap_radar generate() refused open({file!r}, {mode!r})")

  for script in _SCRIPTS:
    dbc_name = script.replace(".py", ".dbc")
    runpy.run_path(str(_DIR / script), run_name="__main__", init_globals={"open": local_open})
    if dbc_name not in outputs:
      raise RuntimeError(f"{script} did not write {dbc_name}")
  return outputs
