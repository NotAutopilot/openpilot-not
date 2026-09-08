"""Default feedforward lookup table for VirtualDAS.

Generated from the existing 3-breakpoint linear interpolation at a grid of
(speed, accel) points, with a conservative field correction on the positive
branch. This is the fallback when no data-driven table is available.
The generate_ff_table.py script produces a refined version from real drive
logs.

Table format: SPEED_BP × ACCEL_BP → pedal_di
Zero-torque offset is applied at runtime (not baked into the table).
"""

# Speed breakpoints (m/s) — matches PEDAL_BP from nap_conf
SPEED_BP = [0.0, 5.0, 12.0, 20.0, 30.0, 40.0]

# Acceleration breakpoints (m/s²) — from REGEN_MAX to ACCEL_MAX
ACCEL_BP = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5]

# pedal_di values: DEFAULT_TABLE[speed_idx][accel_idx]
# Negative and zero values, plus the 0 m/s row, are computed from:
# interp(accel, [REGEN_MAX, 0, ACCEL_MAX], [DI_MIN, 0, max_pedal]).
# Positive values at 5-40 m/s apply a conservative 0.65 field correction to
# the legacy-derived fallback. The 5 m/s correction preserves monotonic
# positive effort and prevents a 5-to-12 m/s drop. External calibrated tables
# remain authoritative.
# where max_pedal = interp(speed, PEDAL_BP, PEDAL_MAX_VALUES)
# and zero_torque_di = 0 (applied as offset at runtime)
#
# Each row is a speed, each column is an accel request.
#         -1.5   -1.0   -0.5    0.0    0.5    1.0    1.5    2.0    2.5
DEFAULT_TABLE = [
    [-5.00, -3.33, -1.67,  0.00, 10.00, 20.00, 30.00, 40.00, 50.00],  # 0 m/s
    [-5.00, -3.33, -1.67,  0.00,  7.54, 15.08, 22.62, 30.16, 37.70],  # 5 m/s
    [-5.00, -3.33, -1.67,  0.00,  8.58, 17.16, 25.74, 34.32, 42.90],  # 12 m/s
    [-5.00, -3.33, -1.67,  0.00,  9.62, 19.24, 28.86, 38.48, 48.10],  # 20 m/s
    [-5.00, -3.33, -1.67,  0.00, 10.66, 21.32, 31.98, 42.64, 53.30],  # 30 m/s
    [-5.00, -3.33, -1.67,  0.00, 11.70, 23.40, 35.10, 46.80, 58.50],  # 40 m/s
]
