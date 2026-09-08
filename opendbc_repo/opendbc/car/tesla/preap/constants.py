# Pre-AP pedal accel envelopes, mapped to openpilot Driving Personality toggle.
# Breakpoints are speed in m/s; values are max accel in m/s².
# Based on Tinkla Pedal profiles but reduced for feedforward architecture:
# Tinkla uses PID which naturally dampens the ramp to the ceiling; our kf=1.0
# feedforward passes MPC targets straight through, so the ceilings must be
# lower to get the same feel.
#   aggressive(0) → spirited but controlled
#   standard(1)   → smooth daily driver
#   relaxed(2)    → gentle, minimal push
ACCEL_PREAP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 30.0, 40.0]  # m/s
#                  0    3    17    33    56    67    90  mph
ACCEL_PREAP_PROFILES = {
  0: [0.3, 0.8, 1.1, 1.0, 0.85, 0.7, 0.6],  # aggressive
  1: [0.3, 0.7, 1.0, 0.9, 0.8, 0.6, 0.5],  # standard
  2: [0.3, 0.6, 0.9, 0.8, 0.7, 0.5, 0.45],  # relaxed (former standard values)
}

# When following a lead car, cap positive accel to these values regardless
# of personality. Prevents overshoot → regen → overshoot oscillation.
# Open road uses the full profile above; this only limits follow mode.
ACCEL_PREAP_FOLLOW = [0.3, 0.6, 0.9, 0.8, 0.7, 0.5, 0.45]

# Generic LongControl passes the planner acceleration target through with
# kf=1.0 and no feedback. VirtualDAS owns acceleration feedback and converts
# the target into pedal DI.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
PEDAL_LONG_KP_V = [0.0, 0.0, 0.0, 0.0]
PEDAL_LONG_KI_V = [0.0, 0.0, 0.0, 0.0]

# Virtual DAS inner PID corrects residual acceleration tracking error before
# the acceleration-to-DI feedforward map. Error input and output trim are both
# m/s²: KP is dimensionless and KI is 1/s.
VDAS_INNER_K_BP = [0.0, 5.0, 35.0]
VDAS_INNER_KP_V = [0.0, 0.0, 0.0]
VDAS_INNER_KI_V = [0.3, 0.2, 0.15]

# Delay compensation: predict a_ego this far into the future using
# estimated jerk. Longer at highway speed where powertrain is slower.
VDAS_FUTURE_T_BP = [2.0, 5.0]
VDAS_FUTURE_T_V = [0.30, 0.55]

# a_ego low-pass filter time constant (seconds). Smooths IMU noise
# without adding too much phase lag. Matches Toyota's 0.25s RC.
VDAS_AEGO_FILTER_RC = 0.25

# Acceleration command shaping. Positive transitions use a comfort-oriented
# bound while braking retains the stronger existing response.
VDAS_ACCEL_JERK_MAX = 1.0  # m/s³
VDAS_DECEL_JERK_MAX = 2.5  # m/s³
VDAS_ACCEL_SNAP_MAX = 4.0  # m/s⁴

# Acceleration interval blended across zero torque to remove the propulsion/
# regen slope discontinuity in the legacy-derived feedforward table.
VDAS_ZERO_TORQUE_TRANSITION_WIDTH = 0.25  # m/s² on each side of zero

# Physical bound on measured acceleration derivative used by delay prediction.
# This limits one-frame sensor/source discontinuities without constraining the
# tighter acceleration-command jerk limits above.
VDAS_EGO_JERK_MAX = 5.0  # m/s³
