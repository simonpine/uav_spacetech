"""Airframe constants for the SNOOP-style tri-motor tiltrotor.

Every number here was computed from the expanded URDF (see tools/mass_props.py),
not estimated.  If you edit vtol_tiltrotor.urdf.xacro, re-run that script and
update this file, otherwise the mixer's moment arms stop matching the model and
the attitude loops will fight a steady bias.

Frame convention (URDF base_link): x forward, y LEFT, z UP.  This is ROS FLU,
so the sign conventions are:

    +Mx / +roll   right wing down   (same as aerospace)
    +My / +pitch  nose DOWN         (opposite of aerospace)
    +Mz / +yaw    nose LEFT         (opposite of aerospace)

The controller works in these raw URDF signs internally and converts at the
pilot-input boundary, so read the mixer with the table above in hand.
"""

import math

# ----------------------------------------------------------------- airframe --
MASS = 5.4960            # kg
GRAVITY = 9.80665        # m/s^2
WEIGHT = MASS * GRAVITY  # 53.897 N

# Inertia about the CoM in base_link axes.  Off-diagonal Ixz is 0.039, small
# enough to ignore in the rate loops.
IXX = 0.3839
IYY = 0.5653
IZZ = 0.8901

# Rotor stations measured from the CoM (x=-0.00422, z=+0.03568 in base_link).
X_FRONT = 0.3642         # front rotors ahead of the CoM
Y_FRONT = 0.3600         # half-track of the front rotors
Z_FRONT = 0.0893         # front rotors above the CoM
X_REAR = -0.5358         # rear rotor behind the CoM (negative)
Z_REAR = 0.1743          # rear rotor above the CoM

# ---------------------------------------------------------------- propulsion --
K_THRUST = 1.6e-05       # T = K_THRUST * w^2  [N], matches MulticopterMotorModel
W_MAX = 1500.0           # rad/s
W_IDLE = 90.0            # keeps the props visibly alive when armed
F_MAX = K_THRUST * W_MAX ** 2   # 36.0 N per rotor

# Hover trim, from the 2x2 solve of (sum of thrust = weight, pitch moment = 0).
F_FRONT_HOVER = 16.043
F_REAR_HOVER = 21.812

# Commanded travel, held just INSIDE the URDF joint limits. With
# <use_velocity_commands> a joint driven onto its hard limit latches dartsim's
# limit constraint and stops responding - which is fatal for rear_tilt, the only
# yaw actuator in hover. Keep a few percent of margin.
TILT_MIN = -0.16         # URDF limit -0.17
TILT_MAX = 1.55          # URDF limit 1.5708; cos(1.55) = 0.021, close enough
REAR_TILT_MAX = 0.50     # URDF limit 0.524, tricopter yaw servo

# ------------------------------------------------------------------ aero ------
RHO = 1.2041             # kg/m^3, matches <air_density> in the plugin layer
S_WING = 0.3620          # m^2, 2*(0.123 inboard) + 2*(0.058 raked tip)
CLA = 4.75               # per rad, from the LiftDrag blocks
A0 = 0.05                # rad, zero-lift offset
ALPHA_STALL = 0.30       # rad
CL_MAX = CLA * (ALPHA_STALL + A0)        # 1.662
V_STALL = math.sqrt(2.0 * WEIGHT / (RHO * S_WING * CL_MAX))   # 12.20 m/s

# Control-surface effectiveness: moment = COEFF * q_dyn * deflection, with
# q_dyn = 0.5*RHO*V^2, so COEFF is in N*m per (Pa * rad).  Deflection conventions:
#   ailerons : (left = -a, right = +a)  ->  +Mx, roll right
#   elevator : both ruddervators +e     ->  -My, nose up
#   rudder   : (left = +r, right = -r)  ->  -Mz, yaw right
#
# MEASURED, not derived, by tools/measure_surfaces.py: it flies the aircraft on
# rotors alone, holds one surface at +-0.30 rad, and reads how much rotor moment
# the controller needs to cancel it.  Analytically, from the LiftDrag areas, cp
# positions and control_joint_rad_to_cl, these come out as +0.1427, -0.1949 and
# -0.1523.  The signs agree; the magnitudes are 40-85% of the analytic values,
# because the panels' local angle of attack changes as the airframe responds and
# the V-tail sees sidewash.  Using the measured numbers keeps the closed-loop
# gain honest, and the rate loops' integral action covers what is left.
AIL_MX_PER_Q = 0.1211    # analytic +0.1427, ratio 0.85
ELE_MY_PER_Q = -0.1193   # analytic -0.1949, ratio 0.61
RUD_MZ_PER_Q = -0.0599   # analytic -0.1523, ratio 0.39

AIL_LIMIT = 0.42         # URDF limit 0.44
RV_LIMIT = 0.50          # URDF limit 0.52

# Joint / topic names in the order the Actuators message expects them.
MOTOR_ORDER = ('front_left', 'front_right', 'rear')

SERVO_TOPICS = (
    'tilt_left', 'tilt_right', 'rear_tilt',
    'aileron_left', 'aileron_right',
    'ruddervator_left', 'ruddervator_right',
)


def thrust_to_speed(thrust_n: float) -> float:
    """Invert T = k w^2, clamped to the motor's usable band."""
    if thrust_n <= 0.0:
        return 0.0
    return min(math.sqrt(thrust_n / K_THRUST), W_MAX)


def dynamic_pressure(airspeed: float) -> float:
    return 0.5 * RHO * airspeed * airspeed


def lift_estimate(airspeed: float, alpha: float) -> float:
    """Wing lift at this airspeed and angle of attack, using the same
    coefficients the LiftDrag plugin uses.  Feed-forwarding this is what makes
    the transition hand lift from the rotors to the wing without the altitude
    integrator having to chase it."""
    cl = CLA * (alpha + A0)
    cl = max(-0.6, min(CL_MAX, cl))
    return dynamic_pressure(airspeed) * S_WING * cl
