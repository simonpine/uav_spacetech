# VTOL tiltrotor — keyboard-flyable simulation

A flying simulation of the SNOOP-style tri-motor tiltrotor VTOL, driven from the
keyboard. **Spacebar switches between drone and aeroplane mode.**

```bash
./fly.sh
```

That builds the workspace on first run, starts Gazebo with a chase camera, and
opens the keyboard pilot in its own window.

```
  SPACE   toggle  DRONE <-> AIRPLANE      (needs 25 m altitude to go out)
  ENTER   arm / disarm                    X  kill motors immediately

  W / S   pitch    nose down / nose up
            DRONE: W flies forward, S flies backward
            AIRPLANE: it is a stick - W dives, S climbs
  A / D   roll     left / right
  Q / E   yaw      left / right   (DRONE: tail rotor tilts. AIRPLANE: rudder)
  R / F   power    DRONE: climb / descend.  AIRPLANE: trims target airspeed
            arrow Up/Down do the same as R/F

  T  auto take-off to 40 m     L  auto land
  H  level off and hold        Z  centre all sticks      ?  reprint help
```

A typical flight: **ENTER**, **T**, wait for the climb, **SPACE**, fly it around,
**SPACE**, **L**.

Other ways to run it:

```bash
./fly.sh --headless     # no Gazebo window; you still get the HUD and can fly
./fly.sh --demo         # scripted mission, no keyboard needed
./fly.sh --rviz         # RViz alongside Gazebo
```

## What you are flying

The Cannon SNOOP 18: 5.496 kg, 1.70 m span, two front rotors that tilt from
hover to cruise, and a rear rotor on a pylon that tilts sideways to yaw,
tricopter-style. Stall is 12.2 m/s and it cruises around 20 m/s.

| | |
|---|---|
| `src/vtol_tiltrotor_description` | the airframe: URDF/xacro, Gazebo plugin layer, world, bridge config |
| `src/vtol_control` | the flight controller, the keyboard pilot, the scripted mission |
| `tools/` | the measurement scripts the controller's constants came from |

The description package was written as a *description* — its own README says so:
"There is no controller here. No stabilisation, no mixer, no transition
sequencing. You supply that." `vtol_control` is that missing layer.

## How the controller works

One set of loops covers hover, transition and cruise. The obvious alternative —
two controllers with a switch at 45° of tilt — fails on this airframe, because at
the switch point neither set of effectors has full authority: the rotors' pitch
arm has collapsed and the wing has not yet got the dynamic pressure.

```
   sticks ─► attitude ─► rate PIDs ─► desired Fz, Mx, My, Mz
                                            │
                        ┌───────────────────┴───────────────────┐
                    rotors                                  surfaces
        front pair + rear rotor + rear tilt        ailerons + ruddervators
             weight ~ cos(tilt)                     weight ~ (V/16)²
```

Every axis's demand is split between the two effector groups in proportion to the
authority each actually has *right now*, and the two shares always sum to the
full demand. There is no switch and no discontinuity.

Three decisions are worth knowing about:

**The tilt angle is scheduled on airspeed, not on a timer.** The nacelles only
reach 90° once the aircraft is genuinely fast enough for the wing to carry it, and
if the acceleration stalls the tilt stops advancing instead of dropping the
aircraft. This makes the front transition self-regulating; it completes in about
3 seconds and holds altitude to within a metre while doing it.

**The rear rotor is solved for pitch, after the front pair thrust is decided.**
Solving pitch and heave together as a 2×2 is exact in hover but goes singular at
84° of tilt. Solving pitch last makes the pitch axis exact at every tilt angle and
leaves the vertical axis — which has a climb-rate integrator behind it — to absorb
the slack.

**In cruise, altitude is held with pitch and power holds airspeed, not the other
way round.** This airframe's drag model is `cd = cda·alpha` with no parasite term,
so drag *falls* as it speeds up and a fixed throttle has no stable trim speed.
R/F therefore trims a target airspeed between 14 and 30 m/s, and the aircraft
finds its own power setting.

The controller also protects itself: it refuses to transition below 25 m AGL, and
it comes back to drone mode by itself if the airspeed decays towards stall or it
gets within 15 m of the ground in aeroplane mode.

## Three defects in the model, and what was done about them

The description package as shipped does not simulate. Each of these is silent —
nothing errors, the aircraft just does not work — so they are documented at the
point of the fix as well as here.

**1. No aerodynamics at all.** sdformat lumps every fixed-joint child into its
parent when it converts URDF to SDF, so `wing_left`, `tip_left`, `vtail_left` and
their mirrors do not survive as SDF links. All six `LiftDrag` plugins failed with
*"Link with name[wing_left] not found. The LiftDrag will not generate forces"* —
an aircraft with no lift and no drag.

Keeping the links alive with `<preserveFixedJoint>` fixes that error and causes a
worse one (below), so instead all six panels now attach to `base_link` with their
`<cp>` and `<upward>` given explicitly in the base frame. That is physically
identical — the panels were rigidly attached anyway — and it keeps the
aerodynamic damping, because `LiftDrag` evaluates the airflow at `<cp>` using the
link's angular velocity as well as its linear velocity.

**2. The servos did not reach their setpoints.** The `JointPositionController`
gains in the original plugin layer are far too low for a force-based PID: a
0.30 rad aileron command settled at **0.02 rad**, so the aircraft had essentially
no pitch or roll control in cruise. They now use `<use_velocity_commands>`, which
makes them behave like real position-controlled servos, rate-limited by `cmd_max`
set from each joint's own URDF `<limit velocity="…">`. Commanded 0.30 rad now
gives 0.300 rad.

**3. Servos latch on their hard limits.** A consequence of the fix above: driving
a joint onto its limit latches dartsim's limit constraint and it stops responding
to anything. For `rear_tilt` — the only yaw actuator in hover — that is fatal. The
controller now commands a few percent inside every joint limit.

This one is worth dwelling on, because it produced the most misleading symptom in
the whole project: in aeroplane mode the aircraft would pitch down and dive into
the ground while the logged elevator command was perfectly sensible and pointing
the right way. The command was fine. The joint was stuck.

## The control-surface coefficients were measured, not derived

`airframe.py`'s `AIL_MX_PER_Q`, `ELE_MY_PER_Q` and `RUD_MZ_PER_Q` convert a
desired moment into a deflection, so their **sign** decides whether the pitch and
roll loops close negative or positive. They come from `tools/measure_surfaces.py`,
which flies the aircraft on rotors alone, holds one surface at ±0.30 rad, and
reads how much rotor moment the controller needs to cancel it. In steady state
the total moment is zero, so the rotors are cancelling the surface exactly, and
the rotor moment follows in closed form from the commanded speeds and the
geometry.

Three open-loop step-response methods were tried first and all of them failed.
The reasons are recorded in that file's docstring, because they are easy to walk
into: two runs at fixed +D and −D are not comparable once an unstabilised aircraft
has diverged; a 0.35 s window under-reports control power four- to five-fold,
because the V-tail's 140 N·m/rad of pitch stiffness cancels the elevator within
the 0.4 s short period; and a 70 ms window short enough to get inside the short
period has a signal-to-noise ratio of about one.

The measured signs agree with the analytical derivation. The magnitudes are
40–85% of it, and the measured numbers are the ones in use.

```bash
# re-measure, e.g. after changing the aero blocks
ros2 launch vtol_control vtol_sim.launch.py gui:=false publish_surfaces:=false
python3 tools/measure_surfaces.py

# re-derive the mass properties after changing the geometry
python3 tools/mass_props.py
```

## Regression test

`demo_flight` flies the whole envelope using exactly the messages the keyboard
sends, so it exercises the real control path rather than a back door:

```bash
ros2 launch vtol_control vtol_sim.launch.py gui:=false &
ros2 run vtol_control demo_flight --csv /tmp/flight.csv
```

Fifteen stages: arm, auto take-off to 40 m, hover, altitude hold, yaw, translate,
transition, cruise, bank right, bank left, climb in cruise, back-transition,
settle, auto land. Every stage asserts an *invariant* as well as a completion
condition — without that a stage whose condition is "mode == DRONE" passes
trivially after the aircraft has crashed and the safety logic has dropped it back
to drone mode. A crash is a failure, not a pass.

Last run, all fifteen passed:

| stage | mode | altitude | airspeed |
|---|---|---|---|
| hover, altitude hold | DRONE | 40.0 → 40.0 m | 1.6 m/s |
| transition, 3.2 s | → AIRPLANE | 39.9 → 40.7 m | 7.6 → 21.8 m/s |
| cruise, 10 s | AIRPLANE | 37.7 → 40.1 m | 20.6 → 24.1 m/s |
| bank ±34° | AIRPLANE | 36 → 38 m | 21 m/s |
| climb in cruise | AIRPLANE | 36 → 55.7 m | 21 m/s |
| back-transition, 3.5 s | → DRONE | 55.7 → 56.8 m | 22.6 → 20.0 m/s |

## If it stops working

**Everything goes quiet — no topics, but the processes look healthy.** FastDDS
leaves its shared-memory segments in `/dev/shm` behind when a node is
`SIGKILL`ed, and enough orphans break discovery. The symptom is misleading: the
aircraft arms, then takes off at a tenth of the commanded climb rate because the
controller is only receiving some of its IMU samples. `fly.sh` clears them at
start-up when nothing else is using ROS; by hand it is

```bash
pkill -f "ign gazebo"           # SIGTERM, not -9
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

**Two simulators at once.** Both publish `/vtol/odom` and both listen to
`/vtol/command/motor_speed`, so the controller sees a position jumping between
two aircraft — airspeed in the thousands, altitude flicking to zero. `fly.sh`
refuses to start a second one. `ROS_DOMAIN_ID=42 ./fly.sh` gives a run its own
domain.

**The Gazebo window will not open.** Try `./fly.sh --headless`, or
`ros2 launch vtol_control vtol_sim.launch.py render_engine:=ogre` if the GPU
driver cannot do ogre2. Headless is fully flyable: the HUD is in the keyboard
pilot's terminal, not in Gazebo.

**You lost the aircraft in the Gazebo view.** Right-click it in the Entity Tree
and pick Follow. The launch does this automatically a few seconds after spawn.

## Environment

Built and tested on ROS 2 Humble with Gazebo Fortress (ign-gazebo 6.17.1) on an
aarch64 Jetson, real-time factor ≈ 1.0 with the GUI up.

`gazebo_harmonic.xacro` is the same plugin layer for Jazzy / Gazebo Harmonic —
identical apart from `gz-sim-*` / `gz::sim::systems::` naming. Select it with
`gz_flavor:=harmonic`. Both are generated from the same source, and the fixes
above are in both.

Kinematics only, no simulator:

```bash
ros2 launch vtol_tiltrotor_description display.launch.py
```
# uav_spacetech
