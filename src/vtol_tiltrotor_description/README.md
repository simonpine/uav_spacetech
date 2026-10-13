# vtol_tiltrotor_description

URDF/xacro of a **SNOOP-style tri-motor tiltrotor VTOL** for ROS 2 Jazzy
(Gazebo Harmonic) or Humble (Gazebo Fortress, see below).

> **This package is now flown by `vtol_control`.** See the workspace README for
> the keyboard controls and `../../README.md#three-defects-in-the-model` for the
> three model defects that had to be fixed before it would simulate: the
> `LiftDrag` plugins were bound to links that do not survive URDF-to-SDF
> conversion, the servo gains were too low to reach their setpoints, and servos
> latch on their hard joint limits. `gazebo_fortress.xacro` is the Humble plugin
> layer; `gazebo_harmonic.xacro` is the same file with Harmonic naming.

Modelled on the **Cannon SNOOP 18**, the aircraft in the reference photo. Per
the manufacturer it is a tri-motor tilt-rotor: the two front rotors swing from
vertical (VTOL) to horizontal (aeroplane flight), and the **rear rotor tilts to
provide steering**.

## Layout

| | |
|---|---|
| Front rotors | On booms projecting ahead of the wing leading edge. Tilt about the **pitch** axis: `0` = hover, `pi/2` = cruise. |
| Rear rotor | On a pylon over the tail boom. Tilts about the **roll** axis, ±30°. **This is the yaw actuator in hover** — tricopter style. It does *not* tilt for transition. |
| Tail | V-tail with two ruddervators. |
| Wing | 1.70 m span, tapered, raked outboard tips, ailerons. |
| Gear | Four skids with hi-vis tips, all four contacting z = −0.240. |

## Verified numbers

Computed from the expanded URDF, not estimated:

```
mass            5.496 kg      (SNOOP 18 MTOW: 5.5 kg)
wingspan        1.700 m       (published: 1700 mm)
length          1.310 m       (published frame length: 1310 mm)
CoM             x = -0.004 m, z = +0.036 m
hover trim      16.0 N per front rotor, 21.8 N rear
max thrust      36.0 N per rotor  ->  T/W = 2.00
links / joints  26 / 25
```

Actuated joints:

```
tilt_left_joint         revolute  axis y   [-0.17, 1.571]   transition
tilt_right_joint        revolute  axis y   [-0.17, 1.571]   transition
rear_tilt_joint         revolute  axis x   [-0.524, 0.524]  YAW in hover
aileron_left_joint      revolute  axis y   [-0.44, 0.44]
aileron_right_joint     revolute  axis y   [-0.44, 0.44]
ruddervator_left_joint  revolute  axis y   [-0.52, 0.52]
ruddervator_right_joint revolute  axis y   [-0.52, 0.52]
rotor_left_joint        continuous
rotor_right_joint       continuous
rotor_rear_joint        continuous
```

V-tail mixing: **elevator** = both ruddervators same sign; **rudder** = opposite
signs.

## Read this before assuming it flies

1. **A URDF has no aerodynamics.** Lift, drag and thrust come entirely from
   `gazebo_harmonic.xacro`. Remove it and you have a static model.
2. **There is no controller in *this* package.** No stabilisation, no mixer, no
   transition sequencing. `vtol_control` supplies all three; alternatively use
   PX4/ArduPilot SITL.
3. **Aero coefficients are generic flat-plate defaults**, not SNOOP data.
   `cla=4.75`, `alpha_stall=0.30`. Retune against real data or XFLR5 before the
   cruise numbers mean anything.
4. **Rear rotor thrust is 36% higher than each front rotor in hover** because of
   the arm lengths. Real tri-motor VTOLs handle this with a larger tail prop or
   a hotter tail motor; if you want them even, move `rear_x` to `-0.720`.
5. **Geometry is primitives.** Swap `<geometry>` for meshes when you have them,
   but keep the link frames: `LiftDrag` `<cp>` is in the link frame and the wing
   and V-tail frames are deliberately placed at their panel aerodynamic centres.

## Build and view

```bash
cd ~/ros2_ws/src && cp -r vtol_tiltrotor_description .
cd ~/ros2_ws && colcon build --packages-select vtol_tiltrotor_description
source install/setup.bash
ros2 launch vtol_tiltrotor_description display.launch.py
```

Drag `tilt_left_joint` / `tilt_right_joint` from 0 to 1.57 for the transition,
and `rear_tilt_joint` for the yaw vectoring.

## Simulate (Jazzy + Gazebo Harmonic)

```bash
xacro urdf/vtol_tiltrotor.urdf.xacro use_gazebo:=true > /tmp/vtol.urdf
gz sim -r empty.sdf
ros2 run ros_gz_sim create -file /tmp/vtol.urdf -name vtol -z 0.3
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix vtol_tiltrotor_description)/share/vtol_tiltrotor_description/config/gz_bridge.yaml
```

```bash
# hover-ish rotor speeds (rad/s): [front_left, front_right, rear]
ros2 topic pub /vtol/command/motor_speed actuator_msgs/msg/Actuators \
  "{velocity: [1000.0, 1000.0, 1167.0]}"

# yaw right in hover
ros2 topic pub /vtol/rear_tilt/cmd std_msgs/msg/Float64 "{data: 0.35}"

# start the transition
ros2 topic pub /vtol/tilt_left/cmd  std_msgs/msg/Float64 "{data: 1.57}"
ros2 topic pub /vtol/tilt_right/cmd std_msgs/msg/Float64 "{data: 1.57}"
```

It will not hold attitude on its own. Expected — see limitation 2. For
stabilised flight use `ros2 launch vtol_control vtol_sim.launch.py`, which does
the bridging above and adds the controller.

## Humble

Humble pairs with Gazebo Fortress. In `gazebo_harmonic.xacro` replace every
`gz-sim-*` filename with `ignition-gazebo-*` and every `gz::sim::systems::`
with `ignition::gazebo::systems::`. Parameter names are otherwise identical.
Humble + Gazebo **Classic** 11 is a different plugin set entirely
(`libgazebo_motor_model.so`, `libLiftDragPlugin.so`) requiring the
`PX4/PX4-SITL_gazebo-classic` plugin library on `GAZEBO_PLUGIN_PATH`.

## If the goal is actually flying transitions

Use PX4 SITL. Its `tiltrotor` airframe is the same two-tilting-front layout and
the VTOL transition state machine is already written and flight-proven; bridge
it to ROS 2 via `px4_ros_com` / uXRCE-DDS. Note PX4's stock tiltrotor yaws with
differential front tilt, so if you want the SNOOP's tilting tail rotor you will
need a custom mixer either way. Reserve this URDF for visualisation, TF, sensor
mounting and `ros2_control` work.
