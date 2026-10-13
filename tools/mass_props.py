#!/usr/bin/env python3
"""Exact mass properties and rotor geometry of the expanded URDF.

This is where the constants at the top of vtol_control/airframe.py come from.
The allocator's moment arms have to match the model to within a few millimetres
or the attitude loops spend their integral term fighting a steady bias, so if
you edit vtol_tiltrotor.urdf.xacro, re-run this and paste the numbers across.

    python3 tools/mass_props.py

It walks the kinematic tree itself rather than using KDL, because KDL refuses a
root link that carries inertia - which base_link does - and because the rotor
stations wanted here are at the all-joints-zero pose, not a live TF snapshot.
"""

import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(HERE, 'src', 'vtol_tiltrotor_description', 'urdf',
                     'vtol_tiltrotor.urdf.xacro')
GRAVITY = 9.80665
K_THRUST = 1.6e-05
W_MAX = 1500.0


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_of(element):
    o = element.find('origin')
    xyz, rpy = np.zeros(3), np.zeros(3)
    if o is not None:
        if o.get('xyz'):
            xyz = np.array([float(v) for v in o.get('xyz').split()])
        if o.get('rpy'):
            rpy = np.array([float(v) for v in o.get('rpy').split()])
    return xyz, rpy


def main():
    urdf = subprocess.run(['xacro', XACRO, 'use_gazebo:=false'],
                          capture_output=True, text=True, check=True).stdout
    root = ET.fromstring(urdf)

    parent = {}
    for j in root.findall('joint'):
        parent[j.find('child').get('link')] = (j.find('parent').get('link'),
                                               *origin_of(j))

    def pose_in_base(link):
        """Link frame in base_link, at the all-joints-zero pose."""
        chain, cur = [], link
        while cur in parent:
            pl, xyz, rpy = parent[cur]
            chain.append((xyz, rpy))
            cur = pl
        pos, rot = np.zeros(3), np.eye(3)
        for xyz, rpy in reversed(chain):
            pos = pos + rot @ xyz
            rot = rot @ rpy_to_matrix(*rpy)
        return pos, rot

    bodies, total_mass, weighted = [], 0.0, np.zeros(3)
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        xyz, rpy = origin_of(inertial)
        i = inertial.find('inertia')
        i_local = np.array([
            [float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
            [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
            [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]])
        lp, lr = pose_in_base(link.get('name'))
        r = lr @ rpy_to_matrix(*rpy)
        centre = lp + lr @ xyz
        bodies.append((m, centre, r @ i_local @ r.T))
        total_mass += m
        weighted += m * centre
    com = weighted / total_mass

    inertia = np.zeros((3, 3))
    for m, centre, i_world in bodies:
        d = centre - com
        inertia += i_world + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    print(f'links with inertia   {len(bodies)}')
    print(f'MASS                 {total_mass:.4f}   kg')
    print(f'CoM in base_link     x={com[0]:+.5f}  y={com[1]:+.5f}  z={com[2]:+.5f}')
    print('inertia about the CoM, base_link axes:')
    for row in inertia:
        print('   ' + '  '.join(f'{v:+.5f}' for v in row))
    print(f'IXX = {inertia[0, 0]:.4f}   IYY = {inertia[1, 1]:.4f}   '
          f'IZZ = {inertia[2, 2]:.4f}')

    arms = {}
    print()
    for rotor in ('rotor_left', 'rotor_right', 'rotor_rear'):
        arms[rotor] = pose_in_base(rotor)[0] - com
        d = arms[rotor]
        print(f'{rotor:12s} arm from CoM  x={d[0]:+.4f}  y={d[1]:+.4f}  '
              f'z={d[2]:+.4f}')
    print()
    print(f'X_FRONT = {arms["rotor_left"][0]:.4f}')
    print(f'Y_FRONT = {arms["rotor_left"][1]:.4f}')
    print(f'Z_FRONT = {arms["rotor_left"][2]:.4f}')
    print(f'X_REAR  = {arms["rotor_rear"][0]:.4f}')
    print(f'Z_REAR  = {arms["rotor_rear"][2]:.4f}')

    # Hover trim: total thrust carries the weight and the pitch moment is zero.
    weight = total_mass * GRAVITY
    x_f, x_r = arms['rotor_left'][0], arms['rotor_rear'][0]
    f_front, f_rear = np.linalg.solve(
        np.array([[2.0, 1.0], [2.0 * x_f, x_r]]), np.array([weight, 0.0]))
    f_max = K_THRUST * W_MAX ** 2
    print()
    print(f'WEIGHT          {weight:.3f} N')
    print(f'F_FRONT_HOVER   {f_front:.3f} N   ({math.sqrt(f_front / K_THRUST):.0f} rad/s)')
    print(f'F_REAR_HOVER    {f_rear:.3f} N   ({math.sqrt(f_rear / K_THRUST):.0f} rad/s)')
    print(f'FRONT_LIFT_SHARE {2 * f_front / weight:.4f}')
    print(f'F_MAX           {f_max:.2f} N per rotor, T/W = {3 * f_max / weight:.3f}')

    # Wing and stall, from the LiftDrag numbers in the plugin layer.
    s_wing, rho, cla, a0, alpha_stall = 2 * 0.123 + 2 * 0.058, 1.2041, 4.75, 0.05, 0.30
    cl_max = cla * (alpha_stall + a0)
    print()
    print(f'S_WING          {s_wing:.4f} m2')
    print(f'CL_MAX          {cl_max:.3f}')
    print(f'V_STALL         {math.sqrt(2 * weight / (rho * s_wing * cl_max)):.2f} m/s')
    for cl in (0.5, 0.7, 1.0):
        v = math.sqrt(2 * weight / (rho * s_wing * cl))
        print(f'   level flight at cl={cl:.1f}: V={v:5.2f} m/s, '
              f'alpha={cl / cla - a0:+.3f} rad')
    return 0


if __name__ == '__main__':
    sys.exit(main())
