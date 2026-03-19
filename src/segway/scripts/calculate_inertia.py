#!/usr/bin/env python3

"""
Calculate inertia tensors from mesh files for URDF.
"""

import trimesh
import numpy as np

BASE_STL  = 'segway/models/segway/meshes/STL/base/body.stl'
WHEEL_STL = 'segway/models/segway/meshes/STL/wheel/wheel_sim.stl'

BODY_MASS  = 80.0 # kg
WHEEL_MASS = 0.5 # kg


def calculate(path, mass, name):
    print(f"{name}")
    mesh = trimesh.load(path)
    print(mesh.bounds)
    print(mesh.extents)

    if not mesh.is_watertight:
        print("mesh is not watertight, inertia may be inaccurate")
        mesh = mesh.convex_hull

    # scale density so total mass matches target
    mesh.density = mass/mesh.volume
    
    print(f"  Volume: {mesh.volume:.6f} m\u00B3")
    print(f"  Mass: {mesh.mass:.3f} kg")
    print(f"  CoM: x={mesh.center_mass[0]:.4f},  y={mesh.center_mass[1]:.4f}, z={mesh.center_mass[2]:.4f}")

    I = mesh.moment_inertia
    print(f"\n  Inertia tensor (at CoM):")
    print(f"    Ixx={I[0,0]:.6f}  Ixy={I[0,1]:.6f}  Ixz={I[0,2]:.6f}")
    print(f"    Iyx={I[1,0]:.6f}  Iyy={I[1,1]:.6f}  Iyz={I[1,2]:.6f}")
    print(f"    Izx={I[2,0]:.6f}  Izy={I[2,1]:.6f}  Izz={I[2,2]:.6f}")
    print("\n")

if __name__ == '__main__':
    calculate(BASE_STL,  BODY_MASS,  'body.stl')
    calculate(WHEEL_STL, WHEEL_MASS, 'wheel_sim.stl')
