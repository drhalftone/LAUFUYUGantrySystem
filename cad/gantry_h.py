"""H-gantry fit check: two FSK40 legs on the table, a third FSK40 across their carriages.

Legs ("X"): two parallel FSK40s, motors at the same end, each carriage carrying
fsk40_carriage_plate. Cross beam ("Y"): an FSK40 turned 90 degrees, its base
resting on the two leg plates. Its carriage carries carriage_hinge_plate with
the hanging chain from carriage_chain.py.

World frame = the TraceParts model frame of the left leg: X along the legs,
Y up, Z across (table top is the underside of the leg bases, Y = -18.69).

Writes gantry_h.glb (not committed: embeds the TraceParts FSK40 model).
Set Y_TRAVEL to slide the cross-beam carriage along the beam, e.g. python gantry_h.py 400
"""
import sys
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix, translation_matrix

FSK40 = r"C:\Users\dllau\Downloads\192475275-21-fsk40-e1250-10c7-bc-b57\fsk40-e1250-10c7-bc-b57.stl"
Y_TRAVEL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0   # cross-beam carriage offset from mid (mm)

# FSK40 model landmarks (TraceParts frame: X travel, Y up, Z across)
BASE_BOTTOM = -18.69                     # underside of the rail base = table
BASE_X = (-97.32, 1277.68)               # ends of the base extrusion
CENTRE_Z = -101.15                       # rail centre line
CARRIAGE_X = (579.18, 644.18)            # carriage as modelled (mid-stroke)
CARRIAGE_TOP = 51.51
PLATE_T = 10.0
SUPPORT_INSET = 40.0                     # leg centre line this far in from each end of the beam's base


def load_rail():
    rail = trimesh.load(FSK40)
    rail.merge_vertices()
    bodies = rail.split(only_watertight=False)
    moving = [b for b in bodies if b.bounds[0][0] > 555 and b.bounds[1][0] < 675 and b.bounds[0][1] > 4]
    fixed = [b for b in bodies if not any(b is m for m in moving)]
    return trimesh.util.concatenate(fixed), trimesh.util.concatenate(moving)


def plain_plate():
    """fsk40_carriage_plate on the modelled carriage (same placement as fsk40_with_plate.py)."""
    p = trimesh.load("fsk40_carriage_plate.stl")
    M = np.eye(4)
    M[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    M[:3, 3] = [612.68, CARRIAGE_TOP, CENTRE_Z]
    return p.apply_transform(M)


rail_fixed, rail_carriage = load_rail()

# Cross-beam placement: rotate the rail model 90 deg about Y (its X -> world -Z, its Z -> world X),
# centre it over the leg carriages, and sit its base on the leg plates.
leg_x = (CARRIAGE_X[0] + CARRIAGE_X[1]) / 2
beam_on_legs = CARRIAGE_TOP + PLATE_T - BASE_BOTTOM
dz = (CENTRE_Z + SUPPORT_INSET) - (-BASE_X[0])          # beam's motor-end base corner just past the left leg
BEAM = translation_matrix([leg_x - CENTRE_Z, beam_on_legs, dz]) @ rotation_matrix(np.pi / 2, [0, 1, 0])
right_leg_z = -BASE_X[1] + dz + SUPPORT_INSET            # right leg centre line
LEG_SPACING = CENTRE_Z - right_leg_z
RIGHT = translation_matrix([0, 0, -LEG_SPACING])
SLIDE = translation_matrix([Y_TRAVEL, 0, 0])             # along the beam, in the beam's own frame

import fsk40_with_chain as fc                            # chain pieces on a carriage, in the model frame

scene = {}
grey, dark, orange, blue, pin = [170, 175, 185, 255], [120, 125, 135, 255], [230, 120, 30, 255], [60, 120, 200, 255], [90, 90, 95, 255]
for name, m, M, c in [
    ("left_leg", rail_fixed, np.eye(4), grey), ("left_carriage", rail_carriage, np.eye(4), dark),
    ("right_leg", rail_fixed, RIGHT, grey), ("right_carriage", rail_carriage, RIGHT, dark),
    ("left_plate", plain_plate(), np.eye(4), orange), ("right_plate", plain_plate(), RIGHT, orange),
    ("beam", rail_fixed, BEAM, grey), ("beam_carriage", rail_carriage, BEAM @ SLIDE, dark),
] + [("chain_" + n, p, BEAM @ SLIDE, orange if n == "plate" else blue if n.startswith("link") else pin)
     for n, p in fc.chain_pieces(beam_on_legs + CARRIAGE_TOP + PLATE_T / 2 - BASE_BOTTOM).items()]:
    mm = m.copy().apply_transform(M)
    mm.visual.face_colors = c
    scene[name] = mm

lo = np.min([m.bounds[0] for m in scene.values()], axis=0)
hi = np.max([m.bounds[1] for m in scene.values()], axis=0)
table = trimesh.creation.box(extents=[hi[0] - lo[0] + 100, 12, hi[2] - lo[2] + 100])
table.apply_translation([(lo[0] + hi[0]) / 2, BASE_BOTTOM - 6, (lo[2] + hi[2]) / 2])
table.visual.face_colors = [205, 185, 150, 255]

if __name__ == "__main__":
    s = trimesh.Scene(scene)
    s.add_geometry(table, node_name="table")
    s.export("gantry_h.glb")
    print(f"leg spacing (centre to centre) {LEG_SPACING:.1f} mm; beam base on the leg plates at Y = {beam_on_legs + BASE_BOTTOM:.2f}")
    chain = {k: v for k, v in scene.items() if k.startswith("chain_") and k != "chain_plate"}
    print(f"chain bottom is {min(v.bounds[0][1] for v in chain.values()) - BASE_BOTTOM:.2f} mm above the table")
    others = {k: v for k, v in scene.items() if not k.startswith("chain_")}
    others["table"] = table
    for cn, cm in chain.items():
        worst = None
        for on, om in others.items():
            near = om.vertices[np.all((om.vertices > cm.bounds[0] - 10) & (om.vertices < cm.bounds[1] + 10), axis=1)]
            if len(near):
                d = (-trimesh.proximity.signed_distance(cm, near)).min()   # >0 outside the chain piece
                if worst is None or d < worst[0]:
                    worst = (d, on)
        print(f"  {cn}: " + (f"closest {worst[0]:.2f} mm to {worst[1]}" if worst else "nothing within 10 mm"))
