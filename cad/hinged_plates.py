"""Two flat plates side by side, hinged along their shared long edge.

Three knuckles run along the seam: a short one on plate A at each end and one
long one on plate B filling the middle. One long M4 screw passes lengthwise
through all of them as the hinge pin: its head sits in a counterbore in one of
A's end knuckles and a nylock nut is captive in a hex pocket in the other, so
nothing sticks out past the ends and it tightens with just a hex key.

Everything is square-edged: each knuckle is a T x T block on the hinge axis, so
the pair is a flat L x 2W x T rectangle when open. Opposite each of the other
plate's knuckles, a plate stops at a straight face SWING_CLEAR from the axis,
far enough that the knuckle's corners (T/2 * sqrt 2 out) clear it as it turns.

Frame: hinge axis along X at Y = 0, Z = T/2. Plate A is Y < 0, plate B is Y > 0.

Run:  python hinged_plates.py
  -> hinged_plate_a.stl, hinged_plate_b.stl   (print both lying flat)
  -> hinged_plates.step, hinged_plates.glb    (assembly with pin, for viewing)
"""
import math
from pathlib import Path
import cadquery as cq

L = 80.0            # plate length (along the hinge)
W = 30.0            # plate width (from hinge axis to outer edge)
T = 10.0            # plate thickness = knuckle size (square)
END_KNUCKLE = 16.0  # length of each of plate A's end knuckles; B's fills the rest
KNUCKLE_GAP = 2.0   # axial space between neighbouring knuckles (1 mm each face)
SWING_GAP = 1.0     # clearance between a turning knuckle's corners and the other plate
PIN_D = 4.4         # M4 hinge-pin hole in plate A (clamped by the screw and nut)
PIVOT_D = 5.0       # M4 hinge-pin hole in plate B (0.5 mm radial play so it turns freely)
HEAD_D = 7.5        # counterbore for the M4 SHCS head (7.0)
HEAD_DEPTH = 7.0    # head sits 3 mm below the end face (room for the hex key)
NUT_AF = 7.3        # hex pocket across flats for the M4 nylock nut (7.0)
NUT_DEPTH = 8.0     # nut is pushed to the bottom of the pocket
# Screw length: runs from the counterbore floor (x = 7) through the nut (x = 72..76.7)
# -> M4 x 70 SHCS ends at x = 77.
PIN_LEN = 70.0

VIEW_FOLD = 45.0    # degrees plate B is folded up in the .step/.glb assembly (STLs stay flat)

r = T / 2
# (start, end, owner) of each knuckle along the seam, before axial gaps
SWING_CLEAR = r * math.sqrt(2) + SWING_GAP   # where a plate stops opposite the other's knuckle
SPANS = [(0, END_KNUCKLE, "a"), (END_KNUCKLE, L - END_KNUCKLE, "b"), (L - END_KNUCKLE, L, "a")]


def along_axis(x0, length, radius):
    """Cylinder on the hinge axis from x0 to x0 + length."""
    return (cq.Workplane("YZ").workplane(offset=x0)
            .center(0, r).circle(radius).extrude(length))


def block(x0, x1, y0, y1):
    """Full-thickness box x0..x1, y0..y1 (any order)."""
    y0, y1 = sorted((y0, y1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, T, centered=False).translate((x0, y0, 0))


def plate(side, owner):
    """side = -1 for A (Y < 0), +1 for B; owner = "a" or "b" (which SPANS it owns)."""
    # main leaf stops short of the axis everywhere...
    body = block(0, L, side * W, side * SWING_CLEAR)
    # ...and reaches across it only as square knuckles in its own spans
    for x0, x1, who in SPANS:
        if who == owner:
            g0 = KNUCKLE_GAP / 2 if x0 > 0 else 0
            g1 = KNUCKLE_GAP / 2 if x1 < L else 0
            body = body.union(block(x0 + g0, x1 - g1, side * SWING_CLEAR, -side * r))
    body = body.cut(along_axis(-1, L + 2, (PIN_D if owner == "a" else PIVOT_D) / 2))
    if owner == "a":
        body = body.cut(along_axis(-1, HEAD_DEPTH + 1, HEAD_D / 2))
        # hex pocket with flats parallel to the plate faces
        nut = (cq.Workplane("YZ").workplane(offset=L - NUT_DEPTH).center(0, r)
               .polygon(6, NUT_AF / math.cos(math.pi / 6)).extrude(NUT_DEPTH + 1))
        body = body.cut(nut)
    return body


plate_a = plate(-1, "a")
plate_b = plate(+1, "b")
pin = (along_axis(HEAD_DEPTH - 4, 4, 3.5)             # M4 SHCS head
       .union(along_axis(HEAD_DEPTH, PIN_LEN, 2.0))   # shank
       .union(cq.Workplane("YZ").workplane(offset=L - NUT_DEPTH).center(0, r)
              .polygon(6, 7.0 / math.cos(math.pi / 6)).extrude(4.7)))  # nylock nut

if __name__ == "__main__":
    here = Path(__file__).parent
    cq.exporters.export(plate_a, str(here / "hinged_plate_a.stl"), tolerance=0.01, angularTolerance=0.1)
    cq.exporters.export(plate_b, str(here / "hinged_plate_b.stl"), tolerance=0.01, angularTolerance=0.1)
    assy = (cq.Assembly()
            .add(plate_a, name="plate_a", color=cq.Color(0.9, 0.47, 0.12))
            .add(plate_b.rotate((0, 0, r), (1, 0, r), VIEW_FOLD), name="plate_b",
                 color=cq.Color(0.24, 0.47, 0.78))
            .add(pin, name="m4_pin", color=cq.Color(0.35, 0.35, 0.37)))
    assy.save(str(here / "hinged_plates.step"))
    assy.save(str(here / "hinged_plates.glb"))
    print("wrote hinged_plate_a/b.stl, hinged_plates.step/.glb")
