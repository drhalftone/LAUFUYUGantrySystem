"""Print-in-place carriage hinge plate + first chain link, hinged together.

Same outline as carriage_chain.py (65 x 48 x 10 carriage plate, counterbored
M4 x 12 into the carriage, one 85 mm link), but the hinge is printed already
assembled instead of pinned with an M4 screw. The link's middle knuckle has a
cone pin on each end face that sits in a cone socket in the plate's end
knuckle, PIP_CLEAR away all round. The knuckle faces are only PIP_GAP apart,
so 3.5 of each 4 mm pin is inside its socket. The 45 degree cones print flat without
supports (pin undersides and socket roofs are 45 degree overhangs) and hold the
link captive both ways along the axis.

Frame and flat print orientation as carriage_chain.py: X along the hinge
(X = 0 at the motor end), Y across the carriage (hinge side +Y), Z up from the
bed / carriage top. The link's far end is plain.

Run:  python carriage_link_pip.py
  -> carriage_link_pip.stl   both bodies in one file, print flat, no supports
  -> carriage_link_pip.step  same, as two solids
"""
import math
from pathlib import Path
import cadquery as cq
from carriage_chain import (L, T, r, PLATE_W, HOLE_PITCH, PATTERN_X, CLEAR_D, CBORE_D, CBORE_DEPTH,
                            SWING_CLEAR, AXIS_Y, SPANS, LINK_PITCH, block)

PIP_CLEAR = 0.4       # gap between pin and socket, normal to the cone faces; tune for your printer
CONE_R = 4.0          # pin radius at the middle knuckle's face (45 deg cone, so also its length)
PIP_GAP = 0.5         # axial gap between knuckle faces (2 mm in the screw hinge); small so the pins reach deep

SOCKET_SHIFT = PIP_CLEAR * math.sqrt(2)   # a 45 deg cone offset by PIP_CLEAR moves its apex this far


def cone(x_base, radius, direction):
    """45 deg cone on the hinge axis, base at x_base, pointing along +X (direction=+1) or -X (-1)."""
    return cq.Workplane("XY").add(cq.Solid.makeCone(
        radius, 0, radius, cq.Vector(x_base, AXIS_Y, r), cq.Vector(direction, 0, 0)))


def knuckles(owner, y_from):
    """Square knuckles in `owner`'s spans, from y_from across the axis to its far side."""
    body = None
    for x0, x1, who in SPANS:
        if who == owner:
            g0 = PIP_GAP / 2 if x0 > 0 else 0
            g1 = PIP_GAP / 2 if x1 < L else 0
            k = block(x0 + g0, x1 - g1, y_from, AXIS_Y + (r if owner == "upper" else -r))
            body = k if body is None else body.union(k)
    return body


# Middle knuckle face positions (where the link's pins start)
(_, mid_x0, _), (_, mid_x1, _) = SPANS[0], SPANS[1]
face0, face1 = mid_x0 + PIP_GAP / 2, mid_x1 - PIP_GAP / 2

# Carriage plate: 65 x 48 plate + end knuckles with cone sockets
pattern = [(PATTERN_X + sx * HOLE_PITCH / 2, sy * HOLE_PITCH / 2) for sx in (-1, 1) for sy in (-1, 1)]
plate = (block(0, L, -PLATE_W / 2, PLATE_W / 2)
         .faces(">Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0))
         .pushPoints(pattern).cboreHole(CLEAR_D, CBORE_D, CBORE_DEPTH))
plate = plate.union(knuckles("upper", PLATE_W / 2))   # AXIS_Y - SWING_CLEAR = plate edge
plate = (plate.cut(cone(face0, CONE_R + SOCKET_SHIFT, -1))
              .cut(cone(face1, CONE_R + SOCKET_SHIFT, +1)))

# Link: middle knuckle with a cone pin on each face, leaf out to a plain far end
link = (block(0, L, AXIS_Y + SWING_CLEAR, AXIS_Y + LINK_PITCH + r)
        .union(knuckles("lower", AXIS_Y + SWING_CLEAR))
        .union(cone(face0, CONE_R, -1))
        .union(cone(face1, CONE_R, +1)))

if __name__ == "__main__":
    here = Path(__file__).parent
    out = here / "carriage_link_pip"
    both = cq.Workplane("XY").add(cq.Compound.makeCompound([plate.val(), link.val()]))
    cq.exporters.export(both, str(out) + ".step")
    cq.exporters.export(both, str(out) + ".stl", tolerance=0.01, angularTolerance=0.1)

    # Check the two bodies never touch, flat and folded down 90 deg and up 180 deg
    for a in (-90, -45, 0, 45, 90):
        moved = link.rotate((0, AXIS_Y, r), (1, AXIS_Y, r), a)
        overlap = plate.intersect(moved).val().Volume() if plate.intersect(moved).vals() else 0.0
        gap = plate.val().distance(moved.val())
        print(f"link at {a:3d} deg: overlap {overlap:.4f} mm^3, min gap {gap:.3f} mm")
    bb = both.val().BoundingBox()
    print(f"print footprint {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm")
