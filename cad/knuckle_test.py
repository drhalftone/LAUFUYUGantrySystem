"""Print-test coupon for the chain hinge: the chain's own knuckles on short leaves.

The outer piece has the chain's end knuckles exactly (4.4 mm pin hole, 8.4 mm pan-head
counterbore, nylock pocket; M4 x 60 Phillips pan head + nylock). The inner pieces have
the chain's middle knuckle (LINK_CLEAR axial clearance) with its pivot hole reduced from
PIVOT_D (5.0) by 0.5 and 1.0 mm, to find how tight it can be and still turn on the screw.
Each inner piece has its hole size engraved on top.

Frame: hinge axis along X at Y = 0, Z = T/2; outer piece at -Y, inner pieces at +Y.

Run:  python knuckle_test.py
  -> knuckle_test_outer.stl, knuckle_test_inner_4.5.stl, knuckle_test_inner_4.0.stl  (print flat)
  -> knuckle_test_plate.stl   all three flat, side by side, for one print
"""
from pathlib import Path
import cadquery as cq
from carriage_chain import L, T, SWING_CLEAR, LINK_CLEAR, PIVOT_D, block, hinge_half

LEAF = 15.0               # leaf width past the swing clearance
REDUCTIONS = (0.5, 1.0)   # taken off PIVOT_D for the inner pieces
LABEL_DEPTH = 0.6
PLATE_GAP = 5.0           # between pieces on the combined print plate

outer = hinge_half(block(0, L, -SWING_CLEAR - LEAF, -SWING_CLEAR), 0, -1, "upper")


def inner(pivot_d):
    piece = hinge_half(block(0, L, SWING_CLEAR, SWING_CLEAR + LEAF), 0, +1, "lower", LINK_CLEAR, pivot_d)
    label = (cq.Workplane("XY").workplane(offset=T - LABEL_DEPTH)
             .center(L / 2, SWING_CLEAR + LEAF / 2).text(f"{pivot_d:.1f}", 8, LABEL_DEPTH + 1, kind="bold"))
    return piece.cut(label)


if __name__ == "__main__":
    here = Path(__file__).parent
    parts = {"knuckle_test_outer": outer}
    parts.update({f"knuckle_test_inner_{PIVOT_D - d:.1f}": inner(PIVOT_D - d) for d in REDUCTIONS})
    for name, part in parts.items():
        cq.exporters.export(part, str(here / f"{name}.stl"), tolerance=0.01, angularTolerance=0.1)
        b = part.val().BoundingBox()
        print(f"{name}.stl  {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm")

    plate, y = cq.Workplane("XY"), 0.0
    for part in parts.values():
        b = part.val().BoundingBox()
        plate = plate.add(part.translate((0, y - b.ymin, -b.zmin)).vals())
        y += b.ylen + PLATE_GAP
    plate = cq.Workplane("XY").add(cq.Compound.makeCompound(plate.vals()))
    cq.exporters.export(plate, str(here / "knuckle_test_plate.stl"), tolerance=0.01, angularTolerance=0.1)
    b = plate.val().BoundingBox()
    print(f"knuckle_test_plate.stl  {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm")
