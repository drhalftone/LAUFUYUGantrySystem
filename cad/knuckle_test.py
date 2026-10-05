"""Test coupon for the bolted chain hinge in carriage_chain.py: one outer and one inner knuckle.

Outer: the two end knuckles (4.4 mm clamped hole, M4 head counterbore, nylock pocket)
on a LEAF-wide strip. Inner: the middle knuckle with a TEST_PIVOT_D hole (the chain
uses PIVOT_D) and LINK_CLEAR to the end knuckles, on its own strip, the hole size
engraved on top. Print both flat, bolt them with the M4 x 55 + nylock, and feel the
wiggle against a 5.0 mm part.

Run:  python knuckle_test.py [pivot_d]
  -> knuckle_test_outer.stl/.step, knuckle_test_inner.stl/.step  (print flat)
  -> knuckle_test.step   both assembled, with the screw and nut
"""
import sys
from pathlib import Path
import cadquery as cq
from carriage_chain import L, T, SWING_CLEAR, LINK_CLEAR, PIVOT_D, block, hinge_half, screw_at, nut_at

TEST_PIVOT_D = float(sys.argv[1]) if len(sys.argv) > 1 else 4.5
LEAF = 10.0           # solid strip beyond each piece's swing clearance
LABEL_DEPTH = 0.6     # engraved hole size on the inner strip

outer = hinge_half(block(0, L, -SWING_CLEAR - LEAF, -SWING_CLEAR), 0, -1, "upper")
inner = hinge_half(block(0, L, SWING_CLEAR, SWING_CLEAR + LEAF), 0, +1, "lower", LINK_CLEAR, TEST_PIVOT_D)
label = (cq.Workplane("XY").workplane(offset=T)
         .center(L / 2, SWING_CLEAR + LEAF / 2).text(f"{TEST_PIVOT_D:g}", 6, -LABEL_DEPTH, kind="bold"))
inner = inner.cut(label)

if __name__ == "__main__":
    here = Path(__file__).parent
    for name, part in (("knuckle_test_outer", outer), ("knuckle_test_inner", inner)):
        cq.exporters.export(part, str(here / f"{name}.step"))
        cq.exporters.export(part, str(here / f"{name}.stl"), tolerance=0.01, angularTolerance=0.1)
    assy = (cq.Assembly().add(outer, name="outer", color=cq.Color(0.9, 0.47, 0.12))
            .add(inner, name="inner", color=cq.Color(0.24, 0.47, 0.78))
            .add(screw_at(0), name="screw", color=cq.Color(0.15, 0.15, 0.17))
            .add(nut_at(0), name="nut", color=cq.Color(0.8, 0.8, 0.82)))
    assy.save(str(here / "knuckle_test.step"))
    for name, part in (("outer", outer), ("inner", inner)):
        bb = part.val().BoundingBox()
        print(f"{name}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {part.val().Volume() / 1000:.1f} cm^3")
    print(f"inner hole {TEST_PIVOT_D} mm (chain uses {PIVOT_D}); "
          f"diametral play on a 4.0 mm screw {TEST_PIVOT_D - 4.0:.1f} mm (was {PIVOT_D - 4.0:.1f})")
