"""Test coupon for the print-in-place hinge in carriage_link_pip.py.

Just the hinge: both parts cut down to LEAF of solid leaf either side of the
knuckles, full 65 mm length so all three knuckles and both cone pins are there.
Print it flat to tune PIP_CLEAR (in carriage_link_pip.py) before printing the
full parts.

Run:  python carriage_link_pip_test.py  -> carriage_link_pip_test.stl/.step
"""
from pathlib import Path
import cadquery as cq
from carriage_link_pip import plate, link, PIP_CLEAR
from carriage_chain import L, T, AXIS_Y, SWING_CLEAR, block

LEAF = 10.0           # solid leaf left on each side, beyond the knuckles

keep = block(-1, L + 1, AXIS_Y - SWING_CLEAR - LEAF, AXIS_Y + SWING_CLEAR + LEAF)
test_plate, test_link = plate.intersect(keep), link.intersect(keep)

if __name__ == "__main__":
    out = Path(__file__).with_suffix("")
    both = cq.Workplane("XY").add(cq.Compound.makeCompound([test_plate.val(), test_link.val()]))
    cq.exporters.export(both, str(out) + ".step")
    cq.exporters.export(both, str(out) + ".stl", tolerance=0.01, angularTolerance=0.1)
    print(f"min gap {test_plate.val().distance(test_link.val()):.3f} mm (PIP_CLEAR {PIP_CLEAR})")
    bb = both.val().BoundingBox()
    print(f"print footprint {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, "
          f"{both.val().Volume() / 1000:.1f} cm^3 vs {(plate.val().Volume() + link.val().Volume()) / 1000:.1f} for the full parts")
