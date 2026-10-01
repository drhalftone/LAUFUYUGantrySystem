"""Print-in-place carriage plate + chain, folded like an accordion so it prints in one go.

The carriage hinge plate and N_LINKS links are printed folded flat against each
other, standing on edge: every hinge pin runs along X (horizontal), the plate and
links stand up as vertical pages 1 mm apart, and the hinges sit alternately along
the top and the bottom of the stack. The print is about 65 x 45 x 60 mm instead of
a 65 x 290 mm strip; 65 mm (the carriage length) is the longest side.

To fold 180 degrees flat, each hinge pin sits in the 1 mm gap between the two pages
it joins (not in the middle of a plate, as in carriage_chain.py). Knuckles are
diamonds (squares turned 45 degrees) so every face prints at 45 degrees, with the
two points along the chain cut flat by CHOP so the bottom knuckles stand on the bed.
The pins are the cone pins of carriage_link_pip.py. Each page stops RELIEF from the
pin opposite the other page's knuckles, and where that notch runs out to the end
of a page its top is cut at 45 degrees so it prints without support.

Links are all the same part; every other one is flipped. Each hinge turns from
folded (-180) through straight (0) to 90 degrees the other way, which covers the
chain hanging over the side of the carriage and lying down on the table.

Frame (flat, as on the carriage): X along the hinges / carriage travel, X = 0 at the
motor end; Y across the carriage, 0 at its centre, hinge side +Y; Z up, 0 on the
carriage top. A link is drawn straight out along +Y with its first pin at Y = 0.

Run:  python folding_chain.py
  -> folding_chain.stl/.step         folded, as printed (stand it on the bed as is)
  -> folding_chain_draped.step       unfolded on the H-gantry cross beam, hanging to the table
"""
from pathlib import Path
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf
from carriage_chain import L, PLATE_W, HOLE_PITCH, PATTERN_X, CLEAR_D, CBORE_D, CBORE_DEPTH, SPANS
from carriage_link_pip import PIP_CLEAR, PIP_GAP, SOCKET_SHIFT

PLATE_T = 10.0        # carriage plate thickness
LINK_T = 6.0          # link thickness
PAGE_GAP = 1.0        # gap between folded pages; the pin sits in the middle of it
D = PAGE_GAP / 2 + LINK_T   # knuckle half-diagonal: a knuckle spans both pages it joins
CHOP = 1.5            # knuckle points along the chain cut off by this (flat on the bed)
RELIEF = D + 1.0      # a page stops this far from the pin opposite the other page's knuckles
CONE_R = 3.5          # cone pin radius at the knuckle face (fits inside the diamond with 1 mm wall)
N_LINKS = 5
# Pin-to-pin length: makes the bottom of the folded plate and the bottom knuckles both sit on the bed
PITCH = PLATE_W + RELIEF - (D - CHOP)
TABLE_Z = -150.4      # table below the carriage top in the H-gantry (see carriage_chain.py)

AXIS_Y = PLATE_W / 2 + RELIEF   # carriage-plate pin: plate edge stays flush with the carriage
AXIS_Z = -PAGE_GAP / 2          # ...just under the plate, so link 1 folds flat underneath it

(_, e0, _), (_, e1, _) = SPANS[0], SPANS[1]
UPPER_X = [(0, e0 - PIP_GAP / 2), (e1 + PIP_GAP / 2, L)]   # end knuckles (sockets)
LOWER_X = (e0 + PIP_GAP / 2, e1 - PIP_GAP / 2)              # middle knuckle (cone pins)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def knuckle(x0, x1, y, z):
    """Diamond knuckle along X on the pin at (y, z), points along Y cut flat by CHOP."""
    a, c = D - CHOP, CHOP
    pts = [(a, c), (0, D), (-a, c), (-a, -c), (0, -D), (a, -c)]
    return cq.Workplane("YZ").workplane(offset=x0).center(y, z).polyline(pts).close().extrude(x1 - x0)


def cone(x_base, radius, direction, y, z):
    return cq.Workplane("XY").add(cq.Solid.makeCone(
        radius, 0, radius, cq.Vector(x_base, y, z), cq.Vector(direction, 0, 0)))


def add_upper(body, y, z):
    """End knuckles with cone sockets on the pin at (y, z)."""
    for x0, x1 in UPPER_X:
        body = body.union(knuckle(x0, x1, y, z))
    return (body.cut(cone(LOWER_X[0], CONE_R + SOCKET_SHIFT, -1, y, z))
                .cut(cone(LOWER_X[1], CONE_R + SOCKET_SHIFT, +1, y, z)))


def add_lower(body, y, z):
    """Middle knuckle with a cone pin on each end on the pin at (y, z)."""
    return (body.union(knuckle(*LOWER_X, y, z))
                .union(cone(LOWER_X[0], CONE_R, -1, y, z))
                .union(cone(LOWER_X[1], CONE_R, +1, y, z)))


# Carriage plate: fsk40 plate, end knuckles on a pin under its +Y edge
pattern = [(PATTERN_X + sx * HOLE_PITCH / 2, sy * HOLE_PITCH / 2) for sx in (-1, 1) for sy in (-1, 1)]
plate = (box(0, L, -PLATE_W / 2, PLATE_W / 2, 0, PLATE_T)
         .faces(">Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0))
         .pushPoints(pattern).cboreHole(CLEAR_D, CBORE_D, CBORE_DEPTH))
for x0, x1 in UPPER_X:
    plate = plate.union(box(x0, x1, PLATE_W / 2, AXIS_Y, 0, PLATE_T))
plate = add_upper(plate, AXIS_Y, AXIS_Z)

# Link: pin A (middle knuckle) at Y = 0 under the link, pin B (end knuckles) at Y = PITCH over it
f, g = UPPER_X[0][1], LOWER_X[0]
outline = [(0, RELIEF + g), (g, RELIEF), (g, 0), (L - g, 0), (L - g, RELIEF), (L, RELIEF + g),   # A end, 45 deg notches
           (L, PITCH), (L - f, PITCH), (L - f, PITCH - RELIEF), (f, PITCH - RELIEF), (f, PITCH), (0, PITCH)]
link = cq.Workplane("XY").polyline(outline).close().extrude(LINK_T)
link = add_lower(link, 0, AXIS_Z)
link = add_upper(link, PITCH, LINK_T + PAGE_GAP / 2)

# Last link: no pin B; its plain end runs on down to the bed instead
end = PITCH + D - CHOP
end_link = cq.Workplane("XY").polyline(outline[:6] + [(L, end), (0, end)]).close().extrude(LINK_T)
end_link = add_lower(end_link, 0, AXIS_Z)
LINKS = [link] * (N_LINKS - 1) + [end_link]


# --- placing the chain: 4x4 transforms -------------------------------------------------
def trans(x, y, z):
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def rot_x(deg, y=0.0, z=0.0):
    """Rotation by deg about the line parallel to X through (y, z)."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    R = np.eye(4)
    R[1:3, 1:3] = [[c, -s], [s, c]]
    return trans(0, y, z) @ R @ trans(0, -y, -z)


FLIP = trans(L, PITCH, LINK_T) @ np.diag([-1.0, 1, -1, 1])   # next link, turned over, its pin A on this one's pin B
HINGE = lambda deg: rot_x(deg, 0, AXIS_Z)                     # a link turning on its own pin A
PRINT = trans(0, PLATE_T, PLATE_W / 2) @ rot_x(90)           # folded stack -> standing on the bed


def placements(angles):
    """World transform of each link for hinge angles (0 straight, -180 folded flat)."""
    W, out = trans(0, AXIS_Y, 0) @ HINGE(angles[0]), []
    out.append(W)
    for a in angles[1:]:
        W = W @ FLIP @ HINGE(a)
        out.append(W)
    return out


def moved(part, M):
    t = gp_Trsf()
    t.SetValues(*M[:3].flatten())
    return part.val().moved(cq.Location(t))


def drape_angles(n=N_LINKS):
    """Hinge angles for the chain hanging from the plate down onto the table.

    Each link hangs straight down if it clears the table; otherwise it tips outward
    (bisected) until it, or the next link lying flat after it, just touches the table.
    """
    angles, slopes = [], []   # slope: degrees below horizontal, outward

    def hinge_angles(sl):     # turned-over links turn the other way
        prev = [0.0] + sl[:-1]
        return [(-1) ** k * -(s - p) for k, (s, p) in enumerate(zip(sl, prev))]

    for k in range(n):
        def low(s):
            sl = (slopes + [s, 0.0])[:n]
            return min(moved(p, M).BoundingBox().zmin
                       for p, M in list(zip(LINKS, placements(hinge_angles(sl))))[k:k + 2]) - TABLE_Z
        if low(90) >= 0:
            s = 90.0
        elif low(-45) <= 0:
            s = -45.0
        else:
            lo, hi = -45.0, 90.0
            for _ in range(30):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if low(mid) > 0 else (lo, mid)
            s = lo
        slopes.append(s)
    return hinge_angles(slopes), slopes


def clash(a, b):
    v = a.intersect(b).Volume()
    return v, (0.0 if v > 1e-6 else a.distance(b))


if __name__ == "__main__":
    here = Path(__file__).parent
    print(f"link pitch {PITCH:.1f} mm x {N_LINKS} = {PITCH * N_LINKS:.0f} mm of chain")

    print("hinge range (overlap mm^3, min gap mm):")
    for a in (-180, -135, -90, -45, 0, 45, 90):
        pl = clash(plate.val(), moved(link, placements([a])[0])) if a <= 0 else None
        ll = clash(link.val(), moved(link, FLIP @ HINGE(a)))
        print(f"  {a:5d} deg  plate-link " + (f"{pl[0]:7.3f} {pl[1]:.3f}" if pl else "      -      ")
              + f"   link-link {ll[0]:7.3f} {ll[1]:.3f}")

    # Folded, standing on the bed
    folded = [moved(plate, PRINT)] + [moved(p, PRINT @ M) for p, M in zip(LINKS, placements([-180] * N_LINKS))]
    worst = min((clash(a, b)[1], i, j) for i, a in enumerate(folded) for j, b in enumerate(folded) if i < j)
    print(f"folded: closest bodies {worst[1]} & {worst[2]} at {worst[0]:.3f} mm")
    stack = cq.Compound.makeCompound(folded)
    bb = stack.BoundingBox()
    print(f"print size {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, bottom at Z = {bb.zmin:.2f}")
    cq.exporters.export(cq.Workplane("XY").add(stack), str(here / "folding_chain.step"))
    cq.exporters.export(cq.Workplane("XY").add(stack), str(here / "folding_chain.stl"),
                        tolerance=0.01, angularTolerance=0.1)

    # Unfolded, hanging from the carriage plate to the table
    angles, slopes = drape_angles()
    hung = [plate.val()] + [moved(p, M) for p, M in zip(LINKS, placements(angles))]
    print("draped: link slopes below horizontal " + ", ".join(f"{s:.1f}" for s in slopes)
          + "; hinge angles " + ", ".join(f"{a:.1f}" for a in angles))
    hits = [(i, j, v) for i, a in enumerate(hung) for j, b in enumerate(hung) if i < j
            for v in [clash(a, b)[0]] if v > 1e-6]
    print("draped: " + ("no overlaps" if not hits else f"overlaps {hits}")
          + f"; lowest point {cq.Compound.makeCompound(hung).BoundingBox().zmin - TABLE_Z:.2f} mm above the table")
    assy = cq.Assembly().add(cq.Workplane("XY").add(hung[0]), name="plate", color=cq.Color(0.9, 0.47, 0.12))
    for i, s in enumerate(hung[1:]):
        c = cq.Color(0.24, 0.47, 0.78) if i % 2 == 0 else cq.Color(0.3, 0.7, 0.45)
        assy.add(cq.Workplane("XY").add(s), name=f"link{i + 1}", color=c)
    assy.add(box(-20, L + 20, -40, 320, TABLE_Z - 5, TABLE_Z), name="table", color=cq.Color(0.8, 0.72, 0.59))
    assy.save(str(here / "folding_chain_draped.step"))
