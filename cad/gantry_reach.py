"""Where the X-Rite i1 aperture can reach on the breadboard: the gantry with its motion limits drawn on the board.

Reads gantry_on_plate.step (run gantry_on_plate.py first): Z-up, breadboard X/Y 0..609.6, top face
Z = 12.7, legs along X, beam along Y, every carriage at mid-stroke. The i1 plate is found in it by its
bounding box and the aperture by the 9.8 mm hole in its mesh, so the numbers follow the chain model.

Envelopes, as thin slabs on the board top (aperture centre positions):
  red     full stroke: legs +/- LEG_STROKE/2, beam +/- BEAM_STROKE/2 about mid-stroke
  yellow  red, clipped where the 64 mm i1 plate meets the inside faces of the leg rails
  green   yellow, clipped so the whole i1 plate rests on the board (X >= 0)
Ghost i1 plates sit at the four corners of the yellow envelope, with a pin at each aperture.

Run:  python gantry_reach.py
  -> gantry_reach.glb   (git-ignored: embeds the TraceParts FSK40 model)
"""
import math
from pathlib import Path
import cadquery as cq
from fsk40_resize import LEG_STROKE, BEAM_STROKE

BOARD, BOARD_TOP = 609.6, 12.7
APERTURE_D = 9.8
LEG_INNER = (42.7, 566.9)     # inside faces of the leg rail bases (Y), from gantry_on_plate.step


def find_plate(shape):
    """The i1 plate: the 64 mm wide solid lying on the board top."""
    for s in shape.solids().vals():
        b = s.BoundingBox()
        if abs(b.zmin - BOARD_TOP) < 0.2 and abs(min(b.xlen, b.ylen) - 64.0) < 0.5 and max(b.xlen, b.ylen) > 120:
            return s
    raise SystemExit("i1 plate not found in gantry_on_plate.step")


def find_aperture(plate):
    """Centre of the 9.8 mm aperture: the point most mesh vertices sit APERTURE_D / 2 from."""
    b = plate.BoundingBox()
    cy = (b.ymin + b.ymax) / 2
    pts = [v.toTuple() for v in plate.Vertices() if v.Z < BOARD_TOP + 1.5]
    best = max((sum(abs(math.hypot(x - cx, y - cy) - APERTURE_D / 2) < 0.15 for x, y, _ in pts), cx)
               for cx in [b.xmin + 0.1 * i for i in range(int(b.xlen * 10))])
    return best[1], cy


def slab(x0, x1, y0, y1, z, t=0.1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, t, centered=False).translate((x0, y0, z)).val()


if __name__ == "__main__":
    here = Path(__file__).parent
    gantry = cq.importers.importStep(str(here / "gantry_on_plate.step"))
    plate = find_plate(gantry)
    pb = plate.BoundingBox()
    ax, ay = find_aperture(plate)
    half_w = pb.ylen / 2
    tail = ax - pb.xmin           # plate overhang past the aperture toward -X

    full = (ax - LEG_STROKE / 2, ax + LEG_STROKE / 2, ay - BEAM_STROKE / 2, ay + BEAM_STROKE / 2)
    clear = (full[0], full[1], max(full[2], LEG_INNER[0] + half_w), min(full[3], LEG_INNER[1] - half_w))
    on_board = (max(clear[0], tail), clear[1], clear[2], clear[3])

    assy = cq.Assembly(name="gantry_reach")
    assy.add(gantry, name="gantry_on_breadboard", color=cq.Color(0.6, 0.62, 0.65))
    for (name, env, col), z in zip((("reach_full_stroke", full, cq.Color(0.9, 0.2, 0.2, 0.5)),
                                    ("reach_clear_of_legs", clear, cq.Color(0.95, 0.8, 0.1, 0.6)),
                                    ("reach_plate_on_board", on_board, cq.Color(0.2, 0.75, 0.3, 0.7))),
                                   (BOARD_TOP + 0.05, BOARD_TOP + 0.15, BOARD_TOP + 0.25)):
        assy.add(slab(env[0], env[1], env[2], env[3], z), name=name, color=col)

    pin = cq.Workplane("XY").circle(APERTURE_D / 2).extrude(40).translate((0, 0, BOARD_TOP)).val()
    assy.add(pin.moved(cq.Location(cq.Vector(ax, ay, 0))), name="aperture_mid_stroke", color=cq.Color(0.1, 0.3, 0.9))
    for i, (x, y) in enumerate((x, y) for x in clear[:2] for y in clear[2:]):
        d = cq.Location(cq.Vector(x - ax, y - ay, 0))
        assy.add(plate.moved(d), name=f"i1_plate_limit_{i + 1}", color=cq.Color(0.85, 0.85, 0.2, 0.45))
        assy.add(pin.moved(cq.Location(cq.Vector(x, y, 0))), name=f"aperture_limit_{i + 1}", color=cq.Color(0.9, 0.1, 0.1))

    out = here / "gantry_reach.glb"
    assy.save(str(out))
    w = lambda e: f"X {e[0]:.1f}..{e[1]:.1f} ({e[1] - e[0]:.1f}) x Y {e[2]:.1f}..{e[3]:.1f} ({e[3] - e[2]:.1f}) mm"
    print(f"aperture at mid-stroke ({ax:.1f}, {ay:.1f}); i1 plate {pb.xlen:.1f} x {pb.ylen:.1f}, "
          f"X {pb.xmin:.1f}..{pb.xmax:.1f} (tail {tail:.1f} mm past the aperture)")
    print(f"  red    full stroke       {w(full)}")
    print(f"  yellow clear of legs     {w(clear)}")
    print(f"  green  plate on board    {w(on_board)}")
    print(f"-> {out.name}")
