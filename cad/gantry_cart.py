"""Push cart for the breadboard gantry, built from 80/20 10-series profile under the 24 x 24 board.

The legs and shelf sit inside the breadboard's 24 x 24 in footprint; only the top rails reach past
it, to carry a laptop table (Z up, inches, origin at the floor under the board's -X/-Y corner; the
board covers X, Y = 0..24 with its top at BOARD_TOP):

  - Top frame: 1010, its centre lines on the board's outermost hole rows/columns (0.5 in in from
    each edge), so the outer faces are flush with the board. The X rails run the full length
    (board + laptop table); the Y rails fit between them. The gantry legs' M4 screws/nuts (outer hole rows) drop into the top
    T-slot of the X rails: slot opening 0.342 in > M4 nut across corners (8.1 mm), 0.323 in deep.
  - Legs: 2020 at the corners, flush with the outside, under the top frame.
  - Casters: 2323 (5 in soft rubber, swivel, top brake locks wheel + swivel, 300 lb) on a 2419
    10-series flange-mount caster base plate under each leg, wheel trailing toward the centre.
  - Laptop table: the two X top rails run on LAPTOP_DEPTH past the -X legs (the end with the leg
    rails' motors), a 1010 end rail joins them, and a 2633 Lite ACM panel on top
    butts against the board.
  - Shelf: 1010 frame between the legs with a 2633 Lite ACM panel, set just low enough under the top
    frame for the tallest item (FUYU driver standing on edge, 75.5 mm) plus SHELF_CLEAR.

Vendor CAD (git-ignored, from 80/20 PARTcommunity, unzipped in ~/Downloads):
  8020_101097/1010-97.stp, 8020_2020145/2020-145.stp   profiles, cut to length here
  8020_2323/2323.stp, 8020_2419/2419.stp               caster and base plate
Run:  python gantry_cart.py [dir holding those folders]
  -> gantry_cart.step   (cart + gantry on breadboard; git-ignored)
"""
import math
import sys
from pathlib import Path
import cadquery as cq
import cart_hardware as hw

IN = 25.4
VENDOR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Downloads"

BOARD = 24.0
BOARD_TOP = 30.0
BOARD_T = 0.5
TOP_Z = BOARD_TOP - BOARD_T          # top of the 1010 top frame = board underside
EDGE_ROW = 0.5                       # outermost hole row/column from the board edge
P10, P20 = 1.0, 2.0                  # 1010 and 2020 section sizes

# 2323 caster STEP frame (in): Y up, flange top Y = 1.893, swivel axis X = -1.196, Z = -0.029,
# wheel on the floor at Y = -4.325, wheel trailing toward -X
CASTER_TOP_Y, CASTER_FLOOR_Y, CASTER_AXIS_XZ = 1.893, -4.325, (-1.196, -0.029)
CASTER_H = CASTER_TOP_Y - CASTER_FLOOR_Y          # 6.218 load height
PLATE_2419 = (3.75, 2.5, 0.5)                     # STEP frame: X 0..3.75, Y 0..2.5, Z 0..0.5
LEG_BOTTOM = CASTER_H + PLATE_2419[2]
LEG_TOP = TOP_Z - P10
LEG_L = LEG_TOP - LEG_BOTTOM

LAPTOP_DEPTH = 14.0                  # top rails cantilever this far past the legs at -X (leg-rail motor end)

TALLEST = 75.5 / IN                  # driver on edge
SHELF_CLEAR = 0.5
PANEL_T = 0.236                      # 80/20 2633 Lite aluminium composite panel
SHELF_PANEL_TOP = LEG_TOP - TALLEST - SHELF_CLEAR
SHELF_RAIL_TOP = SHELF_PANEL_TOP - PANEL_T


def stock_profile(path):
    """Profile STEP with its length along -Z from 0: centred, length along +Z from 0."""
    s = cq.importers.importStep(str(path)).val()
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def member(stock, length, start, direction):
    """Profile of `length` in starting at `start` (in; section centre at the start face), along x/y/z."""
    m = stock.intersect(cq.Solid.makeBox(200, 200, length * IN, cq.Vector(-100, -100, 0)))
    if direction == "x":
        m = m.rotate(cq.Vector(), cq.Vector(0, 1, 0), 90)
    elif direction == "y":
        m = m.rotate(cq.Vector(), cq.Vector(1, 0, 0), -90)
    return m.translate(cq.Vector(*(c * IN for c in start)))


def box(x0, y0, z0, dx, dy, dz):
    return cq.Solid.makeBox(dx * IN, dy * IN, dz * IN, cq.Vector(x0 * IN, y0 * IN, z0 * IN))


LEG_XY = [(P20 / 2, P20 / 2), (BOARD - P20 / 2, P20 / 2), (P20 / 2, BOARD - P20 / 2), (BOARD - P20 / 2, BOARD - P20 / 2)]


def frame(p1010, p2020):
    parts = []
    for i, (x, y) in enumerate(LEG_XY):
        parts.append((f"leg{i + 1}_2020", member(p2020, LEG_L, (x, y, LEG_BOTTOM), "z")))
    e, zc = EDGE_ROW, TOP_Z - P10 / 2
    for side, y in (("front", e), ("back", BOARD - e)):       # one piece each, cantilevered past the legs
        parts.append((f"top_{side}_1010", member(p1010, BOARD + LAPTOP_DEPTH, (-LAPTOP_DEPTH, y, zc), "x")))
    for side, x in (("left", e), ("right", BOARD - e), ("laptop_end", -LAPTOP_DEPTH + e)):
        parts.append((f"top_{side}_1010", member(p1010, BOARD - 2 * P10, (x, P10, zc), "y")))
    zc = SHELF_RAIL_TOP - P10 / 2
    for side, y in (("front", e), ("back", BOARD - e)):
        parts.append((f"shelf_{side}_1010", member(p1010, BOARD - 2 * P20, (P20, y, zc), "x")))
    for side, x in (("left", e), ("right", BOARD - e)):
        parts.append((f"shelf_{side}_1010", member(p1010, BOARD - 2 * P20, (x, P20, zc), "y")))
    return parts


def shelf_panel():
    p = box(0, 0, SHELF_RAIL_TOP, BOARD, BOARD, PANEL_T)
    for x, y in ((0, 0), (BOARD - P20, 0), (0, BOARD - P20), (BOARD - P20, BOARD - P20)):
        p = p.cut(box(x, y, SHELF_RAIL_TOP - 0.1, P20, P20, PANEL_T + 0.2))
    return p


def laptop_panel():
    """2633 panel on the cantilevered top rails, butted against the breadboard's -X edge."""
    return box(-LAPTOP_DEPTH, 0, TOP_Z, LAPTOP_DEPTH, BOARD, PANEL_T)


def electronics():
    """Drivers stand on edge, 50 mm apart (datasheet); controller and PSU are placeholder boxes."""
    z = SHELF_PANEL_TOP
    drv = (118 / IN, 24.3 / IN, 75.5 / IN)
    out = [(f"driver_{n}", box(3.0, 3.0 + i * (drv[1] + 50 / IN), z, *drv)) for i, n in enumerate(("XL", "XR", "Y"))]
    out.append(("controller_FMC4030_placeholder", box(10.5, 3.0, z, 180 / IN, 110 / IN, 40 / IN)))
    out.append(("psu_24V_350W_placeholder", box(3.0, 13.0, z, 215 / IN, 115 / IN, 50 / IN)))
    return out


def caster_assemblies(caster_solids, plate):
    """[(name, shape, colour)] 2419 plate + 2323 caster + screws under each leg.

    The 2419's four counterbored 1/4-20 holes are on a diamond 0.5 in from its centre, so on a
    2020 end (holes at the centre and at +-0.5, +-0.5) the plate sits 0.5 in inboard of the leg
    centre: three of its holes then land on the leg's centre hole and two corner holes (tap all
    three 1/4-20). Counterbores face down; the caster flange covers them.
    The caster flange is bolted square to the plate (long axes along X); only the fork + wheel
    are turned about the swivel axis to trail toward the cart centre.
    """
    to_z_up = lambda s: s.rotate(cq.Vector(), cq.Vector(1, 0, 0), 90).translate(cq.Vector(
        -CASTER_AXIS_XZ[0] * IN, CASTER_AXIS_XZ[1] * IN, -CASTER_TOP_Y * IN))
    fixed = [to_z_up(s) for s in caster_solids if s.BoundingBox().ymin / IN > 1.0]   # flange + top race
    swivel = [to_z_up(s) for s in caster_solids if s.BoundingBox().ymin / IN <= 1.0]
    # plate: counterbores (STEP top face) turned down, centred on the origin, top face at z = 0.5
    p0 = (plate.rotate(cq.Vector(), cq.Vector(1, 0, 0), 180)
          .translate(cq.Vector(-PLATE_2419[0] / 2 * IN, PLATE_2419[1] / 2 * IN, PLATE_2419[2] * IN)))
    out = []
    for i, (x, y) in enumerate(LEG_XY):
        u = 1 if y < BOARD / 2 else -1                       # inboard along Y
        cx, cy = x, y + 0.5 * u                               # plate centre = swivel axis
        mv = lambda s, z: s.translate(cq.Vector(cx * IN, cy * IN, z * IN))
        ang = math.degrees(math.atan2(BOARD / 2 - cy, BOARD / 2 - cx)) - 180
        flange = cq.Compound.makeCompound([mv(s, CASTER_H) for s in fixed])
        fork = cq.Compound.makeCompound([mv(s.rotate(cq.Vector(), cq.Vector(0, 0, 1), ang), CASTER_H) for s in swivel])
        out += [(f"caster{i + 1}_2323_flange", flange, hw.BLACK), (f"caster{i + 1}_2323_fork_wheel", fork, hw.BLACK),
                (f"caster{i + 1}_plate_2419", mv(p0, CASTER_H), hw.ALU)]
        # 4 x 5/16-18 x 5/8 SHCS up through the caster flange into the plate's tapped holes
        for j, (dx, dy) in enumerate(((1.47, 0.875), (-1.47, 0.875), (1.47, -0.875), (-1.47, -0.875))):
            s = hw.bhcs(0.625, 0.3125, hw.SHCS_516).rotate(cq.Vector(), cq.Vector(1, 0, 0), 180)
            out.append((f"caster{i + 1}_shcs516_{j + 1}",
                        s.translate(cq.Vector((cx + dx) * IN, (cy + dy) * IN, (CASTER_H - 0.125) * IN)), hw.STEEL))
        # 3 x 1/4-20 x 3/4 SHCS up through the plate counterbores into the tapped leg end
        for j, (dx, dy) in enumerate(((0, 0), (0.5, 0.5 * u), (-0.5, 0.5 * u))):
            s = hw.bhcs(0.75, 0.25, hw.SHCS_14).rotate(cq.Vector(), cq.Vector(1, 0, 0), 180)
            out.append((f"leg{i + 1}_shcs14_{j + 1}",
                        s.translate(cq.Vector((x + dx) * IN, (y + dy) * IN, (CASTER_H + hw.SHCS_14[1]) * IN)), hw.STEEL))
    return out


def joints():
    """Every 4132 bracket joint: (name, corner, n1, n2) - see cart_hardware.bracket_joint."""
    j = []
    zt = TOP_Z - P10 / 2                                       # top rail mid-height
    xs = {"left": (P10, +1), "right": (BOARD - P10, -1), "laptop_end": (-LAPTOP_DEPTH + P10, +1)}
    for rail, (x, sx) in xs.items():                           # Y top rails to the front/back X rails
        for side, (y, sy) in (("front", (P10, +1)), ("back", (BOARD - P10, -1))):
            j.append((f"top_{rail}_{side}", (x, y, zt), (0, sy, 0), (sx, 0, 0)))
    e = EDGE_ROW
    for i, (lx, ly) in enumerate(LEG_XY):                      # each leg to the X and Y top rails above it
        sx = 1 if lx < BOARD / 2 else -1
        sy = 1 if ly < BOARD / 2 else -1
        rail_y = e if ly < BOARD / 2 else BOARD - e
        rail_x = e if lx < BOARD / 2 else BOARD - e
        j.append((f"leg{i + 1}_top_x", (lx + sx * P20 / 2, rail_y, LEG_TOP), (0, 0, -1), (sx, 0, 0)))
        j.append((f"leg{i + 1}_top_y", (rail_x, ly + sy * P20 / 2, LEG_TOP), (0, 0, -1), (0, sy, 0)))
        zs = SHELF_RAIL_TOP - P10                              # under the shelf rails, against the leg
        j.append((f"leg{i + 1}_shelf_x", (lx + sx * P20 / 2, rail_y, zs), (0, 0, -1), (sx, 0, 0)))
        j.append((f"leg{i + 1}_shelf_y", (rail_x, ly + sy * P20 / 2, zs), (0, 0, -1), (0, sy, 0)))
    return j


PANEL_SCREWS = {   # (x, y) on the rail centre lines; slot direction of the rail underneath
    "shelf": [((3.0, 0.5), "x"), ((21.0, 0.5), "x"), ((3.0, 23.5), "x"), ((21.0, 23.5), "x")],
    "laptop": [((-12.5, 0.5), "x"), ((-1.5, 0.5), "x"), ((-12.5, 23.5), "x"), ((-1.5, 23.5), "x")],
}


def panel_hardware():
    out = []
    for panel, z_top in (("shelf", SHELF_PANEL_TOP), ("laptop", TOP_Z + PANEL_T)):
        for k, ((x, y), d) in enumerate(PANEL_SCREWS[panel]):
            out += hw.panel_screw(f"{panel}_panel_{k + 1}", x, y, z_top, d)
    for side, y in (("front", EDGE_ROW), ("back", BOARD - EDGE_ROW)):
        out += hw.end_cap(f"top_{side}_laptop_end", -LAPTOP_DEPTH, y, TOP_Z - P10 / 2, -1)
        out += hw.end_cap(f"top_{side}_far_end", BOARD, y, TOP_Z - P10 / 2, +1)
    return out


if __name__ == "__main__":
    here = Path(__file__).parent
    p1010 = stock_profile(VENDOR / "8020_101097" / "1010-97.stp")
    p2020 = stock_profile(VENDOR / "8020_2020145" / "2020-145.stp")
    caster = cq.importers.importStep(str(VENDOR / "8020_2323" / "2323.stp")).solids().vals()
    hw.PANEL_T_DEFAULT = PANEL_T
    plate = cq.importers.importStep(str(VENDOR / "8020_2419" / "2419.stp")).val()

    cart = cq.Assembly(name="gantry_cart")
    alu = cq.Color(0.78, 0.79, 0.81)
    for name, solid in frame(p1010, p2020):
        cart.add(solid, name=name, color=alu)
    shelf, laptop = shelf_panel(), laptop_panel()
    for (x, y), _ in PANEL_SCREWS["shelf"]:
        shelf = shelf.cut(hw.countersink(x, y, SHELF_PANEL_TOP))
    for (x, y), _ in PANEL_SCREWS["laptop"]:
        laptop = laptop.cut(hw.countersink(x, y, TOP_Z + PANEL_T))
    cart.add(shelf, name="shelf_panel", color=cq.Color(0.15, 0.15, 0.16))
    cart.add(laptop, name="laptop_panel", color=cq.Color(0.15, 0.15, 0.16))
    for name, solid in electronics():
        cart.add(solid, name=name, color=cq.Color(0.1, 0.1, 0.1) if name.startswith("driver") else cq.Color(0.55, 0.6, 0.65))
    hardware = caster_assemblies(caster, plate) + panel_hardware()
    for name, corner, n1, n2 in joints():
        hardware += hw.bracket_joint(name, corner, n1, n2)
    for name, shape, col in hardware:
        cart.add(shape, name=name, color=col)

    gantry = cq.importers.importStep(str(here / "gantry_on_plate.step"))
    cart.add(gantry, name="gantry_on_breadboard", loc=cq.Location(cq.Vector(0, 0, TOP_Z * IN)))

    out = here / "gantry_cart.step"
    cart.save(str(out))
    print(f"{out.name}: footprint {BOARD:g} x {BOARD:g} in, board top {BOARD_TOP:g} in; "
          f"2020 legs {LEG_L:.3f} in ({LEG_BOTTOM:.3f}..{LEG_TOP:.3f}); "
          f"shelf panel top {SHELF_PANEL_TOP:.3f} in, {LEG_TOP - SHELF_PANEL_TOP:.3f} in clear under the top frame")
    print(f"cut list: 1010 2 @ {BOARD + LAPTOP_DEPTH:g}, 3 @ {BOARD - 2 * P10:g} (top + laptop end); "
          f"4 @ {BOARD - 2 * P20:g} (shelf); "
          f"2020 4 @ {LEG_L:.3f}")
    print(f"hardware: {len(joints())} x 4132 + {2 * len(joints())} x 3393, "
          f"{sum(len(v) for v in PANEL_SCREWS.values())} x 3321 panel screws, 16 x 5/16 SHCS, 12 x 1/4 SHCS, 4 x 2015")
