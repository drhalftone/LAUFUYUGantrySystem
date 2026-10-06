"""Breadboard gantry stand in two parts, built from 80/20 10-series profile under the 24 x 24 board:
a bench box (top frame + electronics shelf) that sits flat on a workbench, and a wheeled cart with
the laptop table that the box lifts onto.

Z up, inches, origin at the floor under the board's -X/-Y corner; the board covers X, Y = 0..24 with
its top at BOARD_TOP when the box is on the cart. The box stays inside the board's footprint; only
the cart's laptop table reaches past it.

Bench box
  - Top frame: 1010, its centre lines on the board's outermost hole rows/columns (0.5 in in from
    each edge), so the outer faces are flush with the board. The X rails run the board's length;
    the Y rails fit between them. The gantry legs' M4 screws/nuts (outer hole rows) drop into the top
    T-slot of the X rails: slot opening 0.342 in > M4 nut across corners (8.1 mm), 0.323 in deep.
  - Posts: short 2020 at the corners, flush with the outside, under the top frame.
  - Shelf: 1010 frame between the posts with a 2633 Lite ACM panel, set just low enough under the top
    frame for the tallest item (FUYU driver standing on edge, 75.5 mm) plus SHELF_CLEAR. The shelf
    rails' and posts' bottoms are flush at BOX_BOTTOM, so the box stands on them; the shelf rails
    join the posts with 4132s on the rails' inner faces, so nothing sticks out below the box.
  - Camera mast on the +X side, bolted to the top and shelf right rails (see camera_mast).

Wheeled cart
  - Legs: 2020 at the corners, directly under the box posts.
  - Top frame: 1010 on the legs, top at CART_TOP = BOX_BOTTOM; the box sits on it. The X rails run on
    LAPTOP_DEPTH past the -X legs (the end with the leg rails' motors), a 1010 end rail joins them,
    and a 2633 Lite ACM laptop panel on top butts against the box.
  - Lower ring: 1010 between the legs, LOWER_RAIL_UP above the leg bottoms, against racking; 4132s
    on the rails' inner faces.
  - Locators: a 2 x 2 x 1/8 aluminium angle on the outside of each leg top, standing LOC_UP above
    CART_TOP, so the box drops into the four corners and can't slide off. A LOC_GAP shim behind each
    flange leaves the box that much clearance. At the -X corners the X rails run on, so the angle's
    X-face flange there only rises from CART_TOP and the laptop panel stops short of it.
  - Casters: 2323 (5 in soft rubber, swivel, top brake locks wheel + swivel, 300 lb) on a 2419
    10-series flange-mount caster base plate under each leg, wheel trailing toward the centre.

Vendor CAD (git-ignored, from 80/20 PARTcommunity, unzipped in ~/Downloads):
  8020_101097/1010-97.stp, 8020_2020145/2020-145.stp   profiles, cut to length here
  8020_2323/2323.stp, 8020_2419/2419.stp               caster and base plate
Run:  python gantry_cart.py [dir holding those folders]
  -> gantry_cart.step    (box + gantry on breadboard, on the cart; git-ignored)
  -> wheeled_cart.step   (the cart alone; git-ignored)
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
TOP_Z = BOARD_TOP - BOARD_T          # top of the box's 1010 top frame = board underside
EDGE_ROW = 0.5                       # outermost hole row/column from the board edge
P10, P20 = 1.0, 2.0                  # 1010 and 2020 section sizes

# 2323 caster STEP frame (in): Y up, flange top Y = 1.893, swivel axis X = -1.196, Z = -0.029,
# wheel on the floor at Y = -4.325, wheel trailing toward -X
CASTER_TOP_Y, CASTER_FLOOR_Y, CASTER_AXIS_XZ = 1.893, -4.325, (-1.196, -0.029)
CASTER_H = CASTER_TOP_Y - CASTER_FLOOR_Y          # 6.218 load height
PLATE_2419 = (3.75, 2.5, 0.5)                     # STEP frame: X 0..3.75, Y 0..2.5, Z 0..0.5
LEG_BOTTOM = CASTER_H + PLATE_2419[2]
POST_TOP = TOP_Z - P10               # top of the box posts

LAPTOP_DEPTH = 14.0                  # cart top rails cantilever this far past the legs at -X (leg-rail motor end)

# Camera mast at +X (opposite the laptop table): 1010 upright on the outside of the box, held to the
# top and shelf right rails; a 1010 boom slides on it (4132s in the T-slots) and carries a USB camera
# looking straight down at the board centre.
CAMERA_HEIGHT = 36.0                 # lens above the board top (adjustable, ~7 in .. 36 in; sets the mast length)
CAMERA_XY = (BOARD / 2, BOARD / 2)   # board centre, on the boom's bottom T-slot
MAST_Y = BOARD / 2
BALLHEAD_H, CAMERA_H = 1.5, 1.0      # ball head (boom underside -> camera top), camera body (lens at bottom)
BOOM_BOTTOM = BOARD_TOP + CAMERA_HEIGHT + CAMERA_H + BALLHEAD_H
BOOM_LEN = BOARD - (CAMERA_XY[0] - 1.0)                       # mast face (x = BOARD) to 1 in past the camera
MAST_TOP = BOOM_BOTTOM + P10

TALLEST = 75.5 / IN                  # driver on edge
SHELF_CLEAR = 0.5
PANEL_T = 0.236                      # 80/20 2633 Lite aluminium composite panel
SHELF_PANEL_TOP = POST_TOP - TALLEST - SHELF_CLEAR
SHELF_RAIL_TOP = SHELF_PANEL_TOP - PANEL_T
BOX_BOTTOM = SHELF_RAIL_TOP - P10    # underside of the box (shelf rails + posts)
POST_L = POST_TOP - BOX_BOTTOM
MAST_BOTTOM = BOX_BOTTOM
MAST_L = MAST_TOP - MAST_BOTTOM

CART_TOP = BOX_BOTTOM                # top of the cart's top frame
LEG_TOP = CART_TOP - P10             # cart legs, under the top frame
LEG_L = LEG_TOP - LEG_BOTTOM
LOWER_RAIL_UP = 4.0                  # cart lower ring: rail tops this far above the leg bottoms
LOWER_RAIL_TOP = LEG_BOTTOM + LOWER_RAIL_UP
LOC_UP, LOC_DOWN, LOC_W, LOC_T = 1.0, 2.5, 2.0, 0.125     # corner angle: above / below CART_TOP, flange, thickness
LOC_GAP = 1 / 32                     # shim behind each angle flange = box clearance


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


def top_frame(p1010, prefix, z_top, x0, y_rails):
    """X rails from x0 to BOARD on the edge rows, Y rails (name, x) between them, tops at z_top."""
    e, zc = EDGE_ROW, z_top - P10 / 2
    parts = [(f"{prefix}_{side}_1010", member(p1010, BOARD - x0, (x0, y, zc), "x"))
             for side, y in (("front", e), ("back", BOARD - e))]
    parts += [(f"{prefix}_{side}_1010", member(p1010, BOARD - 2 * P10, (x, P10, zc), "y")) for side, x in y_rails]
    return parts


def ring(p1010, prefix, z_top):
    """1010 rails between the 2020 corners on the edge rows, tops at z_top."""
    e, zc = EDGE_ROW, z_top - P10 / 2
    parts = [(f"{prefix}_{side}_1010", member(p1010, BOARD - 2 * P20, (P20, y, zc), "x"))
             for side, y in (("front", e), ("back", BOARD - e))]
    parts += [(f"{prefix}_{side}_1010", member(p1010, BOARD - 2 * P20, (x, P20, zc), "y"))
              for side, x in (("left", e), ("right", BOARD - e))]
    return parts


BOX_Y_RAILS = (("left", EDGE_ROW), ("right", BOARD - EDGE_ROW))
CART_Y_RAILS = BOX_Y_RAILS + (("laptop_end", -LAPTOP_DEPTH + EDGE_ROW),)


def box_frame(p1010, p2020):
    parts = [(f"post{i + 1}_2020", member(p2020, POST_L, (x, y, BOX_BOTTOM), "z")) for i, (x, y) in enumerate(LEG_XY)]
    return parts + top_frame(p1010, "top", TOP_Z, 0, BOX_Y_RAILS) + ring(p1010, "shelf", SHELF_RAIL_TOP)


def cart_frame(p1010, p2020):
    parts = [(f"cart_leg{i + 1}_2020", member(p2020, LEG_L, (x, y, LEG_BOTTOM), "z")) for i, (x, y) in enumerate(LEG_XY)]
    return (parts + top_frame(p1010, "cart_top", CART_TOP, -LAPTOP_DEPTH, CART_Y_RAILS)
            + ring(p1010, "cart_lower", LOWER_RAIL_TOP))


def shelf_panel():
    p = box(0, 0, SHELF_RAIL_TOP, BOARD, BOARD, PANEL_T)
    for x, y in ((0, 0), (BOARD - P20, 0), (0, BOARD - P20), (BOARD - P20, BOARD - P20)):
        p = p.cut(box(x, y, SHELF_RAIL_TOP - 0.1, P20, P20, PANEL_T + 0.2))
    return p


def laptop_panel():
    """2633 panel on the cart's cantilevered top rails, up to the -X locator angles."""
    return box(-LAPTOP_DEPTH, 0, CART_TOP, LAPTOP_DEPTH - LOC_GAP - LOC_T, BOARD, PANEL_T)


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
    2020 end (0.205 in corner holes at +-0.5, +-0.5; the centre is a 1.78 in hollow) the plate sits
    0.5 in inboard of the leg centre so two of its holes land on two corner holes (tap both 1/4-20).
    That is only two screws per caster - see the BOM note. Counterbores face down under the flange.
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
        # 2 x 1/4-20 x 3/4 SHCS up through the plate counterbores into the leg's tapped corner holes
        for j, (dx, dy) in enumerate(((0.5, 0.5 * u), (-0.5, 0.5 * u))):
            s = hw.bhcs(0.75, 0.25, hw.SHCS_14).rotate(cq.Vector(), cq.Vector(1, 0, 0), 180)
            out.append((f"leg{i + 1}_shcs14_{j + 1}",
                        s.translate(cq.Vector((x + dx) * IN, (y + dy) * IN, (CASTER_H + hw.SHCS_14[1]) * IN)), hw.STEEL))
    return out


def locators():
    """[(name, shape, colour)] corner angle on each cart leg top + its 3393 bolts (1/4-20 x 3/8 + T-nut).

    Each flange bolts to the top-frame rail behind it (rail slot, 0.5 in below CART_TOP) and to the
    leg (vertical slot, 0.75 in below the leg top). At the -X corners the X-face flange sits above
    the cantilevered X rail, so it has no bolt and the Y-face flange spans both sides of the corner.
    """
    out = []
    g0, g1 = LOC_GAP, LOC_GAP + LOC_T
    z_rail, z_leg = CART_TOP - P10 / 2, LEG_TOP - 0.75
    for i, (cx, cy) in enumerate(((0, 0), (BOARD, 0), (0, BOARD), (BOARD, BOARD))):
        ox, oy = (-1 if cx == 0 else 1), (-1 if cy == 0 else 1)      # outward
        laptop_end = cx == 0
        z_lo, z_hi = CART_TOP - LOC_DOWN, CART_TOP + LOC_UP
        # flange on the Y face (plane y = cy)
        xs = sorted((cx - LOC_W, cx + LOC_W)) if laptop_end else sorted((cx + ox * g1, cx - ox * LOC_W))
        ys = sorted((cy + oy * g0, cy + oy * g1))
        a = box(xs[0], ys[0], z_lo, xs[1] - xs[0], ys[1] - ys[0], z_hi - z_lo)
        # flange on the X face (plane x = cx); only above the rail at the laptop end
        zx = CART_TOP if laptop_end else z_lo
        xs = sorted((cx + ox * g0, cx + ox * g1))
        ys = sorted((cy + oy * g1, cy - oy * LOC_W))
        a = a.fuse(box(xs[0], ys[0], zx, xs[1] - xs[0], ys[1] - ys[0], z_hi - zx)).clean()
        # (face, outward normal, point on the member face, slot direction)
        bolts = [("y_rail", (0, oy, 0), (cx - ox * 0.5, cy, z_rail), (1, 0, 0)),
                 ("y_leg", (0, oy, 0), (cx - ox * 0.5, cy, z_leg), (0, 0, 1))]
        if not laptop_end:
            bolts += [("x_rail", (ox, 0, 0), (cx, cy - oy * 1.5, z_rail), (0, 1, 0)),
                      ("x_leg", (ox, 0, 0), (cx, cy - oy * 0.5, z_leg), (0, 0, 1))]
        hw_parts = []
        for face, n, at, slot in bolts:
            n, at, slot = cq.Vector(*n), cq.Vector(*at), cq.Vector(*slot)
            a = a.cut(cq.Solid.makeCylinder(0.14 * IN, 1 * IN, (at - n * 0.5) * IN, n))
            screw = hw.bhcs(hw.SCREW_L).moved(cq.Location(cq.Plane((at + n * g1) * IN, slot, n)))
            nut = hw.tnut().moved(cq.Location(cq.Plane(at * IN, slot, n)))
            hw_parts += [(f"locator{i + 1}_3393_screw_{face}", screw, hw.STEEL),
                         (f"locator{i + 1}_3393_tnut_{face}", nut, hw.ZINC)]
        out += [(f"locator{i + 1}_angle", a, hw.ALU)] + hw_parts
    return out


def camera_mast(p1010):
    """[(name, shape, colour)] mast, boom, ball head and a placeholder USB webcam."""
    out = [("camera_mast_1010", member(p1010, MAST_L, (BOARD + P10 / 2, MAST_Y, MAST_BOTTOM), "z"), hw.ALU),
           ("camera_boom_1010", member(p1010, BOOM_LEN, (BOARD - BOOM_LEN, MAST_Y, BOOM_BOTTOM + P10 / 2), "x"), hw.ALU)]
    cx, cy = CAMERA_XY
    stem = cq.Solid.makeCylinder(0.25 * IN, 0.75 * IN, cq.Vector(cx * IN, cy * IN, (BOOM_BOTTOM - 0.75) * IN))
    ball = cq.Solid.makeSphere(0.5 * IN, cq.Vector(cx * IN, cy * IN, (BOOM_BOTTOM - 1.0) * IN))
    out.append(("camera_ballhead", stem.fuse(ball), hw.BLACK))
    # placeholder webcam ~ 3.7 x 1.1 x 1.0 in (C920-class), lens on the bottom face
    z_cam = BOOM_BOTTOM - BALLHEAD_H - CAMERA_H
    body = box(cx - 0.55, cy - 1.85, z_cam, 1.1, 3.7, CAMERA_H)
    lens = cq.Solid.makeCylinder(0.35 * IN, 0.08 * IN, cq.Vector(cx * IN, cy * IN, (z_cam - 0.08) * IN))
    out.append(("usb_camera_placeholder", body.fuse(lens), cq.Color(0.1, 0.1, 0.1)))
    # 1/4-20 stud from the ball head up into an economy T-nut in the boom's bottom slot
    stud = cq.Solid.makeCylinder(0.125 * IN, 0.6 * IN, cq.Vector(cx * IN, cy * IN, (BOOM_BOTTOM - 0.35) * IN))
    nut = hw.tnut().rotate(cq.Vector(), cq.Vector(1, 0, 0), 180).translate(cq.Vector(cx * IN, cy * IN, BOOM_BOTTOM * IN))
    out += [("camera_mount_stud", stud, hw.STEEL), ("camera_mount_tnut", nut, hw.ZINC)]
    return out


def top_frame_joints(prefix, z_top, y_rails, leg_top):
    """Y rails to the X rails (side faces, flush top and bottom) and each 2020 to both rails above it."""
    j, zt, e = [], z_top - P10 / 2, EDGE_ROW
    for rail, x in y_rails:                                    # bracket on the Y rail's inboard face
        sx = -1 if x > BOARD / 2 else +1
        for side, (y, sy) in (("front", (P10, +1)), ("back", (BOARD - P10, -1))):
            j.append((f"{prefix}_{rail}_{side}", (x + sx * P10 / 2, y, zt), (0, sy, 0), (sx, 0, 0)))
    for i, (lx, ly) in enumerate(LEG_XY):                      # each 2020 to the X and Y rails above it
        sx = 1 if lx < BOARD / 2 else -1
        sy = 1 if ly < BOARD / 2 else -1
        rail_y = e if ly < BOARD / 2 else BOARD - e
        rail_x = e if lx < BOARD / 2 else BOARD - e
        j.append((f"{prefix}_leg{i + 1}_x", (lx + sx * P20 / 2, rail_y, leg_top), (0, 0, -1), (sx, 0, 0)))
        j.append((f"{prefix}_leg{i + 1}_y", (rail_x, ly + sy * P20 / 2, leg_top), (0, 0, -1), (0, sy, 0)))
    return j


def ring_joints(prefix, z_top):
    """4132s on the ring rails' inner faces against the 2020 corners, flush with the rail top and bottom."""
    j, zc = [], z_top - P10 / 2
    for i, (lx, ly) in enumerate(LEG_XY):
        sx = 1 if lx < BOARD / 2 else -1
        sy = 1 if ly < BOARD / 2 else -1
        rail_y = EDGE_ROW if ly < BOARD / 2 else BOARD - EDGE_ROW
        rail_x = EDGE_ROW if lx < BOARD / 2 else BOARD - EDGE_ROW
        j.append((f"{prefix}{i + 1}_x", (lx + sx * P20 / 2, rail_y + sy * P10 / 2, zc), (0, sy, 0), (sx, 0, 0), "width"))
        j.append((f"{prefix}{i + 1}_y", (rail_x + sx * P10 / 2, ly + sy * P20 / 2, zc), (sx, 0, 0), (0, sy, 0), "width"))
    return j


def box_joints():
    """Every 4132 bracket joint in the box: (name, corner, n1, n2[, slot_b]) - see cart_hardware.bracket_joint."""
    j = []
    # camera mast to the +X faces of the top and shelf right rails, a bracket each side of the mast
    for rail, zc in (("top", TOP_Z - P10 / 2), ("shelf", SHELF_RAIL_TOP - P10 / 2)):
        for side, sy in (("a", -1), ("b", +1)):
            j.append((f"mast_{rail}_{side}", (BOARD, MAST_Y + sy * P10 / 2, zc), (1, 0, 0), (0, sy, 0), "width"))
    # boom to the mast's -X face: one bracket under the boom, one on top (loosen both to slide the boom)
    j.append(("boom_under", (BOARD, MAST_Y, BOOM_BOTTOM), (0, 0, -1), (-1, 0, 0)))
    j.append(("boom_over", (BOARD, MAST_Y, BOOM_BOTTOM + P10), (0, 0, 1), (-1, 0, 0)))
    return j + top_frame_joints("top", TOP_Z, BOX_Y_RAILS, POST_TOP) + ring_joints("shelf_post", SHELF_RAIL_TOP)


def cart_joints():
    return top_frame_joints("cart_top", CART_TOP, CART_Y_RAILS, LEG_TOP) + ring_joints("cart_lower_leg", LOWER_RAIL_TOP)


def joints():
    return box_joints() + cart_joints()


PANEL_SCREWS = {   # (x, y) on the rail centre lines; slot direction of the rail underneath
    "shelf": [((3.0, 0.5), "x"), ((21.0, 0.5), "x"), ((3.0, 23.5), "x"), ((21.0, 23.5), "x")],
    "laptop": [((-12.5, 0.5), "x"), ((-1.5, 0.5), "x"), ((-12.5, 23.5), "x"), ((-1.5, 23.5), "x")],
}
END_CAPS = {       # exposed 1010 X-rail ends: (x, direction)
    "box": [(0, -1), (BOARD, +1)],       # top frame, both ends
    "cart": [(-LAPTOP_DEPTH, -1)],       # laptop end; the +X ends are under the locator angles
}


def box_panel_hardware():
    out = []
    for k, ((x, y), d) in enumerate(PANEL_SCREWS["shelf"]):
        out += hw.panel_screw(f"shelf_panel_{k + 1}", x, y, SHELF_PANEL_TOP, d)
    for side, y in (("front", EDGE_ROW), ("back", BOARD - EDGE_ROW)):
        for x, d in END_CAPS["box"]:
            out += hw.end_cap(f"top_{side}_{'far' if d > 0 else 'near'}_end", x, y, TOP_Z - P10 / 2, d)
    return out


def cart_panel_hardware():
    out = []
    for k, ((x, y), d) in enumerate(PANEL_SCREWS["laptop"]):
        out += hw.panel_screw(f"laptop_panel_{k + 1}", x, y, CART_TOP + PANEL_T, d)
    for side, y in (("front", EDGE_ROW), ("back", BOARD - EDGE_ROW)):
        for x, d in END_CAPS["cart"]:
            out += hw.end_cap(f"cart_top_{side}_laptop_end", x, y, CART_TOP - P10 / 2, d)
    return out


def add_all(asm, solids, color):
    for name, solid in solids:
        asm.add(solid, name=name, color=color)


def add_hardware(asm, hardware, joint_list):
    for name, corner, n1, n2, *slot_b in joint_list:
        hardware += hw.bracket_joint(name, corner, n1, n2, *slot_b)
    for name, shape, col in hardware:
        asm.add(shape, name=name, color=col)


if __name__ == "__main__":
    here = Path(__file__).parent
    p1010 = stock_profile(VENDOR / "8020_101097" / "1010-97.stp")
    p2020 = stock_profile(VENDOR / "8020_2020145" / "2020-145.stp")
    caster = cq.importers.importStep(str(VENDOR / "8020_2323" / "2323.stp")).solids().vals()
    hw.PANEL_T_DEFAULT = PANEL_T
    plate = cq.importers.importStep(str(VENDOR / "8020_2419" / "2419.stp")).val()
    alu, panel_col = cq.Color(0.78, 0.79, 0.81), cq.Color(0.15, 0.15, 0.16)

    cart = cq.Assembly(name="wheeled_cart")
    add_all(cart, cart_frame(p1010, p2020), alu)
    laptop = laptop_panel()
    for (x, y), _ in PANEL_SCREWS["laptop"]:
        laptop = laptop.cut(hw.countersink(x, y, CART_TOP + PANEL_T))
    cart.add(laptop, name="laptop_panel", color=panel_col)
    add_hardware(cart, caster_assemblies(caster, plate) + locators() + cart_panel_hardware(), cart_joints())

    bench = cq.Assembly(name="bench_box")
    add_all(bench, box_frame(p1010, p2020), alu)
    shelf = shelf_panel()
    for (x, y), _ in PANEL_SCREWS["shelf"]:
        shelf = shelf.cut(hw.countersink(x, y, SHELF_PANEL_TOP))
    bench.add(shelf, name="shelf_panel", color=panel_col)
    for name, solid in electronics():
        bench.add(solid, name=name, color=cq.Color(0.1, 0.1, 0.1) if name.startswith("driver") else cq.Color(0.55, 0.6, 0.65))
    add_hardware(bench, box_panel_hardware() + camera_mast(p1010), box_joints())
    gantry = cq.importers.importStep(str(here / "gantry_on_plate.step"))
    bench.add(gantry, name="gantry_on_breadboard", loc=cq.Location(cq.Vector(0, 0, TOP_Z * IN)))

    cart.save(str(here / "wheeled_cart.step"))
    stand = cq.Assembly(name="gantry_cart")
    stand.add(cart, name="wheeled_cart")
    stand.add(bench, name="bench_box")
    out = here / "gantry_cart.step"
    stand.save(str(out))
    print(f"{out.name}: footprint {BOARD:g} x {BOARD:g} in (+{LAPTOP_DEPTH:g} in laptop table), "
          f"board top {BOARD_TOP:g} in on the cart")
    print(f"bench box: {BOARD_TOP - BOX_BOTTOM:.3f} in base to board top; shelf panel top {SHELF_PANEL_TOP - BOX_BOTTOM:.3f} in "
          f"above the base, {POST_TOP - SHELF_PANEL_TOP:.3f} in clear under the top frame")
    print(f"cart: top frame + laptop panel at {CART_TOP:.3f} in (panel top {CART_TOP + PANEL_T:.3f}), "
          f"locators {LOC_UP:g} in proud, lower ring top {LOWER_RAIL_TOP:.3f} in")
    print(f"cut list: box 1010 2 @ {BOARD:g}, 2 @ {BOARD - 2 * P10:g} (top), 4 @ {BOARD - 2 * P20:g} (shelf); "
          f"2020 4 @ {POST_L:.3f} (posts) | cart 1010 2 @ {BOARD + LAPTOP_DEPTH:g}, 3 @ {BOARD - 2 * P10:g} (top + laptop end), "
          f"4 @ {BOARD - 2 * P20:g} (lower ring); 2020 4 @ {LEG_L:.3f} (legs)")
    print(f"camera mast 1010 {MAST_L:.2f} in ({MAST_BOTTOM:.2f}..{MAST_TOP:.2f}), boom 1010 {BOOM_LEN:g} in at "
          f"{BOOM_BOTTOM:.2f} in; lens {CAMERA_HEIGHT:g} in above the board")
    n_caps = 2 * sum(len(v) for v in END_CAPS.values())
    print(f"hardware: {len(joints())} x 4132 (box {len(box_joints())}, cart {len(cart_joints())}) + "
          f"{2 * len(joints()) + 12} x 3393 (12 on the locators), "
          f"{sum(len(v) for v in PANEL_SCREWS.values())} x 3321 panel screws, 16 x 5/16 SHCS, 8 x 1/4 SHCS, "
          f"{n_caps} x 2015, 4 corner angles {LOC_W:g} x {LOC_W:g} x {LOC_T:g} x {LOC_DOWN + LOC_UP:g} in")
