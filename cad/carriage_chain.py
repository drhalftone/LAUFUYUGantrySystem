"""Carriage hinge plate + hanging chain links for the FSK40 carriage.

The carriage plate is the fsk40_carriage_plate (65 x 48 x 10, counterbored M4 x 12
on the carriage's 25 x 25 pattern, flush with the carriage) with a square-knuckle
hinge along one long edge, so a chain of links can hang over the side of the
carriage down to the table. In use the chain is a single long link ending in the
X-Rite i1 plate (xrite_i1_plate.py); standard links are only modelled for N_LINKS > 1.

Every hinge is the one from hinged_plates.py: the upper piece owns two end
knuckles (M4 pan head recessed in one, nylock nut captive in the other) and the
lower piece owns the middle knuckle, which turns on the screw. Both have a 4.0 mm
hole (the knuckle_test.py coupon: 4.0 still turns on an M4 as printed). Each piece stops SWING_CLEAR short of the other's knuckles.

Frame (flat, as printed): X along the hinge / carriage travel, X = 0 at the
motor end of the carriage; Y across the carriage, 0 at its centre, hinge side +Y;
Z up, 0 on the carriage top. Links are drawn flat with their top pin at Y = 0.

Run:  python carriage_chain.py
  -> carriage_hinge_plate.stl/.step, chain_link_long.stl/.step  (print flat)
  -> carriage_chain.step            plate + hanging chain + M4 screws and nuts
  -> fsk40_with_chain.glb           same, sitting on the FSK40 (fit check, not committed)
"""
import math
from pathlib import Path
import cadquery as cq

# --- carriage plate (same as fsk40_carriage_plate.py) ---
L = 65.0              # plate / hinge length along travel (= carriage length)
PLATE_W = 48.0        # across the carriage (= carriage width)
T = 10.0              # all plates and knuckles are T thick
HOLE_PITCH = 25.0     # carriage M4 pattern, square
PATTERN_X = 33.5      # pattern centre from the motor end (21 / 19 mm hole-to-end)
CLEAR_D, CBORE_D, CBORE_DEPTH = 4.5, 8.0, 5.0   # M4 x 12 SHCS into the carriage

# --- 1010 camera-mast socket (80/20 10 series, 1.00 in square) ---
# A square cup on the plate's -Y side (away from the hinge), its foot entirely off the carriage: the
# plate's leaf runs on past the carriage's -Y edge to carry it. The cup is tilted MAST_TILT from
# vertical so the 1010 leans back over the carriage toward the hinge side, high enough over the
# carriage screws for an L-key to reach them under it. A solid wedge fills between the tilted cup and
# the plate. The 1010 seats on the cup floor and is clamped by 1/4-20 x 1/2 BHSCS + economy T-nuts
# (80/20 3393) through the cup's outer (-Y, facing away from the carriage) and +X walls into its slots.
PROFILE = 25.4        # 1010 section
SOCKET_FIT = 0.1      # clearance per side
SOCKET_WALL = 5.0
SOCKET_FLOOR = 5.0    # cup floor under the 1010's end
SOCKET_DEPTH = 50.8   # 2 in of the 1010 held in the cup
SOCKET_IN = PROFILE + 2 * SOCKET_FIT
SOCKET_OUT = SOCKET_IN + 2 * SOCKET_WALL
MAST_TILT = 30.0      # degrees from vertical, leaning toward +Y (the hinge side)
FOOT_GAP = 4.0        # cup footprint stops this far outboard of the carriage -Y edge: ~27 mm of L-key room over the near screws
CORNER_RELIEF_D = 2.0  # relief at the pocket corners so the printed corner radius doesn't hold the 1010 off
CLAMP_D = 7.1          # 1/4-20 clearance (0.28 in)
FIN_T = 6.0            # gusset under the leaning cup, centred between the two columns of carriage screws

_s, _c = math.sin(math.radians(MAST_TILT)), math.cos(math.radians(MAST_TILT))
# Cup frame: origin at the centre of the pocket floor, local Z along the 1010 (leaning to +Y), local X = X,
# local +Y pointing down toward the carriage. Its lowest outer corner sits on the plate top, and its
# footprint's +Y edge is FOOT_GAP off the carriage's -Y edge.
SOCKET_X = PATTERN_X                                       # centred on the screw pattern along travel
SOCKET_Y = -PLATE_W / 2 - FOOT_GAP - _c * SOCKET_OUT / 2 + _s * SOCKET_FLOOR
SOCKET_Z = T + _c * SOCKET_FLOOR + _s * SOCKET_OUT / 2
SOCKET_LOC = cq.Location(cq.Vector(SOCKET_X, SOCKET_Y, SOCKET_Z), cq.Vector(1, 0, 0), -MAST_TILT)

# --- hinge (same as hinged_plates.py) ---
END_KNUCKLE = 16.0    # each end knuckle of the upper piece; the lower piece's fills the middle
KNUCKLE_GAP = 2.0     # axial space between neighbouring knuckles (each stops 1 mm short of the split)
SWING_GAP = 1.0       # clearance between a turning knuckle's corners and the other piece
PIN_D = 4.0           # pin hole in the upper piece (clamped)
PIVOT_D = 4.0         # pin hole in the lower piece (turns on the screw; 4.0 won the knuckle test over 4.5)
# Pin: M4 x 60 Phillips pan head (ISO 7045: head 8.0 dia x 3.1) + M4 nylock (7.0 AF x 5.0).
# Head in a counterbore at one end, nut captive at the other; nothing sticks out past the ends.
HEAD_D, HEAD_H = 8.0, 3.1
HEAD_CBORE_D, HEAD_DEPTH = 8.4, 4.0   # head top 0.9 below the end face
NUT_AF, NUT_H = 7.3, 5.0          # pocket across flats (the 7.0 nut fits it well); nut thickness
NUT_DEPTH = 7.0                   # nut pushed to the pocket floor: x = 58..63
PIN_LEN = 60.0        # counterbore floor (x = 4) to x = 64: 1 mm past the nut, 1 mm inside the end face

# --- chain ---
LINK_PITCH = 85.0     # pin-to-pin length of each link
FIRST_LINK_PITCH = 2 * LINK_PITCH   # first link off the carriage: long enough to reach the table at an angle
LINK_CLEAR = 0.2        # each link's middle knuckle stops this short of the knuckles above it (no side play)
LINK_WALL = 10.0        # every link's leaf is a frame: a window cut through the middle leaves this much all round
WINDOW_FILLET = 6.0     # window corner radius
N_LINKS = 1           # links hanging below the carriage plate: just the long one
PIN_ABOVE_TABLE = 155.4   # carriage-plate pin height in the H-gantry (cross-beam carriage top is 150.4 up)

r = T / 2
SWING_CLEAR = r * math.sqrt(2) + SWING_GAP
AXIS_Y = PLATE_W / 2 + SWING_CLEAR   # carriage-plate hinge axis: plate edge stays flush with the carriage
SPANS = [(0, END_KNUCKLE, "upper"), (END_KNUCKLE, L - END_KNUCKLE, "lower"), (L - END_KNUCKLE, L, "upper")]


def block(x0, x1, y0, y1):
    y0, y1 = sorted((y0, y1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, T, centered=False).translate((x0, y0, 0))


def on_axis(axis_y, x0, length, sketch):
    """Extrude a sketch (drawn on YZ, centred on the pin) along X from x0."""
    return sketch(cq.Workplane("YZ").workplane(offset=x0).center(axis_y, r)).extrude(length)


def hinge_half(body, axis_y, side, role, face_clear=KNUCKLE_GAP, pivot_d=PIVOT_D):
    """Add one piece's half of a hinge whose pin runs along X at (axis_y, T/2).

    side: +1 if this piece's leaf lies at +Y of the pin, -1 if at -Y.
    role: "upper" (end knuckles, clamps the pin) or "lower" (middle knuckle, turns on it).
    face_clear: axial space between each face of the middle knuckle and the end knuckles
    ("lower" only; the end knuckles always stop KNUCKLE_GAP / 2 short of their split).
    pivot_d: pin hole in the middle knuckle ("lower" only).
    The caller's leaf must already stop at axis_y + side * SWING_CLEAR.
    """
    g = KNUCKLE_GAP / 2 if role == "upper" else face_clear - KNUCKLE_GAP / 2
    for x0, x1, who in SPANS:
        if who == role:
            g0 = g if x0 > 0 else 0
            g1 = g if x1 < L else 0
            body = body.union(block(x0 + g0, x1 - g1, axis_y + side * SWING_CLEAR, axis_y - side * r))
    hole = PIN_D if role == "upper" else pivot_d
    body = body.cut(on_axis(axis_y, -1, L + 2, lambda w: w.circle(hole / 2)))
    if role == "upper":
        body = body.cut(on_axis(axis_y, -1, HEAD_DEPTH + 1, lambda w: w.circle(HEAD_CBORE_D / 2)))
        body = body.cut(on_axis(axis_y, L - NUT_DEPTH, NUT_DEPTH + 1,
                                lambda w: w.polygon(6, NUT_AF / math.cos(math.pi / 6))))
    return body


def hex_(af):
    return lambda w: w.polygon(6, af / math.cos(math.pi / 6))


def screw_at(axis_y):
    """M4 x PIN_LEN Phillips pan head screw, head seated on the counterbore floor."""
    head = on_axis(axis_y, HEAD_DEPTH - HEAD_H, HEAD_H, lambda w: w.circle(HEAD_D / 2))
    for wh in ((4.4, 1.0), (1.0, 4.4)):                                   # Phillips cross, 1.8 deep
        head = head.cut(on_axis(axis_y, HEAD_DEPTH - HEAD_H - 1, 2.8, lambda w: w.rect(*wh)))
    return head.union(on_axis(axis_y, HEAD_DEPTH, PIN_LEN, lambda w: w.circle(2.0)))


def nut_at(axis_y):
    """M4 nylock nut, seated at the bottom of its pocket."""
    return (on_axis(axis_y, L - NUT_DEPTH, NUT_H, hex_(7.0))
            .cut(on_axis(axis_y, L - NUT_DEPTH - 1, NUT_H + 2, lambda w: w.circle(2.0))))


def _to_plate(shape):
    """Move a cup-frame shape onto the plate."""
    return shape.moved(SOCKET_LOC)


def _cup_corners(z):
    """The cup's four outer corners at cup-frame height z, in the plate frame (fixed winding)."""
    h = SOCKET_OUT / 2
    pts = []
    for x, y in ((-h, -h), (h, -h), (h, h), (-h, h)):
        p = cq.Vertex.makeVertex(x, y, z).moved(SOCKET_LOC)
        pts.append(cq.Vector(p.X, p.Y, p.Z))
    return pts


def mast_socket():
    """Tilted 1010 cup on a wedge down to the plate: walls, floor, corner reliefs and the two clamp holes."""
    h = SOCKET_DEPTH
    cup = cq.Solid.makeBox(SOCKET_OUT, SOCKET_OUT, SOCKET_FLOOR + h,
                           cq.Vector(-SOCKET_OUT / 2, -SOCKET_OUT / 2, -SOCKET_FLOOR))
    pocket = cq.Solid.makeBox(SOCKET_IN, SOCKET_IN, h + 1, cq.Vector(-SOCKET_IN / 2, -SOCKET_IN / 2, 0))
    for sx in (-1, 1):
        for sy in (-1, 1):
            pocket = pocket.fuse(cq.Solid.makeCylinder(CORNER_RELIEF_D / 2, h + 1,
                                                       cq.Vector(sx * SOCKET_IN / 2, sy * SOCKET_IN / 2, 0)))
    cup = cup.cut(pocket)
    # clamp holes on the slot centre line, mid-way up the cup: the outer wall (cup -Y, facing away from
    # the carriage and up) and the +X wall
    centre = cq.Vector(0, 0, h / 2)
    for direction in (cq.Vector(0, -1, 0), cq.Vector(1, 0, 0)):
        cup = cup.cut(cq.Solid.makeCylinder(CLAMP_D / 2, SOCKET_OUT / 2 + 1, centre, direction))
    cup = _to_plate(cup)

    # wedge: the cup's bottom face lofted straight down to the plate top
    bottom = _cup_corners(-SOCKET_FLOOR)
    flat = [cq.Vector(p.x, p.y, T - 0.01) for p in bottom]
    wedge = cq.Solid.makeLoft([cq.Wire.makePolygon(bottom, close=True), cq.Wire.makePolygon(flat, close=True)], True)
    # fin: a vertical gusset (FIN_T thick, centred on X = SOCKET_X, between the screw columns) filling
    # the triangle between the plate top and the cup's underside (its carriage-facing outer face),
    # from where that face meets the plate out to the cup's top end
    def yz(ly, lz):
        p = cq.Vertex.makeVertex(0, ly, lz).moved(SOCKET_LOC)
        return p.Y, p.Z
    (ya, za), (yb, zb) = yz(SOCKET_OUT / 2, -SOCKET_FLOOR), yz(SOCKET_OUT / 2, SOCKET_DEPTH)
    fin = (cq.Workplane("YZ", origin=(SOCKET_X - FIN_T / 2, 0, 0))
           .polyline([(ya, T - 0.01), (yb, zb), (yb, T - 0.01)]).close().extrude(FIN_T).val())
    return cq.Workplane().add(cup.fuse(wedge).fuse(fin).clean())


def socket_footprint_y():
    """Plate-frame Y range of the wedge's footprint on the plate."""
    ys = [p.y for p in _cup_corners(-SOCKET_FLOOR)]
    return min(ys), max(ys)


# Carriage plate: the 65 x 48 carriage footprint, its leaf carried on past the -Y edge under the 1010
# socket's wedge; hinge knuckles stick out past the +Y edge
pattern = [(PATTERN_X + sx * HOLE_PITCH / 2, sy * HOLE_PITCH / 2) for sx in (-1, 1) for sy in (-1, 1)]
carriage_plate = (block(0, L, socket_footprint_y()[0], PLATE_W / 2)
                  .faces(">Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0))
                  .pushPoints(pattern).cboreHole(CLEAR_D, CBORE_D, CBORE_DEPTH))
carriage_plate = hinge_half(carriage_plate.union(mast_socket()), AXIS_Y, -1, "upper")

MAST_LEN_IN = 8.0     # camera mast (1010) standing in the socket
PROFILE_STEP = Path.home() / "Downloads" / "8020_101097" / "1010-97.stp"   # 80/20 PARTcommunity (git-ignored)


def mast_parts(length_in=MAST_LEN_IN, vendor=False):
    """[(name, Workplane, Color)] for the fit check: the 1010 in the socket plus its two clamp screws and T-nuts.

    The 1010 is a plain 1 in square stand-in, so the committed assemblies carry no 80/20 CAD; with
    vendor=True (git-ignored outputs only) it is the 80/20 1010 STEP from ~/Downloads, cut to length.
    Everything is built in the cup frame (1010 along +Z from the pocket floor) and moved onto the plate.
    """
    length = length_in * 25.4
    if vendor and PROFILE_STEP.exists():
        s = cq.importers.importStep(str(PROFILE_STEP)).val()
        bb = s.BoundingBox()
        s = s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
        profile = s.intersect(cq.Solid.makeBox(100, 100, length, cq.Vector(-50, -50, 0)))
    else:
        profile = cq.Solid.makeBox(PROFILE, PROFILE, length, cq.Vector(-PROFILE / 2, -PROFILE / 2, 0))
    parts = [("mast_1010", profile, cq.Color(0.78, 0.79, 0.81))]

    # 1/4-20 x 1/2 BHSCS (head 0.437 x 0.137 in) on each clamp wall, economy T-nut 0.087 in under the slot lip
    head_d, head_h, shank, lip = 11.1, 3.48, 12.7, 2.21
    nut_l, nut_w, nut_t = 12.7, 11.4, 3.05
    for wall, n in (("outer", cq.Vector(0, -1, 0)), ("pos_x", cq.Vector(1, 0, 0))):
        centre = cq.Vector(0, 0, SOCKET_DEPTH / 2)
        outer = centre + n * (SOCKET_OUT / 2)
        inward = n * -1
        screw = (cq.Solid.makeCylinder(head_d / 2, head_h, outer + n * head_h, inward)
                 .fuse(cq.Solid.makeCylinder(6.35 / 2, shank, outer, inward)))
        c = centre + n * (PROFILE / 2) + inward * (lip + nut_t / 2)
        sx, sy = (nut_w, nut_t) if n.x == 0 else (nut_t, nut_w)
        nut = cq.Solid.makeBox(sx, sy, nut_l, cq.Vector(c.x - sx / 2, c.y - sy / 2, c.z - nut_l / 2))
        nut = nut.cut(cq.Solid.makeCylinder(6.35 / 2, 20, c - inward * 10, inward))
        parts += [(f"clamp_screw_{wall}", screw, cq.Color(0.15, 0.15, 0.17)),
                  (f"clamp_tnut_{wall}", nut, cq.Color(0.62, 0.63, 0.66))]
    return [(name, cq.Workplane().add(_to_plate(shape)), color) for name, shape, color in parts]


def make_link(pitch, top_clear=KNUCKLE_GAP, wall=None):
    """Chain link (flat): top pin at Y = 0 (lower half of that hinge), bottom pin at Y = pitch (upper half).

    wall: if given, cut a window through the leaf leaving this much material all round.
    """
    link = block(0, L, SWING_CLEAR, pitch - SWING_CLEAR)
    if wall:
        y0, y1 = SWING_CLEAR + wall, pitch - SWING_CLEAR - wall
        window = (cq.Workplane("XY").box(L - 2 * wall, y1 - y0, T + 2, centered=False)
                  .translate((wall, y0, -1)).edges("|Z").fillet(WINDOW_FILLET))
        link = link.cut(window)
    link = hinge_half(link, 0, +1, "lower", top_clear)
    return hinge_half(link, pitch, -1, "upper")


chain_link = make_link(LINK_PITCH, LINK_CLEAR, LINK_WALL)
first_link = make_link(FIRST_LINK_PITCH, LINK_CLEAR, LINK_WALL)


def link_pitches(n=N_LINKS):
    return ([FIRST_LINK_PITCH] + [LINK_PITCH] * (n - 1))[:n]


def drape_angles(pin_above_table=PIN_ABOVE_TABLE, n=N_LINKS):
    """Resting angle of each link (degrees below horizontal, outward) for a chain draped onto the table.

    A link hangs straight down if it clears the table; otherwise it tips outward until
    the underside corner at its lower end touches the table, and the rest lie (nearly) flat.
    """
    angles, h = [], pin_above_table          # h: height of the link's top pin above the table
    for pitch in link_pitches(n):
        reach = pitch + r                    # top pin to the far end of the link
        lowest = lambda a: h - reach * math.sin(a) - r * math.cos(a)   # lowest corner, tipped a below horizontal
        if lowest(math.pi / 2) >= 0:
            a = math.pi / 2
        elif lowest(0) <= 0:
            a = 0.0
        else:                                # bisect for the angle where the corner meets the table
            lo, hi = 0.0, math.pi / 2
            for _ in range(60):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if lowest(mid) > 0 else (lo, mid)
            a = lo
        angles.append(math.degrees(a))
        h -= pitch * math.sin(a)
    return angles


def chain_end(angles):
    """(y, z) of the last link's lower pin, carriage-plate frame, for links at these angles."""
    pitches = link_pitches(len(angles))
    return (AXIS_Y + sum(p * math.cos(math.radians(a)) for a, p in zip(angles, pitches)),
            r - sum(p * math.sin(math.radians(a)) for a, p in zip(angles, pitches)))


def hanging_chain(pin_above_table=PIN_ABOVE_TABLE, n=N_LINKS, angles=None):
    """Links, and the screws and nuts joining them, draped from the carriage plate's pin down onto the table."""
    if angles is None:
        angles = drape_angles(pin_above_table, n)
    links, screws, nuts = [], [screw_at(AXIS_Y)], [nut_at(AXIS_Y)]
    y, z = AXIS_Y, r                         # current top pin, carriage-plate frame
    for i, (a, pitch) in enumerate(zip(angles, link_pitches(len(angles)))):
        place = lambda w: (w.translate((0, 0, -r)).rotate((0, 0, 0), (1, 0, 0), -a)
                           .translate((0, y, z)))
        links.append(place(first_link if i == 0 else chain_link))
        if i < len(angles) - 1:
            screws.append(place(screw_at(pitch)))
            nuts.append(place(nut_at(pitch)))
        y += pitch * math.cos(math.radians(a))
        z -= pitch * math.sin(math.radians(a))
    return links, screws, nuts


if __name__ == "__main__":
    here = Path(__file__).parent
    for name, part in (("carriage_hinge_plate", carriage_plate), ("chain_link_long", first_link)):
        cq.exporters.export(part, str(here / f"{name}.step"))
        cq.exporters.export(part, str(here / f"{name}.stl"), tolerance=0.01, angularTolerance=0.1)

    links, screws, nuts = hanging_chain()
    assy = cq.Assembly().add(carriage_plate, name="carriage_plate", color=cq.Color(0.9, 0.47, 0.12))
    for name, part, color in mast_parts():
        assy.add(part, name=name, color=color)
    for i, link in enumerate(links):
        c = cq.Color(0.24, 0.47, 0.78) if i % 2 == 0 else cq.Color(0.3, 0.7, 0.45)
        assy.add(link, name=f"link{i + 1}", color=c)
    for i, (s, n) in enumerate(zip(screws, nuts)):
        assy.add(s, name=f"screw{i + 1}", color=cq.Color(0.15, 0.15, 0.17))
        assy.add(n, name=f"nut{i + 1}", color=cq.Color(0.8, 0.8, 0.82))
    assy.save(str(here / "carriage_chain.step"))
    assy.save(str(here / "carriage_chain.glb"))
    print(f"hinge pin {AXIS_Y:.2f} mm from carriage centre; link angles below horizontal: "
          + ", ".join(f"{a:.1f}" for a in drape_angles()))
