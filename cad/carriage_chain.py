"""Carriage hinge plate + hanging chain links for the FSK40 carriage.

The carriage plate is the fsk40_carriage_plate (65 x 48 x 10, counterbored M4 x 12
on the carriage's 25 x 25 pattern, flush with the carriage) with a square-knuckle
hinge along one long edge, so a chain of identical links can hang over the side
of the carriage down to the table.

Every hinge is the one from hinged_plates.py: the upper piece owns two end
knuckles (M4 SHCS head recessed in one, nylock nut captive in the other, 4.4 mm
pin hole) and the lower piece owns the middle knuckle (5.0 mm hole, turns freely
on the screw). Each piece stops SWING_CLEAR short of the other's knuckles.

Frame (flat, as printed): X along the hinge / carriage travel, X = 0 at the
motor end of the carriage; Y across the carriage, 0 at its centre, hinge side +Y;
Z up, 0 on the carriage top. Links are drawn flat with their top pin at Y = 0.

Run:  python carriage_chain.py
  -> carriage_hinge_plate.stl/.step, chain_link.stl/.step  (print flat)
  -> carriage_chain.step            plate + hanging chain + pins
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

# --- hinge (same as hinged_plates.py) ---
END_KNUCKLE = 16.0    # each end knuckle of the upper piece; the lower piece's fills the middle
KNUCKLE_GAP = 2.0     # axial space between knuckles (1 mm each face)
SWING_GAP = 1.0       # clearance between a turning knuckle's corners and the other piece
PIN_D = 4.4           # pin hole in the upper piece (clamped)
PIVOT_D = 5.0         # pin hole in the lower piece (turns freely)
HEAD_D, HEAD_DEPTH = 7.5, 7.0     # M4 SHCS head counterbore, head 3 mm below the end face
NUT_AF, NUT_DEPTH = 7.3, 8.0      # captive M4 nylock nut pocket
PIN_LEN = 55.0        # M4 x 55 SHCS: counterbore floor (x = 7) through the nut (57..61.7)

# --- chain ---
LINK_PITCH = 85.0     # pin-to-pin length of each link
N_LINKS = 3           # links hanging below the carriage plate
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


def hinge_half(body, axis_y, side, role):
    """Add one piece's half of a hinge whose pin runs along X at (axis_y, T/2).

    side: +1 if this piece's leaf lies at +Y of the pin, -1 if at -Y.
    role: "upper" (end knuckles, clamps the pin) or "lower" (middle knuckle, turns on it).
    The caller's leaf must already stop at axis_y + side * SWING_CLEAR.
    """
    for x0, x1, who in SPANS:
        if who == role:
            g0 = KNUCKLE_GAP / 2 if x0 > 0 else 0
            g1 = KNUCKLE_GAP / 2 if x1 < L else 0
            body = body.union(block(x0 + g0, x1 - g1, axis_y + side * SWING_CLEAR, axis_y - side * r))
    hole = PIN_D if role == "upper" else PIVOT_D
    body = body.cut(on_axis(axis_y, -1, L + 2, lambda w: w.circle(hole / 2)))
    if role == "upper":
        body = body.cut(on_axis(axis_y, -1, HEAD_DEPTH + 1, lambda w: w.circle(HEAD_D / 2)))
        body = body.cut(on_axis(axis_y, L - NUT_DEPTH, NUT_DEPTH + 1,
                                lambda w: w.polygon(6, NUT_AF / math.cos(math.pi / 6))))
    return body


def pin_at(axis_y):
    head = on_axis(axis_y, HEAD_DEPTH - 4, 4, lambda w: w.circle(3.5))
    shank = on_axis(axis_y, HEAD_DEPTH, PIN_LEN, lambda w: w.circle(2.0))
    nut = on_axis(axis_y, L - NUT_DEPTH, 4.7, lambda w: w.polygon(6, 7.0 / math.cos(math.pi / 6)))
    return head.union(shank).union(nut)


# Carriage plate: leaf is the whole 65 x 48 plate, hinge knuckles stick out past its +Y edge
pattern = [(PATTERN_X + sx * HOLE_PITCH / 2, sy * HOLE_PITCH / 2) for sx in (-1, 1) for sy in (-1, 1)]
carriage_plate = (block(0, L, -PLATE_W / 2, PLATE_W / 2)
                  .faces(">Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0))
                  .pushPoints(pattern).cboreHole(CLEAR_D, CBORE_D, CBORE_DEPTH))
carriage_plate = hinge_half(carriage_plate, AXIS_Y, -1, "upper")

# Chain link (flat): top pin at Y = 0 (lower half of that hinge), bottom pin at Y = LINK_PITCH (upper half)
chain_link = block(0, L, SWING_CLEAR, LINK_PITCH - SWING_CLEAR)
chain_link = hinge_half(chain_link, 0, +1, "lower")
chain_link = hinge_half(chain_link, LINK_PITCH, -1, "upper")


def drape_angles(pin_above_table=PIN_ABOVE_TABLE, n=N_LINKS):
    """Resting angle of each link (degrees below horizontal, outward) for a chain draped onto the table.

    A link hangs straight down if it clears the table; otherwise it tips outward until
    the underside corner at its lower end touches the table, and the rest lie (nearly) flat.
    """
    angles, h = [], pin_above_table          # h: height of the link's top pin above the table
    reach = LINK_PITCH + r                   # top pin to the far end of the link
    for _ in range(n):
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
        h -= LINK_PITCH * math.sin(a)
    return angles


def hanging_chain(pin_above_table=PIN_ABOVE_TABLE, n=N_LINKS):
    """Links (and their pins) draped from the carriage plate's pin down onto the table."""
    links, pins = [], [pin_at(AXIS_Y)]
    y, z = AXIS_Y, r                         # current top pin, carriage-plate frame
    for i, a in enumerate(drape_angles(pin_above_table, n)):
        place = lambda w: (w.translate((0, 0, -r)).rotate((0, 0, 0), (1, 0, 0), -a)
                           .translate((0, y, z)))
        links.append(place(chain_link))
        if i < n - 1:
            pins.append(place(pin_at(LINK_PITCH)))
        y += LINK_PITCH * math.cos(math.radians(a))
        z -= LINK_PITCH * math.sin(math.radians(a))
    return links, pins


if __name__ == "__main__":
    here = Path(__file__).parent
    for name, part in (("carriage_hinge_plate", carriage_plate), ("chain_link", chain_link)):
        cq.exporters.export(part, str(here / f"{name}.step"))
        cq.exporters.export(part, str(here / f"{name}.stl"), tolerance=0.01, angularTolerance=0.1)

    links, pins = hanging_chain()
    assy = cq.Assembly().add(carriage_plate, name="carriage_plate", color=cq.Color(0.9, 0.47, 0.12))
    for i, link in enumerate(links):
        c = cq.Color(0.24, 0.47, 0.78) if i % 2 == 0 else cq.Color(0.3, 0.7, 0.45)
        assy.add(link, name=f"link{i + 1}", color=c)
    for i, p in enumerate(pins):
        assy.add(p, name=f"pin{i + 1}", color=cq.Color(0.35, 0.35, 0.37))
    assy.save(str(here / "carriage_chain.step"))
    assy.save(str(here / "carriage_chain.glb"))
    print(f"hinge pin {AXIS_Y:.2f} mm from carriage centre; link angles below horizontal: "
          + ", ".join(f"{a:.1f}" for a in drape_angles()))
