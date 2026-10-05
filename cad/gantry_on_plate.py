"""The FSK40XY-H1 gantry bolted to a 24 x 24 in optical breadboard, as built in the lab.

Breadboard (step_03679.step): 609.6 x 609.6 x 12.7 mm, 1/4-20 tapped holes on a 25.4 mm grid,
outer rows 12.7 mm in from the edges. Each leg's outer bottom T-slot (10 mm outboard of the
rail centre line) sits over the breadboard's outermost hole row on its side, which fixes the
leg spacing at 609.6 - 2 * (12.7 + 10) = 564.2 mm centre to centre. The beam is centred
between the legs; the legs are centred on the board along their length.

Fasteners: the bottom T-slot is a 4.5 mm opening over an 8.2 x 4.0 mm cavity, which fits an
M4 SHCS head (7.0 x 4.0) exactly, and an M4 shank passes through a 1/4-20 tapped hole
(5.1 mm minor dia). So each fixing is an M4 x 20 SHCS, head in the slot, through the board,
with a nut underneath.

Run:  python gantry_on_plate.py [path\\to\\step_03679.step]
  -> gantry_on_plate.step   (git-ignored: embeds the TraceParts FSK40 model)
"""
import sys
from pathlib import Path
import cadquery as cq
from fsk40_resize import (build_rails, h_gantry, base_length, LEG_STROKE, BEAM_STROKE, SRC_STROKE,
                          BASE_BOTTOM, BASE_X0, CENTRE_Z, CARRIAGE_TOP, PLATE_T)
from carriage_chain import carriage_plate as hinge_plate, hanging_chain, T as HINGE_T

BOARD_SRC = sys.argv[1] if len(sys.argv) > 1 else str(Path.home() / "Downloads" / "step_03679.step")
BOARD = 609.6         # square
BOARD_T = 12.7
PITCH = 25.4
EDGE = 12.7           # outer hole row from the board edge
SLOT_OFFSET = 10.0    # bottom T-slots are +/- this from the rail centre line

LEG_SPACING = BOARD - 2 * (EDGE + SLOT_OFFSET)
SCREW_COLUMNS = (4, 12, 19)   # board hole columns (from the motor end) used on each leg

# M4 x 20 SHCS seated in the slot cavity (head bottom on the 1.5 mm lip), nut under the board
LIP_TOP = BASE_BOTTOM + 1.5
HEAD_D, HEAD_H, SHANK_D, SCREW_L = 7.0, 4.0, 4.0, 20.0
NUT_AF, NUT_H = 7.0, 3.2


def m4_screw_and_nut():
    head = cq.Workplane("XZ").circle(HEAD_D / 2).extrude(-HEAD_H).translate((0, LIP_TOP, 0))
    shank = cq.Workplane("XZ").circle(SHANK_D / 2).extrude(SCREW_L).translate((0, LIP_TOP, 0))
    nut_top = BASE_BOTTOM - BOARD_T
    nut = (cq.Workplane("XZ").polygon(6, NUT_AF / 0.8660254).circle(SHANK_D / 2)
           .extrude(NUT_H).translate((0, nut_top, 0)))
    return head.union(shank), nut


if __name__ == "__main__":
    here = Path(__file__).parent
    board = cq.importers.importStep(BOARD_SRC)
    bb = board.val().BoundingBox()
    if abs(bb.xlen - BOARD) > 0.1 or abs(bb.zlen - BOARD) > 0.1 or abs(bb.ylen - BOARD_T) > 0.1:
        sys.exit(f"unexpected breadboard size {bb.xlen:.1f} x {bb.zlen:.1f} x {bb.ylen:.1f}")

    # Board frame: X 0..609.6, Z -609.6..0, top face Y = 12.7; first hole row at Z = -12.7.
    # Put its top on the table (rail base underside), its Z = -12.7 row under the left leg's
    # outer slot, and centre the leg base extrusions along it.
    leg_base_mid = BASE_X0 + base_length(LEG_STROKE) / 2
    board_off = cq.Vector(leg_base_mid - BOARD / 2, BASE_BOTTOM - BOARD_T,
                          (CENTRE_Z + SLOT_OFFSET) + EDGE)

    leg, beam = build_rails()

    # Hinge plate + hanging chain (carriage_chain.py) on the beam carriage. Chain frame -> rail frame
    # as in fsk40_with_chain.py: X along travel from the carriage's motor-end face, chain Y -> rail -Z,
    # chain Z -> rail Y (carriage top); the resized beam's carriage sits at mid-stroke.
    beam_carriage_x = 579.18 - (SRC_STROKE - BEAM_STROKE) / 2
    on_carriage = cq.Location(cq.Vector(beam_carriage_x, CARRIAGE_TOP, CENTRE_Z), cq.Vector(1, 0, 0), -90)
    # beam carriage top above the board: beam base on the leg plates, plus the beam's own base-to-carriage height
    beam_top = (CARRIAGE_TOP + PLATE_T - BASE_BOTTOM) + (CARRIAGE_TOP - BASE_BOTTOM)
    links, pins = hanging_chain(beam_top + HINGE_T / 2)
    chain = cq.Assembly(name="hinge_chain")
    chain.add(hinge_plate, name="carriage_hinge_plate", color=cq.Color(0.9, 0.47, 0.12))
    for i, link in enumerate(links):
        chain.add(link, name=f"link{i + 1}",
                  color=cq.Color(0.24, 0.47, 0.78) if i % 2 == 0 else cq.Color(0.3, 0.7, 0.45))
    for i, p in enumerate(pins):
        chain.add(p, name=f"pin{i + 1}", color=cq.Color(0.35, 0.35, 0.37))
    beam.add(chain, name="hinge_chain", loc=on_carriage)

    model = h_gantry(leg, beam, LEG_SPACING)
    model.name = "FSK40XY_H1_on_breadboard"
    model.add(board, name="breadboard_24x24", loc=cq.Location(board_off), color=cq.Color(0.15, 0.15, 0.17))

    screw, nut = m4_screw_and_nut()
    slots = {"left": CENTRE_Z + SLOT_OFFSET, "right": CENTRE_Z - LEG_SPACING - SLOT_OFFSET}
    for side, z in slots.items():
        for k in SCREW_COLUMNS:
            x = board_off.x + EDGE + k * PITCH
            loc = cq.Location(cq.Vector(x, 0, z))
            model.add(screw, name=f"m4x20_{side}_{k}", loc=loc, color=cq.Color(0.2, 0.2, 0.22))
            model.add(nut, name=f"m4_nut_{side}_{k}", loc=loc, color=cq.Color(0.75, 0.75, 0.78))

    # Lay it flat for CAD: Y-up TraceParts frame -> Z-up (+90 deg about X: Y -> Z, Z -> -Y), with the
    # breadboard's underside on the XY plane and its corner at the origin (board in +X, +Y).
    flat = cq.Assembly(name="FSK40XY_H1_on_breadboard")
    flat.add(model, name="gantry",
             loc=cq.Location(cq.Vector(-board_off.x, board_off.z, -board_off.y), cq.Vector(1, 0, 0), 90))

    out = here / "gantry_on_plate.step"
    flat.save(str(out))
    print(f"{out.name}: Z-up, breadboard X/Y 0..{BOARD}, top face Z = {BOARD_T}; "
          f"leg spacing {LEG_SPACING:.1f} mm centre to centre")
    for side, z in slots.items():
        xs = ", ".join(f"{EDGE + k * PITCH:.1f}" for k in SCREW_COLUMNS)
        print(f"  {side} leg: M4 screws at Y = {board_off.z - z:.1f}, X = {xs}")
