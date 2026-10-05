"""X-Rite i1 cradle plate, hinged onto the end of the carriage chain.

The cradle is xRitei1.stl (SketchUp, from iCloud Drive/SketchUp): a 64 x 131.6 plate
with a 30.6 mm ring around a 9.8 mm measuring aperture (1 mm floor, ring 6.76 tall),
a triangular open frame whose arms ramp up to a 60 x 23 mm pocket at the wide end
(10.76 tall). The mesh is sewn into a solid as-is and only moved into the chain
frame; this script adds:

- a solid bar across the wide end (the pocket's end wall is only 1.3 mm), and
- the "lower" half of the chain hinge (middle knuckle, PIVOT_D pivot hole, LINK_CLEAR
  axial clearance like the links) on that bar, so the plate pins to the bottom end of
  the last chain link like another link.

With the knuckle axis T/2 above the plate's underside, the plate lies flat on the
table with the aperture floor on the chart.

Frame (flat, as printed): same as a chain link in carriage_chain.py, X along the
hinge (0..L), pin axis at Y = 0, Z = T/2, plate at +Y, underside at Z = 0.

Run:  python xrite_i1_plate.py
  -> xrite_i1_plate.stl/.step       (print flat, underside down)
  -> carriage_chain_xrite.step/.glb carriage plate + chain + screws, nuts + i1 plate on the table
  -> chain_print_plate.stl          carriage hinge plate, long first link and i1 plate, flat side by side
"""
import math
from pathlib import Path
import cadquery as cq
import trimesh
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from carriage_chain import (L, r, first_link, SWING_CLEAR, LINK_CLEAR, PIN_ABOVE_TABLE, N_LINKS, hinge_half, screw_at,
                            nut_at, carriage_plate, link_pitches, drape_angles, hanging_chain, chain_end)

SOURCE = Path(__file__).parent / "xRitei1.stl"
BAR = 5.0             # solid bar between the hinge leaf line and the cradle's wide end
PLATE_GAP = 8.0       # between parts on the combined print plate


def mesh_solid(path):
    """Closed STL mesh -> OCC solid: sew the triangles into a shell, then merge coplanar faces."""
    m = trimesh.load(path)
    m.merge_vertices(digits_vertex=3)
    sew = BRepBuilderAPI_Sewing(1e-3)
    for tri in m.triangles[m.area_faces > 1e-6]:      # SketchUp leaves zero-area slivers at a corner
        sew.Add(cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*map(float, p)) for p in tri],
                                                          close=True)).wrapped)
    sew.Perform()
    shell = cq.Shape.cast(sew.SewedShape())
    return cq.Solid.makeSolid(shell if isinstance(shell, cq.Shell) else shell.Shells()[0]).clean().fix()


src = mesh_solid(SOURCE)
bb = src.BoundingBox()
CRADLE_H = bb.zlen    # 10.76 at the raised pocket end

# Rotate 180 deg about Z so the wide (pocket) end faces the hinge and the ring points away,
# centre it on the hinge length, put its wide end BAR past the leaf line and its underside on Z = 0.
cradle = (cq.Workplane().add(src)
          .rotate((0, 0, 0), (0, 0, 1), 180)
          .translate((L / 2 + (bb.xmin + bb.xmax) / 2, SWING_CLEAR + BAR + bb.ymax, -bb.zmin)))
cb = cradle.val().BoundingBox()

bar = (cq.Workplane("XY").box(cb.xlen, BAR + 0.01, CRADLE_H, centered=False)
       .translate((cb.xmin, SWING_CLEAR, 0)))
xrite_plate = hinge_half(cradle.union(bar), 0, +1, "lower", LINK_CLEAR).clean()

# Aperture centre in the plate frame (source ring centre 35.85, 51.29)
APERTURE = (L / 2 + (bb.xmin + bb.xmax) / 2 - 35.85, SWING_CLEAR + BAR + bb.ymax - 51.29)


def chain_angles(pin_above_table=PIN_ABOVE_TABLE, n=N_LINKS):
    """Link angles with the i1 plate flat on the table: the last link's lower pin sits T/2 up."""
    angles = drape_angles(pin_above_table, n - 1)
    h = pin_above_table - r + chain_end(angles)[1]          # last link's top pin above the table
    pitch = link_pitches(n)[-1]
    angles.append(math.degrees(math.asin(min(1.0, max(0.0, (h - r) / pitch)))))
    return angles


def chain_with_plate(pin_above_table=PIN_ABOVE_TABLE, n=N_LINKS):
    """Links, screws and nuts (including the plate's) and the flat i1 plate, carriage-plate frame."""
    angles = chain_angles(pin_above_table, n)
    links, screws, nuts = hanging_chain(pin_above_table, n, angles)
    y, z = chain_end(angles)
    place = lambda w: w.translate((0, y, z - r))
    return links, screws + [place(screw_at(0))], nuts + [place(nut_at(0))], place(xrite_plate)


if __name__ == "__main__":
    here = Path(__file__).parent
    cq.exporters.export(xrite_plate, str(here / "xrite_i1_plate.step"))
    cq.exporters.export(xrite_plate, str(here / "xrite_i1_plate.stl"), tolerance=0.01, angularTolerance=0.1)
    b = xrite_plate.val().BoundingBox()
    print(f"i1 plate {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.2f} mm, aperture at "
          f"({APERTURE[0]:.2f}, {APERTURE[1]:.2f}) from the hinge end / pin axis")

    # One print: the three parts flat (underside on Z = 0), side by side along X
    tray, x = [], 0.0
    for part in (carriage_plate, first_link, xrite_plate):
        pb = part.val().BoundingBox()
        tray.append(part.translate((x - pb.xmin, -pb.ymin, -pb.zmin)).val())
        x += pb.xlen + PLATE_GAP
    tray = cq.Workplane("XY").add(cq.Compound.makeCompound(tray))
    cq.exporters.export(tray, str(here / "chain_print_plate.stl"), tolerance=0.01, angularTolerance=0.1)
    tb = tray.val().BoundingBox()
    print(f"chain_print_plate.stl  {tb.xlen:.1f} x {tb.ylen:.1f} x {tb.zlen:.2f} mm")

    links, screws, nuts, plate = chain_with_plate()
    assy = cq.Assembly().add(carriage_plate, name="carriage_plate", color=cq.Color(0.9, 0.47, 0.12))
    for i, link in enumerate(links):
        c = cq.Color(0.24, 0.47, 0.78) if i % 2 == 0 else cq.Color(0.3, 0.7, 0.45)
        assy.add(link, name=f"link{i + 1}", color=c)
    for i, (s, n) in enumerate(zip(screws, nuts)):
        assy.add(s, name=f"screw{i + 1}", color=cq.Color(0.15, 0.15, 0.17))
        assy.add(n, name=f"nut{i + 1}", color=cq.Color(0.8, 0.8, 0.82))
    assy.add(plate, name="xrite_i1_plate", color=cq.Color(0.85, 0.85, 0.2))
    table = PIN_ABOVE_TABLE - r
    assy.add(cq.Workplane("XY").box(L + 100, 600, 6, centered=(True, False, False))
             .translate((L / 2, -150, -table - 6)), name="table", color=cq.Color(0.8, 0.73, 0.59))
    assy.save(str(here / "carriage_chain_xrite.step"))
    assy.save(str(here / "carriage_chain_xrite.glb"))
    pb = plate.val().BoundingBox()
    print("link angles below horizontal: " + ", ".join(f"{a:.1f}" for a in chain_angles())
          + f"; i1 plate underside {pb.zmin + table:.2f} mm above the table, "
          f"aperture {chain_end(chain_angles())[0] + APERTURE[1]:.1f} mm out from the carriage centre")
