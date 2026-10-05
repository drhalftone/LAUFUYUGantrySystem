"""Resize the TraceParts FSK40-E1250 STEP to the strokes FUYU actually shipped, and build the H gantry.

The FUYU order (FSK40XY-H1, quoted 2026-05-27) has a 310 x 610 mm work area: the two
parallel legs are 310 mm stroke and the cross beam is 610 mm. TraceParts only has the
1250 mm configuration, so each rail is made from it by splitting the parts in four:

  stretch   base extrusion and ball screw: cut out a slab in the plain middle section
            and close the gap, so both machined ends are kept
  trim      guide rail: shortened at the far end so its 25 mm hole pitch stays put
            relative to the motor end; rail screws past the new end are dropped
  far end   end plate, its screws and cap: moved back by the change in stroke
  carriage  carriage, ball nut and their screws: moved to mid-stroke
  (rest)    motor, coupling, motor-end bearing block, motor-end hardware: unchanged

Frame (TraceParts): X along travel (motor at -X), Y up, Z across; the base underside
(= table) is Y = -18.69. Base extrusion length = stroke + 125.

Run:  python fsk40_resize.py [path\\to\\FSK40-E1250-10C7-BC-B57.STEP]
  -> fsk40_s310.step, fsk40_s610.step   single rails, carriage at mid-stroke
  -> gantry_h1.step                     two 310 legs + 610 beam on carriage plates
These embed the TraceParts model, so they are git-ignored.
"""
import sys
from pathlib import Path
import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Quantity import Quantity_Color
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain

SRC = (sys.argv[1] if len(sys.argv) > 1 else
       str(Path.home() / "Downloads" / "192830955-3-fsk40-e1250-10c7-bc-b57" / "FSK40-E1250-10C7-BC-B57.STEP"))
SRC_STROKE = 1250.0
LEG_STROKE, BEAM_STROKE = 310.0, 610.0

# Part names in the TraceParts STEP (SolidWorks NAUO instance names)
STRETCH = {"NAUO1", "NAUO3"}                  # base extrusion, ball screw
SPLICE_X = 300.0                              # cut point: plain section, clear of both ends
RAIL = "NAUO4"                                # guide rail, ends 1 mm short of the far end plate
RAIL_SCREWS = {"NAUO14"} | {f"NAUO{i}" for i in (*range(31, 35), *range(48, 96))}
FAR_END = {"NAUO8", "NAUO13", "NAUO19", "NAUO20", "NAUO43"}
CARRIAGE = {"NAUO2", "NAUO5", "NAUO6"} | {f"NAUO{i}" for i in range(21, 31)}

# Landmarks (same as gantry_h.py)
BASE_BOTTOM = -18.69
BASE_X0 = -97.32                              # motor-end face of the base extrusion
CENTRE_Z = -101.15
CARRIAGE_TOP = 51.51
PLATE_T = 10.0
SUPPORT_INSET = 40.0                          # leg centre line this far in from each end of the beam's base


def read_parts(path):
    """[(name, shape in the assembly frame, Quantity_Color or None)] for every leaf part."""
    doc = TDocStd_Document(TCollection_ExtendedString("fsk40"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(path) != 1:
        sys.exit(f"cannot read {path}")
    reader.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "?"

    def color(*labels):
        c = Quantity_Color()
        for label in labels:
            for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
                if colors.GetColor_s(label, kind, c):
                    return c
        return None

    parts = []

    def walk(label, loc, top):
        ref = label
        if shapes.IsReference_s(label):
            ref = TDF_Label()
            shapes.GetReferredShape_s(label, ref)
            loc = loc.Multiplied(shapes.GetLocation_s(label))
        kids = TDF_LabelSequence()
        shapes.GetComponents_s(ref, kids)
        if kids.Length() == 0:
            parts.append((top or name(label), shapes.GetShape_s(ref).Moved(loc), color(label, ref)))
        for i in range(1, kids.Length() + 1):
            # sub-assemblies (the motor) keep their top-level name with a suffix
            walk(kids.Value(i), loc, f"{top}.{name(kids.Value(i))}" if top else None)

    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    kids = TDF_LabelSequence()
    shapes.GetComponents_s(roots.Value(1), kids)
    for i in range(1, kids.Length() + 1):
        k = kids.Value(i)
        ref = TDF_Label()
        shapes.GetReferredShape_s(k, ref)
        sub = TDF_LabelSequence()
        shapes.GetComponents_s(ref, sub)
        walk(k, TopLoc_Location(), name(k) if sub.Length() else None)
    return parts


BIG = 1e4


def keep_x(shape, x0, x1):
    """The part of shape with x0 <= X <= x1."""
    box = cq.Solid.makeBox(x1 - x0, 2 * BIG, 2 * BIG, cq.Vector(x0, -BIG, -BIG))
    return shape.intersect(box)


def unify(shape):
    u = ShapeUpgrade_UnifySameDomain(shape.wrapped, True, True, True)
    u.Build()
    return cq.Shape.cast(u.Shape())


def stretch(shape, dl):
    """Remove the slab SPLICE_X..SPLICE_X + dl and close the gap (dl > 0 shortens)."""
    left = keep_x(shape, -BIG, SPLICE_X)
    right = keep_x(shape, SPLICE_X + dl, BIG).translate(cq.Vector(-dl, 0, 0))
    return unify(left.fuse(right))


def rail(stroke, parts):
    """cq.Assembly of one FSK40 at the given stroke, carriage at mid-stroke."""
    dl = SRC_STROKE - stroke
    rail_end = bb_rail_end(parts, dl)
    assy = cq.Assembly(name=f"FSK40_S{stroke:.0f}")
    for name, shape, col in parts:
        top = name.split(".")[0]
        s = cq.Shape.cast(shape)
        if top in STRETCH:
            s = stretch(s, dl)
        elif top == RAIL:
            s = keep_x(s, -BIG, rail_end)
        elif top in RAIL_SCREWS:
            if s.BoundingBox().xmax > rail_end:
                continue
        elif top in FAR_END:
            s = s.translate(cq.Vector(-dl, 0, 0))
        elif top in CARRIAGE:
            s = s.translate(cq.Vector(-dl / 2, 0, 0))
        c = cq.Color(col.Red(), col.Green(), col.Blue()) if col is not None else cq.Color(0.7, 0.72, 0.75)
        assy.add(s, name=name.replace(".", "_"), color=c)
    return assy


def bb_rail_end(parts, dl):
    for name, shape, _ in parts:
        if name == RAIL:
            return cq.Shape.cast(shape).BoundingBox().xmax - dl
    raise KeyError(RAIL)


def base_length(stroke):
    return stroke + 125.0


def h_gantry(leg, beam, leg_spacing):
    """Legs along X (left leg in the TraceParts frame, right leg leg_spacing toward -Z); beam turned
    90 deg about Y, its base resting on the two leg carriage plates and centred between the legs.
    All carriages sit at mid-stroke."""
    here = Path(__file__).parent
    leg_x = (579.18 + 644.18) / 2 - (SRC_STROKE - LEG_STROKE) / 2   # leg carriage centre
    beam_y = CARRIAGE_TOP + PLATE_T - BASE_BOTTOM           # lift so the beam base sits on the plates
    # beam X -> world -Z: put the centre of its base midway between the legs
    dz = CENTRE_Z - leg_spacing / 2 + BASE_X0 + base_length(BEAM_STROKE) / 2
    plate = cq.importers.importStep(str(here / "fsk40_carriage_plate.step"))
    # plate frame: origin at hole-pattern centre on its bottom face, X along travel
    plate_loc = lambda z: cq.Location(cq.Vector(leg_x + 1.0, CARRIAGE_TOP, z), cq.Vector(1, 0, 0), -90)

    gantry = cq.Assembly(name="FSK40XY_H1")
    gantry.add(leg, name="leg_left")
    gantry.add(leg, name="leg_right", loc=cq.Location(cq.Vector(0, 0, -leg_spacing)))
    gantry.add(plate, name="plate_left", loc=plate_loc(CENTRE_Z), color=cq.Color(0.9, 0.47, 0.12))
    gantry.add(plate, name="plate_right", loc=plate_loc(CENTRE_Z - leg_spacing), color=cq.Color(0.9, 0.47, 0.12))
    gantry.add(beam, name="beam",
               loc=cq.Location(cq.Vector(leg_x - CENTRE_Z, beam_y, dz), cq.Vector(0, 1, 0), 90))
    return gantry


def build_rails():
    parts = read_parts(SRC)
    found = {n.split(".")[0] for n, _, _ in parts}
    missing = (STRETCH | FAR_END | CARRIAGE | {RAIL}) - found
    if missing:
        sys.exit(f"unexpected STEP structure, missing {sorted(missing)}")
    return rail(LEG_STROKE, parts), rail(BEAM_STROKE, parts)


if __name__ == "__main__":
    here = Path(__file__).parent
    leg, beam = build_rails()
    for stroke, a in ((LEG_STROKE, leg), (BEAM_STROKE, beam)):
        out = here / f"fsk40_s{stroke:.0f}.step"
        a.save(str(out))
        bb = a.toCompound().BoundingBox()
        print(f"{out.name}: overall X {bb.xmin:.2f}..{bb.xmax:.2f} ({bb.xlen:.1f} mm incl. motor), "
              f"base extrusion {base_length(stroke):.0f} mm")

    leg_spacing = base_length(BEAM_STROKE) - 2 * SUPPORT_INSET
    h_gantry(leg, beam, leg_spacing).save(str(here / "gantry_h1.step"))
    print(f"gantry_h1.step: leg spacing {leg_spacing:.1f} mm centre to centre, "
          f"beam base {base_length(BEAM_STROKE):.0f} mm on legs with {base_length(LEG_STROKE):.0f} mm bases")
