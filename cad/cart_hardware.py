"""Connecting hardware for gantry_cart.py, modelled from 80/20's published dimensions.

  4132   10 series 2-hole gusseted inside corner bracket: 0.875 in flanges, 1.0 in wide, one
         1/4 in hole per flange on the centre line, with a gusset rib along each edge
  3393   bolt assembly: 1/4-20 x 0.5 BHSCS + slide-in economy T-nut (nut sits under the
         0.087 in slot lip; 10 series slot measured from the 1010 STEP)
  3321   bolt assembly: 1/4-20 x 0.5 FBHSCS + economy T-nut, panels to the rails
  SHCS   5/16-18 x 5/8 (caster to 2419 plate), 1/4-20 x 3/4 (2419 plate to leg end)
  2015   10 series 1 x 1 end cap on the exposed 1010 ends

All functions take/return inches in the cart frame and return [(name, shape, colour)].
"""
import cadquery as cq

IN = 25.4
STEEL = cq.Color(0.2, 0.2, 0.22)
ZINC = cq.Color(0.62, 0.63, 0.66)
ALU = cq.Color(0.78, 0.79, 0.81)
BLACK = cq.Color(0.08, 0.08, 0.08)

# 4132: two side ribs (gussets) along the edges, holes on the centre line between them
FL, W, T, RIB_T = 0.875, 1.0, 0.125, 0.10
HOLE_AT, HOLE_D = 0.5, 0.28
SCREW_L = 0.375          # 1/4-20 x 3/8 BHSCS: a 1/2 in screw bottoms out in the 0.323 in deep slot
# 10 series slot / economy T-nut
LIP = 0.087
NUT_L, NUT_W, NUT_T = 0.50, 0.45, 0.12
# screws (head dia, head height)
BHCS_14 = (0.437, 0.137)
SHCS_14 = (0.375, 0.250)
SHCS_516 = (0.469, 0.3125)
FHCS_14_HEAD = 0.500


def _cyl(r, h, z0=0.0):
    return cq.Solid.makeCylinder(r * IN, h * IN, cq.Vector(0, 0, z0 * IN))


def _box(x0, y0, z0, dx, dy, dz):
    return cq.Solid.makeBox(dx * IN, dy * IN, dz * IN, cq.Vector(x0 * IN, y0 * IN, z0 * IN))


def bhcs(length, d=0.25, head=BHCS_14):
    """Button/socket head screw, head on z >= 0, shank down to z = -length."""
    return _cyl(head[0] / 2, head[1]).fuse(_cyl(d / 2, length, -length))


def fhcs(length, d=0.25, head=FHCS_14_HEAD):
    """Flat head screw, countersunk head top at z = 0, shank down to z = -length (overall)."""
    hh = (head - d) / 2                       # 82 deg-ish cone, approximated at 90 deg
    cone = cq.Solid.makeCone(d / 2 * IN, head / 2 * IN, hh * IN, cq.Vector(0, 0, -hh * IN))
    return cone.fuse(_cyl(d / 2, length - hh, -length))


def tnut():
    """Economy T-nut body, long axis along local x, top face at z = -LIP (under the slot lip)."""
    b = _box(-NUT_L / 2, -NUT_W / 2, -LIP - NUT_T, NUT_L, NUT_W, NUT_T)
    return b.cut(_cyl(0.25 / 2, NUT_T + 0.02, -LIP - NUT_T - 0.01))


def gusset_4132():
    """Local frame: corner line along y (centred), flange A on z in [0, T] extending +x,
    flange B on x in [0, T] extending +z, web in the middle."""
    a = _box(0, -W / 2, 0, FL, W, T)
    b = _box(0, -W / 2, 0, T, W, FL)
    rib = lambda y0: (cq.Workplane("XZ", origin=(0, y0 * IN, 0)).polyline([(0, 0), (FL * IN, 0), (0, FL * IN)]).close()
                      .extrude(-RIB_T * IN).val())
    g = a.fuse(b).fuse(rib(-W / 2)).fuse(rib(W / 2 - RIB_T))
    g = g.cut(cq.Solid.makeCylinder(HOLE_D / 2 * IN, 2 * IN, cq.Vector(HOLE_AT * IN, 0, -1 * IN)))
    g = g.cut(cq.Solid.makeCylinder(HOLE_D / 2 * IN, 2 * IN, cq.Vector(-1 * IN, 0, HOLE_AT * IN), cq.Vector(1, 0, 0)))
    return g


def _loc(corner, n1, n2):
    """Local z -> n1, local x -> n2, origin at corner (inches)."""
    pl = cq.Plane(origin=cq.Vector(*corner) * IN, xDir=cq.Vector(*n2), normal=cq.Vector(*n1))
    return cq.Location(pl)


def bracket_joint(name, corner, n1, n2, slot_b="n1"):
    """4132 in the inside corner between face 1 (normal n1) and face 2 (normal n2), with its two
    3393 bolt assemblies. Flange A lies on face 1 and runs along n2; flange B on face 2 along n1.
    T-nuts lie along their slots: A's along n2; B's along n1 by default, or along the bracket's
    width (n1 x n2) when member B's slot runs that way (slot_b="width", e.g. a vertical mast)."""
    loc = _loc(corner, n1, n2)
    out = [(f"{name}_4132", gusset_4132().moved(loc), ALU)]
    # bolt A: head on flange A, shank into face 1; T-nut in that member's slot (slot runs along n2 = local x)
    bolt_a = bhcs(SCREW_L).translate(cq.Vector(HOLE_AT * IN, 0, T * IN))
    nut_a = tnut().translate(cq.Vector(HOLE_AT * IN, 0, 0))
    # bolt B: same thing in a frame turned so local z -> n2 (rotate -90 about y: x -> z, z -> -x)
    rot = lambda s: s.rotate(cq.Vector(), cq.Vector(0, 1, 0), 90)
    bolt_b = rot(bhcs(SCREW_L).translate(cq.Vector(-HOLE_AT * IN, 0, T * IN)))
    nb = tnut() if slot_b == "n1" else tnut().rotate(cq.Vector(), cq.Vector(0, 0, 1), 90)
    nut_b = rot(nb.translate(cq.Vector(-HOLE_AT * IN, 0, 0)))
    out += [(f"{name}_3393_screw_a", bolt_a.moved(loc), STEEL), (f"{name}_3393_tnut_a", nut_a.moved(loc), ZINC),
            (f"{name}_3393_screw_b", bolt_b.moved(loc), STEEL), (f"{name}_3393_tnut_b", nut_b.moved(loc), ZINC)]
    return out


def panel_screw(name, x, y, z_top, slot_dir):
    """3321: FBHSCS through a panel (countersunk, head flush at z_top) into the rail's top slot."""
    screw = fhcs(0.5).translate(cq.Vector(x * IN, y * IN, z_top * IN))
    nut = tnut()
    if slot_dir == "y":
        nut = nut.rotate(cq.Vector(), cq.Vector(0, 0, 1), 90)
    return [(f"{name}_3321_screw", screw, STEEL),
            (f"{name}_3321_tnut", nut.translate(cq.Vector(x * IN, y * IN, (z_top - PANEL_T_DEFAULT) * IN)), ZINC)]


PANEL_T_DEFAULT = 0.236      # set by gantry_cart to its PANEL_T


def countersink(x, y, z_top):
    """Cutter for a 1/4 FBHSCS countersink + clearance hole in a panel whose top is z_top."""
    return fhcs(1.0).translate(cq.Vector(x * IN, y * IN, z_top * IN))


def end_cap(name, x, y, z, direction):
    """2015: 1 x 1 x 0.1 cap on a 1010 end face at (x, y, z), pointing along `direction` (+-x)."""
    t = 0.1
    x0 = x if direction > 0 else x - t
    return [(f"{name}_2015", _box(x0, y - 0.5, z - 0.5, t, 1.0, 1.0), BLACK)]
