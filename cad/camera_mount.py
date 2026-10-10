"""Basler ace 2 USB camera bracket for the top of the 1010 camera mast, aimed at the i1 cradle's aperture.

A "7" seen from the side (along the travel), in the spirit of the SICK Visionary-T Mini plates
(LAUCowProcessingTools/CAD/SICK/build_sick_plate_*.py): nothing wraps around the camera or the 1010.

- The stem is a flat 7-shaped plate on the 1010's +X side face (along the travel), bolted with two
  1/4-20 x 1/2 BHSCS into economy T-nuts in that face's slot (80/20 3393; slide the T-nuts in from the
  1010's top end). Its top follows the bar as a gusset.
- The bar comes off the top of the stem, just past the 1010's end, slopes down toward the i1 aperture,
  and runs across from the side plate to under the camera. The camera sits on top of the bar, its optical axis along the bar and through the aperture,
  held by three M3 x 8 SHCS driven up from under the bar into the camera's own threads (counterbored,
  ~4.5 mm into the camera). The camera sits close to the 7's joint, with every counterbore whole; a
  gusset fills the 7's inside corner, in the side plate's plane beside the camera.
  Assembly: screw the camera to the bracket first (the rear M3 has only ~18 mm of straight room once the
  bracket is on the 1010), then bolt the bracket to the 1010's side face.

Camera: Basler ace 2 USB 3.0 (a2A...), housing 36.3 x 29 x 29 mm without lens mount and connector
(42.8 mm long overall). Its M3 holes on the bottom face were measured from Basler's to-scale drawing
(docs.baslerweb.com/mounting-instructions, drawing-mounting-instructions-ace-2-usb-m3.svg), good to
about +-0.3 mm: (a) two holes 21.0 mm apart, 2.3 mm behind the front face; (b) one on the centre
line 31.2 mm behind them. The bar's holes have some slop and (b) is a short slot.

Frames: built in the mast-top frame (origin at the centre of the 1010's top end, Z along the 1010,
Y toward its carriage-facing / hinge-side face, X = carriage-plate X), then moved into the
carriage-plate frame of carriage_chain.py, so it drops into the same assemblies as the plate.

Run:  python camera_mount.py
  -> camera_mount.step/.stl            the printed bracket (as installed; print it on its side, the 7 flat)
  -> camera_mount_assembly.step/.glb   carriage plate + mast + bracket + camera placeholder + i1 chain
"""
import math
from pathlib import Path
import cadquery as cq
import carriage_chain as cc
from xrite_i1_plate import chain_with_plate, chain_angles, APERTURE, PIVOT_Z

# --- camera: Basler ace 2 USB 3.0 ---
CAM_W = 29.0
CAM_L = 36.3
CAM_MOUNT_L, CAM_MOUNT_D = 6.5, 30.0     # C-mount front (placeholder)
CAM_CONN_L = 6.0                         # USB connector stub behind (placeholder)
LENS_D, LENS_L = 30.0, 35.0              # placeholder C-mount lens, only for clearance / line-of-sight checks
HOLE_A_X = 21.0 / 2                      # (a) pair, either side of the camera's centre line
HOLE_A_FROM_FRONT = 2.3
HOLE_B_BEHIND_A = 31.2

# --- "7" bracket: a stem bolted along the 1010's carriage-facing (+Y) face, and a bar off its top that
# slopes down toward the aperture; the camera sits on the bar, screwed up into from underneath ---
T = 8.0                # stem and bar thickness
BRACKET_W = CAM_W + 4.0                                  # across (plate X)
F_W = 6.0              # the 7's top corner (bar top meets the stem's inner face) this far past the 1010's end
BAR_GAP = 16.0         # camera's back face this far along the bar from the corner: (b)'s counterbore stays
                       # whole, clear of the joint, with ~19 mm of straight L-key room under it
BAR_NOSE = 3.0         # bar runs on this far past the camera's front face so the front counterbores stay whole
GUSSET_LEG = 25.0      # gusset in the 7's inside corner: this far down the stem and out along the bar
RAIL_W = [-15.0, -40.4]                                  # 1/4-20s along the 1010's side-face slot, 1 in apart
STEM_TAIL = 12.0       # stem runs on this far below the lower 1/4-20
M3_CLEAR, M3_CB_D, M3_CB_DEPTH = 3.6, 6.2, 4.5          # 3.5 mm under the head: M3 x 8 -> 4.5 mm into the camera
HOLE_B_SLOT = 2.0      # (b) is a slot along the camera axis
Q_CLEAR, Q_CB_D, RAIL_CB = 7.1, 12.0, 2.5                # head 1 mm proud; 1/2 in screw ends 7.2 mm into the slot
SCREW_HEAD_D, SCREW_HEAD_H = 5.5, 3.0

HALF = cc.PROFILE / 2                                    # the side plate's inner face sits on the 1010's +X face
TOP_LOC = cc.SOCKET_LOC * cq.Location(cq.Vector(0, 0, cc.MAST_LEN_IN * 25.4))


def to_top(p):
    """Carriage-plate frame point -> mast-top frame."""
    v = cq.Vertex.makeVertex(*p.toTuple()).moved(TOP_LOC.inverse)
    return cq.Vector(v.X, v.Y, v.Z)


def aperture_centre():
    """Centre of the i1 cradle's aperture (on the table), carriage-plate frame."""
    y, z = cc.chain_end(chain_angles())
    return cq.Vector(APERTURE[0], y + APERTURE[1], z - PIVOT_Z)


# Work in the mast-top frame's (Y, Z) plane: the 7 is a profile there, extruded across X.
AP = to_top(aperture_centre())
F = (HALF, F_W)                                          # the 7's top-left corner


def _solve():
    """Bar direction d (unit, (y, z)) such that the camera sitting on the bar looks straight at the aperture."""
    d = (0.86, -0.5)
    for _ in range(50):
        n = (-d[1], d[0])                                    # bar's top-surface normal (up / out)
        s = BAR_GAP + CAM_L / 2
        c = (F[0] + d[0] * s + n[0] * CAM_W / 2, F[1] + d[1] * s + n[1] * CAM_W / 2)
        t = (AP.y - c[0], AP.z - c[1])
        L = math.hypot(*t)
        d = (t[0] / L, t[1] / L)
    return d, n, c


D, N, C = _solve()
CAM_C = cq.Vector(0, *C)
CAM_D = cq.Vector(0, *D)
CAM_LOC_TOP = cq.Location(cq.Plane(origin=CAM_C, xDir=cq.Vector(-1, 0, 0), normal=CAM_D))   # camera +Y = bar normal
CAM_LOC = TOP_LOC * CAM_LOC_TOP
BOTTOM = -CAM_W / 2
FRONT, BACK = CAM_L / 2, -CAM_L / 2
HOLES = [(-HOLE_A_X, FRONT - HOLE_A_FROM_FRONT), (HOLE_A_X, FRONT - HOLE_A_FROM_FRONT),
         (0.0, FRONT - HOLE_A_FROM_FRONT - HOLE_B_BEHIND_A)]    # (x, z) on the camera's bottom face


def cam_to_top(x, y, z):
    v = cq.Vertex.makeVertex(x, y, z).moved(CAM_LOC_TOP)
    return cq.Vector(v.X, v.Y, v.Z)


def _line_hit(p, d, v):
    """Point where the line p + t d reaches Y = v."""
    t = (v - p[0]) / d[0]
    return (v, p[1] + t * d[1])


BAR_END = BAR_GAP + CAM_L + BAR_NOSE                     # bar ends just past the camera's front face
_under0 = (F[0] - N[0] * T, F[1] - N[1] * T)             # bar underside line passes here, direction D
CORNER_IN = _line_hit(_under0, D, HALF)                  # bar underside meets the line of the 1010's +Y face
STEM_BOTTOM = min(RAIL_W) - STEM_TAIL


def _bar():
    end_top = (F[0] + D[0] * BAR_END, F[1] + D[1] * BAR_END)
    end_under = (end_top[0] - N[0] * T, end_top[1] - N[1] * T)
    return [CORNER_IN, end_under, end_top, F]


def profile():
    """The 7 in (Y, Z): stem down the 1010's side face (its full width), bar off its top along D."""
    return [(-HALF, STEM_BOTTOM), (HALF, STEM_BOTTOM)] + _bar() + [(-HALF, F[1])]


def _gusset():
    """Triangle in the 7's inside corner, between the stem's front edge and the bar's underside."""
    return [CORNER_IN, (HALF, CORNER_IN[1] - GUSSET_LEG),
            (CORNER_IN[0] + D[0] * GUSSET_LEG, CORNER_IN[1] + D[1] * GUSSET_LEG)]


def _extrude(pts, x0, x1):
    pts = [(round(a, 4), round(b, 4)) for a, b in pts]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def plate():
    """Side plate (the 7) on the 1010's +X face, plus the bar run across under the camera; screw holes."""
    body = (_extrude(profile(), HALF, HALF + T).fuse(_extrude(_gusset(), HALF, HALF + T))
            .fuse(_extrude(_bar(), -BRACKET_W / 2, HALF)).clean())
    global SOLID_BEFORE_HOLES
    SOLID_BEFORE_HOLES = body
    # 1/4-20 into T-nuts in the 1010's +X slot (centre line Y = 0), counterbored on the side plate's outer face
    for w in RAIL_W:
        body = body.cut(cq.Solid.makeCylinder(Q_CLEAR / 2, T + 2, cq.Vector(HALF - 1, 0, w), cq.Vector(1, 0, 0)))
        body = body.cut(cq.Solid.makeCylinder(Q_CB_D / 2, RAIL_CB + 1, cq.Vector(HALF + T - RAIL_CB, 0, w), cq.Vector(1, 0, 0)))
    # M3 up into the camera, counterbored on the bar's underside; (b) slotted along the camera axis
    n = cq.Vector(0, *N)
    for i, (x, z) in enumerate(HOLES):
        spots = [z] if i < 2 else [z - HOLE_B_SLOT / 2, z, z + HOLE_B_SLOT / 2]
        for zz in spots:
            top = cam_to_top(x, BOTTOM, zz)                    # on the bar's top surface
            under = top - n * T
            body = body.cut(cq.Solid.makeCylinder(M3_CLEAR / 2, T + 2, under - n, n))
            body = body.cut(cq.Solid.makeCylinder(M3_CB_D / 2, M3_CB_DEPTH + 1, under - n, n))
    return body.moved(TOP_LOC)


def camera_placeholder():
    """Housing + C-mount front + USB connector + a lens, camera frame (optical axis +Z)."""
    hw = CAM_W / 2
    body = cq.Solid.makeBox(CAM_W, CAM_W, CAM_L, cq.Vector(-hw, -hw, BACK))
    front = cq.Solid.makeCylinder(CAM_MOUNT_D / 2 - 1, CAM_MOUNT_L, cq.Vector(0, 0, FRONT))
    conn = cq.Solid.makeBox(14, 9, CAM_CONN_L, cq.Vector(-7, -4.5, BACK - CAM_CONN_L))
    lens = cq.Solid.makeCylinder(LENS_D / 2, LENS_L, cq.Vector(0, 0, FRONT + CAM_MOUNT_L))
    return body.fuse(front).fuse(conn).moved(CAM_LOC), lens.moved(CAM_LOC)


def screws():
    """M3 x 8 SHCS up into the camera and 1/4-20 x 1/2 BHSCS into the 1010, heads in their counterbores."""
    out = []
    n = cq.Vector(0, *N)
    for x, z in HOLES:
        top = cam_to_top(x, BOTTOM, z)
        head = top - n * (T - M3_CB_DEPTH)                     # head top face
        s = (cq.Solid.makeCylinder(SCREW_HEAD_D / 2, SCREW_HEAD_H, head, n * -1)
             .fuse(cq.Solid.makeCylinder(1.5, 8.0, head, n)))
        out.append(("camera_m3", s.moved(TOP_LOC)))
    for w in RAIL_W:
        head = cq.Vector(HALF + T - RAIL_CB, 0, w)
        s = (cq.Solid.makeCylinder(11.1 / 2, 3.48, head, cq.Vector(1, 0, 0))
             .fuse(cq.Solid.makeCylinder(6.35 / 2, 12.7, head, cq.Vector(-1, 0, 0))))
        out.append(("rail_bhscs", s.moved(TOP_LOC)))
    return out


def driver_paths(m3_length=15.0, rail_length=40.0, d=7.0):
    """Straight driver access out of every counterbore: down from the bar's underside (an M3 L-key's short
    arm is ~15 mm), out from the stem (1/4-20 hex key or driver)."""
    out = []
    n = cq.Vector(0, *N)
    for x, z in HOLES:
        under = cam_to_top(x, BOTTOM, z) - n * (T + 0.5)
        out.append(cq.Solid.makeCylinder(d / 2, m3_length, under, n * -1).moved(TOP_LOC))
    for w in RAIL_W:
        out.append(cq.Solid.makeCylinder(d / 2, rail_length, cq.Vector(HALF + T + 2, 0, w), cq.Vector(1, 0, 0)).moved(TOP_LOC))
    return out


SOLID_BEFORE_HOLES = None
mount = cq.Workplane().add(plate().clean())


def holes_whole(margin=1.0):
    """True for each hole if a ring `margin` outside its counterbore is solid all the way through the part
    (mast-top frame): nothing breaks out of an edge or into the joint."""
    body = SOLID_BEFORE_HOLES
    n = cq.Vector(0, *N)
    out = []
    rings = []
    for x, z in HOLES:
        top = cam_to_top(x, BOTTOM, z)
        rings.append(("camera M3", top, n * -1, M3_CB_D / 2 + margin, T))
    for w in RAIL_W:
        rings.append(("rail 1/4-20", cq.Vector(HALF + T, 0, w), cq.Vector(-1, 0, 0), Q_CB_D / 2 + margin, T))
    for name, face, inward, r, depth in rings:
        a = (inward.cross(cq.Vector(1, 0, 0)) if abs(inward.x) < 0.9 else inward.cross(cq.Vector(0, 1, 0))).normalized()
        b = inward.cross(a)
        ok = all(body.isInside(face + inward * dd + (a * math.cos(t) + b * math.sin(t)) * r)
                 for dd in (0.5, depth / 2, depth - 0.5) for t in [k * math.pi / 8 for k in range(16)])
        out.append((name, ok))
    return out


def camera_parts():
    """[(name, Workplane, Color)] for assemblies: the printed plate, camera, lens and the screws."""
    cam, lens = camera_placeholder()
    parts = [("camera_bracket", mount, cq.Color(0.2, 0.55, 0.85)),
             ("basler_ace2_placeholder", cq.Workplane().add(cam), cq.Color(0.1, 0.1, 0.12)),
             ("lens_placeholder", cq.Workplane().add(lens), cq.Color(0.3, 0.3, 0.32))]
    return parts + [(f"{n}_{i + 1}", cq.Workplane().add(s), cq.Color(0.15, 0.15, 0.17)) for i, (n, s) in enumerate(screws())]


if __name__ == "__main__":
    here = Path(__file__).parent
    m = mount.val()
    ap = aperture_centre()
    c = cq.Vertex.makeVertex(0, 0, 0).moved(CAM_LOC)
    centre = cq.Vector(c.X, c.Y, c.Z)
    a = cq.Vertex.makeVertex(0, 0, 1).moved(CAM_LOC)
    axis = (cq.Vector(a.X, a.Y, a.Z) - centre).normalized()
    t = (ap - centre).dot(axis)
    miss = (centre + axis * t - ap).Length
    bb = m.BoundingBox()
    print(f"bracket valid {m.isValid()}, solids {len(mount.solids().vals())}, {m.Volume() / 1000:.1f} cm3, {T:g} mm thick")
    print(f"camera centre {tuple(round(v, 1) for v in centre.toTuple())}, rail screws {-RAIL_W[0]:.1f} / {-RAIL_W[1]:.1f} mm below the 1010's end, "
          f"{(ap - centre).Length:.0f} mm from the aperture; optical axis {math.degrees(math.acos(-axis.z)):.1f} deg "
          f"from vertical, passes {miss:.1f} mm from the aperture centre")

    mast = cc.mast_parts(vendor=True)[0][1].val()          # checks use the real 1010 profile when it's on hand
    cam, lens = camera_placeholder()
    print(f"bracket/1010 overlap {m.intersect(mast).Volume():.2f} mm3, bracket/camera overlap {m.intersect(cam).Volume():.2f} mm3, "
          f"camera/1010 closest {cam.distance(mast):.1f} mm, lens/1010 closest {lens.distance(mast):.1f} mm")
    for name, ok in holes_whole():
        print(f"{name} hole + counterbore whole (1 mm of material all round): {ok}")
    for i, p in enumerate(driver_paths()):
        hit = sum(p.intersect(o).Volume() for o in (m, mast, cam, lens))
        kind = "M3 (camera)" if i < len(HOLES) else "1/4-20 (rail)"
        print(f"driver path {i + 1} {kind}: {'clear' if hit < 0.01 else f'BLOCKED ({hit:.0f} mm3)'}")
    sight = cq.Solid.makeCylinder(2.0, t - CAM_L / 2 - CAM_MOUNT_L - LENS_L - 5,
                                  centre + axis * (CAM_L / 2 + CAM_MOUNT_L + LENS_L), axis)
    links, screws_, nuts, i1 = chain_with_plate()
    for name, s in [("1010", mast), ("carriage plate", cc.carriage_plate.val()), ("long link", links[0].val()), ("camera bracket", m)]:
        print(f"line of sight vs {name}: closest {sight.distance(s):.1f} mm")

    cq.exporters.export(mount, str(here / "camera_mount.step"))
    cq.exporters.export(mount, str(here / "camera_mount.stl"), tolerance=0.01, angularTolerance=0.1)

    assy = cq.Assembly().add(cc.carriage_plate, name="carriage_plate", color=cq.Color(0.9, 0.47, 0.12))
    for name, part, color in cc.mast_parts() + camera_parts():
        assy.add(part, name=name, color=color)
    for i, l in enumerate(links):
        assy.add(l, name=f"link{i + 1}", color=cq.Color(0.24, 0.47, 0.78))
    assy.add(i1, name="xrite_i1_plate", color=cq.Color(0.85, 0.85, 0.2))
    assy.save(str(here / "camera_mount_assembly.step"))
    assy.save(str(here / "camera_mount_assembly.glb"))
    print("wrote camera_mount.step/.stl and camera_mount_assembly.step/.glb")
