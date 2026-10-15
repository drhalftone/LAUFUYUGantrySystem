"""Basler ace USB 3.0 camera bracket for the top of the 1010 camera mast, aimed at the i1 cradle's aperture.

A "7" seen from the side (along the travel), in the spirit of the SICK Visionary-T Mini plates
(LAUCowProcessingTools/CAD/SICK/build_sick_plate_*.py): nothing wraps around the camera or the 1010.

- The stem is a flat 7-shaped plate on the 1010's +X side face (along the travel), bolted with two
  1/4-20 x 1/2 FBHSCS (flanged button head) into economy T-nuts in that face's slot (80/20 3393; slide the T-nuts in from the
  1010's top end). Its top follows the bar as a gusset.
- The bar comes off the top of the stem, just past the 1010's end, slopes down toward the i1 aperture,
  and runs across from the side plate to under the camera. The camera sits on top of the bar, its optical axis along the bar and through the aperture,
  held by three M3 x 6 SHCS driven up from under the bar into the camera's own threads (counterbored,
  2.5 mm into the camera's 3 mm deep threads). The camera sits close to the 7's joint, with every counterbore whole; a
  gusset fills the 7's inside corner, in the side plate's plane beside the camera.
  Assembly: screw the camera to the bracket first (the rear M3 has only ~18 mm of straight room once the
  bracket is on the 1010), then bolt the bracket to the 1010's side face.

Camera: Basler ace USB 3.0 classic, C-mount (acA1920-150um; Basler drawing IB019417 rev 02 and its STEP,
"Basler ace USB3 C-Mount v02"): housing 29.3 x 29 x 29 mm, a 12 mm C-mount in front and a 6.9 mm
connector block behind (48.2 mm overall). M3 x 3 deep on the bottom face: (a) two holes 20.0 mm apart,
3.0 mm behind the housing's front face; (b) one on the centre line 23.7 mm behind them.
The bar's holes are slots, so it also takes the ace 2 USB (CAMERAS): (a) widened across to its 21.0 mm
spacing, (b) stretched back to its 31.2 mm. The (a) pair is the datum, so every body's optical axis is
the same line; only how far forward the housing sits changes.
Basler's STEP is not redistributed: with it in ~/Downloads (BASLER_STEP) the checks run against it and a
git-ignored camera_mount_assembly_basler.step carries it; the committed outputs use a placeholder of the
same envelope.

Frames: built in the mast-top frame (origin at the centre of the 1010's top end, Z along the 1010,
Y toward its carriage-facing / hinge-side face, X = carriage-plate X), then moved into the
carriage-plate frame of carriage_chain.py, so it drops into the same assemblies as the plate.

Run:  python camera_mount.py
  -> camera_mount.step/.stl            the printed bracket (as installed; print it on its side, the 7 flat)
  -> camera_mount_print.stl            the bracket laid for printing: camera seat face down on the bed
  -> camera_mount_assembly.step/.glb   carriage plate + mast + bracket + camera placeholder + i1 chain
  -> camera_mount_assembly_basler.step the same with Basler's camera, Edmund's lens and the 80/20 1010 (git-ignored)
"""
import math
from pathlib import Path
import cadquery as cq
import carriage_chain as cc
from xrite_i1_plate import chain_with_plate, chain_angles, APERTURE, PIVOT_Z

# --- camera: Basler ace USB 3.0 classic, C-mount ---
CAM_W = 29.0
CAM_L = 29.3                             # housing, without the C-mount and the connector block
CAM_MOUNT_L, CAM_MOUNT_D = 12.0, 24.5    # C-mount front
CAM_CONN_L = 6.9                         # connector block behind
CONN_X, CONN_Y = (-12.78, 11.15), (-11.29, 14.22)   # its extent, camera frame (from the STEP)
USB_Y = -CAM_W / 2 + 7.6                 # USB 3.0 Micro-B socket centre, on the centre line
PLUG_W, PLUG_H, PLUG_L = 20.0, 12.0, 35.0   # screw-lock cable plug + strain relief (rough envelope, clearance only)
LENS_D, LENS_L = 36.0, 43.72             # Edmund 6 mm C Series (#67-709): front of the lens this far ahead of its flange
HOLE_A_X = 20.0 / 2                      # (a) pair, either side of the camera's centre line
HOLE_A_FROM_FRONT = 3.0
HOLE_B_BEHIND_A = 23.7

# Other Basler 29 x 29 bodies the bar also takes, with the (a) pair as the common datum: the bar's holes are
# slots spanning every body's (a) spacing and (b) offset. ace 2 USB from Basler's to-scale mounting drawing
# (docs.baslerweb.com/mounting-instructions, drawing-mounting-instructions-ace-2-usb-m3.svg, +-0.3 mm).
# name: (housing length, (a) half spacing, (a) behind the housing front, (b) behind (a))
CAMERAS = {"ace USB3 classic (acA)": (CAM_L, HOLE_A_X, HOLE_A_FROM_FRONT, HOLE_B_BEHIND_A),
           "ace 2 USB (a2A)": (36.3, 21.0 / 2, 2.3, 31.2)}
BASLER_STEP = Path.home() / "Downloads" / "Basler_ace_USB3_C-Mount_v02" / "Basler ace USB3 C-Mount v02.STP"
LENS_STEP = Path.home() / "Downloads" / "step_67709.step"   # Edmund Optics' model of #67-709 (not redistributed)

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
M3_CLEAR, M3_CB_D, M3_CB_DEPTH = 3.6, 6.2, 4.5          # 3.5 mm under the head: M3 x 6 -> 2.5 mm into the camera
M3_LEN = 6.0           # the camera's M3 threads are 3 mm deep
Q_HEAD_D, Q_HEAD_H = cc.RAIL_HEAD_D, cc.RAIL_HEAD_H    # 1/4-20 flanged button head
Q_CLEAR, Q_CB_D, RAIL_CB = 7.1, Q_HEAD_D + 1.0, 1.5     # 6.5 mm under the head: the 1/2 in screw ends 6.2 mm into the slot
Q_LEN = 12.7           # 1/4-20 x 1/2, under the head
SLOT_FLOOR = 8.21      # 1010 T-slot floor below the face (80/20 1010-97.stp): the screw tip stays ~2 mm clear
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
CAM_X = AP.x                                             # across: the camera sits over the aperture, not the 1010's centre
CAM_C = cq.Vector(CAM_X, *C)
CAM_D = cq.Vector(0, *D)
CAM_LOC_TOP = cq.Location(cq.Plane(origin=CAM_C, xDir=cq.Vector(-1, 0, 0), normal=CAM_D))   # camera +Y = bar normal
CAM_LOC = TOP_LOC * CAM_LOC_TOP
BOTTOM = -CAM_W / 2
FRONT, BACK = CAM_L / 2, -CAM_L / 2
PAIR_Z = FRONT - HOLE_A_FROM_FRONT                       # the (a) pair's datum line
HOLES = [(-HOLE_A_X, PAIR_Z), (HOLE_A_X, PAIR_Z),
         (0.0, PAIR_Z - HOLE_B_BEHIND_A)]                # (x, z) on the camera's bottom face, design camera
_ax = [c[1] for c in CAMERAS.values()]
_bz = [PAIR_Z - c[3] for c in CAMERAS.values()]
SLOTS = [((-min(_ax), PAIR_Z), (-max(_ax), PAIR_Z)), ((min(_ax), PAIR_Z), (max(_ax), PAIR_Z)),
         ((0.0, max(_bz)), (0.0, min(_bz)))]             # each hole's two ends over every camera
SLOT_ENDS = list(dict.fromkeys(e for slot in SLOTS for e in slot))


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
            .fuse(_extrude(_bar(), CAM_X - BRACKET_W / 2, HALF)).clean())
    global SOLID_BEFORE_HOLES
    SOLID_BEFORE_HOLES = body
    # 1/4-20 into T-nuts in the 1010's +X slot (centre line Y = 0), counterbored on the side plate's outer face
    for w in RAIL_W:
        body = body.cut(cq.Solid.makeCylinder(Q_CLEAR / 2, T + 2, cq.Vector(HALF - 1, 0, w), cq.Vector(1, 0, 0)))
        body = body.cut(cq.Solid.makeCylinder(Q_CB_D / 2, RAIL_CB + 1, cq.Vector(HALF + T - RAIL_CB, 0, w), cq.Vector(1, 0, 0)))
    # M3 up into the camera, counterbored on the bar's underside; slotted to take every body in CAMERAS
    n = cq.Vector(0, *N)
    for (x0, z0), (x1, z1) in SLOTS:
        p0, p1 = cam_to_top(x0, BOTTOM, z0) - n * (T + 1), cam_to_top(x1, BOTTOM, z1) - n * (T + 1)
        body = body.cut(_slot(p0, p1, n, M3_CLEAR, T + 2)).cut(_slot(p0, p1, n, M3_CB_D, M3_CB_DEPTH + 1))
    return body.moved(TOP_LOC)


def _slot(p0, p1, n, d, h):
    """Stadium of width d from p0 to p1 (a round hole if they meet), extruded h along n."""
    u = p1 - p0
    if u.Length < 1e-6:
        return cq.Solid.makeCylinder(d / 2, h, p0, n)
    plane = cq.Plane(origin=(p0 + p1) * 0.5, xDir=u.normalized(), normal=n)
    return cq.Workplane(plane).slot2D(u.Length + d, d).extrude(h).val()


def basler_model():
    """Basler's STEP in the camera frame, or None if it isn't in ~/Downloads. Its own frame has the C-mount
    front at Z = 0 looking toward -Z, the housing from Z = 12 to 41.3, the M3 face at Y = -14.5."""
    if not BASLER_STEP.exists():
        return None
    s = cq.importers.importStep(str(BASLER_STEP))
    c = cq.Compound.makeCompound(s.solids().vals())
    return (c.rotate(cq.Vector(), cq.Vector(0, 1, 0), 180)
            .translate(cq.Vector(0, 0, CAM_MOUNT_L + CAM_L / 2)))


def lens_model():
    """Edmund's STEP in the camera frame, flange on the camera's C-mount face, or None if it isn't in
    ~/Downloads. Its own frame has the flange at X = 0 looking toward -X, the thread behind it to X = 5.1,
    and the iris / focus lock screws toward -Y: turned to the camera's top here, where they can be reached
    (in practice they end up wherever the thread stops)."""
    if not LENS_STEP.exists():
        return None
    s = cq.importers.importStep(str(LENS_STEP))
    c = cq.Compound.makeCompound(s.solids().vals())
    return (c.rotate(cq.Vector(), cq.Vector(1, 0, 0), 180).rotate(cq.Vector(), cq.Vector(0, 1, 0), 90)
            .translate(cq.Vector(0, 0, FRONT + CAM_MOUNT_L)))


def camera_placeholder(vendor=False):
    """Camera (housing + C-mount + connector block, or Basler's model with vendor=True when it is on hand)
    and lens (a cylinder of the Edmund lens's envelope, or Edmund's model), both in place."""
    lens = (lens_model() if vendor else None) or cq.Solid.makeCylinder(LENS_D / 2, LENS_L, cq.Vector(0, 0, FRONT + CAM_MOUNT_L))
    real = basler_model() if vendor else None
    if real is not None:
        return real.moved(CAM_LOC), lens.moved(CAM_LOC)
    hw = CAM_W / 2
    body = cq.Solid.makeBox(CAM_W, CAM_W, CAM_L, cq.Vector(-hw, -hw, BACK))
    front = cq.Solid.makeCylinder(CAM_MOUNT_D / 2, CAM_MOUNT_L, cq.Vector(0, 0, FRONT))
    conn = cq.Solid.makeBox(CONN_X[1] - CONN_X[0], CONN_Y[1] - CONN_Y[0], CAM_CONN_L,
                            cq.Vector(CONN_X[0], CONN_Y[0], BACK - CAM_CONN_L))
    return body.fuse(front).fuse(conn).moved(CAM_LOC), lens.moved(CAM_LOC)


def usb_plug():
    """Rough envelope of the screw-lock USB cable plug in the camera's Micro-B socket, for clearance only."""
    return cq.Solid.makeBox(PLUG_W, PLUG_H, PLUG_L,
                            cq.Vector(-PLUG_W / 2, USB_Y - PLUG_H / 2, BACK - CAM_CONN_L - PLUG_L)).moved(CAM_LOC)


def screws():
    """M3 x 6 SHCS up into the camera and 1/4-20 x 1/2 FBHSCS into the 1010, heads in their counterbores."""
    out = []
    n = cq.Vector(0, *N)
    for x, z in HOLES:
        top = cam_to_top(x, BOTTOM, z)
        head = top - n * (T - M3_CB_DEPTH)                     # head top face
        s = (cq.Solid.makeCylinder(SCREW_HEAD_D / 2, SCREW_HEAD_H, head, n * -1)
             .fuse(cq.Solid.makeCylinder(1.5, M3_LEN, head, n)))
        out.append(("camera_m3", s.moved(TOP_LOC)))
    for w in RAIL_W:
        head = cq.Vector(HALF + T - RAIL_CB, 0, w)
        s = (cq.Solid.makeCylinder(Q_HEAD_D / 2, Q_HEAD_H, head, cq.Vector(1, 0, 0))
             .fuse(cq.Solid.makeCylinder(6.35 / 2, Q_LEN, head, cq.Vector(-1, 0, 0))))
        out.append(("rail_fbhscs", s.moved(TOP_LOC)))
    return out


def driver_paths(m3_length=15.0, rail_length=40.0, d=7.0):
    """Straight driver access out of every counterbore: down from the bar's underside (an M3 L-key's short
    arm is ~15 mm), out from the stem (1/4-20 hex key or driver)."""
    out = []
    n = cq.Vector(0, *N)
    for x, z in SLOT_ENDS:
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
    for x, z in SLOT_ENDS:
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


def print_pose():
    """The bracket with the bar's top (the camera seat) face down on the bed at Z = 0, the bar along +X."""
    def world(p):
        v = cq.Vertex.makeVertex(*p).moved(TOP_LOC)
        return cq.Vector(v.X, v.Y, v.Z)
    o = world((0, 0, 0))
    n = (world((0, *N)) - o).normalized()
    d = (world((0, *D)) - o).normalized()
    bed = cq.Location(cq.Plane(origin=world((0, *F)), xDir=d, normal=-n))
    return cq.Workplane().add(mount.val().moved(bed.inverse))


def camera_parts(vendor=False):
    """[(name, Workplane, Color)] for assemblies: the printed plate, camera, lens and the screws.
    vendor=True puts Basler's model in (git-ignored outputs only)."""
    cam, lens = camera_placeholder(vendor)
    name = "basler_ace" if vendor and BASLER_STEP.exists() else "basler_ace_placeholder"
    parts = [("camera_bracket", mount, cq.Color(0.2, 0.55, 0.85)),
             (name, cq.Workplane().add(cam), cq.Color(0.1, 0.1, 0.12)),
             ("edmund_67709" if vendor and LENS_STEP.exists() else "lens_placeholder", cq.Workplane().add(lens),
              cq.Color(0.3, 0.3, 0.32))]
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

    mast = cc.mast_parts(vendor=True)[0][1].val()          # checks use the real 1010 profile and camera when on hand
    cam, lens = camera_placeholder(vendor=True)
    print(f"camera model: {'Basler STEP' if BASLER_STEP.exists() else 'placeholder (no Basler STEP in ~/Downloads)'}, "
          f"lens model: {'Edmund STEP' if LENS_STEP.exists() else 'placeholder (no Edmund STEP in ~/Downloads)'}")
    print(f"lens/bracket closest {lens.distance(m):.1f} mm, lens/camera overlap {lens.intersect(cam).Volume():.1f} mm3")
    print(f"bracket/1010 overlap {m.intersect(mast).Volume():.2f} mm3, bracket/camera overlap {m.intersect(cam).Volume():.2f} mm3, "
          f"camera/1010 closest {cam.distance(mast):.1f} mm, lens/1010 closest {lens.distance(mast):.1f} mm")
    if BASLER_STEP.exists():                                # each bar hole lands on one of the camera's threaded holes
        for i, (x, z) in enumerate(HOLES):
            p = cam_to_top(x, BOTTOM + 1.5, z)
            v = cq.Vertex.makeVertex(*p.toTuple()).moved(TOP_LOC)
            print(f"camera M3 {i + 1}: {'on a threaded hole' if not cam.isInside(cq.Vector(v.X, v.Y, v.Z)) else 'MISSES the hole'}")
    plug = usb_plug()
    print(f"USB plug envelope: bracket closest {plug.distance(m):.1f} mm, 1010 closest {plug.distance(mast):.1f} mm")
    for name, (L, ax, af, bz) in CAMERAS.items():       # every body: its screws pass the slots, its housing clears
        front = PAIR_Z + af
        box = cq.Solid.makeBox(CAM_W, CAM_W, L, cq.Vector(-CAM_W / 2, -CAM_W / 2, front - L)).moved(CAM_LOC)
        tail = cq.Solid.makeBox(PLUG_W, PLUG_H, CAM_CONN_L + PLUG_L,
                                cq.Vector(-PLUG_W / 2, -PLUG_H / 2, front - L - CAM_CONN_L - PLUG_L)).moved(CAM_LOC)
        free = []
        for x, z in [(-ax, PAIR_Z), (ax, PAIR_Z), (0.0, PAIR_Z - bz)]:
            ok = True
            for dx, dz in [(0, 0), (1.5, 0), (-1.5, 0), (0, 1.5), (0, -1.5)]:  # M3 shank edge, under the camera
                for h in (0.5, T / 2, T - 0.5):
                    p = cq.Vertex.makeVertex(*cam_to_top(x + dx * 0.99, BOTTOM - h, z + dz * 0.99).toTuple()).moved(TOP_LOC)
                    ok &= not m.isInside(cq.Vector(p.X, p.Y, p.Z))
            free.append(ok)
        print(f"{name}: screws pass the slots {all(free)}, housing/bracket overlap {m.intersect(box).Volume():.2f} mm3, "
              f"housing/1010 closest {box.distance(mast):.1f} mm, connector + plug envelope/bracket closest {tail.distance(m):.1f} mm")
    tip = Q_LEN - (T - RAIL_CB)
    print(f"1/4-20 tip {tip:.1f} mm into the 1010 slot, {SLOT_FLOOR - tip:.1f} mm clear of its floor")
    for name, ok in holes_whole():
        print(f"{name} hole + counterbore whole (1 mm of material all round): {ok}")
    for i, p in enumerate(driver_paths()):
        hit = sum(p.intersect(o).Volume() for o in (m, mast, cam, lens))
        kind = "M3 (camera)" if i < len(SLOT_ENDS) else "1/4-20 (rail)"
        print(f"driver path {i + 1} {kind}: {'clear' if hit < 0.01 else f'BLOCKED ({hit:.0f} mm3)'}")
    sight = cq.Solid.makeCylinder(2.0, t - CAM_L / 2 - CAM_MOUNT_L - LENS_L - 5,
                                  centre + axis * (CAM_L / 2 + CAM_MOUNT_L + LENS_L), axis)
    links, screws_, nuts, i1 = chain_with_plate()
    for name, s in [("1010", mast), ("carriage plate", cc.carriage_plate.val()), ("long link", links[0].val()), ("camera bracket", m)]:
        print(f"line of sight vs {name}: closest {sight.distance(s):.1f} mm")

    cq.exporters.export(mount, str(here / "camera_mount.step"))
    cq.exporters.export(mount, str(here / "camera_mount.stl"), tolerance=0.01, angularTolerance=0.1)
    cq.exporters.export(print_pose(), str(here / "camera_mount_print.stl"), tolerance=0.01, angularTolerance=0.1)

    def assembly(vendor):
        assy = cq.Assembly().add(cc.carriage_plate, name="carriage_plate", color=cq.Color(0.9, 0.47, 0.12))
        for name, part, color in cc.mast_parts(vendor=vendor) + camera_parts(vendor):
            assy.add(part, name=name, color=color)
        for i, l in enumerate(links):
            assy.add(l, name=f"link{i + 1}", color=cq.Color(0.24, 0.47, 0.78))
        assy.add(i1, name="xrite_i1_plate", color=cq.Color(0.85, 0.85, 0.2))
        return assy

    assy = assembly(False)
    assy.save(str(here / "camera_mount_assembly.step"))
    assy.save(str(here / "camera_mount_assembly.glb"))
    print("wrote camera_mount.step/.stl, camera_mount_print.stl and camera_mount_assembly.step/.glb")
    if BASLER_STEP.exists():
        assembly(True).save(str(here / "camera_mount_assembly_basler.step"))
        print("wrote camera_mount_assembly_basler.step (Basler's model; git-ignored)")
