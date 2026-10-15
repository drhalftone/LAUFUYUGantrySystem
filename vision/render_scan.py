"""Synthetic camera frames of the breadboard for developing the hole-grid filter, with exact ground truth.

The Basler on the carriage mast (cad/camera_mount.py) looks down at ~65 deg past the i1 cradle. This
renders what it sees at a grid of carriage positions, as the scan would: the carriage moves, the camera
and the i1 chain move with it, the board and the leg rails stay put.

Scene: cad/gantry_on_plate.glb (Y-up; breadboard top at y = 12.7, board x 0..609.6, z -609.6..0).
- Breadboard: drawn analytically per pixel (ray -> top plane -> nearest hole), so the holes are exact:
  r 3.19 mm at the top, 45 deg chamfer to r 2.55 mm, through the 12.7 mm board, on the 25.4 mm grid
  starting 12.7 mm from the edges (measured from the board mesh). Matte brushed-aluminium top.
- Everything else (cradle, chain, rails, ...): z-buffered triangles from the GLB.
- Camera: 6 mm lens on the acA1920-150um (camera.py: 1920 x 1200, 5.86 um), with barrel distortion, vignetting, defocus,
  and shot/read noise, saved as 8-bit grey PNGs.

Board frame (used for everything downstream): origin at the board's corner on its top face,
Xb = glb x (along the legs, the X axis), Yb = -glb z (along the beam, the Y axis), Zb up. Hole (i, j)
is at (12.7 + 25.4 i, 12.7 + 25.4 j), i, j = 0..23.

Carriage position (x, y) is mm from the model's pose (both axes at mid-stroke): x moves the beam along
the legs (+Xb), y moves the beam carriage along the beam (+Yb).

Run:  python render_scan.py [--step 120] [--n 3] [--out scan]
      python render_scan.py --xs=-150,-75,0,75,150 --ys=-180,-90,0,90,180 --out scan_bare
      python render_scan.py --xs=... --ys=... --paper --out scan_paper     (same positions + test target)
  -> <out>/frame_<x>_<y>.png and <out>/scan.json (camera model, carriage positions, true hole pixels,
     and with --paper the sheet's true corners and patch centres in board mm)
"""
import argparse, json, re
from pathlib import Path
import numpy as np, trimesh, cv2
from camera import W, H, PIX, F_MM, nominal_K

HERE = Path(__file__).parent
GLB = HERE.parent / "cad" / "gantry_on_plate.glb"

K = nominal_K()
FX = K[0, 0]
DIST = np.array([-0.08, 0.02, 0.0, 0.0, 0.0])        # k1 k2 p1 p2 k3: barrel, ~90 px at the corners
SS = 2                                                # supersampling
# extra pinhole canvas around the image: barrel distortion pulls in what lies outside the pinhole frame
_corner = cv2.undistortPointsIter(np.array([[[0.0, 0.0]]]), K, DIST, None, None,
                                  (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-10)).ravel() * FX
MARGIN = float(max(-_corner[0] / (W / 2), -_corner[1] / (H / 2)) - 1 + 0.01)

BOARD, BOARD_T, PITCH, EDGE, NHOLE = 609.6, 12.7, 25.4, 12.7, 24
R_TOP, R_IN = 3.19, 2.55
SKIP = {"lens_placeholder", "basler_ace_placeholder", "camera_bracket", "camera_m3"}
CARRIAGE = {"NAUO2", "NAUO5", "NAUO6"} | {f"NAUO{i}" for i in range(21, 31)}   # fsk40_resize.CARRIAGE

# glb (x, y, z) -> board (Xb, Yb, Zb):  Xb = x, Yb = -z, Zb = y - 12.7
M_GB = np.array([[1.0, 0, 0], [0, 0, -1], [0, 1, 0]])


def hole_xy(i, j):
    return EDGE + PITCH * np.asarray(i), EDGE + PITCH * np.asarray(j)


# ----------------------------------------------------------------------------------------- paper target
class PaperTarget:
    """A letter sheet (8.5 x 11 in) taped flat on the platen, printed with a block of packed swatches
    (random grey levels: what a mono camera makes of a colour chart), centred, no registration marks.
    Default: 32 x 43 swatches of 6 mm, no gaps (192 x 258 mm, 11-12 mm margins).
    Pose: centre (board mm) and rotation (deg) of the sheet; its long side runs along Yb."""
    W, L = 215.9, 279.4                       # sheet, along its own u (short) and v (long) axes
    PAPER, INK_MIN = 0.86, 0.04

    def __init__(self, centre=(162.0, 305.0), angle=1.5, seed=3, cols=32, rows=43, patch=6.0, gap=0.0):
        self.COLS, self.ROWS, self.PATCH, self.GAP = cols, rows, patch, gap
        self.c = np.array(centre, float)
        a = np.deg2rad(angle)
        self.R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])   # sheet (u, v) -> board
        self.angle = angle
        self.levels = np.random.default_rng(seed).uniform(self.INK_MIN, 0.80, (self.ROWS, self.COLS))
        gw = self.COLS * self.PATCH + (self.COLS - 1) * self.GAP
        gl = self.ROWS * self.PATCH + (self.ROWS - 1) * self.GAP
        self.u0, self.v0 = -gw / 2, -gl / 2

    def to_sheet(self, Xb, Yb):
        d = np.stack([Xb - self.c[0], Yb - self.c[1]], -1)
        uv = d @ self.R                                       # R^T d
        return uv[..., 0], uv[..., 1]

    def albedo(self, Xb, Yb):
        """Reflectance at board points; NaN off the sheet."""
        u, v = self.to_sheet(Xb, Yb)
        out = np.full(u.shape, np.nan, np.float32)
        on = (np.abs(u) <= self.W / 2) & (np.abs(v) <= self.L / 2)
        out[on] = self.PAPER
        pu, pv = (u - self.u0) / (self.PATCH + self.GAP), (v - self.v0) / (self.PATCH + self.GAP)
        ci, ri = np.floor(pu).astype(int), np.floor(pv).astype(int)
        inp = (on & (ci >= 0) & (ci < self.COLS) & (ri >= 0) & (ri < self.ROWS)
               & ((pu - ci) * (self.PATCH + self.GAP) < self.PATCH) & ((pv - ri) * (self.PATCH + self.GAP) < self.PATCH))
        out[inp] = self.levels[ri[inp], ci[inp]]
        return out

    def truth(self):
        to_b = lambda u, v: (self.c + self.R @ np.array([u, v])).tolist()
        corners = [to_b(su * self.W / 2, sv * self.L / 2) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        patches = []
        for r in range(self.ROWS):
            for c in range(self.COLS):
                u = self.u0 + c * (self.PATCH + self.GAP) + self.PATCH / 2
                v = self.v0 + r * (self.PATCH + self.GAP) + self.PATCH / 2
                patches.append({"row": r, "col": c, "centre": to_b(u, v), "level": float(self.levels[r, c])})
        return {"size_mm": [self.W, self.L], "centre": self.c.tolist(), "angle_deg": self.angle,
                "corners": corners, "patch_mm": self.PATCH, "gap_mm": self.GAP, "cols": self.COLS, "rows": self.ROWS,
                "patches": patches}


# ----------------------------------------------------------------------------------------- scene
def load_scene(plate_grey=None):
    """Non-board triangles in glb coordinates, colours, and which carriage they ride on.
    plate_grey: paint the i1 plate this grey (e.g. 0.05 for black PLA) instead of its model colour."""
    s = trimesh.load(GLB)
    G = s.graph.to_networkx()

    def ancestors(n):
        out = []
        while True:
            p = list(G.predecessors(n))
            if not p:
                return out
            n = p[0]
            out.append(n)

    tris, cols, grp, cam = [], [], [], {}
    for node in s.graph.nodes_geometry:
        T, g = s.graph[node]
        base = re.sub(r'_\d+$', '', g)
        mesh = s.geometry[g]
        if base in ("lens_placeholder", "basler_ace_placeholder"):
            cam.setdefault(base, []).append(trimesh.transform_points(mesh.vertices, T))
        if base in SKIP or base == "breadboard_24x24":
            continue
        anc = ancestors(node)
        leg = "leg_left" in anc or "leg_right" in anc
        on_y = "hinge_chain" in anc or ("beam" in anc and base in CARRIAGE)
        on_x = on_y or "beam" in anc or "plate_left" in anc or "plate_right" in anc or (leg and base in CARRIAGE)
        try:
            c = np.array(mesh.visual.material.baseColorFactor[:3]) / 255.0
            if plate_grey is not None and base == "xrite_i1_plate":
                c = np.full(3, plate_grey)
        except Exception:
            c = np.array([.7, .7, .7])
        t = trimesh.transform_points(mesh.vertices, T)[mesh.faces]
        tris.append(t)
        cols.append(np.tile(c, (len(t), 1)))
        grp.append(np.full(len(t), 2 if on_y else 1 if on_x else 0))
    lens = np.concatenate(cam["lens_placeholder"])
    body = np.concatenate(cam["basler_ace_placeholder"])
    axis = lens.mean(0) - body.mean(0)
    axis /= np.linalg.norm(axis)
    front = lens.mean(0) + axis * ((lens - lens.mean(0)) @ axis).max()   # pinhole at the lens front
    return np.concatenate(tris), np.concatenate(cols), np.concatenate(grp), front, axis


def camera_axes(axis):
    """Rows: camera x (right), y (down), z (optical axis), in glb. No roll about the vertical."""
    z = axis / np.linalg.norm(axis)
    x = np.cross(z, [0, 1.0, 0])
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z])


def board_pose(Rg, Cg):
    """Camera extrinsics in the board frame: p_cam = R p_board + t."""
    R = Rg @ M_GB.T
    t = Rg @ (M_GB.T @ np.array([0, 0, BOARD_T]) - Cg)
    return R, t


# ----------------------------------------------------------------------------------------- texture
rng = np.random.default_rng(7)
S1 = [rng.standard_normal(1 << 16).astype(np.float32) for _ in range(4)]
G1 = [rng.standard_normal((256, 256)).astype(np.float32) for _ in range(3)]


def _n1(c, res, a):
    i = c / res
    i0 = np.floor(i).astype(np.int64)
    f = i - i0
    f = f * f * (3 - 2 * f)
    return a[i0 % len(a)] * (1 - f) + a[(i0 + 1) % len(a)] * f


def _n2(px, pz, res, g):
    i, j = px / res, pz / res
    i0, j0 = np.floor(i).astype(np.int64), np.floor(j).astype(np.int64)
    fi, fj = i - i0, j - j0
    fi, fj = fi * fi * (3 - 2 * fi), fj * fj * (3 - 2 * fj)
    N = g.shape[0]
    q = lambda a, b: g[a % N, b % N]
    return ((q(i0, j0) * (1 - fi) + q(i0 + 1, j0) * fi) * (1 - fj)
            + (q(i0, j0 + 1) * (1 - fi) + q(i0 + 1, j0 + 1) * fi) * fj)


def brushed(px, pz):
    """Matte brushed-aluminium albedo, brushing along glb x."""
    streak = 0.50 * _n1(pz, 0.06, S1[0]) + 0.35 * _n1(pz, 0.25, S1[1]) + 0.25 * _n1(pz, 1.3, S1[2])
    streak = streak * (0.6 + 0.4 * np.abs(_n2(px, pz, 40.0, G1[0])))
    scratch = (_n1(pz, 0.04, S1[3]) > 2.6) * (_n2(px, pz * 30, 25.0, G1[1]) > 0.3) * 0.12
    mottle = 0.5 * _n2(px, pz, 60.0, G1[2]) + 0.3 * _n2(px, pz, 12.0, G1[0])
    return np.clip(0.56 + 0.035 * streak + 0.025 * mottle + scratch, 0, 1)


# ----------------------------------------------------------------------------------------- render
class TriangleRaster:
    """Which triangle covers each pixel of the pinhole canvas, drawn by VTK (OpenGL, offscreen): every
    triangle is painted flat, unlit, in a colour that encodes its index. Depth and shading are then
    worked out per pixel from that triangle, exactly as the CPU rasteriser does."""

    def __init__(self, tris, grp, Wc, Hc, fp):
        import vtk
        from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
        self.vtk, self.Wc, self.Hc = vtk, Wc, Hc
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(0, 0, 0)
        self.actors = []
        for g in (0, 1, 2):                                       # static, X carriage, Y carriage
            idx = np.nonzero(grp == g)[0]
            pd = vtk.vtkPolyData()
            pts = vtk.vtkPoints()
            pts.SetData(numpy_to_vtk(np.ascontiguousarray(tris[idx].reshape(-1, 3), np.float64), deep=True))
            pd.SetPoints(pts)
            cells = np.c_[np.full(len(idx), 3), np.arange(3 * len(idx)).reshape(-1, 3)].astype(np.int64).ravel()
            ca = vtk.vtkCellArray()
            ca.SetCells(len(idx), numpy_to_vtkIdTypeArray(cells, deep=True))
            pd.SetPolys(ca)
            code = idx + 1                                        # 0 = no triangle
            rgb = np.stack([code & 255, (code >> 8) & 255, (code >> 16) & 255], 1).astype(np.uint8)
            sc = numpy_to_vtk(rgb, deep=True)
            pd.GetCellData().SetScalars(sc)
            m = vtk.vtkPolyDataMapper()
            m.SetInputData(pd)
            m.SetScalarModeToUseCellData()
            m.SetColorModeToDirectScalars()
            a = vtk.vtkActor()
            a.SetMapper(m)
            a.GetProperty().LightingOff()
            a.GetProperty().SetInterpolationToFlat()
            self.ren.AddActor(a)
            self.actors.append(a)
        self.win = vtk.vtkRenderWindow()
        self.win.SetOffScreenRendering(1)
        self.win.SetMultiSamples(0)
        self.win.AddRenderer(self.ren)
        self.win.SetSize(Wc, Hc)
        cam = self.ren.GetActiveCamera()
        cam.SetViewAngle(np.degrees(2 * np.arctan(Hc / 2 / fp)))

    def ids(self, dx, dy, Cg, Rg):
        """(Hc, Wc) triangle index per pixel, -1 where none."""
        self.actors[1].SetPosition(dx, 0, 0)
        self.actors[2].SetPosition(dx, 0, -dy)
        cam = self.ren.GetActiveCamera()
        cam.SetPosition(*Cg)
        cam.SetFocalPoint(*(Cg + 100 * Rg[2]))
        cam.SetViewUp(*(-Rg[1]))                                  # image y runs down
        cam.SetClippingRange(5, 5000)
        self.win.Render()
        w2i = self.vtk.vtkWindowToImageFilter()
        w2i.SetInput(self.win)
        w2i.SetInputBufferTypeToRGB()
        w2i.Update()
        from vtk.util.numpy_support import vtk_to_numpy
        a = vtk_to_numpy(w2i.GetOutput().GetPointData().GetScalars()).reshape(self.Hc, self.Wc, 3)[::-1]
        a = a.astype(np.int32)
        return a[..., 0] + (a[..., 1] << 8) + (a[..., 2] << 16) - 1


def canvas():
    return int(W * SS * (1 + 2 * MARGIN)), int(H * SS * (1 + 2 * MARGIN)), FX * SS


def render(tris, cols, grp, front0, axis, dx, dy, undist_map, paper=None, raster=None):
    """One frame at carriage offset (dx, dy) mm. Returns 8-bit grey image, camera (R, t), z-buffer info.
    raster: a TriangleRaster for the gantry triangles (fast); None uses the CPU rasteriser."""
    shift = np.array([[0, 0, 0], [dx, 0, 0], [dx, 0, -dy]], float)
    Tw = tris + shift[grp][:, None, :]
    Cg = front0 + np.array([dx, 0, -dy])
    Rg = camera_axes(axis)
    z = Rg[2]
    Wc, Hc = int(W * SS * (1 + 2 * MARGIN)), int(H * SS * (1 + 2 * MARGIN))
    fp = FX * SS
    cxc, cyc = Wc / 2, Hc / 2

    # --- triangles (everything but the board)
    if raster is not None:                                        # GPU: which triangle covers each pixel
        tid = raster.ids(dx, dy, Cg, Rg)
        yy, xx = np.nonzero(tid >= 0)
        used, inv = np.unique(tid[yy, xx], return_inverse=True)
        Tu = Tw[used]
        P = (Tu - Cg) @ Rg.T
        nc = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
        dc = np.stack([(xx + 0.5 - cxc) / fp, (yy + 0.5 - cyc) / fp, np.ones(len(xx))], 1)
        den = (nc[inv] * dc).sum(1)
        num = (nc[inv] * P[inv, 0]).sum(1)
        zb = np.full((Hc, Wc), np.inf, np.float32)
        zb[yy, xx] = np.where(np.abs(den) > 1e-12, num / np.where(den == 0, 1, den), P[inv, 0, 2])
        n = np.cross(Tu[:, 1] - Tu[:, 0], Tu[:, 2] - Tu[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
        n[(n @ z) > 0] *= -1
        C = cols[used]
        tid[yy, xx] = inv                                         # index into used / C / n
        u = ()
    else:
        n = np.cross(Tw[:, 1] - Tw[:, 0], Tw[:, 2] - Tw[:, 0])
        nl = np.linalg.norm(n, axis=1)
        ok = nl > 1e-9
        Tw, C, n = Tw[ok], cols[ok], n[ok] / nl[ok, None]
        n[(n @ z) > 0] *= -1
        P = (Tw - Cg) @ Rg.T
        keep = (P[..., 2] > 5).all(1)
        P, C, n = P[keep], C[keep], n[keep]
        u = fp * P[..., 0] / P[..., 2] + cxc
        v = fp * P[..., 1] / P[..., 2] + cyc
        inb = ~((u.max(1) < 0) | (u.min(1) > Wc) | (v.max(1) < 0) | (v.min(1) > Hc))
        u, v, P, C, n = u[inb], v[inb], P[inb], C[inb], n[inb]
        zb = np.full((Hc, Wc), np.inf, np.float32)
        tid = np.full((Hc, Wc), -1, np.int32)
    for i in range(len(u)):                                       # CPU fallback: z-buffered rasteriser
        x0, x1 = int(max(np.floor(u[i].min()), 0)), int(min(np.ceil(u[i].max()), Wc - 1))
        y0, y1 = int(max(np.floor(v[i].min()), 0)), int(min(np.ceil(v[i].max()), Hc - 1))
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + .5, np.arange(y0, y1 + 1) + .5)
        (a_, bx, cx), (ay, by, cy) = u[i], v[i]
        d = (by - cy) * (a_ - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-12:
            continue
        l1 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / d
        l2 = ((cy - ay) * (gx - cx) + (a_ - cx) * (gy - cy)) / d
        l3 = 1 - l1 - l2
        m = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
        if not m.any():
            continue
        depth = 1 / (l1 / P[i, 0, 2] + l2 / P[i, 1, 2] + l3 / P[i, 2, 2])
        sub = zb[y0:y1 + 1, x0:x1 + 1]
        win = m & (depth < sub)
        sub[win] = depth[win]
        tid[y0:y1 + 1, x0:x1 + 1][win] = i

    L1 = np.array([0.25, 1.0, 0.35]); L1 /= np.linalg.norm(L1)
    L2 = -z + np.array([0, 0.3, 0]); L2 /= np.linalg.norm(L2)
    light = lambda N: 0.62 * np.clip(N @ L1, 0, 1) + 0.30 * np.clip(N @ L2, 0, 1) + 0.22

    img = np.zeros((Hc, Wc), np.float32)
    lum = np.array([0.299, 0.587, 0.114])
    plane_depth = np.full((Hc, Wc), np.inf, np.float32)
    for r0 in range(0, Hc, 256):                                  # board, analytically, in row chunks
        r1 = min(r0 + 256, Hc)
        gy, gx = np.mgrid[r0:r1, 0:Wc].astype(np.float32) + 0.5
        dc = np.stack([(gx - cxc) / fp, (gy - cyc) / fp, np.ones_like(gx)], -1)
        d = dc @ Rg                                               # ray directions, glb
        t = (BOARD_T - Cg[1]) / np.where(d[..., 1] < -1e-6, d[..., 1], -1e-6)
        p = Cg + t[..., None] * d
        pd = t * (d @ z)
        shade = np.full(gx.shape, 0.06, np.float32)               # off the board: dark table
        on = (t > 0) & (p[..., 0] > 0) & (p[..., 0] < BOARD) & (p[..., 2] < 0) & (p[..., 2] > -BOARD)
        px, pz = p[..., 0], p[..., 2]
        ci = np.clip(np.round((px - EDGE) / PITCH), 0, NHOLE - 1)
        cj = np.clip(np.round((-pz - EDGE) / PITCH), 0, NHOLE - 1)
        hx, hz = EDGE + PITCH * ci, -(EDGE + PITCH * cj)
        ox, oz = px - hx, pz - hz
        rho = np.hypot(ox, oz)
        N = np.zeros(p.shape, np.float32); N[..., 1] = 1
        sheet = np.zeros(on.shape, bool)
        if paper is not None:                                     # the sheet covers board and holes
            alb = paper.albedo(px, -pz)
            sheet = on & ~np.isnan(alb)
            shade[sheet] = alb[sheet] * light(N[sheet])
            on = on & ~sheet
        top = on & (rho >= R_TOP)
        shade[top] = brushed(px[top], pz[top]) * light(N[top])
        ch = on & (rho < R_TOP) & (rho >= R_IN)                    # 45 deg chamfer, facing the axis
        Nc = np.stack([-ox / np.maximum(rho, 1e-6) * 0.7071, np.full_like(rho, 0.7071), -oz / np.maximum(rho, 1e-6) * 0.7071], -1)
        shade[ch] = 0.50 * light(Nc[ch])
        hole = on & (rho < R_IN)                                  # ray continues down: far wall or through
        dh = d[..., [0, 2]]
        a = (dh ** 2).sum(-1)
        b = 2 * (ox * dh[..., 0] + oz * dh[..., 1])
        c = rho ** 2 - R_IN ** 2
        sx = (-b + np.sqrt(np.maximum(b * b - 4 * a * c, 0))) / (2 * np.maximum(a, 1e-12))
        depth_in = -sx * d[..., 1]
        wx, wz = ox + sx * dh[..., 0], oz + sx * dh[..., 1]
        Nw = np.stack([-wx / R_IN, np.zeros_like(wx), -wz / R_IN], -1)
        thread = 1 + 0.18 * np.sin(2 * np.pi * depth_in / 1.27 + np.arctan2(wz, wx))
        wall = 0.42 * np.exp(-depth_in / 2.5) * thread * (0.3 + 0.7 * light(Nw))
        wall = np.where(depth_in > BOARD_T - 0.64, 0.02, wall)
        shade[hole] = wall[hole]
        plane_depth[r0:r1] = np.where(on | sheet | (t > 0), pd, np.inf)
        img[r0:r1] = shade

    # --- composite the triangles in front of the board
    front_m = (tid >= 0) & (zb < plane_depth)
    yy, xx = np.nonzero(front_m)
    t_ = tid[yy, xx]
    rgb = np.clip(C[t_] * light(n[t_])[:, None], 0, 1)
    img[yy, xx] = rgb @ lum
    occluder = front_m

    # --- lens distortion: sample the pinhole canvas at each distorted pixel's undistorted position
    mx, my = undist_map
    img = cv2.remap(img, mx, my, cv2.INTER_LINEAR)
    occ = cv2.remap(occluder.astype(np.uint8), mx, my, cv2.INTER_NEAREST)
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    occ = cv2.resize(occ, (W, H), interpolation=cv2.INTER_NEAREST)
    return img, occ, Rg, Cg


def undistort_map():
    """For each supersampled distorted pixel, its position on the enlarged pinhole canvas."""
    gy, gx = np.mgrid[0:H * SS, 0:W * SS].astype(np.float32)
    pts = np.stack([(gx + 0.5) / SS - 0.5, (gy + 0.5) / SS - 0.5], -1).reshape(-1, 1, 2)
    und = cv2.undistortPointsIter(pts, K, DIST, None, None,
                                  (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 1e-6)).reshape(H * SS, W * SS, 2)
    Wc, Hc = W * SS * (1 + 2 * MARGIN), H * SS * (1 + 2 * MARGIN)
    mx = (und[..., 0] * FX * SS + Wc / 2 - 0.5).astype(np.float32)
    my = (und[..., 1] * FX * SS + Hc / 2 - 0.5).astype(np.float32)
    return mx, my


def sensor(img, seed):
    """Vignetting, defocus, shot + read noise, 8 bit."""
    r = np.random.default_rng(seed)
    gy, gx = np.mgrid[0:H, 0:W]
    rr = np.hypot((gx - W / 2) / FX, (gy - H / 2) / FX)
    img = img * (1 + rr ** 2) ** -2 * 1.15                       # cos^4 falloff
    img = cv2.GaussianBlur(img, (0, 0), 0.7)
    e = np.clip(img, 0, 1) * 4000                                # electrons
    e = r.poisson(e).astype(np.float32) + r.normal(0, 6, e.shape)
    return np.clip(e / 4000 * 255, 0, 255).astype(np.uint8)


def true_holes(R, t, occ, paper=None):
    """Every hole whose rim centre lands in the image: pixel, and whether something hides it."""
    I, J = np.meshgrid(np.arange(NHOLE), np.arange(NHOLE), indexing="ij")
    X, Y = hole_xy(I.ravel(), J.ravel())
    P = np.stack([X, Y, np.zeros_like(X)], 1)
    pc = P @ R.T + t
    uv, _ = cv2.projectPoints(P, cv2.Rodrigues(R)[0], t, K, DIST)
    uv = uv.reshape(-1, 2)
    out = []
    for k in range(len(P)):
        u, v = uv[k]
        if pc[k, 2] > 0 and 0 <= u < W and 0 <= v < H:
            hidden = bool(occ[int(v), int(u)])
            if paper is not None and not np.isnan(paper.albedo(np.array([X[k]]), np.array([Y[k]]))[0]):
                hidden = True
            out.append([int(I.ravel()[k]), int(J.ravel()[k]), float(u), float(v), not hidden])
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=120.0, help="carriage step between frames, mm")
    ap.add_argument("--n", type=int, default=3, help="frames per axis (odd: the middle one is the model pose)")
    ap.add_argument("--out", default="scan")
    ap.add_argument("--xs", help="carriage X positions, mm, comma separated (overrides --step/--n)")
    ap.add_argument("--ys", help="carriage Y positions, mm, comma separated")
    ap.add_argument("--paper", action="store_true", help="tape the letter-size test target to the platen")
    ap.add_argument("--blank-at", help="X,Y (board mm): a blank white letter sheet centred here instead of the target")
    ap.add_argument("--black-plate", action="store_true", help="i1 plate in black PLA")
    ap.add_argument("--cpu", action="store_true", help="draw the gantry on the CPU (slow) instead of with VTK")
    a = ap.parse_args()
    out = HERE / a.out
    out.mkdir(exist_ok=True)
    tris, cols, grp, front0, axis = load_scene(0.05 if a.black_plate else None)
    umap = undistort_map()
    raster = None if a.cpu else TriangleRaster(tris, grp, *canvas())
    offs = (np.arange(a.n) - (a.n - 1) / 2) * a.step
    xs = [float(v) for v in a.xs.split(",")] if a.xs else offs
    ys = [float(v) for v in a.ys.split(",")] if a.ys else offs
    paper = PaperTarget() if a.paper else None
    if a.blank_at:
        paper = PaperTarget(tuple(float(v) for v in a.blank_at.split(",")), 0.0, cols=0, rows=0)
    R0, t0 = board_pose(camera_axes(axis), front0)
    header = {"image_size": [W, H], "pixel_mm": PIX, "focal_mm": F_MM,
              "true_K": K.tolist(), "true_dist": DIST.tolist(),
              "board": {"pitch": PITCH, "edge": EDGE, "holes": NHOLE, "size": BOARD},
              "camera_at_home": {"R": R0.tolist(), "t": t0.tolist()},
              "paper": paper.truth() if paper else None}
    # scan.json is rewritten after every frame, so a stopped run resumes where it left off
    done = {}
    if (out / "scan.json").exists():
        done = {f["file"]: f for f in json.load(open(out / "scan.json"))["frames"]}
    frames = []
    for k, (dx, dy) in enumerate([(x, y) for y in ys for x in xs]):
        name = f"frame_{dx:+04.0f}_{dy:+04.0f}.png"
        if name in done and (out / name).exists():
            frames.append(done[name])
            continue
        img, occ, Rg, Cg = render(tris, cols, grp, front0, axis, dx, dy, umap, paper, raster)
        R, t = board_pose(Rg, Cg)
        cv2.imwrite(str(out / name), sensor(img, k))
        holes = true_holes(R, t, occ, paper)
        frames.append({"file": name, "carriage_mm": [float(dx), float(dy)],
                       "true_R": R.tolist(), "true_t": t.tolist(), "true_holes": holes})
        json.dump({**header, "frames": frames}, open(out / "scan.json", "w"), indent=1)
        print(f"{name}: {sum(h[4] for h in holes)} visible holes ({len(holes)} in frame)", flush=True)
    json.dump({**header, "frames": frames}, open(out / "scan.json", "w"), indent=1)
