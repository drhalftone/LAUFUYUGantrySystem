"""Breadboard hole detector and grid fit for the carriage camera.

Three stages, each usable on its own:

1. detect_holes(gray)  -> sub-pixel centres of hole-like dark ellipses.
   Flat-field the image with a grey closing (which erases anything smaller than a hole, i.e. the
   holes themselves, leaving the board's illumination), threshold the ratio image, keep contours
   that are filled, convex ellipses of a plausible size, then refine each centre by fitting an
   ellipse to sub-pixel edge crossings along rays from the blob centre.

2. grow_lattice(pts)   -> relative (i, j) for each detection, without any pose knowledge.
   Seed at the detection nearest the image centre, take its two shortest non-parallel neighbour
   vectors as the local basis, and flood outward: each step predicts the next hole by linear
   extrapolation from the already-labelled neighbours (so the spacing may change smoothly with
   perspective and lens distortion), and accepts the nearest detection within a fraction of the
   pitch. A homography fit over the grown set then fills gaps (e.g. holes beside the i1 cradle)
   and drops detections that do not sit on the lattice.

3. index_lattice(...)  -> absolute board indices (I, J) from the carriage position.
   The grid alone is ambiguous by whole pitches (and by the lattice's 8 symmetries). The carriage
   position from the FUYU controller plus a nominal camera model predicts where each detection is
   on the board; every symmetry + offset is scored by how many detections it puts on the predicted
   hole, so the prior only has to be good to about half a pitch (12.7 mm). The residual between the
   prior and the snapped grid is reported: it is the prior's pose error, useful for calibration.

Board frame: origin at the board corner on its top face; hole (I, J) at (EDGE + PITCH*I, EDGE + PITCH*J).
"""
from dataclasses import dataclass, field
import numpy as np, cv2
from scipy.spatial import cKDTree

PITCH, EDGE, NHOLE = 25.4, 12.7, 24
BRIGHT = 1.4          # ratio above which a pixel is a highlight (thread / lit hole wall), not board
SYMMETRIES = [np.array(m) for m in ([[1, 0], [0, 1]], [[0, -1], [1, 0]], [[-1, 0], [0, -1]], [[0, 1], [-1, 0]],
                                    [[1, 0], [0, -1]], [[-1, 0], [0, 1]], [[0, 1], [1, 0]], [[0, -1], [-1, 0]])]


# ----------------------------------------------------------------------------------------- 1. detect
@dataclass
class Detections:
    centers: np.ndarray            # (N, 2) px
    axes: np.ndarray               # (N, 2) full ellipse axes, px
    angles: np.ndarray             # (N,) deg
    contrast: np.ndarray           # (N,) mean ratio inside (0 = black, 1 = board)
    ratio: np.ndarray = field(repr=False, default=None)    # flat-fielded image, for debugging


def _odd(n):
    n = int(round(n))
    return n + 1 - n % 2


def detect_holes(gray, min_d=10, max_d=70, thresh=0.6):
    """gray: 2-D uint8/float image. min_d/max_d: plausible hole diameters in px (smallest far, largest near)."""
    g = gray.astype(np.float32)
    k = _odd(max_d * 1.4)
    # The background (board illumination with the holes closed over) is smooth -- it is blurred by k/6
    # px anyway -- so estimate it at 1/4 scale: ~16x less work than the full-res closing, same result.
    s = 4
    small = cv2.resize(g, (g.shape[1] // s, g.shape[0] // s), interpolation=cv2.INTER_AREA)
    ks = _odd(k / s)
    bg = cv2.morphologyEx(small, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks)))
    bg = cv2.GaussianBlur(bg, (0, 0), k / 6 / s)
    bg = cv2.resize(bg, (g.shape[1], g.shape[0]), interpolation=cv2.INTER_LINEAR)
    ratio = cv2.GaussianBlur(g, (0, 0), 0.8) / np.maximum(bg, 1)
    dark = (ratio < thresh).astype(np.uint8)
    # Under a low, angled light a tapped hole isn't one dark disc: the threads throw bright stripes
    # across the interior and the lit far wall shows as a bright crescent on the rim.  Count a bright
    # highlight that TOUCHES a dark hole as part of the hole (isolated bright debris stays out), then
    # close small gaps, so the blob is the whole opening again.
    bright = (ratio > BRIGHT).astype(np.uint8)
    near = cv2.dilate(dark, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (_odd(min_d * 0.5),) * 2))
    mask = dark | (bright & near)
    c_ = _odd(min_d * 0.6)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (c_, c_)))
    # open with a disc a bit smaller than the smallest hole: erases thin dark lines (edges of the i1
    # cradle, shadows) that would otherwise merge with a hole they touch; centres come from `ratio`
    o = _odd(min_d * 0.8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (o, o)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    a_min, a_max = np.pi * (min_d / 2) ** 2 * 0.5, np.pi * (max_d / 2) ** 2 * 1.5
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (a_min < area < a_max) or len(c) < 10:
            continue
        (cx, cy), (A, B), ang = cv2.fitEllipse(c)
        if min(A, B) < min_d * 0.5 or max(A, B) / max(min(A, B), 1e-6) > 3.5:
            continue
        fill = area / (np.pi * A * B / 4)
        solidity = area / max(cv2.contourArea(cv2.convexHull(c)), 1e-6)
        # (tolerant: a hole seen obliquely under directional light is the dark part of the opening only,
        # slightly crescent-shaped; the lattice fit downstream rejects anything not on the grid)
        if not (0.72 < fill < 1.15) or solidity < 0.85:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if x == 0 or y == 0 or x + w >= g.shape[1] or y + h >= g.shape[0]:
            continue                                         # cut by the image edge: centre would be biased
        refined = _refine(ratio, (cx, cy), (A, B), thresh)
        if refined is None:
            continue
        (cx, cy), (A, B), ang = refined
        inside = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(inside, [c], -1, 1, -1)
        out.append((cx, cy, A, B, ang, float(ratio[inside > 0].mean())))
    a = np.array(out, float).reshape(-1, 6)
    return Detections(a[:, :2], a[:, 2:4], a[:, 4], a[:, 5], ratio)


def _refine(ratio, c, axes, thresh, n_rays=48):
    """Sub-pixel ellipse: where the ratio image crosses `thresh` along rays from the centre.

    The crossing is searched from the OUTSIDE in (the outermost dark->board transition), so bright
    thread highlights inside a tapped hole -- which a centre-out search would hit first -- don't
    pull the edge inward."""
    cx, cy = c
    rmax = max(axes) * 0.9 + 3
    rs = np.linspace(0, rmax, int(rmax * 4))
    th = np.linspace(0, 2 * np.pi, n_rays, endpoint=False)
    xs = cx + np.outer(np.cos(th), rs)
    ys = cy + np.outer(np.sin(th), rs)
    prof = cv2.remap(ratio, xs.astype(np.float32), ys.astype(np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE)
    pts = []
    for k in range(n_rays):
        p = prof[k]
        # "not board" = dark, or a highlight inside the hole (thread / lit wall); fold highlights onto
        # the dark side so the outermost not-board -> board step is the opening's real rim
        q = np.where(p > BRIGHT, 0.0, p)
        below = np.nonzero(q < thresh)[0]
        if len(below) == 0 or below[-1] >= len(q) - 1:
            continue                                   # never dark, or still dark at rmax: no clean edge
        i = below[-1] + 1                              # first board sample after the outermost hole one
        if p[i - 1] > BRIGHT:
            r = rs[i - 1] + 0.5 * (rs[i] - rs[i - 1])  # highlight -> board: no gradient to interpolate
            pts.append((cx + r * np.cos(th[k]), cy + r * np.sin(th[k])))
            continue
        f = (thresh - p[i - 1]) / (p[i] - p[i - 1])
        r = rs[i - 1] + f * (rs[i] - rs[i - 1])
        pts.append((cx + r * np.cos(th[k]), cy + r * np.sin(th[k])))
    if len(pts) < n_rays * 0.75:
        return None
    pts = np.array(pts, np.float32)
    e = cv2.fitEllipse(pts)
    d = np.hypot(*(np.array(e[0]) - c))
    return e if d < max(axes) * 0.25 else None


# ----------------------------------------------------------------------------------------- 2. lattice
class GridModel:
    """Lattice (i, j) -> pixel through a real lens: a homography to the ideal (pinhole) image, then
    radial distortion (k1, k2, OpenCV's model) about the image centre.  A homography alone keeps rows of
    holes straight; a real wide lens bows them by several px at the edges, which is what made a plain
    homography reject correct holes on real frames.  f only sets the units of k1, k2 (any value near the
    true focal length in px will do; the default is 0.55 x the larger image side, ~a 6 mm lens here)."""

    def __init__(self, Hm, k=(0.0, 0.0), centre=(0.0, 0.0), f=1000.0):
        self.H = np.asarray(Hm, float) / Hm[2, 2]
        self.k = np.asarray(k, float)
        self.centre = np.asarray(centre, float)
        self.f = float(f)

    def ideal(self, ij):
        return cv2.perspectiveTransform(np.asarray(ij, np.float64).reshape(1, -1, 2), self.H)[0]

    def __call__(self, ij):
        x = (self.ideal(ij) - self.centre) / self.f
        r2 = (x * x).sum(1, keepdims=True)
        return self.centre + self.f * x * (1 + self.k[0] * r2 + self.k[1] * r2 * r2)

    def jacobian(self, ij, h=1e-3):
        """(N, 2, 2) d(pixel)/d(i, j), numerically."""
        ij = np.asarray(ij, float)
        p0 = self(ij)
        return np.stack([(self(ij + [h, 0]) - p0) / h, (self(ij + [0, h]) - p0) / h], -1)

    def corner_shift(self, size):
        """Distortion at the image corner, px (how far the lens moves a corner pixel)."""
        x = (np.asarray(size, float) - self.centre) / self.f
        r2 = (x * x).sum()
        return float(self.f * np.sqrt(r2) * abs(self.k[0] * r2 + self.k[1] * r2 * r2))


def fit_grid_model(ij, uv, centre, f, H0=None, k0=(0.0, 0.0), fit_k=True):
    """Robust least squares of GridModel to labelled holes.  H0: starting homography (default: a RANSAC
    homography).  soft-L1 loss so a few mislabelled holes don't bend the fit."""
    from scipy.optimize import least_squares
    ij, uv = np.asarray(ij, float), np.asarray(uv, float)
    if H0 is None:
        H0, _ = cv2.findHomography(ij, uv, cv2.RANSAC, 0.3 * _pitch_px(uv, ij))
        if H0 is None:
            return None
    H0 = H0 / H0[2, 2]
    x0 = np.r_[H0.ravel()[:8], k0 if fit_k else []]

    def build(x):
        Hm = np.r_[x[:8], 1.0].reshape(3, 3)
        return GridModel(Hm, x[8:10] if fit_k else k0, centre, f)

    def res(x):
        return (build(x)(ij) - uv).ravel()

    r = least_squares(res, x0, loss="soft_l1", f_scale=1.0, x_scale="jac", max_nfev=200)
    return build(r.x)


@dataclass
class Lattice:
    ij: np.ndarray                 # (M, 2) relative lattice indices
    uv: np.ndarray                 # (M, 2) px
    det: np.ndarray                # (M,) index into the detections
    H: np.ndarray                  # lattice (i, j, 1) -> ideal-pixel homography (model.H)
    rms: float                     # grid-model residual over the lattice, px
    model: GridModel = None        # lattice -> pixel including lens distortion
    expected: np.ndarray = None    # (K, 2) cells the model puts in view with no detection on them


def grow_lattice(pts, tol=0.28, image_size=None, shapes=None):
    """Assign relative (i, j) to detections that sit on one regular grid. shapes: (axes, angles) of
    each detection's ellipse, used to drop holes that are partly hidden (e.g. behind the i1 chain)."""
    n = len(pts)
    if n < 6:
        return None
    tree = cKDTree(pts)
    centre = np.array(image_size, float) / 2 if image_size is not None else pts.mean(0)
    d, nb = tree.query(pts, k=min(7, n))
    order = np.argsort(np.linalg.norm(pts - centre, axis=1))
    seed, basis = None, None
    for s in order[:60]:
        # seed: a central detection whose 4 nearest neighbours are a full "+" -- two opposite pairs.
        # (A missing neighbour, e.g. hidden by the i1 cradle, would otherwise let a diagonal in as a
        # basis vector and shear every label.)
        v4 = pts[nb[s, 1:5]] - pts[s]
        pairs = [(p, q) for p in range(4) for q in range(p + 1, 4)
                 if np.linalg.norm(v4[p] + v4[q]) < tol * min(np.linalg.norm(v4[p]), np.linalg.norm(v4[q]))]
        if len(pairs) != 2 or len({*pairs[0], *pairs[1]}) != 4:
            continue
        a, b = v4[pairs[0][0]], v4[pairs[1][0]]
        if abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b)) < 0.5:
            seed, basis = s, (a, b)
            break
    if seed is None:
        return None

    idx = {seed: (0, 0)}
    at = {(0, 0): seed}
    queue = [seed]
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    while queue:
        s = queue.pop(0)
        i, j = idx[s]
        p = pts[s]
        for di, dj in steps:
            key = (i + di, j + dj)
            if key in at:
                continue
            prev = at.get((i - di, j - dj))                   # linear extrapolation along this direction
            if prev is not None:
                step = p - pts[prev]
            else:
                step = di * basis[0] + dj * basis[1]
                # use a local basis vector from any labelled neighbour along the same axis
                for (a_i, a_j), sign in (((i + dj, j + di), 1), ((i - dj, j - di), 1)):
                    q, r = at.get((a_i, a_j)), at.get((a_i - di, a_j - dj))
                    if q is not None and r is not None:
                        step = pts[q] - pts[r]
                        break
            pred = p + step
            dist, q = tree.query(pred)
            if dist < tol * np.linalg.norm(step) and q not in idx:
                idx[q] = key
                at[key] = q
                queue.append(q)
    if len(idx) < 6:
        return None
    det = np.array(list(idx.keys()))
    ij = np.array([idx[k] for k in det], float)
    return _fit_and_fill(pts, tree, det, ij, tol, image_size, shapes)


def _cells_in_view(model, ij, image_size, margin=3):
    lo, hi = ij.min(0) - margin, ij.max(0) + margin
    gi, gj = np.meshgrid(np.arange(lo[0], hi[0] + 1), np.arange(lo[1], hi[1] + 1), indexing="ij")
    cells = np.stack([gi.ravel(), gj.ravel()], 1).astype(float)
    pred = model(cells)
    if image_size is not None:
        ok = ((pred[:, 0] > -20) & (pred[:, 1] > -20) &
              (pred[:, 0] < image_size[0] + 20) & (pred[:, 1] < image_size[1] + 20))
        cells, pred = cells[ok], pred[ok]
    return cells, pred


def _fit_and_fill(pts, tree, det, ij, tol, image_size, shapes=None, rounds=4, f=None):
    """Grid model (homography + radial lens distortion) over the grown set; re-predict every lattice
    cell in view and snap to detections; refit; repeat."""
    size = np.array(image_size if image_size is not None else pts.max(0) + pts.min(0), float)
    centre, f = size / 2, (f if f else 0.55 * size.max())
    model = None
    for rnd in range(rounds):
        # first round: straight homography only (the grown set may still hold a few bad labels);
        # afterwards the lens distortion is fitted too, starting from the previous model
        model = fit_grid_model(ij, pts[det], centre, f,
                               H0=None if model is None else model.H,
                               k0=(0.0, 0.0) if model is None else model.k, fit_k=rnd > 0)
        if model is None:
            return None
        cells, pred = _cells_in_view(model, ij, image_size)
        J = model.jacobian(cells)
        loc = np.minimum(np.linalg.norm(J[:, :, 0], axis=1), np.linalg.norm(J[:, :, 1], axis=1))
        dist, q = tree.query(pred)
        good = dist < tol * loc
        # one detection per cell, one cell per detection (keep the closest)
        best = {}
        for c, qq, dd in zip(cells[good], q[good], dist[good]):
            if qq not in best or dd < best[qq][1]:
                best[qq] = (c, dd)
        if len(best) < 6:
            return None
        det = np.array(list(best.keys()))
        ij = np.array([best[k][0] for k in det], float)
    model = fit_grid_model(ij, pts[det], centre, f, H0=model.H, k0=model.k)
    if shapes is not None:
        det, ij, _ = _prune_shapes(det, ij, model, shapes)
    if len(det) < 6:
        return None
    model = fit_grid_model(ij, pts[det], centre, f, H0=model.H, k0=model.k)
    res = np.linalg.norm(model(ij) - pts[det], axis=1)
    # cells the model puts inside the image (with a hole's margin) that no detection claimed
    cells, pred = _cells_in_view(model, ij, image_size, margin=6)
    have = {tuple(c) for c in ij.astype(int)}
    inside = ((pred[:, 0] > 15) & (pred[:, 1] > 15) &
              (pred[:, 0] < size[0] - 15) & (pred[:, 1] < size[1] - 15))
    missing = np.array([c for c, ok in zip(cells.astype(int), inside) if ok and tuple(c) not in have]).reshape(-1, 2)
    return Lattice(ij.astype(int), pts[det], det, model.H, float(np.sqrt((res ** 2).mean())), model, missing)


def _ellipse_cov(axes, angle_deg):
    a, b = axes[..., 0] / 2, axes[..., 1] / 2
    t = np.deg2rad(angle_deg)
    c, s = np.cos(t), np.sin(t)
    R = np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)
    D = np.zeros(R.shape)
    D[..., 0, 0], D[..., 1, 1] = a * a, b * b
    return R @ D @ np.swapaxes(R, -1, -2)


def _prune_shapes(det, ij, model, shapes, max_dev=0.25, radius=2.01):
    """Drop detections whose ellipse doesn't match their lattice neighbours' -- partly hidden behind the
    i1 chain, merged with a shadow, cut by an edge.

    Compared LOCALLY (against the median ellipse of the matched holes within `radius` cells), not
    against one global "flat circle seen through the grid model": a real hole is a cylinder, and away
    from the optical axis the camera sees down its wall, so the dark ellipse changes shape smoothly across
    the frame (plus defocus and chamfers) in a way a flat-disc model doesn't predict -- on real frames that
    rejected most of the outer holes.  A smooth change passes; a single odd hole still stands out.
    `model` is unused (kept for the call signature)."""
    axes, angles = shapes
    C = _ellipse_cov(axes[det], angles[det])
    tree = cKDTree(ij)
    dev = np.zeros(len(det))
    for n in range(len(det)):
        nb = [m for m in tree.query_ball_point(ij[n], radius) if m != n]
        if len(nb) < 3:
            continue                                     # too isolated to judge: keep
        Cm = np.median(C[nb], axis=0)
        dev[n] = np.linalg.norm(C[n] - Cm) / np.linalg.norm(Cm)
    keep = dev < max_dev
    return det[keep], ij[keep], dev


def _pitch_px(uv, ij):
    tree = cKDTree(ij)
    d, k = tree.query(ij, k=2)
    one = d[:, 1] == 1
    return float(np.median(np.linalg.norm(uv[one] - uv[k[one, 1]], axis=1))) if one.any() else 50.0


# ----------------------------------------------------------------------------------------- 3. index
@dataclass
class CameraModel:
    """Pinhole + distortion camera on the carriage. Pose in the board frame at carriage (0, 0):
    p_cam = R p_board + t; the carriage moves the camera centre along axes ex, ey (board frame, per mm)."""
    K: np.ndarray
    dist: np.ndarray
    R: np.ndarray
    t: np.ndarray
    ex: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0]))
    ey: np.ndarray = field(default_factory=lambda: np.array([0, 1.0, 0]))

    def pose(self, x, y):
        C = -self.R.T @ self.t + x * self.ex + y * self.ey
        return self.R, -self.R @ C

    def to_board(self, uv, x, y):
        """Pixels -> board-plane (Xb, Yb) at carriage (x, y)."""
        R, t = self.pose(x, y)
        n = cv2.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float64), self.K, self.dist).reshape(-1, 2)
        rays = np.c_[n, np.ones(len(n))] @ R                    # board-frame directions (R^T d)
        C = -R.T @ t
        s = -C[2] / rays[:, 2]
        return C[:2] + s[:, None] * rays[:, :2]

    def to_image(self, XY, x, y):
        R, t = self.pose(x, y)
        P = np.c_[XY, np.zeros(len(XY))]
        uv, _ = cv2.projectPoints(P, cv2.Rodrigues(R)[0], t, self.K, self.dist)
        return uv.reshape(-1, 2)


@dataclass
class IndexedHoles:
    IJ: np.ndarray                 # (M, 2) absolute board indices
    uv: np.ndarray                 # (M, 2) px
    agree: float                   # fraction of holes where the prior and the grid agree on the index
    prior_offset_mm: np.ndarray    # mean (board - prior) position, mm: the prior pose error at this frame
    symmetry: int


def index_lattice(lat, cam, x, y, clip=True):
    """clip: drop holes the indexing puts off the board (False keeps them: for priors whose board origin
    is not yet known, scan_map.bootstrap_prior)."""
    prior = cam.to_board(lat.uv, x, y)
    f = (prior - EDGE) / PITCH                                  # fractional board indices
    best = None
    for s, D in enumerate(SYMMETRIES):
        IJ0 = lat.ij @ D.T
        off = np.round(np.median(f - IJ0, axis=0))
        IJ = IJ0 + off
        agree = np.mean(np.all(np.abs(f - IJ) < 0.5, axis=1))
        resid = np.abs(f - IJ).mean()
        if best is None or (agree, -resid) > (best[0], -best[1]):
            best = (agree, resid, s, IJ)
    agree, _, s, IJ = best
    on_board = np.all((IJ >= 0) & (IJ < NHOLE), axis=1) if clip else np.ones(len(IJ), bool)
    off_mm = ((EDGE + PITCH * IJ) - prior).mean(0)
    return IndexedHoles(IJ[on_board].astype(int), lat.uv[on_board], float(agree), off_mm, s)


def find_board_holes(gray, cam, x, y, **kw):
    """The whole filter: image + carriage position -> absolute hole indices and pixels."""
    det = detect_holes(gray, **kw)
    lat = grow_lattice(det.centers, image_size=gray.shape[::-1], shapes=(det.axes, det.angles))
    if lat is None:
        return det, None, None
    return det, lat, index_lattice(lat, cam, x, y)
