"""Scan -> holes -> camera calibration -> board map, with the carriage position as the only pose input.

For each frame (image + carriage (x, y) from the FUYU controller):
  1. holegrid: detect holes, fit the grid (detect_holes, grow_lattice), label every hole with its absolute
     board index (I, J) using a nominal camera model (only has to be good to ~half a pitch). A synthetic
     scan's nominal camera is its CAD pose, deliberately put off; a real scan's comes from the scan itself
     (bootstrap_prior), since the printed bracket and the slider's zero are not known to half a pitch.
Over the whole scan:
  2. Self-calibration: the board is a 24 x 24 grid of known points, so the labelled holes from every
     frame calibrate the camera (focal length, distortion) and give each frame's pose. All frames share
     one camera orientation (the carriage only translates), so the principal point is held at the image
     centre -- with pure translation it cannot be told apart from a small camera tilt.
  3. Carriage model: the per-frame camera centres are fitted as C0 + x*ex + y*ey, which measures how the
     gantry's X and Y axes actually lie on the board (direction, scale, squareness). With it, any pixel
     at any carriage position maps to board millimetres from the controller position alone.
  4. Hole matching across positions: the same (I, J) seen from different frames, back-projected
     from the carriage position alone, should land on the same board point; the spread is reported.
  5. Board map: every frame is re-projected onto the board plane (ortho, MAP_MM per px), the camera's
     own carriage-fixed clutter (i1 cradle and chain, which sit at the same pixels in every frame) is
     masked out, and frames are blended favouring the closest (sharpest) view.

Real scans (grab.py): carriage positions are the Qt app's slider mm. The board frame's X and Y are taken
to increase with the slider's x and y (homed at the motor ends); if an axis was homed the other way, pass
--axes -x,+y (etc.). Check on the live view (grab.py --live): jogging +x moves the camera away from where it
looks, so the board slides up the image; jogging +y slides it left. Step the carriage by amounts that are not
multiples of 12.7 mm (half a pitch): a wrong sign then shows as a scale error in step 3 rather than not at all.

Run:  python scan_map.py [scan] [--prior-error 1.0] [--bootstrap] [--axes +x,+y] [--first-hole I,J]
  -> scan/out/: report.txt, holes.json, det_<frame>.png, mask.png, board_map.png, board_map_grid.png,
     and for later scans at the same positions (mosaic.py): calibration.json, mask.png, flat.npy
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2
from scipy.spatial import cKDTree
import holegrid as hg
from holegrid import PITCH, EDGE, NHOLE
from camera import R_CAD, F_MM, nominal_K

MAP_MM = 0.25
BOARD = 609.6
# hole diameters (px) detect_holes' defaults were tuned for, at the focal length (px) they were tuned at:
# the 6 mm lens on 3.45 um pixels. Hole size scales with the focal length in pixels.
HOLE_D_PX, HOLE_D_F = (10.0, 70.0), 6.0 / 3.45e-3
CALIB_FLAGS = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
               | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)


def rot(axis, deg):
    return cv2.Rodrigues(np.asarray(axis, float) * np.deg2rad(deg))[0]


def nominal_camera(d, err=1.0):
    """The camera as CAD would give it -- deliberately off, as a real build would be: ~1 deg of tilt
    and roll, a few mm of position, 3 % in focal length, and no idea of the lens distortion.
    err scales all of it (0 = the exact camera the frames were rendered with)."""
    K = np.array(d["true_K"])
    K[0, 0] *= 1 + 0.03 * err
    K[1, 1] *= 1 + 0.03 * err
    R0, t0 = np.array(d["camera_at_home"]["R"]), np.array(d["camera_at_home"]["t"])
    C0 = -R0.T @ t0
    R = rot([1, 0, 0], 0.8 * err) @ rot([0, 0, 1], 0.6 * err) @ R0
    C = C0 + err * np.array([3.0, -3.0, 4.0])
    return hg.CameraModel(K, np.zeros(5), R, -R @ C)


def bootstrap_prior(frames, lats, K0, size, axes=(1, 1), first_hole=None):
    """Nominal camera for a real scan, from the scan itself: no CAD pose is needed beyond which way the
    camera faces (camera.R_CAD, to 45 deg).
    1. The frame with the most holes calibrates the camera against its own lattice: K, dist and its pose
       in lattice coordinates (indices relative to an arbitrary hole, in one of the lattice's 8 symmetries).
    2. Of the symmetries that leave the camera above a right-handed board, the one whose camera
       orientation is closest to the bracket's sets the board axes.
    3. Every frame is indexed with that camera moved by the carriage (axes: signs of the slider's x, y
       along board X, Y; the slider mm are taken at face value, so a frame indexes right if the move is
       good to half a pitch).
    4. The board origin: when the scan reaches the board's edges the holes seen span it, so the lowest
       index is hole 0. Otherwise first_hole gives the board index of the lowest hole seen.
    Returns the prior camera, the indexed holes per frame, and notes for the report."""
    r = max(range(len(lats)), key=lambda k: len(lats[k].ij))
    lat, (xr, yr) = lats[r], frames[r]["carriage_mm"]
    obj = np.c_[PITCH * lat.ij, np.zeros(len(lat.ij))].astype(np.float32)
    rms, K, dist, rv, tv = cv2.calibrateCamera([obj], [lat.uv.astype(np.float32)], size, K0.copy(), np.zeros(5),
                                               flags=CALIB_FLAGS)
    RL = cv2.Rodrigues(rv[0])[0]
    CL = -RL.T @ tv[0].ravel()
    ex, ey = np.array([axes[0], 0.0, 0.0]), np.array([0.0, axes[1], 0.0])
    cands = []
    for D in hg.SYMMETRIES:
        det = round(np.linalg.det(D))
        if det * CL[2] <= 0:
            continue
        A = np.eye(3)
        A[:2, :2], A[2, 2] = D, det
        Rb = RL @ A.T                                 # board point P = A p_lattice + (EDGE, EDGE, 0)
        off_cad = np.rad2deg(np.linalg.norm(cv2.Rodrigues(Rb @ R_CAD.T)[0]))
        cands.append((off_cad, Rb, A @ CL + [EDGE, EDGE, 0]))
    off_cad, Rb, Cb = min(cands, key=lambda c: c[0])
    C0 = Cb - xr * ex - yr * ey
    cam = hg.CameraModel(K, dist, Rb, -Rb @ C0, ex, ey)
    idxs = [hg.index_lattice(l, cam, *f["carriage_mm"], clip=False) for l, f in zip(lats, frames)]
    allIJ = np.concatenate([i.IJ for i in idxs])
    lo, hi = allIJ.min(0), allIJ.max(0)
    span = hi - lo + 1
    notes = [f"  reference frame {frames[r]['file']}: {len(lat.ij)} holes, lattice calibration RMS {rms:.2f} px, "
             f"focal {K[0, 0]:.0f} px, k1 {dist[0]:+.3f} k2 {dist[1]:+.3f}",
             f"  board orientation from the bracket: camera {off_cad:.1f} deg from the CAD orientation"
             + ("  ** over 20 deg: is the camera rolled on its bracket? **" if off_cad > 20 else ""),
             f"  holes seen span {span[0]} x {span[1]} of {NHOLE} x {NHOLE}"]
    if first_hole is not None:
        lo = lo - np.array(first_hole)
    elif np.any(span < NHOLE):
        notes.append("  ** the scan does not reach both board edges along "
                     + " and ".join(n for n, s_ in zip("XY", span) if s_ < NHOLE)
                     + ": the lowest hole seen is taken as hole 0 (override with --first-hole) **")
    if np.any(span > NHOLE):
        notes.append(f"  ** the holes span more than the board: frames are mis-indexed -- check the slider "
                     f"positions and --axes **")
    for i in idxs:
        i.IJ = i.IJ - lo
        keep = np.all((i.IJ >= 0) & (i.IJ < NHOLE), axis=1)
        i.IJ, i.uv = i.IJ[keep], i.uv[keep]
    C0 = C0 - np.r_[PITCH * lo, 0]
    return hg.CameraModel(K, dist, Rb, -Rb @ C0, ex, ey), idxs, notes


def score(frame, idx):
    """Against the renderer's ground truth: index correctness, centre error, holes found."""
    gt = np.array(frame["true_holes"])
    vis = gt[gt[:, 4] == 1]
    dd, k = cKDTree(gt[:, 2:4]).query(idx.uv)
    ok = np.all(gt[k, :2] == idx.IJ, 1) & (dd < 5)
    found = cKDTree(idx.uv).query(vis[:, 2:4])[0] < 5
    return dict(n=len(idx.IJ), index_ok=float(ok.mean()), found=float(found.mean()), n_visible=len(vis),
                err_med=float(np.median(dd)), err_p95=float(np.percentile(dd, 95)), err_max=float(dd.max()),
                bias=(idx.uv - gt[k, 2:4]).mean(0).tolist())


def draw_fiducials(g, det, idx, f):
    """Fitted hole ellipses (green = indexed board hole, red = rejected blob), centre crosses, (I, J)."""
    img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    used = set()
    near = cKDTree(det.centers)
    for (I, J), (u, v) in zip(idx.IJ, idx.uv):
        k = near.query([u, v])[1]
        used.add(k)
        cv2.ellipse(img, ((float(u), float(v)), tuple(det.axes[k] + 6), float(det.angles[k])), (0, 220, 0), 2, cv2.LINE_AA)
        cv2.drawMarker(img, (int(round(u)), int(round(v))), (0, 255, 255), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
        cv2.putText(img, f"{I},{J}", (int(u) + 10, int(v) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, f"{I},{J}", (int(u) + 10, int(v) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    for k, c in enumerate(det.centers):
        if k not in used:
            cv2.ellipse(img, ((float(c[0]), float(c[1])), tuple(det.axes[k] + 6), float(det.angles[k])), (0, 0, 255), 2, cv2.LINE_AA)
    label = f"carriage x {f['carriage_mm'][0]:+.0f} mm, y {f['carriage_mm'][1]:+.0f} mm   {len(idx.IJ)} holes indexed"
    cv2.rectangle(img, (0, 0), (620, 34), (0, 0, 0), -1)
    cv2.putText(img, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def calibrate(frames, holes, K0, size):
    obj = [np.c_[EDGE + PITCH * h.IJ, np.zeros(len(h.IJ))].astype(np.float32) for h in holes]
    img = [h.uv.astype(np.float32) for h in holes]
    rms, K, dist, rv, tv = cv2.calibrateCamera(obj, img, size, K0.copy(), np.zeros(5), flags=CALIB_FLAGS)
    Rs = [cv2.Rodrigues(r)[0] for r in rv]
    ts = [t.ravel() for t in tv]
    return rms, K, dist.ravel(), Rs, ts


def carriage_model(frames, Rs, ts):
    """Fit C_k = C0 + x_k ex + y_k ey over the frames; one mean rotation."""
    C = np.array([-R.T @ t for R, t in zip(Rs, ts)])
    xy = np.array([f["carriage_mm"] for f in frames])
    A = np.c_[np.ones(len(xy)), xy]
    sol, *_ = np.linalg.lstsq(A, C, rcond=None)
    C0, ex, ey = sol
    resid = C - A @ sol
    U, _, Vt = np.linalg.svd(sum(Rs))                          # chordal mean rotation
    Rm = U @ Vt
    spread = max(np.rad2deg(np.linalg.norm(cv2.Rodrigues(R @ Rm.T)[0])) for R in Rs)
    return Rm, C0, ex, ey, resid, spread


def background(g):
    """The board's brightness with the holes closed over: illumination x vignetting x board finish."""
    g = g.astype(np.float32)
    bg = cv2.morphologyEx(g, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (99, 99)))
    return cv2.GaussianBlur(bg, (0, 0), 16)


def flat(g):
    """Divide out illumination and vignetting locally (fine for the bare board, which is uniform)."""
    return np.clip(g.astype(np.float32) / np.maximum(background(g), 1) * 170, 0, 255)


def carriage_mask(frames, grays, cam, sub=2, thresh=8.0):
    """Pixels on the carriage (i1 cradle, chain): they stay at the same pixels from frame to frame,
    while the board stays put only after warping one frame onto another through the board plane.
    So a pixel is carriage if frames agree there unwarped but disagree once warped. (Plain board
    agrees both ways and is kept.) A one-off calibration product: the mask is the same for every scan."""
    h, w = grays[0].shape
    gy, gx = np.mgrid[0:h:sub, 0:w:sub].astype(np.float32)
    uv = np.stack([gx.ravel(), gy.ravel()], 1)
    sm = [cv2.GaussianBlur(flat(g), (0, 0), 1.2) for g in grays]
    small = [s[::sub, ::sub][:gx.shape[0], :gx.shape[1]] for s in sm]
    # unwarped, the carriage looks the same in *every* frame; board spots don't (holes pass through them)
    du_all = np.stack([np.abs(small[k] - small[l]) for k in range(len(sm)) for l in range(k + 1, len(sm))])
    static = np.percentile(du_all, 90, axis=0) < thresh
    votes = np.zeros(gx.shape, np.float32)
    count = np.zeros(gx.shape, np.float32)
    for k, fk in enumerate(frames):
        XY = cam.to_board(uv, *fk["carriage_mm"])
        for l, fl in enumerate(frames):
            if l == k:
                continue
            p = cam.to_image(XY, *fl["carriage_mm"]).astype(np.float32)
            ok = ((p[:, 0] > 0) & (p[:, 1] > 0) & (p[:, 0] < w - 1) & (p[:, 1] < h - 1)).reshape(gx.shape)
            Iw = cv2.remap(sm[l], p[:, 0].reshape(gx.shape), p[:, 1].reshape(gx.shape), cv2.INTER_LINEAR)
            votes += ok * (np.abs(small[k] - Iw) > thresh)        # ...but disagrees once warped by the board
            count += ok
    frac = votes / np.maximum(count, 1)
    m = (static & (frac > 0.2)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    keep = np.zeros_like(m)
    big = stats[1:, cv2.CC_STAT_AREA].max() if n > 1 else 0
    for c in range(1, n):                                           # the carriage is one body; ghosts are crumbs
        if stats[c, cv2.CC_STAT_AREA] > 0.2 * big:
            keep[lab == c] = 1
    contours, _ = cv2.findContours(keep, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(keep, contours, -1, 1, -1)                     # fill the cradle's outline
    keep = cv2.resize(keep, (w, h), interpolation=cv2.INTER_NEAREST)
    return cv2.dilate(keep, np.ones((21, 21), np.uint8)).astype(bool)


def board_map(frames, grays, cam, mask, Rs, ts, prep=flat):
    n = int(round(BOARD / MAP_MM))
    gy, gx = np.mgrid[0:n, 0:n].astype(np.float32)
    XY = np.stack([(gx.ravel() + 0.5) * MAP_MM, (gy.ravel() + 0.5) * MAP_MM], 1)
    P = np.c_[XY, np.zeros(len(XY))]
    acc = np.zeros(n * n, np.float32)
    wsum = np.zeros(n * n, np.float32)
    h, w = grays[0].shape
    for g, R, t in zip(grays, Rs, ts):
        uv, _ = cv2.projectPoints(P, cv2.Rodrigues(R)[0], t, cam.K, cam.dist)
        uv = uv.reshape(-1, 2).astype(np.float32)
        val = cv2.remap(prep(g), uv[:, 0].reshape(n, n), uv[:, 1].reshape(n, n),
                        cv2.INTER_LINEAR, borderValue=0).ravel()
        ui, vi = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        inside = (ui >= 0) & (vi >= 0) & (ui < w) & (vi < h)
        ok = inside.copy()
        ok[inside] = ~mask[vi[inside], ui[inside]]
        depth = (P @ R.T + t)[:, 2]
        edge = np.minimum.reduce([uv[:, 0], uv[:, 1], w - uv[:, 0], h - uv[:, 1]])
        wt = np.where(ok, np.clip(edge / 60, 0, 1) * (300 / depth) ** 6, 0)   # closest view dominates, feathered
        acc += wt * val
        wsum += wt
    out = np.where(wsum > 0, acc / np.maximum(wsum, 1e-9), 0).reshape(n, n)
    return np.clip(out, 0, 255).astype(np.uint8), (wsum > 0).reshape(n, n)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", nargs="?", default="scan")
    ap.add_argument("--prior-error", type=float, default=1.0, help="scale of the nominal camera's error (synthetic scans)")
    ap.add_argument("--bootstrap", action="store_true", help="prior from the scan itself, even for a synthetic scan")
    ap.add_argument("--axes", default="+x,+y", help="signs of the slider's x, y along board X, Y (real scans)")
    ap.add_argument("--first-hole", help="I,J: board index of the lowest hole seen, when the scan misses a board edge")
    a = ap.parse_args()
    folder = Path(__file__).parent / a.scan
    out = folder / "out"
    out.mkdir(exist_ok=True)
    d = json.load(open(folder / "scan.json"))
    W, H = d["image_size"]
    K0 = nominal_K(d.get("focal_mm", F_MM), d["pixel_mm"], (W, H))
    hole_px = {"min_d": HOLE_D_PX[0] * K0[0, 0] / HOLE_D_F, "max_d": HOLE_D_PX[1] * K0[0, 0] / HOLE_D_F}
    lines = []
    log = lambda s="": (print(s), lines.append(s))

    # 1. holes in every frame
    grays, dets, lats, frames = [], [], [], []
    for f in d["frames"]:
        g = cv2.imread(str(folder / f["file"]), 0)
        det = hg.detect_holes(g, **hole_px)
        lat = hg.grow_lattice(det.centers, image_size=g.shape[::-1], shapes=(det.axes, det.angles))
        if lat is None:
            log(f"  {f['file']}: no hole grid ({len(det.centers)} hole-like blobs), frame skipped")
            continue
        grays.append(g), dets.append(det), lats.append(lat), frames.append(f)
    if "camera_at_home" in d and not a.bootstrap:
        prior = nominal_camera(d, a.prior_error)
        holes = [hg.index_lattice(l, prior, *f["carriage_mm"]) for l, f in zip(lats, frames)]
        log(f"1. Hole filter, nominal camera prior (error scale {a.prior_error:g})")
    else:
        sx, sy = (-1 if v.strip().startswith("-") else 1 for v in a.axes.split(","))
        first = [int(v) for v in a.first_hole.split(",")] if a.first_hole else None
        prior, holes, notes = bootstrap_prior(frames, lats, K0, (W, H), (sx, sy), first)
        log(f"1. Hole filter (hole diameters {hole_px['min_d']:.0f}..{hole_px['max_d']:.0f} px), "
            f"prior from the scan itself")
        for n in notes:
            log(n)
    for f, g, det, idx in zip(frames, grays, dets, holes):
        s = score(f, idx) if "true_holes" in f else None
        msg = (f"  {f['file']}  carriage ({f['carriage_mm'][0]:+5.0f},{f['carriage_mm'][1]:+5.0f}) mm: "
               f"{len(idx.IJ):3d} holes, prior agrees {idx.agree:.0%}, prior off by "
               f"({idx.prior_offset_mm[0]:+.1f}, {idx.prior_offset_mm[1]:+.1f}) mm")
        if s:
            msg += (f" | truth: index ok {s['index_ok']:.0%}, found {s['found']:.0%} of {s['n_visible']}, "
                    f"centre err median {s['err_med']:.2f} px, p95 {s['err_p95']:.2f}, max {s['err_max']:.2f}")
        log(msg)
        cv2.imwrite(str(out / f"det_{f['file']}"), draw_fiducials(g, det, idx, f))
    json.dump([{"file": f["file"], "carriage_mm": f["carriage_mm"],
                "holes": [[int(I), int(J), float(u), float(v)] for (I, J), (u, v) in zip(h.IJ, h.uv)]}
               for f, h in zip(frames, holes)], open(out / "holes.json", "w"), indent=1)

    # 2. calibration from the board itself
    rms, K, dist, Rs, ts = calibrate(frames, holes, prior.K, (W, H))
    log()
    log(f"2. Self-calibration from {sum(len(h.IJ) for h in holes)} labelled holes in {len(frames)} frames: "
        f"reprojection RMS {rms:.2f} px")
    log(f"  focal {K[0, 0] * d['pixel_mm']:.3f} mm (prior {prior.K[0, 0] * d['pixel_mm']:.3f}), "
        f"k1 {dist[0]:+.4f} k2 {dist[1]:+.4f}")
    if "true_K" in d:
        Kt, Dt = np.array(d["true_K"]), np.array(d["true_dist"])
        log(f"  truth: focal {Kt[0, 0] * d['pixel_mm']:.3f} mm, k1 {Dt[0]:+.4f} k2 {Dt[1]:+.4f}")

    # 3. carriage model
    Rm, C0, ex, ey, resid, spread = carriage_model(frames, Rs, ts)
    cam = hg.CameraModel(K, dist, Rm, -Rm @ C0, ex, ey)
    ang = lambda v, ref: np.rad2deg(np.arctan2(np.linalg.norm(np.cross(v, ref)), v @ ref))
    log()
    log("3. Carriage model (camera centre = C0 + x ex + y ey, board frame)")
    log(f"  camera orientation spread over frames {spread:.3f} deg (one rigid camera: should be ~0)")
    log(f"  ex = ({ex[0]:+.4f}, {ex[1]:+.4f}, {ex[2]:+.4f})  scale {np.linalg.norm(ex):.4f}, {ang(ex, [1, 0, 0]):.3f} deg off board X")
    log(f"  ey = ({ey[0]:+.4f}, {ey[1]:+.4f}, {ey[2]:+.4f})  scale {np.linalg.norm(ey):.4f}, {ang(ey, [0, 1, 0]):.3f} deg off board Y")
    log(f"  X-Y squareness {90 - ang(ex, ey):+.3f} deg; centre fit residual RMS {np.sqrt((resid ** 2).sum(1).mean()):.3f} mm")
    for n, e in (("X", ex), ("Y", ey)):
        # an axis indexed with the wrong sign still fits, each move snapped to whole pitches: only its scale shows it
        if abs(np.linalg.norm(e) - 1) > 0.005:
            log(f"  ** {n} scale is {np.linalg.norm(e):.4f} board mm per slider mm: the slider's {n.lower()} runs the "
                f"other way (--axes), or its mm are off (FuyuRailController/tools/fmc_set_scale.py) **")
    if "camera_at_home" in d:
        Rt, tt = np.array(d["camera_at_home"]["R"]), np.array(d["camera_at_home"]["t"])
        log(f"  truth: camera centre at home {(-Rt.T @ tt).round(2).tolist()}, fitted {C0.round(2).tolist()}")

    # 4. holes across positions, from the carriage position alone
    log()
    log("4. Same hole from different carriage positions (calibrated camera + carriage position only)")
    seen = {}
    for f, h in zip(frames, holes):
        XY = cam.to_board(h.uv, *f["carriage_mm"])
        pred = cam.to_image(EDGE + PITCH * h.IJ.astype(float), *f["carriage_mm"])
        for (I, J), p in zip(h.IJ, XY):
            seen.setdefault((I, J), []).append(p)
        f["_pred_px"] = np.linalg.norm(pred - h.uv, axis=1)
    multi = {k: np.array(v) for k, v in seen.items() if len(v) > 1}
    spread_mm = np.concatenate([np.linalg.norm(v - v.mean(0), axis=1) for v in multi.values()])
    true_err = np.concatenate([np.linalg.norm(v - (EDGE + PITCH * np.array(k)), axis=1) for k, v in seen.items()])
    allpx = np.concatenate([f["_pred_px"] for f in frames])
    log(f"  {len(seen)} distinct holes mapped, {len(multi)} seen from 2+ positions "
        f"(up to {max(len(v) for v in multi.values())})")
    log(f"  spread of one hole across positions: RMS {np.sqrt((spread_mm ** 2).mean()):.3f} mm, max {spread_mm.max():.3f} mm")
    log(f"  board position vs the hole's nominal position: RMS {np.sqrt((true_err ** 2).mean()):.3f} mm, max {true_err.max():.3f} mm")
    log(f"  hole pixel predicted from carriage position alone vs detected: RMS {np.sqrt((allpx ** 2).mean()):.2f} px, max {allpx.max():.2f} px")

    # 5. board map
    mask = carriage_mask(frames, grays, cam)
    cv2.imwrite(str(out / "mask.png"), mask.astype(np.uint8) * 255)
    # flat-field for later scans: the bare board's background, median over the scan (camera-fixed
    # vignetting and the lighting as the camera sees it), normalised to 1 at its median
    ff = np.median(np.stack([background(g) for g in grays]), axis=0)
    np.save(out / "flat.npy", (ff / np.median(ff)).astype(np.float32))
    json.dump({"image_size": [W, H], "pixel_mm": d["pixel_mm"], "reprojection_rms_px": rms,
               "K": K.tolist(), "dist": dist.tolist(), "R": Rm.tolist(), "C0": C0.tolist(),
               "ex": ex.tolist(), "ey": ey.tolist(),
               "carriage_mm": [f["carriage_mm"] for f in frames]},
              open(out / "calibration.json", "w"), indent=1)
    Rmap = [Rm] * len(frames)
    tmap = [cam.pose(*f["carriage_mm"])[1] for f in frames]           # carriage position only
    bmap, covered = board_map(frames, grays, cam, mask, Rmap, tmap)
    cv2.imwrite(str(out / "board_map.png"), bmap)
    grid = cv2.cvtColor(bmap, cv2.COLOR_GRAY2BGR)
    for I in range(NHOLE):                                           # map image: column = Xb, row = Yb
        for J in range(NHOLE):
            c = (int(round((EDGE + PITCH * I) / MAP_MM)), int(round((EDGE + PITCH * J) / MAP_MM)))
            cv2.circle(grid, c, int(round(3.19 / MAP_MM)), (0, 200, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(out / "board_map_grid.png"), grid)
    log()
    log(f"5. Board map {bmap.shape[1]} x {bmap.shape[0]} px at {MAP_MM} mm/px, {covered.mean():.0%} of the board covered; "
        f"carriage mask {mask.mean():.1%} of each frame")
    (out / "report.txt").write_text("\n".join(lines) + "\n")
