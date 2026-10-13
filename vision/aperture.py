"""Where the i1 looks: the measuring aperture's platen position at carriage home (A0), from the camera.

Procedure (after the bare-platen calibration, scan_map.py): take the i1 out of its cradle, slide a white
sheet under the black i1 plate, and take frames at a few carriage positions. Through the plate's
aperture hole the camera sees a white disc on black. Each frame's disc is mapped onto the platen with the
calibrated camera at that carriage position; the carriage model then takes it back to home:
    A(x, y) = A0 + x ex + y ey      ->  rail move to put the aperture on platen point P:
    (x, y) = [ex ey]^-1 (P - A0)
The camera looks ~25 deg off vertical, so the hole's top rim (the plate floor is h thick) hides part of
the disc: what shows is the bottom circle cut by the top circle, which projects onto the paper shifted by
s = h (c - C) / (Cz - h) away from the camera. The visible lens is symmetric between the two, so its
centroid sits s/2 from the true (bottom) centre: corrected here. h and the CAD prior come from the plate.

Run:  python aperture.py scan_aperture --calib scan_bare15/out
  -> scan_aperture/out/: aperture.json (A0, ex, ey, h), aperture_report.txt, aperture_<frame>.png
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2, trimesh
from mosaic import load_calibration

HERE = Path(__file__).parent
GLB = HERE.parent / "cad" / "gantry_on_plate.glb"
WIN, RES = 10.0, 0.02          # local platen window half-size and resolution, mm


def cad_aperture():
    """Aperture centre (board mm) at home and plate floor thickness at the hole, from the CAD plate."""
    s = trimesh.load(GLB)
    parts = []
    for n in s.graph.nodes_geometry:
        T, g = s.graph[n]
        if g.startswith("xrite_i1_plate"):
            m = s.geometry[g].copy()
            m.apply_transform(T)
            parts.append(m)
    m = trimesh.util.concatenate(parts)
    m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, -12.7], [0, 0, 0, 1.0]]))  # glb -> board
    best = None
    for L in m.section([0, 0, 1], [0, 0, 0.2]).discrete:      # the small round loop: the aperture
        x, y = L[:, 0], L[:, 1]
        if np.hypot(x - x.mean(), y - y.mean()).max() > 8:
            continue
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        cr = x * y1 - x1 * y                                  # area centroid: the facets' vertices are uneven
        c = np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (3 * cr.sum())
        r = np.linalg.norm(L[:, :2] - c, axis=1)
        if r.std() < 0.2 and (best is None or r.mean() < best[1]):
            best = (c, r.mean())
    c, r = best
    h = 0.2
    while h < 10:                                             # floor top: where the hole stops being r
        sec = m.section([0, 0, 1], [0, 0, h + 0.05])
        rr = np.linalg.norm(np.vstack([e[:, :2] for e in sec.discrete]) - c, axis=1) if sec is not None else []
        if not len(rr) or rr.min() > r + 1.0:
            break
        h += 0.05
    return c, r, h


# Rail travel the i1 can use, in the scans' carriage coordinates (here mm from mid-stroke, as simulated):
# below X -133 the plate leaves the platen's -X edge; outside Y the plate meets the rail bases
# (cad/gantry_on_plate.py, cad/gantry_reach.py). On the real gantry, record carriage positions as the Qt
# app's slider mm and set these to that app's soft limits.
RAIL_X = (-133.0, 155.0)
RAIL_Y = (54.7 - 326.3, 554.9 - 326.3)


def rail_for(P, A0, ex, ey):
    """Rail position (x, y) mm from home that puts the aperture on platen point P (board mm)."""
    return np.linalg.solve(np.c_[ex[:2], ey[:2]], np.asarray(P, float) - A0)


def disc(img, cam, x, y, guess):
    """Visible white disc near the board point guess: centroid (board mm) and the local ortho crop."""
    R, t = cam.pose(x, y)
    g = np.arange(-WIN, WIN + RES / 2, RES)
    gx, gy = np.meshgrid(guess[0] + g, guess[1] + g)
    P = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], 1)
    uv = cv2.projectPoints(P, cv2.Rodrigues(R)[0], t, cam.K, cam.dist)[0].reshape(gx.shape + (2,)).astype(np.float32)
    L = cv2.remap(img.astype(np.float32), uv[..., 0], uv[..., 1], cv2.INTER_LINEAR)
    t_, bw = cv2.threshold(cv2.GaussianBlur(L, (0, 0), 2).astype(np.uint8), 0, 255,
                           cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, lab, st, cen = cv2.connectedComponentsWithStats(bw)
    h, w = bw.shape
    ok = [k for k in range(1, n) if st[k, 0] > 0 and st[k, 1] > 0 and st[k, 0] + st[k, 2] < w
          and st[k, 1] + st[k, 3] < h and st[k, 4] * RES ** 2 > 20]    # enclosed by black, > 20 mm^2
    if not ok:
        return None, L
    k = min(ok, key=lambda k: np.hypot(*(cen[k] - [w / 2, h / 2])))
    reg = cv2.dilate((lab == k).astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    black = np.median(L[~reg])
    white = np.percentile(L[lab == k], 90)
    wgt = np.clip((L - black) / (white - black), 0, 1) * reg     # soft edge: sub-pixel centroid
    cy, cx = (np.indices(L.shape) * wgt).reshape(2, -1).sum(1) / wgt.sum()
    return guess - WIN + np.array([cx, cy]) * RES, L


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", nargs="?", default="scan_aperture")
    ap.add_argument("--calib", default="scan_bare15/out")
    a = ap.parse_args()
    folder = HERE / a.scan
    out = folder / "out"
    out.mkdir(exist_ok=True)
    cam, _, _, _ = load_calibration(HERE / a.calib)
    A_cad, r_cad, h = cad_aperture()
    C0 = -cam.R.T @ cam.t
    Ex, Ey = cam.ex[:2], cam.ey[:2]
    lines = []
    log = lambda s="": (print(s), lines.append(s))
    log(f"CAD: aperture at home {A_cad.round(2)}, radius {r_cad:.2f} mm, plate floor {h:.2f} mm thick")
    d = json.load(open(folder / "scan.json"))
    est, raw = [], []
    for f in d["frames"]:
        x, y = f["carriage_mm"]
        img = cv2.imread(str(folder / f["file"]), 0)
        guess = A_cad + x * Ex + y * Ey
        c_vis, L = disc(img, cam, x, y, guess)
        if c_vis is None:
            log(f"  {f['file']}: no disc found")
            continue
        C = C0 + x * cam.ex + y * cam.ey                       # camera centre, board frame
        c = c_vis.copy()
        for _ in range(3):                                     # undo the rim's half shift
            s = h * (c - C[:2]) / (C[2] - h)
            c = c_vis - s / 2
        A0 = c - x * Ex - y * Ey                               # back to home, board mm
        A0_raw = c_vis - x * Ex - y * Ey
        est.append(A0); raw.append(A0_raw)
        log(f"  {f['file']}  carriage ({x:+4.0f}, {y:+4.0f}):  disc seen at {c_vis.round(3)}, "
            f"rim shift {np.linalg.norm(s):.3f} mm -> A0 {A0.round(3)}")
        vis = cv2.cvtColor(np.clip(L, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        q = lambda p, col: cv2.drawMarker(vis, tuple(int(v) for v in np.round((p - guess + WIN) / RES)), col,
                                          cv2.MARKER_CROSS, 40, 2)
        q(c_vis, (0, 200, 255)); q(c, (0, 0, 255))
        cv2.circle(vis, tuple(int(v) for v in np.round((c - guess + WIN) / RES)), int(round(r_cad / RES)),
                   (0, 0, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(out / f"aperture_{f['file']}"), vis)
    est, raw = np.array(est), np.array(raw)
    A0 = est.mean(0)
    log(f"A0 = ({A0[0]:.3f}, {A0[1]:.3f}) mm from {len(est)} frames; spread RMS "
        f"{np.sqrt(((est - A0) ** 2).sum(1).mean()):.3f} mm")
    log(f"  vs CAD: off by {(A0 - A_cad).round(3)} mm ({np.linalg.norm(A0 - A_cad):.3f}); "
        f"without the rim correction {(raw.mean(0) - A_cad).round(3)} mm")
    json.dump({"A0_mm": A0.tolist(), "ex": cam.ex.tolist(), "ey": cam.ey.tolist(), "floor_mm": h,
               "radius_mm": r_cad, "frames": len(est)}, open(out / "aperture.json", "w"), indent=1)
    (out / "aperture_report.txt").write_text("\n".join(lines) + "\n")
