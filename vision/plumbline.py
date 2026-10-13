"""Lens radial distortion from straightness alone (plumb-line calibration) on the platen's hole rows.

The holes lie on straight lines on the platen: rows, columns and both diagonals. A pinhole camera keeps
straight lines straight, so whatever bends them in the image is lens distortion. Here k1, k2 (OpenCV's
radial model) are chosen so that, once each frame's detected holes are undistorted, every line of 4+ holes
is as straight as possible. No hole spacing, camera pose or carriage position is used: only which holes
share a line (the lattice indices from scan_map.py) and a nominal focal length.

The focal length only sets the units: distortion seen in pixels is k1 r^3 / f^2 + ..., so a wrong nominal f
changes k1, k2 but not the correction itself. The result is therefore also given as the pixel shift at the
image corner, and with a synthetic scan compared with the true lens.

Run:  python plumbline.py scan_bare15 [--f-mm 6.0] [--centre]
  -> scan/out/: plumbline.json (K, dist), plumbline_report.txt,
     plumbline_lines.png (one frame: holes and fitted lines, raw vs corrected, bow exaggerated x20)
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2
from scipy.optimize import least_squares

HERE = Path(__file__).parent
PIX = 3.45e-3
W, H = 1920, 1200


def lines_of(holes, min_pts=4):
    """Index arrays of the holes on each lattice line: rows, columns, diagonals."""
    IJ = np.array([h[:2] for h in holes])
    out = []
    for key in (IJ[:, 0], IJ[:, 1], IJ[:, 0] - IJ[:, 1], IJ[:, 0] + IJ[:, 1]):
        for k in np.unique(key):
            idx = np.nonzero(key == k)[0]
            if len(idx) >= min_pts:
                out.append(idx)
    return out


def undistort(uv, K, dist):
    n = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, dist,
                            criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-10)).reshape(-1, 2)
    return n * [K[0, 0], K[1, 1]] + K[:2, 2]                     # back to pixels, distortion-free


def off_line(p):
    """Perpendicular distances of points p from their total-least-squares line."""
    c = p - p.mean(0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    return c @ vt[1]


def model(params, f, centre):
    k1, k2 = params[:2]
    cx, cy = params[2:4] if centre else (W / 2, H / 2)
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]]), np.array([k1, k2, 0, 0, 0.0])


def residuals(params, frames, f, centre):
    K, dist = model(params, f, centre)
    r = []
    for uv, lines in frames:
        q = undistort(uv, K, dist)
        r.extend(off_line(q[idx]) for idx in lines)
    return np.concatenate(r)


def corner_shift(K, dist):
    """Pixel shift distortion causes at the image corner (distorted - ideal), magnitude."""
    q = undistort(np.array([[0.0, 0.0]]), K, dist)
    return float(np.hypot(*(q[0] - [0, 0])))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", nargs="?", default="scan_bare15")
    ap.add_argument("--f-mm", type=float, default=6.0, help="nominal focal length (sets the units of k1, k2)")
    ap.add_argument("--centre", action="store_true", help="also fit the distortion centre")
    a = ap.parse_args()
    folder = HERE / a.scan
    out = folder / "out"
    f = a.f_mm / PIX
    holes = json.load(open(out / "holes.json"))
    frames = [(np.array([h[2:] for h in fr["holes"]], np.float64), lines_of(fr["holes"])) for fr in holes]
    nlines = sum(len(l) for _, l in frames)
    lines = []
    log = lambda s="": (print(s), lines.append(s))

    p0 = np.array([0.0, 0.0] + ([W / 2, H / 2] if a.centre else []))
    r0 = residuals(p0, frames, f, a.centre)
    fit = least_squares(residuals, p0, args=(frames, f, a.centre), x_scale=[0.05, 0.05] + [50, 50] * a.centre)
    K, dist = model(fit.x, f, a.centre)
    r1 = fit.fun
    rms = lambda r: np.sqrt(np.mean(r ** 2))
    log(f"{sum(len(u) for u, _ in frames)} holes on {nlines} lines (rows, columns, diagonals) in {len(frames)} frames; "
        f"nominal f {a.f_mm} mm")
    log(f"off-line distance: uncorrected RMS {rms(r0):.3f} px (max {np.abs(r0).max():.2f}); "
        f"corrected RMS {rms(r1):.3f} px (max {np.abs(r1).max():.2f})")
    log(f"k1 {dist[0]:+.4f}, k2 {dist[1]:+.4f}" + (f", centre ({K[0, 2]:.1f}, {K[1, 2]:.1f})" if a.centre else
                                                   ", centre fixed at the image centre"))
    log(f"distortion at the image corner: {corner_shift(K, dist):.2f} px")

    scan = json.load(open(folder / "scan.json"))
    if "true_K" in scan:
        Kt, dt = np.array(scan["true_K"]), np.array(scan["true_dist"])
        log(f"truth: k1 {dt[0]:+.4f}, k2 {dt[1]:+.4f} at f {Kt[0, 0] * PIX:.3f} mm; corner {corner_shift(Kt, dt):.2f} px")
        gy, gx = np.mgrid[0:H:20, 0:W:20]
        g = np.c_[gx.ravel(), gy.ravel()].astype(np.float64)
        e = np.hypot(*(undistort(g, K, dist) - undistort(g, Kt, dt)).T)
        # the two corrections may differ by a scale about the centre (that is what f absorbs): remove it
        a_, b_ = undistort(g, K, dist) - [W / 2, H / 2], undistort(g, Kt, dt) - [W / 2, H / 2]
        s = (a_ * b_).sum() / (a_ * a_).sum()
        e2 = np.hypot(*(s * a_ - b_).T)
        log(f"  corrected image vs the true lens: max difference {e2.max():.3f} px, RMS {np.sqrt((e2 ** 2).mean()):.3f} px "
            f"(after a {abs(s - 1) * 100:.2f}% scale, which the focal length takes up)")

    json.dump({"K": K.tolist(), "dist": dist.tolist(), "f_mm_nominal": a.f_mm,
               "rms_px_before": rms(r0), "rms_px_after": rms(r1)}, open(out / "plumbline.json", "w"), indent=1)
    (out / "plumbline_report.txt").write_text("\n".join(lines) + "\n")

    # picture: the busiest frame, raw vs corrected, lines fitted through the holes, bow exaggerated x20
    k = max(range(len(frames)), key=lambda i: len(frames[i][0]))
    fr = holes[k]
    img = cv2.imread(str(folder / fr["file"]), 0)
    uv, lns = frames[k]
    panels = []
    for name, pts, im in (("uncorrected", uv, img),
                          ("corrected", undistort(uv, K, dist), cv2.undistort(img, K, dist))):
        vis = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        IJ = np.array([h[:2] for h in fr["holes"]])
        for idx in lns:
            if len(np.unique(IJ[idx, 0])) > 1 and len(np.unique(IJ[idx, 1])) > 1:
                continue                                          # draw rows and columns only
            p = pts[idx]
            c = p.mean(0)
            _, _, vt = np.linalg.svd(p - c)
            t = (p - c) @ vt[0]
            a0, a1 = c + t.min() * vt[0], c + t.max() * vt[0]
            cv2.line(vis, tuple(np.int32(a0)), tuple(np.int32(a1)), (0, 200, 0), 1, cv2.LINE_AA)
            d = (p - c) @ vt[1]
            for q, dd in zip(p, d):                               # off-line distance, x20, in red
                e = q + 20 * dd * vt[1] - dd * vt[1]
                cv2.line(vis, tuple(np.int32(q - dd * vt[1])), tuple(np.int32(e)), (0, 0, 255), 2, cv2.LINE_AA)
        for q in pts:
            cv2.circle(vis, tuple(np.int32(q)), 4, (0, 220, 255), 1, cv2.LINE_AA)
        r = np.concatenate([off_line(pts[i]) for i in lns])
        cv2.putText(vis, f"{name}: off-line RMS {rms(r):.2f} px (red = x20)", (30, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    1.4, (0, 0, 255), 3, cv2.LINE_AA)
        panels.append(vis)
    cv2.imwrite(str(out / "plumbline_lines.png"), np.vstack(panels))
