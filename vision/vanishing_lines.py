"""Draw the board's hole columns that run away from the camera ("into the distance") on the
lens-corrected frame, and where they meet (the vanishing point).

Holes are detected on the raw frame (linear levelling), labelled with the lattice, then their centres
are moved into the corrected image with the calibration (cv2.undistortPoints). Each column of holes gets
a straight total-least-squares line, extended across the picture; the vanishing point is the point
closest to all of them. With the lens distortion removed the holes sit on straight lines, so the
off-line distances printed are the check.

Run:  python vanishing_lines.py [calibration.json]     (default: the newest vision/calib/*/calibration.json)
  -> snaps/vanishing_lines.png (opened in Chrome)
"""
import json, sys, subprocess
from pathlib import Path
import numpy as np, cv2
from pypylon import pylon
import holegrid as hg
import live_clahe as LV

HERE = Path(__file__).resolve().parent


def grab():
    cam = LV.open_camera()
    cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    try:
        for _ in range(30):
            r = cam.RetrieveResult(5000); raw = r.Array.copy(); r.Release()
            e = cam.ExposureTime.GetValue(); LV.adjust_exposure(cam, raw)
            if cam.ExposureTime.GetValue() == e:
                break
        r = cam.RetrieveResult(5000); raw = r.Array.copy(); r.Release()
    finally:
        cam.StopGrabbing(); cam.Close()
    return raw


def tls_line(p):
    c = p.mean(0)
    _, _, vt = np.linalg.svd(p - c)
    return c, vt[0], (p - c) @ vt[1]          # point, direction, signed off-line distances


def main():
    cal_path = Path(sys.argv[1]) if len(sys.argv) > 1 else sorted((HERE / "calib").glob("*/calibration.json"))[-1]
    cal = json.load(open(cal_path))
    K, dist = np.array(cal["K"]), np.array(cal["dist"])

    raw = grab()
    H, W = raw.shape
    lin8 = np.clip(LV.level_linear(raw) * 240.0, 0, 255).astype(np.uint8)
    det = hg.detect_holes(lin8)
    lat = hg.grow_lattice(det.centers, image_size=(W, H), shapes=(det.axes, det.angles))

    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (W, H), 0)
    und = cv2.undistort(LV.picture(raw), K, dist, None, newK)
    uv = cv2.undistortPoints(lat.uv.reshape(-1, 1, 2).astype(np.float64), K, dist, P=newK).reshape(-1, 2)

    # which lattice index runs into the distance: the one whose step moves mostly vertically in the image
    Jm = lat.model.jacobian(np.zeros((1, 2)))[0]
    axis = 0 if abs(Jm[1, 0]) > abs(Jm[1, 1]) else 1          # step along `axis` = mostly up/down
    key = lat.ij[:, 1 - axis]                                 # holes sharing this index form one column

    lines, offs = [], []
    for k in np.unique(key):
        idx = np.nonzero(key == k)[0]
        if len(idx) < 4:
            continue
        c, d, off = tls_line(uv[idx])
        lines.append((c, d, idx))
        offs.append(off)
    offs = np.concatenate(offs)

    # vanishing point: least-squares point closest to every line
    A, b = np.zeros((2, 2)), np.zeros(2)
    for c, d, _ in lines:
        n = np.array([-d[1], d[0]])
        A += np.outer(n, n); b += np.outer(n, n) @ c
    vp = np.linalg.solve(A, b)
    miss = [abs(np.array([-d[1], d[0]]) @ (vp - c)) for c, d, _ in lines]

    v = cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)
    for n, (c, d, idx) in enumerate(lines):
        col = (0, 200, 255) if n % 2 == 0 else (255, 200, 0)
        # extend each line across the whole picture (top to bottom)
        if abs(d[1]) > 1e-6:
            t0, t1 = (0 - c[1]) / d[1], (H - c[1]) / d[1]
            p0, p1 = c + t0 * d, c + t1 * d
            cv2.line(v, tuple(np.int32(p0)), tuple(np.int32(p1)), col, 2, cv2.LINE_AA)
        for q in uv[idx]:
            cv2.circle(v, tuple(np.int32(q)), 5, col, -1, cv2.LINE_AA)
    rms = float(np.sqrt(np.mean(offs ** 2)))
    vp_txt = (f"they meet {-vp[1]:.0f} px above the top of the picture" if vp[1] < 0
              else f"they meet at ({vp[0]:.0f}, {vp[1]:.0f})")
    text = [f"{len(lines)} hole columns running into the distance (lens distortion removed)",
            f"holes off their straight line: RMS {rms:.2f} px, max {np.abs(offs).max():.2f} px;   {vp_txt}"]
    cv2.rectangle(v, (0, 0), (W, 34 * len(text) + 14), (0, 0, 0), -1)
    for n, s in enumerate(text):
        cv2.putText(v, s, (14, 38 + 34 * n), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 2, cv2.LINE_AA)
    out = HERE / "snaps" / "vanishing_lines.png"
    cv2.imwrite(str(out), v)

    # a second, zoomed-out view so the meeting point itself is visible
    pad_top = int(min(max(-vp[1] + 150, 0), 6000))
    s = 1200 / (H + pad_top)
    canvas = np.full((H + pad_top, W, 3), 30, np.uint8)
    canvas[pad_top:] = cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)
    for n, (c, d, idx) in enumerate(lines):
        col = (0, 200, 255) if n % 2 == 0 else (255, 200, 0)
        p0 = vp + [0, pad_top]
        t1 = (H - c[1]) / d[1]
        p1 = c + t1 * d + [0, pad_top]
        cv2.line(canvas, tuple(np.int32(p0)), tuple(np.int32(p1)), col, max(int(3 / s / 2), 2), cv2.LINE_AA)
    cv2.circle(canvas, tuple(np.int32(vp + [0, pad_top])), int(12 / s), (0, 0, 255), -1, cv2.LINE_AA)
    small = cv2.resize(canvas, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    cv2.putText(small, "vanishing point", tuple(np.int32((vp + [0, pad_top]) * s + [14, 6])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    out2 = HERE / "snaps" / "vanishing_point.png"
    cv2.imwrite(str(out2), small)

    print(text[0]); print(text[1])
    print(f"lines miss the common point by: median {np.median(miss):.1f} px, max {np.max(miss):.1f} px")
    print(f"calibration: {cal_path}")
    subprocess.Popen(["cmd", "/c", "start", "chrome", out.as_uri(), out2.as_uri()])


if __name__ == "__main__":
    main()

