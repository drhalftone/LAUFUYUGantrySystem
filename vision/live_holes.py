"""Live hole detection on the carriage camera: every frame goes through holegrid's detector and the
lens-aware grid fit, and the result is drawn over the video.

  green circle + dot   CONFIRMED  -- detected, and on the fitted grid (homography + radial lens distortion)
  orange circle        SUSPECTED  -- detected as a hole, but not on the grid (debris, glare, a cut-off hole)
  magenta x            EXPECTED   -- the grid puts a hole here, but nothing was detected
  cyan curves          the fitted grid rows / columns, bent by the fitted lens distortion (g toggles)

Header: counts, the grid fit's RMS residual, the fitted k1/k2 (units: f = 0.55 x the larger image side)
and how far the lens moves the image corner, and processing time per frame.

Run:  python live_holes.py [--exposure 120] [--gain 0]
Keys: q/Esc quit, g grid on/off, s save raw + overlay to snaps/, +/- exposure x1.25, a auto-expose
"""
import argparse, time
from pathlib import Path
import numpy as np, cv2
from pypylon import pylon
import holegrid as hg
from grab import Camera

HERE = Path(__file__).parent
GREEN, ORANGE, MAGENTA, CYAN = (0, 220, 0), (0, 165, 255), (255, 0, 255), (200, 200, 0)


def process(g):
    H, W = g.shape
    t0 = time.perf_counter()
    det = hg.detect_holes(g)
    lat = None
    if len(det.centers) >= 6:
        lat = hg.grow_lattice(det.centers, image_size=(W, H), shapes=(det.axes, det.angles))
    return det, lat, (time.perf_counter() - t0) * 1000


def draw(g, det, lat, ms, show_grid):
    H, W = g.shape
    v = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    on = set(lat.det.tolist()) if lat is not None else set()

    if lat is not None and show_grid:
        lo, hi = lat.ij.min(0) - 1, lat.ij.max(0) + 1
        for axis in (0, 1):
            other = 1 - axis
            for k in range(lo[axis], hi[axis] + 1):
                t = np.linspace(lo[other], hi[other], 200)
                ij = np.zeros((len(t), 2))
                ij[:, axis], ij[:, other] = k, t
                p = lat.model(ij)
                inside = (p[:, 0] > -50) & (p[:, 0] < W + 50) & (p[:, 1] > -50) & (p[:, 1] < H + 50)
                if inside.sum() > 1:
                    cv2.polylines(v, [np.int32(np.round(p[inside] * 4))], False, CYAN, 1, cv2.LINE_AA, shift=2)

    for k, c in enumerate(det.centers):
        r = int(det.axes[k].max() / 2) + 4
        cc = tuple(np.int32(np.round(c * 4)))
        if k in on:
            cv2.circle(v, cc, r * 4, GREEN, 2, cv2.LINE_AA, shift=2)
            cv2.circle(v, cc, 2 * 4, GREEN, -1, cv2.LINE_AA, shift=2)
        else:
            cv2.circle(v, cc, r * 4, ORANGE, 3, cv2.LINE_AA, shift=2)

    n_exp = 0
    if lat is not None and lat.expected is not None and len(lat.expected):
        n_exp = len(lat.expected)
        for p in lat.model(lat.expected.astype(float)):
            x, y = int(round(p[0])), int(round(p[1]))
            cv2.line(v, (x - 9, y - 9), (x + 9, y + 9), MAGENTA, 2, cv2.LINE_AA)
            cv2.line(v, (x - 9, y + 9), (x + 9, y - 9), MAGENTA, 2, cv2.LINE_AA)

    n_det, n_on = len(det.centers), len(on)
    lines = [f"detected {n_det}   confirmed {n_on}   suspected {n_det - n_on}   expected-not-seen {n_exp}"]
    if lat is not None:
        m = lat.model
        lines.append(f"grid fit RMS {lat.rms:.2f} px   k1 {m.k[0]:+.4f}  k2 {m.k[1]:+.4f}   "
                     f"lens moves corner {m.corner_shift((W, H)):.0f} px")
    else:
        lines.append("no grid found")
    lines.append(f"{ms:.0f} ms/frame   exposure {draw.exposure_ms:.1f} ms")
    cv2.rectangle(v, (0, 0), (W, 34 * len(lines) + 14), (0, 0, 0), -1)
    for n, s in enumerate(lines):
        cv2.putText(v, s, (14, 38 + 34 * n), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    return v


draw.exposure_ms = float("nan")   # set by main() each frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exposure", type=float, default=120.0, help="ms")
    ap.add_argument("--gain", type=float, default=0.0, help="dB")
    ap.add_argument("--serial")
    ap.add_argument("--auto", action="store_true", help="auto-expose once at startup (a re-runs it live)")
    a = ap.parse_args()

    cam = Camera(a.serial, a.exposure, a.gain)
    if a.auto:
        print(f"auto exposure: {cam.auto_expose():.2f} ms")
    win = "Live hole detection"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1440, 900)
    show_grid = True
    print(__doc__.split("Run:")[0].strip())
    try:
        cam.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        while True:
            r = cam.cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if not r.GrabSucceeded():
                r.Release()
                continue
            g = r.Array.copy()
            r.Release()
            det, lat, ms = process(g)
            draw.exposure_ms = cam.exposure_ms
            v = draw(g, det, lat, ms, show_grid)
            cv2.imshow(win, v)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord("g"):
                show_grid = not show_grid
            elif k in (ord("+"), ord("=")):
                cam.exposure_ms *= 1.25
            elif k in (ord("-"), ord("_")):
                cam.exposure_ms /= 1.25
            elif k in (ord("a"), ord("s")):
                cam.cam.StopGrabbing()
                if k == ord("a"):
                    print(f"auto exposure: {cam.auto_expose():.2f} ms")
                else:
                    out = HERE / "snaps"
                    out.mkdir(exist_ok=True)
                    stem = time.strftime("live_%Y%m%d_%H%M%S")
                    cv2.imwrite(str(out / f"{stem}.png"), g)
                    cv2.imwrite(str(out / f"{stem}_holes.png"), v)
                    print(f"saved {out / stem}.png and _holes.png")
                cam.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    finally:
        cam.cam.StopGrabbing()
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
