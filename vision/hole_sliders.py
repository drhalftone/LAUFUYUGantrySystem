"""Slider positions that centre the i1 aperture on each platen hole, in the app's slider mm.

The aperture is measured from a bare-platen scan: the board shows through the plate's hole as a bright disc, and
each frame's disc is put on the platen with the calibrated camera (aperture.py's method, without the white sheet).
Only frames with no platen hole under the aperture are used, since a dark hole in the disc would bias its centroid.

Hole (X, Y) in inches is the world grid of label_mosaic.py: X along the single rail, Y along the rail pair.
Output columns slider_x_mm / slider_y_mm are the app's X (single rail) and Y (rail pair) slider values.

Run:  python hole_sliders.py <bare scan> [--pixel 978,576] [--plate-floor 1.0]
  -> <bare scan>/out/hole_sliders.csv, aperture_check.png
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2
from holegrid import PITCH, EDGE
from mosaic import load_calibration
from aperture import cad_aperture, disc, rail_for, RES, WIN

HERE = Path(__file__).parent
FIRST_ROW, LAST_ROW, FIRST_COL = 3, 26, 0       # lattice rows/columns of the real board, from label_mosaic.py
N_X, N_Y = 24, 21                               # holes along X and Y that the scan shows
SLIDER_X = (5.0, 436.244)                       # the app's scan range, single rail
SLIDER_Y = (5.0, 266.288)                       # the app's scan range, rail pair


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan")
    ap.add_argument("--pixel", default="978,576", help="aperture centre in the image, roughly")
    ap.add_argument("--plate-floor", type=float, help="plate floor thickness at the hole, mm (default: from CAD)")
    a = ap.parse_args()
    folder = HERE / a.scan
    cam, _, _, _ = load_calibration(folder / "out")
    d = json.load(open(folder / "scan.json"))
    u, v = (float(s) for s in a.pixel.split(","))
    Ex, Ey = cam.ex[:2], cam.ey[:2]
    C0 = -cam.R.T @ cam.t
    try:
        _, radius, floor = cad_aperture()
        print(f"CAD: aperture radius {radius:.2f} mm, plate floor {floor:.2f} mm")
    except Exception as e:
        radius, floor = 4.9, 1.0                               # cad/xrite_i1_plate.py: 9.8 mm aperture, 1 mm floor
        print(f"CAD model not readable ({e}); using radius {radius} mm, floor {floor} mm from cad/xrite_i1_plate.py")
    if a.plate_floor is not None:
        floor = a.plate_floor

    seen = []
    crops = []
    for f in d["frames"]:
        x, y = f["carriage_mm"]
        img = cv2.imread(str(folder / f["file"]), 0)
        guess = cam.to_board(np.array([[u, v]], float), x, y)[0]
        c_vis, L = disc(img, cam, x, y, guess)
        if c_vis is None:
            continue
        C = C0 + x * cam.ex + y * cam.ey
        c = c_vis.copy()
        for _ in range(3):
            s = floor * (c - C[:2]) / (C[2] - floor)
            c = c_vis - s / 2
        seen.append((f["file"], x, y, *(c - x * Ex - y * Ey)))
        crops.append((L, c_vis, guess))
    A = np.array([s[3:] for s in seen])
    A0 = np.median(A, axis=0)

    def clear(x, y, A0_):
        P = A0_ + x * Ex + y * Ey
        lat = np.round((P - EDGE) / PITCH) * PITCH + EDGE
        return np.linalg.norm(P - lat) > radius + 3.2 + 1.0

    for _ in range(3):                                  # which frames have plain board under the aperture
        keep = np.array([clear(s[1], s[2], A0) for s in seen])
        keep &= np.linalg.norm(A - A0, axis=1) < 1.0
        A0 = A[keep].mean(0)
    spread = np.sqrt(((A[keep] - A0) ** 2).sum(1).mean())
    print(f"{len(seen)} frames with a disc, {keep.sum()} with plain board under the aperture")
    print(f"aperture at carriage (0,0): board ({A0[0]:.3f}, {A0[1]:.3f}) mm; spread RMS {spread:.3f} mm")

    rows = ["x_in,y_in,slider_x_mm,slider_y_mm,reachable"]
    sx, sy, ok_all = [], [], []
    for Y in range(N_Y):
        for X in range(N_X):
            P = (EDGE + PITCH * (FIRST_COL + Y), EDGE + PITCH * (LAST_ROW - X))
            pair, single = rail_for(P, A0, cam.ex, cam.ey)      # carriage_mm order: [rail pair, single rail]
            ok = SLIDER_X[0] <= single <= SLIDER_X[1] and SLIDER_Y[0] <= pair <= SLIDER_Y[1]
            rows.append(f"{X},{Y},{single:.3f},{pair:.3f},{int(ok)}")
            sx.append(single); sy.append(pair); ok_all.append(ok)
    ok_all = np.array(ok_all)
    (folder / "out" / "hole_sliders.csv").write_text(chr(10).join(rows) + chr(10))
    print(f"{ok_all.sum()} of {ok_all.size} holes reachable within the slider range")
    print(f"slider X needed {min(sx):.1f} .. {max(sx):.1f} (range {SLIDER_X}); slider Y needed {min(sy):.1f} .. {max(sy):.1f} (range {SLIDER_Y})")
    r = np.array(ok_all).reshape(N_Y, N_X)
    xs = np.nonzero(r.any(0))[0]; ys = np.nonzero(r.any(1))[0]
    print(f"reachable holes: X {xs.min()}..{xs.max()} in, Y {ys.min()}..{ys.max()} in")
    for X, Y in ((0, 0), (12, 10), (23, 20)):
        print(f"  hole ({X},{Y}) -> " + rows[1 + Y * N_X + X])

    # picture: the disc as the camera saw it, in a few of the frames used
    tiles = []
    for k in np.nonzero(keep)[0][:6]:
        L, c_vis, g = crops[k]
        vis = cv2.cvtColor(np.clip(L, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        q = tuple(int(n) for n in np.round((c_vis - g + WIN) / RES))
        cv2.drawMarker(vis, q, (0, 0, 255), cv2.MARKER_CROSS, 60, 2)
        cv2.circle(vis, q, int(round(radius / RES)), (0, 255, 255), 1, cv2.LINE_AA)
        tiles.append(cv2.resize(vis, (300, 300)))
    if tiles:
        cv2.imwrite(str(folder / "out" / "aperture_check.png"), np.hstack(tiles))
