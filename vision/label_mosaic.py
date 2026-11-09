"""Platen mosaic over the whole board, with the hole grid drawn and every hole labelled in inches.

World XY plane, the gantry's own axes: X is the single rail and runs along the picture's columns, Y is the rail
pair and runs down its rows; both increase with their slider. The origin is the board's first hole at the sliders'
low ends, so holes sit on whole inches. The board frame of scan_map.py has X along the rail pair and Y against the
single rail, so the mosaic is turned 90 degrees clockwise here. A lattice position counts as a real hole only if
the mosaic is actually dark there, which also rejects rows that scan_map.py indexed on the rail.

Run:  python label_mosaic.py <scan> --calib <bare scan>/out [--window 900]
  -> <scan>/out/platen_mosaic_full.png, platen_mosaic_grid.png, and holes_inches.csv
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2
import scan_map
from holegrid import PITCH, EDGE
from mosaic import load_calibration

HERE = Path(__file__).parent
MM = scan_map.MAP_MM
HOLE_R_MM, RING_MM = 2.5, (6.0, 9.0)
MIN_RATIO = 0.75                       # hole centre at most this bright relative to the board around it
MIN_ROW_FILL = 0.3                     # a row or column is real if this much of it is holes (the rails hide part of the end rows)


def px(mm):
    return mm / MM - 0.5


def is_hole(mos, covered, cx, cy):
    r2 = int(RING_MM[1] / MM) + 1
    x, y = int(round(px(cx))), int(round(px(cy)))
    if x - r2 < 0 or y - r2 < 0 or x + r2 >= mos.shape[1] or y + r2 >= mos.shape[0] or not covered[y, x]:
        return False
    yy, xx = np.ogrid[-r2:r2 + 1, -r2:r2 + 1]
    d = np.hypot(xx, yy) * MM
    patch, ok = mos[y - r2:y + r2 + 1, x - r2:x + r2 + 1].astype(float), covered[y - r2:y + r2 + 1, x - r2:x + r2 + 1]
    inner, ring = (d <= HOLE_R_MM) & ok, (d >= RING_MM[0]) & (d <= RING_MM[1]) & ok
    return inner.sum() > 10 and ring.sum() > 30 and np.mean(patch[inner]) < MIN_RATIO * np.median(patch[ring])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--window", type=float, default=900.0, help="side of the rendered square, mm")
    a = ap.parse_args()
    folder, cfold = HERE / a.scan, HERE / a.calib
    d = json.load(open(folder / "scan.json"))
    cam, mask, ff, calib = load_calibration(cfold)
    frames = d["frames"]
    grays = [cv2.imread(str(folder / f["file"]), 0) for f in frames]
    prep = lambda g: np.clip(g.astype(np.float32) / np.maximum(ff, 1e-3), 0, 255)
    scan_map.BOARD = a.window                                       # board_map reads it at call time
    R = [cam.R] * len(frames)
    T = [cam.pose(*f["carriage_mm"])[1] for f in frames]
    mos, covered = scan_map.board_map(frames, grays, cam, mask, R, T, prep=prep)

    # which lattice positions are really holes
    n = int((a.window - EDGE) / PITCH) + 1
    found = np.array([[is_hole(mos, covered, EDGE + PITCH * I, EDGE + PITCH * J) for I in range(n)] for J in range(n)])
    in_view = np.array([[0 <= int(px(EDGE + PITCH * J)) < mos.shape[0] and 0 <= int(px(EDGE + PITCH * I)) < mos.shape[1]
                         and covered[int(px(EDGE + PITCH * J)), int(px(EDGE + PITCH * I))] for I in range(n)] for J in range(n)])
    frac = lambda ax: found.sum(ax) / np.maximum(in_view.sum(ax), 1)
    cols, rows = np.nonzero(frac(0) >= MIN_ROW_FILL)[0], np.nonzero(frac(1) >= MIN_ROW_FILL)[0]
    I0, I1, J0, J1 = cols.min(), cols.max(), rows.min(), rows.max()
    print(f"real holes: lattice columns {I0}..{I1}, rows {J0}..{J1} -> {I1 - I0 + 1} x {J1 - J0 + 1} holes, "
          f"{found[J0:J1 + 1, I0:I1 + 1].sum()} of {(I1 - I0 + 1) * (J1 - J0 + 1)} positions confirmed dark")
    print(f"  fill by row: " + " ".join(f"{J}:{frac(1)[J]:.1f}" for J in range(0, min(n, J1 + 3))))

    # draw in the gantry's axes: world X = J1 - J (columns), world Y = I - I0 (rows)
    rotated = cv2.rotate(mos, cv2.ROTATE_90_CLOCKWISE)
    H = mos.shape[0]
    ipx = lambda v: int(round(v))
    hole_px = lambda I, J: (H - 1 - px(EDGE + PITCH * J), px(EDGE + PITCH * I))
    nX, nY = J1 - J0 + 1, I1 - I0 + 1
    world = lambda X, Y: hole_px(Y + I0, J1 - X)
    img = cv2.cvtColor(rotated, cv2.COLOR_GRAY2BGR)
    for X in range(nX):
        a_, b_ = world(X, 0), world(X, nY - 1)
        cv2.line(img, (ipx(a_[0]), ipx(a_[1])), (ipx(b_[0]), ipx(b_[1])), (0, 200, 0), 1, cv2.LINE_AA)
    for Y in range(nY):
        a_, b_ = world(0, Y), world(nX - 1, Y)
        cv2.line(img, (ipx(a_[0]), ipx(a_[1])), (ipx(b_[0]), ipx(b_[1])), (0, 200, 0), 1, cv2.LINE_AA)
    rows_csv = ["x_in,y_in,board_x_mm,board_y_mm,confirmed_dark"]
    for Y in range(nY):
        for X in range(nX):
            I, J = Y + I0, J1 - X
            rows_csv.append(f"{X},{Y},{EDGE + PITCH * I:.2f},{EDGE + PITCH * J:.2f},{int(found[J, I])}")
            if not in_view[J, I]:
                continue
            u, v = world(X, Y)
            c = (ipx(u), ipx(v))
            cv2.circle(img, c, ipx(3.19 / MM), (0, 255, 255) if found[J, I] else (0, 0, 255), 1, cv2.LINE_AA)
            t, o = f"{X},{Y}", (c[0] + 7, c[1] - 7)
            cv2.putText(img, t, o, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, t, o, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    u, v = world(0, 0)
    O = (ipx(u), ipx(v))
    cv2.arrowedLine(img, O, (O[0] + 200, O[1]), (0, 0, 255), 3, cv2.LINE_AA, tipLength=0.1)
    cv2.arrowedLine(img, O, (O[0], O[1] + 200), (255, 120, 0), 3, cv2.LINE_AA, tipLength=0.1)
    cv2.putText(img, "X (in)", (O[0] + 205, O[1] + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "Y (in)", (O[0] - 40, O[1] + 228), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 120, 0), 2, cv2.LINE_AA)
    # crop to the board: one inch around the holes
    m = PITCH / MM
    far = world(nX - 1, nY - 1)
    x_a, y_a = max(ipx(O[0] - m), 0), max(ipx(O[1] - m), 0)
    x_b, y_b = min(ipx(far[0] + m), img.shape[1]), min(ipx(far[1] + m), img.shape[0])
    cv2.imwrite(str(folder / "out" / "platen_mosaic_full.png"), rotated[y_a:y_b, x_a:x_b])
    cv2.imwrite(str(folder / "out" / "platen_mosaic_grid.png"), img[y_a:y_b, x_a:x_b])
    (folder / "out" / "holes_inches.csv").write_text(chr(10).join(rows_csv) + chr(10))
    print(f"world grid: X 0..{nX - 1} in, Y 0..{nY - 1} in; picture {x_b - x_a} x {y_b - y_a} px")
