"""The measurement list: for every swatch, where it is on the platen and the rail position that puts the
i1 aperture on its centre, in a serpentine order, checked against the rail travel the i1 can use.

Inputs: <scan>/out/swatches.json (target.py: swatch centres, platen mm) and <aperture>/out/aperture.json
(aperture.py: aperture at home A0 and the carriage axes ex, ey). Rail positions come out in the same
carriage coordinates the scans recorded -- mm from mid-stroke in the simulation; on the real gantry, record
the Qt app's slider mm in scan.json and the list is in slider mm, ready to send.

Order: serpentine through the rows (or columns), so each move is one pitch and the carriage never runs
back across the sheet. Swatch numbers count row-major from row 0, column 0 (match them to the chart file).

Run:  python swatch_list.py scan_paper15 --aperture scan_aperture [--order rows|cols]
                            [--x-range LO,HI] [--y-range LO,HI] [--speed 20] [--dwell 1.5]
  -> <scan>/out/swatch_list.csv (order, swatch, row, col, platen_x_mm, platen_y_mm, rail_x_mm, rail_y_mm,
     reachable) and a summary
"""
import argparse, csv, json
from pathlib import Path
import numpy as np
from aperture import rail_for, RAIL_X, RAIL_Y

HERE = Path(__file__).parent


def serpentine(rows, cols, by="rows"):
    """(row, col) in serpentine order: along each row (or column), alternating direction."""
    if by == "rows":
        return [(r, c if r % 2 == 0 else cols - 1 - c) for r in range(rows) for c in range(cols)]
    return [(r if c % 2 == 0 else rows - 1 - r, c) for c in range(cols) for r in range(rows)]


def pair(s):
    lo, hi = (float(v) for v in s.split(","))
    return lo, hi


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", nargs="?", default="scan_paper15")
    ap.add_argument("--aperture", default="scan_aperture", help="folder whose out/ holds aperture.json")
    ap.add_argument("--order", choices=("rows", "cols"), default="rows")
    ap.add_argument("--x-range", type=pair, default=RAIL_X, help="usable rail X travel LO,HI (carriage mm)")
    ap.add_argument("--y-range", type=pair, default=RAIL_Y, help="usable rail Y travel LO,HI (carriage mm)")
    ap.add_argument("--speed", type=float, default=20.0, help="move speed for the time estimate, mm/s")
    ap.add_argument("--dwell", type=float, default=1.5, help="settle + i1 reading per swatch, s")
    ap.add_argument("--out", help="CSV path (default <scan>/out/swatch_list.csv)")
    a = ap.parse_args()
    sw = json.load(open(HERE / a.scan / "out" / "swatches.json"))
    apj = json.load(open(HERE / a.aperture / "out" / "aperture.json"))
    A0, ex, ey = np.array(apj["A0_mm"]), np.array(apj["ex"]), np.array(apj["ey"])
    rows, cols = sw["rows"], sw["cols"]
    by_rc = {(s["row"], s["col"]): np.array(s["centre_mm"]) for s in sw["swatches"]}

    out = Path(a.out) if a.out else HERE / a.scan / "out" / "swatch_list.csv"
    xr, yr = a.x_range, a.y_range
    entries = []
    for k, (r, c) in enumerate(serpentine(rows, cols, a.order)):
        P = by_rc[(r, c)]
        x, y = rail_for(P, A0, ex, ey)
        ok = xr[0] <= x <= xr[1] and yr[0] <= y <= yr[1]
        entries.append((k + 1, r * cols + c + 1, r, c, P[0], P[1], x, y, ok))
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["order", "swatch", "row", "col", "platen_x_mm", "platen_y_mm", "rail_x_mm", "rail_y_mm", "reachable"])
        for e in entries:
            w.writerow([e[0], e[1], e[2], e[3], f"{e[4]:.3f}", f"{e[5]:.3f}", f"{e[6]:.3f}", f"{e[7]:.3f}", int(e[8])])

    R = np.array([[e[6], e[7]] for e in entries])
    bad = [e for e in entries if not e[8]]
    steps = np.diff(R, axis=0)
    move_s = np.maximum(np.abs(steps[:, 0]), np.abs(steps[:, 1])).sum() / a.speed   # axes move together
    t = move_s + a.dwell * len(entries)
    print(f"{len(entries)} swatches ({rows} rows x {cols} cols), serpentine by {a.order} -> {out.relative_to(HERE)}")
    print(f"rail X {R[:, 0].min():.2f} .. {R[:, 0].max():.2f}  (usable {xr[0]:g} .. {xr[1]:g})")
    print(f"rail Y {R[:, 1].min():.2f} .. {R[:, 1].max():.2f}  (usable {yr[0]:g} .. {yr[1]:g})")
    print(f"unreachable: {len(bad)}" + ("" if not bad else " -- e.g. " + ", ".join(
        f"row {e[2]} col {e[3]} at rail ({e[6]:.1f}, {e[7]:.1f})" for e in bad[:5])))
    print(f"path {np.hypot(*steps.T).sum() / 1000:.2f} m; estimated time {t / 60:.0f} min "
          f"(at {a.speed:g} mm/s and {a.dwell:g} s per swatch)")
