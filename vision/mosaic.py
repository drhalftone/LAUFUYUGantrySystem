"""Platen mosaic from a scan with something on the platen (e.g. a paper test target).

Two passes at the same carriage positions:
  1. bare platen  -> scan_map.py: holes -> camera calibration + carriage axes, the carriage mask
                     (i1 cradle and chain), and a flat-field. Saved in <bare>/out/.
  2. with target  -> this script: each frame is put on the platen from its carriage position and
                     the pass-1 calibration alone -- the target hides most holes, and none are needed.
Each frame is divided by the flat-field (vignetting, lighting), the carriage mask is dropped, and the
frames are re-projected onto the platen plane and blended, favouring the closest (sharpest) view.

If the scan is synthetic (render_scan.py --paper), the mosaic is checked against the sheet's true
position: whole-sheet and per-region offsets by phase correlation against an ideal ortho image of it.

Run:  python mosaic.py scan_paper --calib scan_bare/out [--mm 0.25]
  -> scan_paper/out/: platen_mosaic.png, sheet_check.png, mosaic_report.txt
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2
import holegrid as hg
from scan_map import board_map, BOARD

HERE = Path(__file__).parent


def load_calibration(folder):
    c = json.load(open(folder / "calibration.json"))
    R, C0 = np.array(c["R"]), np.array(c["C0"])
    cam = hg.CameraModel(np.array(c["K"]), np.array(c["dist"]), R, -R @ C0, np.array(c["ex"]), np.array(c["ey"]))
    mask = cv2.imread(str(folder / "mask.png"), 0) > 0
    ff = np.load(folder / "flat.npy")
    return cam, mask, ff, c


def ideal_sheet(paper, mm, n):
    """The sheet as it truly lies on the platen, ortho, same grid as the mosaic (NaN off the sheet)."""
    from render_scan import PaperTarget
    p = PaperTarget(paper["centre"], paper["angle_deg"], cols=paper.get("cols", 7), rows=paper.get("rows", 9),
                    patch=paper["patch_mm"], gap=paper.get("gap_mm", 4.0))
    gy, gx = np.mgrid[0:n, 0:n].astype(np.float32)
    return p.albedo((gx + 0.5) * mm, (gy + 0.5) * mm)


def shift_mm(a, b, mm):
    """Offset of b relative to a (mm), by phase correlation of their normalised gradients."""
    def prep(x):
        x = np.nan_to_num(x.astype(np.float32))
        gx, gy = cv2.Sobel(x, cv2.CV_32F, 1, 0), cv2.Sobel(x, cv2.CV_32F, 0, 1)
        g = np.hypot(gx, gy)
        return (g - g.mean()) / (g.std() + 1e-6)
    (dx, dy), resp = cv2.phaseCorrelate(prep(a), prep(b), cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))
    return np.array([dx, dy]) * mm, resp


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", nargs="?", default="scan_paper")
    ap.add_argument("--calib", default="scan_bare/out", help="output folder of scan_map.py on the bare-platen pass")
    ap.add_argument("--mm", type=float, default=0.25, help="mosaic resolution, mm per pixel")
    a = ap.parse_args()
    folder, cfold = HERE / a.scan, HERE / a.calib
    out = folder / "out"
    out.mkdir(exist_ok=True)
    d = json.load(open(folder / "scan.json"))
    cam, mask, ff, calib = load_calibration(cfold)
    lines = []
    log = lambda s="": (print(s), lines.append(s))

    frames = d["frames"]
    calib_pos = {tuple(p) for p in calib["carriage_mm"]}
    missing = [f["carriage_mm"] for f in frames if tuple(f["carriage_mm"]) not in calib_pos]
    log(f"{len(frames)} frames; calibration from {cfold.relative_to(HERE)} "
        f"({len(calib_pos)} positions, reprojection RMS {calib['reprojection_rms_px']:.2f} px)")
    if missing:
        log(f"  note: {len(missing)} positions not in the calibration scan (fine -- the carriage model covers them)")

    grays = [cv2.imread(str(folder / f["file"]), 0) for f in frames]
    prep = lambda g: np.clip(g.astype(np.float32) / np.maximum(ff, 1e-3), 0, 255)       # ff is 1 at its median
    import scan_map
    scan_map.MAP_MM = a.mm                                                # board_map reads it at call time
    R = [cam.R] * len(frames)
    T = [cam.pose(*f["carriage_mm"])[1] for f in frames]                   # carriage position only
    mos, covered = board_map(frames, grays, cam, mask, R, T, prep=prep)
    cv2.imwrite(str(out / "platen_mosaic.png"), mos)
    log(f"platen mosaic {mos.shape[1]} x {mos.shape[0]} px at {a.mm} mm/px (columns = Xb, rows = Yb): "
        f"{covered.mean():.0%} of the platen covered")

    paper = d.get("paper")
    if paper:
        n = mos.shape[0]
        ideal = ideal_sheet(paper, a.mm, n)
        on = ~np.isnan(ideal)
        cov = covered[on].mean()
        ys, xs = np.nonzero(on)
        y0, y1, x0, x1 = ys.min() - 40, ys.max() + 40, xs.min() - 40, xs.max() + 40
        m = mos[y0:y1, x0:x1].astype(np.float32)
        ref = np.where(on, ideal, 0.56)[y0:y1, x0:x1]
        s, resp = shift_mm(ref, m, a.mm)
        log(f"sheet: {cov:.1%} of it covered; whole-sheet offset vs truth ({s[0]:+.3f}, {s[1]:+.3f}) mm "
            f"(correlation peak {resp:.2f})")
        # per-region offsets: 3 x 3 tiles over the sheet
        hh, ww = m.shape
        offs = []
        for i in range(3):
            for j in range(3):
                sl = (slice(i * hh // 3, (i + 1) * hh // 3), slice(j * ww // 3, (j + 1) * ww // 3))
                o, r = shift_mm(ref[sl], m[sl], a.mm)
                offs.append(np.hypot(*o))
        log(f"  3 x 3 regions across the sheet: offset RMS {np.sqrt(np.mean(np.square(offs))):.3f} mm, "
            f"max {max(offs):.3f} mm")
        # overlay: true sheet outline and patch outlines on the mosaic
        chk = cv2.cvtColor(mos, cv2.COLOR_GRAY2BGR)
        px = lambda p: (int(round(p[0] / a.mm)), int(round(p[1] / a.mm)))
        cv2.polylines(chk, [np.array([px(c) for c in paper["corners"]])], True, (0, 220, 0), 2, cv2.LINE_AA)
        ang = np.deg2rad(paper["angle_deg"])
        Rp = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        h2 = paper["patch_mm"] / 2
        for p in paper["patches"]:
            c = np.array(p["centre"])
            sq = [c + Rp @ np.array(v) for v in ((-h2, -h2), (h2, -h2), (h2, h2), (-h2, h2))]
            cv2.polylines(chk, [np.array([px(q) for q in sq])], True, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(out / "sheet_check.png"), chk[max(y0, 0):y1, max(x0, 0):x1])
    (out / "mosaic_report.txt").write_text("\n".join(lines) + "\n")
