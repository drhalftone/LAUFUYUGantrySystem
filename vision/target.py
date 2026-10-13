"""Swatch centres on the platen from the platen mosaic: paper edges -> sheet, swatch edges -> block, grid -> centres.

No registration marks: the sheet's four edges are found where the white paper meets the dark platen, then,
inside the sheet, the swatch block's four edges are found where the white margin meets the first swatch.
Each edge is a straight-line fit through hundreds of sub-pixel edge points, robust to the odd scanline
that runs down a gap between swatches or over a swatch nearly as light as the paper. The block's corners
and the chart layout (rows, columns, gap between swatches) give every swatch centre in board mm.

The sheet is a rectangle, so it looks the same turned 180 deg: place it with row 0 toward -Y (the
beam's start) and column 0 toward -X (the motor end), within +/-45 deg.

Run:  python target.py scan_paper --rows 9 --cols 7 --gap 4 [--mm 0.25]
  -> scan_paper/out/: swatches.json (board mm), swatch_check.png; with a synthetic scan, errors vs truth
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2

HERE = Path(__file__).parent
RECT_MM = 0.1          # sheet-frame image resolution


def fit_line(p, tol):
    """Robust total-least-squares line through points p (n, 2): point on it and unit direction."""
    keep = np.ones(len(p), bool)
    for _ in range(6):
        c = p[keep].mean(0)
        _, _, vt = np.linalg.svd(p[keep] - c)
        d, n = vt[0], vt[1]
        r = np.abs((p - c) @ n)
        lim = max(tol, 3.0 * 1.4826 * np.median(r[keep]))
        new = r < lim
        if (new == keep).all():
            break
        keep = new
    return c, d, keep


def intersect(l1, l2):
    (c1, d1), (c2, d2) = l1, l2
    t = np.linalg.solve(np.c_[d1, -d2], c2 - c1)
    return c1 + t[0] * d1


def sample(img, pts):
    """Bilinear samples of img at float pixel coordinates pts (..., 2) = (x, y)."""
    p = np.asarray(pts, np.float32)
    s = cv2.remap(img, p[..., 0].reshape(-1, 1), p[..., 1].reshape(-1, 1), cv2.INTER_LINEAR,
                  borderMode=cv2.BORDER_REPLICATE)
    return s.reshape(p.shape[:-1])


def edge_along(prof, step):
    """Sub-sample position of the strongest step in a 1-D profile (index units), or NaN."""
    g = np.gradient(prof)
    k = int(np.argmax(np.abs(g[1:-1]))) + 1
    a, b, c = np.abs(g[k - 1:k + 2])
    den = a - 2 * b + c
    return k + (0.5 * (a - c) / den if den != 0 else 0.0)


def find_sheet(mos, mm):
    """Sheet corners in mosaic pixels, ordered (u0,v0) (u1,v0) (u1,v1) (u0,v1): u ~ +X, v ~ +Y."""
    m = mos.astype(np.float32)
    seen = mos[mos > 0]                                   # 0 = never seen by the camera
    t = (np.median(seen) + np.percentile(seen, 99.5)) / 2  # platen (most of the seen area) / white paper
    bw = (mos > t).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(bw)
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    cs, _ = cv2.findContours((lab == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cs, key=cv2.contourArea)
    quad = cv2.boxPoints(cv2.minAreaRect(c))                     # rough corners
    # order: u along the short side, roughly +X; v roughly +Y
    ctr = quad.mean(0)
    quad = quad[np.argsort(np.arctan2(quad[:, 1] - ctr[1], quad[:, 0] - ctr[0]))]   # clockwise on screen (y down)
    best = None
    for s in range(4):
        q = np.roll(quad, -s, 0)
        u, v = q[1] - q[0], q[3] - q[0]
        if np.linalg.norm(u) > np.linalg.norm(v):
            continue
        score = u[0] / np.linalg.norm(u) + v[1] / np.linalg.norm(v)
        if best is None or score > best[0]:
            best = (score, q)
    quad = best[1]
    # refine each side: profiles across it, sub-pixel step, robust line
    half = 4.0 / mm
    lines = []
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        d = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([-d[1], d[0]])
        ts = np.linspace(0.08, 0.92, 300)
        offs = np.arange(-half, half + 1)
        base = a[None] + ts[:, None] * (b - a)[None]
        pts = base[:, None, :] + offs[None, :, None] * nrm[None, None, :]
        prof = sample(m, pts)
        e = np.array([edge_along(p, 1) for p in prof]) - half
        lines.append(fit_line(base + e[:, None] * nrm, tol=0.3 / mm * 1.0)[:2])
    return np.array([intersect(lines[i - 1], lines[i]) for i in range(4)])


def rectify(mos, corners_px, mm):
    """Sheet-frame image at RECT_MM: u right along corners 0->1, v down along 0->3. Returns image, H (rect px -> mosaic px)."""
    W = np.linalg.norm(corners_px[1] - corners_px[0]) * mm
    L = np.linalg.norm(corners_px[3] - corners_px[0]) * mm
    w, h = int(round(W / RECT_MM)), int(round(L / RECT_MM))
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    H = cv2.getPerspectiveTransform(src, np.float32(corners_px))
    S = cv2.warpPerspective(mos.astype(np.float32), H, (w, h), flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)
    return S, H, W, L


def find_block(S, skip_mm=2.5, drop=0.04):
    """Swatch block edges in the sheet image: scan in from each paper edge to the first swatch."""
    h, w = S.shape
    sk = int(skip_mm / RECT_MM)
    paper = np.median(np.concatenate([S[:, sk:2 * sk].ravel(), S[:, -2 * sk:-sk].ravel(),
                                      S[sk:2 * sk].ravel(), S[-2 * sk:-sk].ravel()]))
    thr = paper * (1 - drop)
    lines = []
    # (scan axis, from which end): left, top, right, bottom -> u0, v0, u1, v1
    for axis, rev in ((1, False), (0, False), (1, True), (0, True)):
        A = S if axis == 1 else S.T                                  # rows of A are scanlines
        A = A[:, ::-1] if rev else A
        n_lines, n = A.shape
        pts = []
        for r in range(int(0.05 * n_lines), int(0.95 * n_lines), 3):
            row = cv2.GaussianBlur(A[r:r + 1].astype(np.float32), (0, 0), 1.0).ravel()
            below = np.nonzero(row[sk:n // 2] < thr)[0]
            if not len(below):
                continue
            k = sk + below[0]
            lo, hi = max(k - 15, 1), min(k + 15, n - 2)
            e = lo + edge_along(row[lo:hi], 1)
            e = n - 1 - e if rev else e
            pts.append((e, r) if axis == 1 else (r, e))
        p = np.array(pts, float)
        c, d, keep = fit_line(p, tol=0.4 / RECT_MM)
        lines.append((c, d, keep.mean(), len(p)))
    # corners (u0,v0) (u1,v0) (u1,v1) (u0,v1); lines are u0, v0, u1, v1
    corners = np.array([intersect(lines[0][:2], lines[1][:2]), intersect(lines[2][:2], lines[1][:2]),
                        intersect(lines[2][:2], lines[3][:2]), intersect(lines[0][:2], lines[3][:2])])
    return corners, paper, [(l[2], l[3]) for l in lines]


def swatch_centres(block_rect_px, rows, cols, gap):
    """Swatch centres in sheet-image pixels: block corners + layout (gap in mm), by block homography."""
    Wb = np.linalg.norm(block_rect_px[1] - block_rect_px[0]) * RECT_MM
    Lb = np.linalg.norm(block_rect_px[3] - block_rect_px[0]) * RECT_MM
    pu, pv = (Wb + gap) / cols, (Lb + gap) / rows                   # pitch
    sw, sh = pu - gap, pv - gap                                     # swatch size
    Hb = cv2.getPerspectiveTransform(np.float32([[0, 0], [Wb, 0], [Wb, Lb], [0, Lb]]), np.float32(block_rect_px))
    uv = np.array([[c * pu + sw / 2, r * pv + sh / 2] for r in range(rows) for c in range(cols)], np.float32)
    return cv2.perspectiveTransform(uv[None], Hb)[0], (Wb, Lb, sw, sh)


def to_mosaic(p, H):
    return cv2.perspectiveTransform(np.float32(p)[None], H)[0]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", nargs="?", default="scan_paper")
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--gap", type=float, default=0.0, help="gap between swatches on the chart, mm")
    ap.add_argument("--mm", type=float, default=0.25, help="mosaic resolution, mm per pixel (as mosaic.py)")
    a = ap.parse_args()
    folder = HERE / a.scan
    out = folder / "out"
    mos = cv2.imread(str(out / "platen_mosaic.png"), 0)
    px2mm = lambda p: (np.asarray(p) + 0.5) * a.mm                  # mosaic pixel -> board mm

    sheet_px = find_sheet(mos, a.mm)
    S, H, W, L = rectify(mos, sheet_px, a.mm)
    block_r, paper, fits = find_block(S)
    cen_r, (Wb, Lb, sw, sh) = swatch_centres(block_r, a.rows, a.cols, a.gap)
    sheet_mm = px2mm(sheet_px)
    block_mm = px2mm(to_mosaic(block_r, H))
    cen_mm = px2mm(to_mosaic(cen_r, H))

    lines = []
    log = lambda s="": (print(s), lines.append(s))
    side = lambda c: np.linalg.norm(np.roll(c, -1, 0) - c, axis=1)
    log(f"sheet  {W:.1f} x {L:.1f} mm; sides {np.round(side(sheet_mm), 1).tolist()}")
    log(f"block  {Wb:.1f} x {Lb:.1f} mm -> swatches {sw:.2f} x {sh:.2f} mm at pitch "
        f"{(Wb + a.gap) / a.cols:.2f} x {(Lb + a.gap) / a.rows:.2f} mm")
    log("block edges (u0 v0 u1 v1): " + ", ".join(f"{k:.0%} of {n} scanlines on the line" for k, n in fits))
    margins = block_r * RECT_MM
    log(f"margins: left {margins[0, 0]:.2f}, top {margins[0, 1]:.2f}, right {W - margins[1, 0]:.2f}, "
        f"bottom {L - margins[2, 1]:.2f} mm")

    res = {"sheet_corners_mm": sheet_mm.tolist(), "block_corners_mm": block_mm.tolist(),
           "rows": a.rows, "cols": a.cols, "swatch_mm": [sw, sh],
           "swatches": [{"row": r, "col": c, "centre_mm": cen_mm[r * a.cols + c].tolist()}
                        for r in range(a.rows) for c in range(a.cols)]}
    json.dump(res, open(out / "swatches.json", "w"), indent=1)

    truth = json.load(open(folder / "scan.json")).get("paper")
    if truth:
        tc = np.array(truth["corners"])
        e = np.linalg.norm(sheet_mm - tc, axis=1)
        log(f"truth: sheet corners off by {np.round(e, 3).tolist()} mm")
        tp = np.array([[p["row"], p["col"], *p["centre"]] for p in truth["patches"]])
        tp = tp[np.lexsort((tp[:, 1], tp[:, 0]))][:, 2:]
        d = cen_mm - tp
        r = np.linalg.norm(d, axis=1)
        log(f"truth: swatch centres off by mean ({d[:, 0].mean():+.3f}, {d[:, 1].mean():+.3f}) mm, "
            f"RMS {np.sqrt((r ** 2).mean()):.3f} mm, max {r.max():.3f} mm")
        d0 = d - d.mean(0)
        log(f"       with the common offset removed: RMS {np.sqrt((d0 ** 2).sum(1).mean()):.3f} mm, "
            f"max {np.linalg.norm(d0, axis=1).max():.3f} mm")

    # overlay on the sheet region of the mosaic
    chk = cv2.cvtColor(mos, cv2.COLOR_GRAY2BGR)
    up = 4
    x0, y0 = np.floor(sheet_px.min(0) - 20).astype(int)
    x1, y1 = np.ceil(sheet_px.max(0) + 20).astype(int)
    crop = cv2.resize(chk[y0:y1, x0:x1], None, fx=up, fy=up, interpolation=cv2.INTER_CUBIC)
    P = lambda p: np.int32(np.round((np.asarray(p) - [x0, y0] + 0.5) * up - 0.5) * 16)
    cv2.polylines(crop, [P(sheet_px)], True, (0, 200, 0), 2, cv2.LINE_AA, 4)
    cv2.polylines(crop, [P(to_mosaic(block_r, H))], True, (255, 160, 0), 2, cv2.LINE_AA, 4)
    hs = np.array([[-sw, -sh], [sw, -sh], [sw, sh], [-sw, sh]], np.float32) / 2 / RECT_MM
    for c in cen_r:
        cv2.polylines(crop, [P(to_mosaic(c + hs, H))], True, (0, 220, 255), 1, cv2.LINE_AA, 4)
        q = P(to_mosaic(c[None], H))[0]
        cv2.drawMarker(crop, tuple(int(v) >> 4 for v in q), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    if truth:
        for t in tp:
            q = (np.asarray(t) / a.mm - 0.5 - [x0, y0] + 0.5) * up - 0.5
            cv2.circle(crop, tuple(int(round(v)) for v in q), 4, (255, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out / "swatch_check.png"), crop)
    (out / "swatches_report.txt").write_text("\n".join(lines) + "\n")
