"""Overview of the platen: mosaic, detected sheet and swatch centres, what the camera saw, and where the
i1 aperture can go (full motor travel, and the usable part where the plate stays on the platen and clear
of the rails). Flags any swatch the i1 cannot reach.

Run:  python overview.py scan_paper15 --aperture scan_aperture [--mm 0.25]
  -> <scan>/out/platen_overview.png
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

HERE = Path(__file__).parent
LEG_STROKE, BEAM_STROKE = 310.0, 610.0
from aperture import RAIL_X, RAIL_Y
RAILS_AT_Y, RAILS_X = (2.7, 606.9), (100.6, 609.6)   # rail centre lines and motor-to-end-plate extent (gantry_on_plate.py)

ap_ = argparse.ArgumentParser()
ap_.add_argument("scan", nargs="?", default="scan_paper15")
ap_.add_argument("--aperture", default="scan_aperture", help="folder whose out/ holds aperture.json")
ap_.add_argument("--mm", type=float, default=0.25, help="mosaic resolution (as mosaic.py)")
a = ap_.parse_args()
MM = a.mm
out = HERE / a.scan / "out"
mos = cv2.imread(str(out / "platen_mosaic.png"), 0)
sw = json.load(open(out / "swatches.json"))
ap = json.load(open(HERE / a.aperture / "out" / "aperture.json")); A0 = np.array(ap["A0_mm"])
ex, ey = np.array(ap["ex"])[:2], np.array(ap["ey"])[:2]
XL, YL = RAIL_X, RAIL_Y
hx, hy = LEG_STROKE / 2, BEAM_STROKE / 2
reach = np.array([A0 + x * ex + y * ey for x, y in ((XL[0], YL[0]), (XL[1], YL[0]), (XL[1], YL[1]), (XL[0], YL[1]))])
full = np.array([A0 + x * ex + y * ey for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])
cen = np.array([s["centre_mm"] for s in sw["swatches"]])
inside = cv2.pointPolygonTest
ok = np.array([inside(reach.astype(np.float32), tuple(map(float, c)), False) >= 0 for c in cen])

fig, ax = plt.subplots(figsize=(13, 12.5))
ax.imshow(mos, cmap="gray", extent=(0, mos.shape[1] * MM, mos.shape[0] * MM, 0), vmin=0, vmax=255)
seen = (mos > 0).astype(np.uint8)
cs, _ = cv2.findContours(seen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for k, c in enumerate(cs):
    if cv2.contourArea(c) > 2000:
        p = (c.reshape(-1, 2) + 0.5) * MM
        ax.add_patch(Polygon(p, fill=False, ec="#33d0ff", lw=1.8, label="seen by the camera (15 frames)" if k == 0 else None))
ax.add_patch(Rectangle((0, 0), 609.6, 609.6, fill=False, ec="w", lw=1.2, ls=":", label="platen edge"))
for yc in RAILS_AT_Y:
    ax.add_patch(Rectangle((RAILS_X[0], yc - 20), RAILS_X[1] - RAILS_X[0], 40, fc="#8d8f94", ec="#55575c", alpha=0.6,
                           label="X rails" if yc < 10 else None))
ax.add_patch(Polygon(full, fill=False, ec="#ff9f1c", lw=1.5, ls="--", label="i1 aperture, full motor travel"))
ax.add_patch(Polygon(reach, fill=False, ec="#ffd21f", lw=2.5, label="i1 aperture, usable (plate on platen, clear of rails)"))
ax.add_patch(Polygon(np.array(sw["sheet_corners_mm"]), fill=False, ec="#2ecc40", lw=1.8, label="sheet (detected)"))
ax.add_patch(Polygon(np.array(sw["block_corners_mm"]), fill=False, ec="#1f77ff", lw=1.8, label="swatch block (detected)"))
ax.plot(cen[ok, 0], cen[ok, 1], "+", color="#ff2050", ms=3.2, mew=0.7, label=f"swatch centres ({ok.sum()} reachable)")
if (~ok).any():
    ax.plot(cen[~ok, 0], cen[~ok, 1], "x", color="#ff00ff", ms=4, label=f"unreachable ({(~ok).sum()})")
ax.plot(*A0, "o", ms=9, mfc="none", mec="#ffd21f", mew=2, label="i1 aperture at home (measured)")
ax.set_xlim(-40, 650); ax.set_ylim(650, -40)
ax.set_xlabel("X along the rails (mm)  ->  far end"); ax.set_ylabel("Y along the beam (mm)")
ax.legend(loc="lower right", fontsize=9, framealpha=0.92)
ax.set_title(f"Platen mosaic ({a.scan}) with the detected sheet, swatch centres and the i1's scannable area")
fig.tight_layout(); fig.savefig(str(out / "platen_overview.png"), dpi=130)
print("reach X", reach[:, 0].min().round(1), reach[:, 0].max().round(1), "Y", reach[:, 1].min().round(1), reach[:, 1].max().round(1),
      "| swatches reachable", ok.sum(), "/", len(ok))
