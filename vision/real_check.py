"""Hole detection + lattice on one real camera frame, with a diagnostic overlay.

Run:  python real_check.py snaps/snap_....png
  -> <image>_det.png beside it: detections (yellow), lattice holes (green, labelled i,j every 4th),
     rejected detections (red); prints counts, pitch and the homography residual.
"""
import sys
from pathlib import Path
import numpy as np, cv2
import holegrid as hg

path = Path(sys.argv[1])
g = cv2.imread(str(path), 0)
H, W = g.shape
det = hg.detect_holes(g)
print(f"{path.name}: {W}x{H}, {len(det.centers)} hole detections; "
      f"diameter px median {np.median(det.axes.max(1)):.1f} (min {det.axes.max(1).min():.1f}, max {det.axes.max(1).max():.1f})")
lat = hg.grow_lattice(det.centers, image_size=(W, H), shapes=(det.axes, det.angles))
if lat is None:
    print("lattice: none found")
else:
    print(f"lattice: {len(lat.det)} holes, i {lat.ij[:, 0].min():.0f}..{lat.ij[:, 0].max():.0f}, "
          f"j {lat.ij[:, 1].min():.0f}..{lat.ij[:, 1].max():.0f}, homography RMS {lat.rms:.2f} px")

vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
on = set(lat.det.tolist()) if lat is not None else set()
for k, c in enumerate(det.centers):
    col = (0, 200, 0) if k in on else (0, 0, 255)
    cv2.circle(vis, tuple(np.int32(np.round(c))), int(det.axes[k].max() / 2) + 3, col, 2, cv2.LINE_AA)
if lat is not None:
    for (i, j), c in zip(lat.ij.astype(int), lat.uv):
        if i % 4 == 0 and j % 4 == 0:
            cv2.putText(vis, f"{i},{j}", tuple(np.int32(c + [8, -8])), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2, cv2.LINE_AA)
out = path.with_name(path.stem + "_det.png")
cv2.imwrite(str(out), vis)
print(f"overlay: {out}")
