"""Lens calibration from ONE real frame of the breadboard.

1. Grab a Mono10 frame (auto-exposed) -- or load a saved *_raw10.png -- and level it with the same
   local brightness levelling as the live view (live_clahe.picture).
2. Detect holes and fit the lattice with the lens-aware grid model (holegrid).
3. The board is a plane of points at a known 25.4 mm pitch, so cv2.calibrateCamera on the labelled
   holes measures the focal length, radial distortion (k1, k2) and the camera's pose over the board.
   Principal point held at the image centre, square pixels, no tangential term (one view can't
   separate those from the pose).

Outputs in vision/calib/<stamp>/:
  frame_raw10.png, frame_level.png      the frame (16-bit 10-bit values; levelled 8-bit)
  holes.png       confirmed (green) / suspected (orange) / expected-not-seen (magenta x),
                  reprojection error of each hole as a red line x50
  undistorted.png the levelled frame with the lens distortion removed (rows of holes come out straight)
  calibration.json, report.txt

Run:  python calibrate_frame.py              grab from the camera (close the live view first)
      python calibrate_frame.py FILE_raw10.png
"""
import json, sys, time
from pathlib import Path
import numpy as np, cv2
import holegrid as hg
from holegrid import PITCH
from camera import PIX, F_MM, TILT_DEG
import live_clahe as LV

HERE = Path(__file__).parent


def grab_raw10():
    cam = LV.open_camera()
    try:
        cam.StartGrabbing(LV.pylon.GrabStrategy_LatestImageOnly)
        raw = None
        for _ in range(40):                         # let the auto-exposure settle
            r = cam.RetrieveResult(5000, LV.pylon.TimeoutHandling_ThrowException)
            raw = r.Array.copy()
            r.Release()
            before = cam.ExposureTime.GetValue()
            LV.adjust_exposure(cam, raw)
            if cam.ExposureTime.GetValue() == before:
                break
        cam.StopGrabbing()
        # average 8 frames at the settled exposure: less noise for the sub-pixel hole centres
        acc = np.zeros(raw.shape, np.float64)
        cam.StartGrabbingMax(8)
        while cam.IsGrabbing():
            r = cam.RetrieveResult(5000, LV.pylon.TimeoutHandling_ThrowException)
            acc += r.Array
            r.Release()
        exp_ms = cam.ExposureTime.GetValue() / 1000
    finally:
        cam.Close()
    return np.round(acc / 8).astype(np.uint16), exp_ms


def main():
    if len(sys.argv) > 1:
        raw = cv2.imread(sys.argv[1], cv2.IMREAD_UNCHANGED)
        exp_ms, source = None, sys.argv[1]
    else:
        raw, exp_ms = grab_raw10()
        source = "camera"
    H, W = raw.shape
    out = HERE / "calib" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True)
    level = LV.picture(raw)
    cv2.imwrite(str(out / "frame_raw10.png"), raw)
    cv2.imwrite(str(out / "frame_level.png"), level)

    # detect on the LINEAR levelled frame (hole-to-board ratios intact); `level` is log-encoded for viewing
    lin8 = np.clip(LV.level_linear(raw) * 240.0, 0, 255).astype(np.uint8)
    cv2.imwrite(str(out / "frame_level_linear.png"), lin8)
    det = hg.detect_holes(lin8)
    lat = hg.grow_lattice(det.centers, image_size=(W, H), shapes=(det.axes, det.angles))
    if lat is None:
        sys.exit("no hole grid found")

    # --- planar calibration on the labelled holes
    obj = np.c_[lat.ij * PITCH, np.zeros(len(lat.ij))].astype(np.float32)
    img = lat.uv.astype(np.float32)
    f0 = F_MM / PIX
    K0 = np.array([[f0, 0, W / 2], [0, f0, H / 2], [0, 0, 1]])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
             | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    rms, K, dist, rv, tv = cv2.calibrateCamera([obj], [img], (W, H), K0, np.zeros(5), flags=flags)
    dist = np.asarray(dist).ravel()
    # drop gross outliers (> 3x RMS) and refit once
    proj = cv2.projectPoints(obj, rv[0], tv[0], K, dist)[0].reshape(-1, 2)
    err = np.linalg.norm(proj - img, axis=1)
    keep = err < max(3 * rms, 1.0)
    if (~keep).any():
        rms, K, dist, rv, tv = cv2.calibrateCamera([obj[keep]], [img[keep]], (W, H), K, dist, flags=flags)
        dist = np.asarray(dist).ravel()
        proj = cv2.projectPoints(obj, rv[0], tv[0], K, dist)[0].reshape(-1, 2)
        err = np.linalg.norm(proj - img, axis=1)

    R = cv2.Rodrigues(rv[0])[0]
    C = (-R.T @ tv[0]).ravel()                       # camera centre in the board (lattice) frame, mm
    axis = R.T @ [0, 0, 1]                          # optical axis in the board frame
    tilt = np.degrees(np.arccos(abs(axis[2])))
    f_mm = K[0, 0] * PIX
    # distortion at the image corner, px
    corner = np.array([[[0.0, 0.0]]])
    und = cv2.undistortPoints(corner, K, dist, P=K).reshape(2)
    corner_px = float(np.hypot(*(und - corner.reshape(2))))

    rep = [f"source: {source}" + (f", exposure {exp_ms:.2f} ms (8-frame average)" if exp_ms else ""),
           f"holes: {len(det.centers)} detected, {len(lat.ij)} on the grid "
           f"({len(lat.ij) - int(keep.sum())} dropped as calibration outliers), "
           f"{0 if lat.expected is None else len(lat.expected)} expected but not detected",
           f"grid-model fit (homography + radial): RMS {lat.rms:.2f} px",
           "",
           f"CALIBRATION (one view, principal point fixed at the centre)",
           f"  reprojection RMS {rms:.3f} px   (max {err[keep].max():.2f} px)",
           f"  focal length {K[0, 0]:.1f} px = {f_mm:.3f} mm   (nominal lens {F_MM} mm)",
           f"  distortion k1 {dist[0]:+.4f}  k2 {dist[1]:+.4f}   -> moves the image corner {corner_px:.0f} px",
           f"  camera {abs(C[2]):.0f} mm above the board, optical axis tilted {tilt:.1f} deg from vertical "
           f"(CAD {TILT_DEG:.1f} deg)",
           "",
           "notes: hole centres are the centres of the dark ellipses; off-axis the camera sees down the hole",
           "wall, which shifts them slightly. One view fixes f and k1/k2 well only because the board is",
           "tilted in view; several views (carriage positions) will firm them up."]
    print("\n".join(rep))
    (out / "report.txt").write_text("\n".join(rep) + "\n")
    json.dump({"K": K.tolist(), "dist": dist.ravel().tolist(), "rms_px": rms, "image_size": [W, H],
               "pixel_mm": PIX, "f_mm": f_mm, "tilt_deg": tilt, "camera_height_mm": float(abs(C[2])),
               "n_holes": int(keep.sum()), "exposure_ms": exp_ms, "source": source},
              open(out / "calibration.json", "w"), indent=1)

    # --- pictures
    v = cv2.cvtColor(level, cv2.COLOR_GRAY2BGR)
    on = set(lat.det.tolist())
    for k, c in enumerate(det.centers):
        r = int(det.axes[k].max() / 2) + 4
        cc = tuple(np.int32(np.round(c)))
        cv2.circle(v, cc, r, (0, 220, 0) if k in on else (0, 165, 255), 2 if k in on else 3, cv2.LINE_AA)
    if lat.expected is not None:
        for p in lat.model(lat.expected.astype(float)):
            x, y = int(round(p[0])), int(round(p[1]))
            cv2.line(v, (x - 9, y - 9), (x + 9, y + 9), (255, 0, 255), 2, cv2.LINE_AA)
            cv2.line(v, (x - 9, y + 9), (x + 9, y - 9), (255, 0, 255), 2, cv2.LINE_AA)
    for p, q, ok in zip(img, proj, keep):           # reprojection error x50
        e = p + 50 * (q - p)
        cv2.line(v, tuple(np.int32(p)), tuple(np.int32(e)), (0, 0, 255) if ok else (128, 0, 128), 2, cv2.LINE_AA)
    lines = [f"{len(lat.ij)} holes on the grid   reprojection RMS {rms:.2f} px (red lines = error x50)",
             f"f {f_mm:.2f} mm   k1 {dist[0]:+.3f}  k2 {dist[1]:+.3f}   tilt {tilt:.1f} deg   "
             f"height {abs(C[2]):.0f} mm"]
    cv2.rectangle(v, (0, 0), (W, 34 * len(lines) + 14), (0, 0, 0), -1)
    for n, s in enumerate(lines):
        cv2.putText(v, s, (14, 38 + 34 * n), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(out / "holes.png"), v)

    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (W, H), 0)
    und = cv2.undistort(level, K, dist, None, newK)
    cv2.imwrite(str(out / "undistorted.png"), und)
    print(f"\nwritten to {out}")
    return out


if __name__ == "__main__":
    main()

