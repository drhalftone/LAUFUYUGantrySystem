"""Simple live camera view: the picture, automatically exposed and evenly lit. Nothing to set.

- Grabs at the camera's full bit depth (Mono10, 0..1023).
- Exposure adjusts itself continuously: the brightest 0.5% of the picture is kept just under clipping.
- Local brightness levelling is always on: the brightness of each neighbourhood (a wide blur, much
  larger than a hole) is pulled toward the picture's overall level -- dark regions brightened, bright
  regions darkened -- while detail smaller than the neighbourhood (holes, edges, marks) keeps its
  contrast. Done on the full 10-bit data in log space; only the final picture is reduced to 8 bits.

Run:  python live_clahe.py
Keys: S save a picture (to vision/snaps/)   Q or Esc close
"""
import time
from pathlib import Path
import numpy as np, cv2
from pypylon import pylon

HERE = Path(__file__).parent
FULL = 1023                 # Mono10 full scale
TARGET = 0.90 * FULL        # where the bright end (99.5th percentile) is kept
NEIGHBOURHOOD_PX = 120      # size of a "neighbourhood" (blur sigma, px): several hole pitches
STRENGTH = 0.85             # 0 = no levelling, 1 = every neighbourhood pulled fully to the same level
WIN = "Camera"


def open_camera():
    cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    cam.Open()
    cam.PixelFormat.SetValue("Mono10")
    cam.ExposureAuto.SetValue("Off")
    cam.GainAuto.SetValue("Off")
    cam.Gain.SetValue(0.0)
    return cam


def adjust_exposure(cam, raw):
    """One gentle auto-exposure step: move the bright end toward TARGET (damped, so it doesn't flicker)."""
    v = float(np.percentile(raw[::4, ::4], 99.5))
    e = cam.ExposureTime
    if v >= FULL - 1:
        factor = 0.5                                   # clipped: the true level is unknown, back off
    elif abs(v - TARGET) < 0.05 * TARGET:
        return                                         # close enough: leave it alone
    else:
        factor = (TARGET / max(v, 1.0)) ** 0.5         # half the way there each step (in log terms)
        factor = min(max(factor, 0.5), 2.0)
    e.SetValue(float(np.clip(e.GetValue() * factor, e.GetMin(), e.GetMax())))


def level_linear(raw):
    """The same local levelling, but kept LINEAR in brightness (float32, ~0..1): each neighbourhood is
    scaled by (overall level / local level)^STRENGTH, so a hole stays exactly as many times darker than
    the board around it as in the raw frame.  Use this for measuring (hole detection); picture() is
    log-encoded for the screen, which compresses those ratios."""
    x = raw.astype(np.float32) + 1.0
    lg = np.log(x)
    s = 8
    small = cv2.resize(lg, (lg.shape[1] // s, lg.shape[0] // s), interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), NEIGHBOURHOOD_PX / s)
    local = cv2.resize(small, (lg.shape[1], lg.shape[0]), interpolation=cv2.INTER_LINEAR)
    out = x * np.exp(-STRENGTH * (local - float(np.median(small))))
    return out / max(float(np.percentile(out[::4, ::4], 99.8)), 1e-6)


def picture(raw):
    """10-bit frame -> evenly lit 8-bit picture.

    In log space the frame is (local brightness) + (detail).  The local brightness is a wide blur
    (computed at 1/8 scale -- it is smooth anyway); subtracting STRENGTH x its deviation from the
    overall level brightens dark neighbourhoods and darkens bright ones without flattening the detail.
    The result is then stretched to the screen with one global mapping (no per-region curves)."""
    lg = np.log(raw.astype(np.float32) + 1.0)
    s = 8
    small = cv2.resize(lg, (lg.shape[1] // s, lg.shape[0] // s), interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), NEIGHBOURHOOD_PX / s)
    local = cv2.resize(small, (lg.shape[1], lg.shape[0]), interpolation=cv2.INTER_LINEAR)
    level = float(np.median(small))
    out = lg - STRENGTH * (local - level)
    lo, hi = np.percentile(out[::4, ::4], (0.5, 99.8))
    return np.clip((out - lo) * (245.0 / max(hi - lo, 1e-6)) + 5.0, 0, 255).astype(np.uint8)


def main():
    cam = open_camera()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 800)
    saved_msg_until = 0.0
    n = 0
    try:
        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        while True:
            r = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if not r.GrabSucceeded():
                r.Release()
                continue
            raw = r.Array.copy()
            r.Release()
            n += 1
            if n % 3 == 0:                             # a new exposure takes a frame or two to show up
                adjust_exposure(cam, raw)

            pic = picture(raw)
            v = cv2.cvtColor(pic, cv2.COLOR_GRAY2BGR)
            hint = "Saved" if time.time() < saved_msg_until else "S = save picture     Q = close"
            cv2.putText(v, hint, (20, v.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(v, hint, (20, v.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WIN, v)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k in (ord("s"), ord("S")):
                out = HERE / "snaps"
                out.mkdir(exist_ok=True)
                stem = time.strftime("picture_%Y%m%d_%H%M%S")
                cv2.imwrite(str(out / f"{stem}.png"), pic)
                cv2.imwrite(str(out / f"{stem}_raw10.png"), raw)      # 16-bit PNG holding the 10-bit values
                saved_msg_until = time.time() + 1.5
    finally:
        cam.StopGrabbing()
        cam.Close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
