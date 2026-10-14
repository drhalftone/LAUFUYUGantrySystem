"""Frame grabbing from the Basler on the carriage mast (camera.py: acA1920-150um, mono, 1920 x 1200, 5.86 um).

Real-camera counterpart of render_scan.py: frames are saved as 8-bit grey PNGs named frame_<x>_<y>.png with
a scan.json beside them (image size, pixel pitch, nominal lens, camera settings, and each frame's carriage
position: the Qt app's slider mm), so scans from the gantry go through the same pipeline as the synthetic ones.

Camera(): opens the first Basler (or one by serial), fixes exposure and gain (no auto: the frames of a scan
must be comparable for the flat-field), and grab(n) returns the mean of n frames as float32 -- averaging
cuts the read and shot noise for the sub-pixel hole and edge fits.

Run:  python grab.py --list                                 cameras on the bus
      python grab.py --live [--exposure 20]                 preview: +/- exposure, s save, a auto-expose, q quit
      python grab.py --auto                                 find an exposure that puts the 99th percentile at ~85% full scale
      python grab.py --out scan_real --at X,Y [--avg 8] [--lens-mm 6]
                                                            one frame at carriage (X, Y) mm, appended to scan.json
"""
import argparse, json, time
from pathlib import Path
import numpy as np, cv2
from pypylon import pylon
from camera import PIX, F_MM

HERE = Path(__file__).parent


def devices():
    return pylon.TlFactory.GetInstance().EnumerateDevices()


class Camera:
    def __init__(self, serial=None, exposure_ms=None, gain_db=0.0):
        tl = pylon.TlFactory.GetInstance()
        if serial:
            info = pylon.DeviceInfo(); info.SetSerialNumber(str(serial))
            dev = tl.CreateDevice(info)
        else:
            dev = tl.CreateFirstDevice()
        self.cam = pylon.InstantCamera(dev)
        self.cam.Open()
        c = self.cam
        c.PixelFormat.SetValue("Mono8")
        c.ExposureAuto.SetValue("Off")
        c.GainAuto.SetValue("Off")
        c.Gain.SetValue(gain_db)
        if exposure_ms is not None:
            self.exposure_ms = exposure_ms

    @property
    def exposure_ms(self):
        return self.cam.ExposureTime.GetValue() / 1000.0

    @exposure_ms.setter
    def exposure_ms(self, ms):
        e = self.cam.ExposureTime
        e.SetValue(float(np.clip(ms * 1000.0, e.GetMin(), e.GetMax())))

    def info(self):
        c, d = self.cam, self.cam.GetDeviceInfo()
        return {"model": d.GetModelName(), "serial": d.GetSerialNumber(),
                "firmware": c.DeviceFirmwareVersion.GetValue(), "pixel_format": c.PixelFormat.GetValue(),
                "exposure_ms": self.exposure_ms, "gain_db": c.Gain.GetValue()}

    def grab(self, n=1, timeout_ms=5000):
        """Mean of n consecutive frames, float32 (H, W), 0..255."""
        acc = None
        self.cam.StartGrabbingMax(n)
        try:
            while self.cam.IsGrabbing():
                r = self.cam.RetrieveResult(timeout_ms, pylon.TimeoutHandling_ThrowException)
                try:
                    if not r.GrabSucceeded():
                        raise RuntimeError(f"grab failed: {r.ErrorCode} {r.ErrorDescription}")
                    a = r.Array.astype(np.float32)
                    acc = a if acc is None else acc + a
                finally:
                    r.Release()
        finally:
            self.cam.StopGrabbing()
        return acc / n

    def auto_expose(self, target=0.85 * 255, pct=99.0, rounds=8):
        """Scale the exposure until the pct-th percentile sits at target (the frame's bright end, not its mean)."""
        for _ in range(rounds):
            v = np.percentile(self.grab(1), pct)
            if abs(v - target) < 0.04 * target:
                break
            k = 4.0 if v >= 254 else target / max(v, 1.0)     # clipped: back off hard, the true level is unknown
            self.exposure_ms = self.exposure_ms * (1 / k if v >= 254 else min(k, 8.0))
        return self.exposure_ms

    def close(self):
        self.cam.Close()


def save_frame(cam, out, at, avg, lens_mm=F_MM):
    out.mkdir(exist_ok=True)
    path = out / "scan.json"
    scan = json.load(open(path)) if path.exists() else {"frames": []}
    img = cam.grab(avg)
    H, W = img.shape
    scan.update({"image_size": [W, H], "pixel_mm": PIX, "focal_mm": lens_mm, "camera": cam.info()})
    name = f"frame_{at[0]:+04.0f}_{at[1]:+04.0f}.png"
    cv2.imwrite(str(out / name), np.clip(img + 0.5, 0, 255).astype(np.uint8))
    scan["frames"] = [f for f in scan["frames"] if f["file"] != name] + [
        {"file": name, "carriage_mm": [float(at[0]), float(at[1])], "averaged": avg,
         "exposure_ms": cam.exposure_ms, "time": time.strftime("%Y-%m-%dT%H:%M:%S")}]
    json.dump(scan, open(path, "w"), indent=1)
    print(f"{out.name}/{name}: mean {img.mean():.1f}, p99 {np.percentile(img, 99):.1f}, "
          f"saturated {np.mean(img >= 254.5) * 100:.2f}%  ({len(scan['frames'])} frames in scan.json)")


def live(cam, out):
    """Preview at half size with the clipped pixels in red. +/- exposure x1.25, a auto, s save to out/, q quit."""
    print(__doc__.split("Run:")[0].strip().splitlines()[0])
    print("keys: + / - exposure, a auto-expose, s save snapshot, q or Esc quit")
    win = "Basler"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cam.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    try:
        while cam.cam.IsGrabbing():
            r = cam.cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if not r.GrabSucceeded():
                r.Release(); continue
            g = r.Array.copy(); r.Release()
            v = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            v[g >= 255] = (0, 0, 255)
            v = cv2.resize(v, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.putText(v, f"exp {cam.exposure_ms:.2f} ms  mean {g.mean():.0f}  p99 {np.percentile(g[::4, ::4], 99):.0f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(win, v)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k in (ord("+"), ord("=")):
                cam.exposure_ms *= 1.25
            elif k in (ord("-"), ord("_")):
                cam.exposure_ms /= 1.25
            elif k in (ord("a"), ord("s")):
                cam.cam.StopGrabbing()
                if k == ord("a"):
                    print(f"auto exposure: {cam.auto_expose():.2f} ms")
                else:
                    out.mkdir(exist_ok=True)
                    name = out / time.strftime("snap_%Y%m%d_%H%M%S.png")
                    cv2.imwrite(str(name), g); print(f"saved {name}")
                cam.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    finally:
        cam.cam.StopGrabbing()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="list cameras and exit")
    ap.add_argument("--serial", help="camera serial number (default: the first one found)")
    ap.add_argument("--exposure", type=float, help="exposure, ms (default: whatever the camera has)")
    ap.add_argument("--gain", type=float, default=0.0, help="gain, dB")
    ap.add_argument("--auto", action="store_true", help="auto-expose once before grabbing")
    ap.add_argument("--live", action="store_true", help="live preview window")
    ap.add_argument("--out", default="snaps", help="output folder (under vision/)")
    ap.add_argument("--at", help="carriage X,Y in mm: save one frame of a scan at this position")
    ap.add_argument("--avg", type=int, default=8, help="frames averaged per saved frame")
    ap.add_argument("--lens-mm", type=float, default=F_MM, help="nominal lens focal length, recorded in scan.json")
    a = ap.parse_args()

    if a.list:
        for d in devices():
            print(f"{d.GetModelName()}  serial {d.GetSerialNumber()}  ({d.GetDeviceClass()})")
        raise SystemExit
    cam = Camera(a.serial, a.exposure, a.gain)
    try:
        info = cam.info()
        print(f"{info['model']} {info['serial']}: exposure {info['exposure_ms']:.2f} ms, gain {info['gain_db']:.1f} dB")
        if a.auto:
            print(f"auto exposure: {cam.auto_expose():.2f} ms")
        out = HERE / a.out
        if a.live:
            live(cam, out)
        elif a.at:
            save_frame(cam, out, [float(v) for v in a.at.split(",")], a.avg, a.lens_mm)
        elif not a.auto:
            out.mkdir(exist_ok=True)
            name = out / time.strftime("snap_%Y%m%d_%H%M%S.png")
            cv2.imwrite(str(name), np.clip(cam.grab(a.avg) + 0.5, 0, 255).astype(np.uint8))
            print(f"saved {name}")
    finally:
        cam.close()
