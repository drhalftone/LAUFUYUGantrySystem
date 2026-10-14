"""The carriage camera: Basler acA1920-150um (ace classic USB 3, Sony IMX174 mono, 1920 x 1200, 5.86 um pixels)
behind a nominal 6 mm C-mount lens. One place for the sensor numbers the simulator, the grabber and the
calibration share; the lens is only a starting guess (scan_map.py measures the focal length and distortion).
"""
import numpy as np

MODEL = "acA1920-150um"
W, H = 1920, 1200          # full-resolution ROI (the sensor's 1984 x 1264 less Basler's border pixels)
PIX = 5.86e-3              # pixel pitch, mm
F_MM = 6.0                 # nominal lens focal length, mm

# Orientation in the board frame (rows: image x right, image y down, optical axis), from the bracket in
# cad/camera_mount.py carried through the gantry (cad/gantry_on_plate.py: chain frame -> board is +90 deg
# about Zb): the camera looks toward -Xb (the legs' motor end), TILT_DEG off vertical, image x along +Yb,
# sitting upright on its bottom face. Only the CAD's word for which way the board is turned in the image:
# scan_map.py measures the real orientation.
TILT_DEG = 25.07
_s, _c = np.sin(np.deg2rad(TILT_DEG)), np.cos(np.deg2rad(TILT_DEG))
R_CAD = np.array([[0.0, 1.0, 0.0], [_c, 0.0, -_s], [-_s, 0.0, -_c]])


def nominal_K(f_mm=F_MM, pixel_mm=PIX, size=(W, H)):
    """Pinhole matrix for a lens of f_mm on this sensor, principal point at the image centre."""
    f = f_mm / pixel_mm
    return np.array([[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1]])
