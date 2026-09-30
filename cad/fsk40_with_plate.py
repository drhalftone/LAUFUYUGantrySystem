"""Place fsk40_carriage_plate on the FSK40 carriage (TraceParts STL) for a fit check.

The TraceParts model is Y-up with travel along X. Carriage top face is at
Y = 51.51; its 25 x 25 M4 pattern is centred at X = 612.68, Z = -101.15.
Writes fsk40_with_plate.stl (single mesh) and fsk40_with_plate.glb (coloured).
"""
import sys
import numpy as np
import trimesh

FSK40 = (sys.argv[1] if len(sys.argv) > 1 else
         r"C:\Users\dllau\Downloads\192475275-21-fsk40-e1250-10c7-bc-b57\fsk40-e1250-10c7-bc-b57.stl")
CARRIAGE_TOP_Y = 51.51
PATTERN_CENTER = (612.68, -101.15)   # (X, Z)

rail = trimesh.load(FSK40)
plate = trimesh.load("fsk40_carriage_plate.stl")

# plate local Z-up -> model Y-up: (x, y, z) -> (x, z, -y), then onto the carriage
T = np.eye(4)
T[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
T[:3, 3] = [PATTERN_CENTER[0], CARRIAGE_TOP_Y, PATTERN_CENTER[1]]
plate.apply_transform(T)

trimesh.util.concatenate([rail, plate]).export("fsk40_with_plate.stl")
rail.visual.face_colors = [170, 175, 185, 255]
plate.visual.face_colors = [230, 120, 30, 255]
trimesh.Scene({"fsk40": rail, "plate": plate}).export("fsk40_with_plate.glb")
print("plate bounds:", plate.bounds.round(2).tolist())
