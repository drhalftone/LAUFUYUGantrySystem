"""Put carriage_chain (plate + hanging links) on the FSK40 carriage for a fit check.

Writes fsk40_with_chain.glb (not committed: embeds the TraceParts FSK40 model).
Chain frame -> TraceParts model: X -> X + 579.18 (carriage motor-end face),
Z -> Y + 51.51 (carriage top), Y -> -Z about -101.15 (carriage centre line).
"""
import numpy as np
import trimesh
from carriage_chain import carriage_plate, hanging_chain

FSK40 = r"C:\Users\dllau\Downloads\192475275-21-fsk40-e1250-10c7-bc-b57\fsk40-e1250-10c7-bc-b57.stl"
T = np.eye(4)
T[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
T[:3, 3] = [579.18, 51.51, -101.15]


def mesh(wp):
    v, f = wp.val().tessellate(0.05, 0.2)
    return trimesh.Trimesh([(p.x, p.y, p.z) for p in v], f).apply_transform(T)


def chain_pieces(pin_above_table):
    """Hinge plate, draped links, screws and nuts as meshes in the model frame."""
    links, screws, nuts = hanging_chain(pin_above_table)
    pieces = {"plate": mesh(carriage_plate)}
    pieces.update({f"link{i + 1}": mesh(l) for i, l in enumerate(links)})
    pieces.update({f"screw{i + 1}": mesh(s) for i, s in enumerate(screws)})
    pieces.update({f"nut{i + 1}": mesh(n) for i, n in enumerate(nuts)})
    return pieces


if __name__ == "__main__":
    rail = trimesh.load(FSK40)
    pieces = chain_pieces(51.51 + 5.0 + 18.69)   # rail base on the table: pin is 75.2 mm up
    rail.visual.face_colors = [170, 175, 185, 255]
    scene = trimesh.Scene({"fsk40": rail})
    colors = {"plate": [230, 120, 30, 255], "link": [60, 120, 200, 255],
              "screw": [40, 40, 45, 255], "nut": [205, 205, 210, 255]}
    for name, m in pieces.items():
        m.visual.face_colors = colors[name.rstrip("0123456789")]
        scene.add_geometry(m, node_name=name)
    scene.export("fsk40_with_chain.glb")

    # clearance: closest FSK40 vertex to each piece (negative = inside it)
    for name, m in pieces.items():
        near = rail.vertices[np.all((rail.vertices > m.bounds[0] - 15) & (rail.vertices < m.bounds[1] + 15), axis=1)]
        if len(near):
            d = -trimesh.proximity.signed_distance(m, near)   # >0 outside
            print(f"{name}: closest FSK40 point {d.min():.2f} mm")
        else:
            print(f"{name}: nothing of the FSK40 within 15 mm")
    print("chain bottom Y =", round(min(m.bounds[0][1] for m in pieces.values()), 2),
          "(rail base underside is Y = -18.69)")
