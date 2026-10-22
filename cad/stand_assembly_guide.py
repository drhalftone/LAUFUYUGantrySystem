"""Step-by-step assembly guide for the 80/20 gantry stand (bench box + wheeled cart).

Every rail, post, bracket, T-nut and panel position comes from gantry_cart.py, so the guide tracks
the model. The vendor STEP profiles aren't needed: members are drawn as plain boxes (their sizes and
positions from gantry_cart.member calls), and each step is rendered with a small z-buffer rasteriser
into a PNG with SVG callouts on top.

Run:  python stand_assembly_guide.py   -> stand_assembly_guide.html (images embedded)
"""
import base64
import html
import io
from collections import defaultdict
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MplPath
from PIL import Image

import cart_hardware as hw
import gantry_cart as g

W, H, SS = 1100, 720, 2              # output image size, supersampling
# gantry_cart puts the boom top flush with the mast top, which leaves the boom's upper 4132 with no
# mast to bolt to. The mast is already cut, so the guide drops the boom this far (lens 35 in up).
BOOM_DROP = 1.0
P10, P20, E = g.P10, g.P20, g.EDGE_ROW


# ---------------------------------------------------------------- geometry
def box_faces(lo, hi):
    """Axis-aligned box -> [(4x3 vertices, normal)]."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)
    quads = [((0, 1, 3, 2), (-1, 0, 0)), ((4, 6, 7, 5), (1, 0, 0)), ((0, 4, 5, 1), (0, -1, 0)),
             ((2, 3, 7, 6), (0, 1, 0)), ((0, 2, 6, 4), (0, 0, -1)), ((1, 5, 7, 3), (0, 0, 1))]
    return [(v[list(q)], np.array(n, float)) for q, n in quads]


def cyl_faces(c, axis, r, length, n=20):
    """Cylinder from c along unit axis (x/y/z vector) -> prism faces."""
    c, a = np.array(c, float), np.array(axis, float)
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < 0.9 else [1, 0, 0]); u /= np.linalg.norm(u)
    w = np.cross(a, u)
    ring = [c + r * (np.cos(t) * u + np.sin(t) * w) for t in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    top = [p + a * length for p in ring]
    faces = [(np.array(ring[::-1]), -a), (np.array(top), a)]
    for i in range(n):
        j = (i + 1) % n
        mid = (ring[i] + ring[j]) / 2 - c
        mid -= a * (mid @ a)
        faces.append((np.array([ring[i], ring[j], top[j], top[i]]), mid / np.linalg.norm(mid)))
    return faces


class Part:
    def __init__(self, name, kind, faces, bbox=None):
        self.name, self.kind, self.faces = name, kind, faces
        pts = np.vstack([f[0] for f in faces])
        self.lo, self.hi = (pts.min(0), pts.max(0)) if bbox is None else bbox
        self.center = (self.lo + self.hi) / 2


def box_part(name, kind, lo, hi):
    return Part(name, kind, box_faces(lo, hi), (np.array(lo, float), np.array(hi, float)))


def member_box(stock, length, start, direction):
    """Same placement rule as gantry_cart.member: section centred on start, length along direction."""
    s, (x, y, z) = stock / 2, start
    if direction == "x":
        return (x, y - s, z - s), (x + length, y + s, z + s)
    if direction == "y":
        return (x - s, y, z - s), (x + s, y + length, z + s)
    return (x - s, y - s, z), (x + s, y + s, z + length)


def frame_members():
    """{name: Part} for every 1010/2020 member, by calling gantry_cart's frame builders."""
    real = g.member
    g.member = lambda stock, length, start, d: member_box(stock, length, start, d)
    try:
        items = g.box_frame(P10, P20) + g.cart_frame(P10, P20) + g.camera_mast(P10)[:2]
    finally:
        g.member = real
    out = {n: box_part(n, "2020" if n.endswith("2020") else "1010", *lohi) for n, lohi, *_ in items}
    b = out["camera_boom_1010"]
    out["camera_boom_1010"] = box_part(b.name, b.kind, b.lo - [0, 0, BOOM_DROP], b.hi - [0, 0, BOOM_DROP])
    return out


def bracket(name, corner, n1, n2):
    """4132 as two plates: flange A on face 1 running along n2, flange B on face 2 running along n1."""
    c, n1, n2 = (np.array(v, float) for v in (corner, n1, n2))
    wdir = np.cross(n1, n2)
    corners = lambda la, lt: np.array([c + a * n2 + b * wdir + t * n1 for a in (0, la) for b in (-hw.W / 2, hw.W / 2) for t in (0, lt)])
    faces = []
    for P in (corners(hw.FL, hw.T), corners(hw.T, hw.FL)):
        faces += box_faces(P.min(0), P.max(0))
    return Part(name, "4132", faces)


def bracket_nuts(corner, n1, n2):
    """(point just inside the member, its face normal) for bolt A into face 1 and bolt B into face 2."""
    c, n1, n2 = (np.array(v, float) for v in (corner, n1, n2))
    return [(c + n2 * hw.HOLE_AT - n1 * 0.2, n1), (c + n1 * hw.HOLE_AT - n2 * 0.2, n2)]


def shelf_panel():
    z0, z1, B = g.SHELF_RAIL_TOP, g.SHELF_PANEL_TOP, g.BOARD
    faces = box_faces((P20, 0, z0), (B - P20, B, z1)) + box_faces((0, P20, z0), (P20, B - P20, z1)) \
        + box_faces((B - P20, P20, z0), (B, B - P20, z1))
    return Part("shelf_panel", "panel", faces, (np.array([0, 0, z0]), np.array([B, B, z1])))


def laptop_panel():
    return box_part("laptop_panel", "panel", (-g.LAPTOP_DEPTH, 0, g.CART_TOP),
                    (-g.LAPTOP_DEPTH + g.LAPTOP_PANEL_L, g.BOARD, g.CART_TOP + g.PANEL_T))


def locator_parts():
    """4166 locator plates (same layout as gantry_cart.locators) and the bolt points into rail/leg slots."""
    parts, nuts = [], []
    g0, g1 = g.LOC_GAP, g.LOC_GAP + g.LOC_T
    z_rail, z_leg = g.CART_TOP - P10 / 2, g.LEG_TOP - 0.5
    z_lo, z_hi = g.CART_TOP - g.LOC_DOWN, g.CART_TOP + g.LOC_UP
    for i, (cx, cy) in enumerate(((0, 0), (g.BOARD, 0), (0, g.BOARD), (g.BOARD, g.BOARD))):
        ox, oy = (-1 if cx == 0 else 1), (-1 if cy == 0 else 1)
        laptop_end = cx == 0
        xs, ys = sorted((cx, cx - ox * g.LOC_W)), sorted((cy + oy * g0, cy + oy * g1))
        faces = box_faces((xs[0], ys[0], z_lo), (xs[1], ys[1], z_hi))
        pts = [((0, oy, 0), (cx - ox * a, cy, z)) for a in (0.5, 1.5) for z in (z_rail, z_leg)]
        if not laptop_end:
            xs, ys = sorted((cx + ox * g0, cx + ox * g1)), sorted((cy, cy - oy * g.LOC_W))
            faces += box_faces((xs[0], ys[0], z_lo), (xs[1], ys[1], z_hi))
            pts += [((ox, 0, 0), (cx, cy - oy * 1.5, z_rail))] + [((ox, 0, 0), (cx, cy - oy * a, z_leg)) for a in (0.5, 1.5)]
        parts.append(Part(f"locator{i + 1}", "angle", faces))
        nuts += [(np.array(at) - 0.2 * np.array(n, float), np.array(n, float)) for n, at in pts]
    return parts, nuts


def caster_parts():
    out = []
    for i, (x, y) in enumerate(g.LEG_XY):
        u = 1 if y < g.BOARD / 2 else -1
        cx, cy, ch = x, y + 0.5 * u, g.CASTER_H
        out.append(box_part(f"plate{i + 1}", "2419", (cx - 1.875, cy - 1.25, ch), (cx + 1.875, cy + 1.25, ch + 0.5)))
        cf = [box_faces((cx - 1.8, cy - 1.2, ch - 0.22), (cx + 1.8, cy + 1.2, ch))]
        cf.append(cyl_faces((cx, cy, ch - 1.0), (0, 0, 1), 1.1, 0.78))
        d = np.array([g.BOARD / 2 - cx, g.BOARD / 2 - cy]); d /= np.linalg.norm(d)
        wx, wy, R = cx + 1.2 * d[0], cy + 1.2 * d[1], 2.5
        perp = np.array([-d[1], d[0], 0.0])
        for s in (-1, 1):                                       # fork sides
            p = np.array([wx, wy]) + perp[:2] * s * 0.95
            cf.append(box_faces((*(p - 0.12), R), (*(p + 0.12), ch - 1.0)))
        cf.append(cyl_faces(np.array([wx, wy, R]) - perp * 0.8, perp, R, 1.6, 28))
        out.append(Part(f"caster{i + 1}", "caster", [f for fs in cf for f in fs]))
    return out


def camera_parts():
    cx, cy = g.CAMERA_XY
    zb = g.BOOM_BOTTOM - BOOM_DROP
    return [Part("ballhead", "camera", cyl_faces((cx, cy, zb - 1.5), (0, 0, 1), 0.45, 1.5)),
            box_part("camera", "camera", (cx - 0.6, cy - 0.6, zb - 1.5 - g.CAMERA_H - 0.6), (cx + 0.6, cy + 0.6, zb - 1.5))]


def end_caps(which):
    out = []
    caps = [(f"box_cap_{s}_{x}", x, y, g.TOP_Z - 0.5, d) for s, y in (("f", E), ("b", g.BOARD - E)) for x, d in g.END_CAPS["box"]] \
        if which == "box" else [(f"cart_cap_{s}", x, y, g.CART_TOP - 0.5, d) for s, y in (("f", E), ("b", g.BOARD - E)) for x, d in g.END_CAPS["cart"]]
    for n, x, y, z, d in caps:
        x0 = x if d > 0 else x - 0.12
        out.append(box_part(n, "cap", (x0, y - 0.5, z - 0.5), (x0 + 0.12, y + 0.5, z + 0.5)))
    return out


# ---------------------------------------------------------------- renderer
BASE = {  # kind: (old colour, new colour)
    "1010": ("#d9dce0", "#f29e4c"), "2020": ("#d9dce0", "#e07b2a"), "4132": ("#c6cace", "#2f74d0"),
    "panel": ("#8f959c", "#2a9d8f"), "2419": ("#c9ccd0", "#7a5cc7"), "caster": ("#7d8187", "#3b3f45"),
    "angle": ("#c9ccd0", "#d1495b"), "camera": ("#6c7076", "#2b2f35"), "cap": ("#55595e", "#111111"),
    "board": ("#4a5058", "#4a5058"), "nut": ("#e6c229", "#e6c229"),
}


def hexrgb(h):
    return np.array([int(h[i:i + 2], 16) for i in (1, 3, 5)], float)


class View:
    def __init__(self, direction, fit_lo, fit_hi, w=W, h=H, margin=40, shift=(0, 0)):
        v = np.array(direction, float); self.v = v / np.linalg.norm(v)
        r = np.cross([0, 0, 1], self.v); self.r = r / np.linalg.norm(r)
        self.u = np.cross(self.v, self.r)
        corners = np.array([[x, y, z] for x in (fit_lo[0], fit_hi[0]) for y in (fit_lo[1], fit_hi[1]) for z in (fit_lo[2], fit_hi[2])])
        su, sv = corners @ self.r, corners @ self.u
        self.w, self.h = w, h
        self.s = min((w - 2 * margin) / (su.max() - su.min()), (h - 2 * margin) / (sv.max() - sv.min()))
        self.u0 = (su.max() + su.min()) / 2 - shift[0] / self.s
        self.v0 = (sv.max() + sv.min()) / 2 + shift[1] / self.s

    def px(self, p):
        p = np.atleast_2d(p)
        return np.stack([(p @ self.r - self.u0) * self.s + self.w / 2, (self.v0 - p @ self.u) * self.s + self.h / 2], 1)


def render(parts, view, new=()):
    w, h = view.w * SS, view.h * SS
    zbuf = np.full((h, w), -np.inf); fid = np.full((h, w), -1, np.int32)
    rgb = np.ones((h, w, 3)) * 255
    L = view.u * 1.0 - view.r * 0.45 + view.v * 0.7
    L /= np.linalg.norm(L)
    ys, xs = np.mgrid[0:h, 0:w]
    k = 0
    for part in parts:
        col = hexrgb(BASE[part.kind][1 if part.name in new else 0])
        for verts, n in part.faces:
            k += 1
            nv = n @ view.v
            if nv <= 1e-6:
                continue
            p2 = view.px(verts) * SS
            x0, y0 = np.floor(p2.min(0)).astype(int); x1, y1 = np.ceil(p2.max(0)).astype(int)
            x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w - 1), min(y1, h - 1)
            if x1 < x0 or y1 < y0:
                continue
            gx, gy = xs[y0:y1 + 1, x0:x1 + 1] + 0.5, ys[y0:y1 + 1, x0:x1 + 1] + 0.5
            mask = MplPath(p2).contains_points(np.stack([gx.ravel(), gy.ravel()], 1)).reshape(gx.shape)
            if not mask.any():
                continue
            uu = (gx / SS - view.w / 2) / view.s + view.u0
            vv = view.v0 - (gy / SS - view.h / 2) / view.s
            t = (n @ verts[0] - uu * (n @ view.r) - vv * (n @ view.u)) / nv
            zb = zbuf[y0:y1 + 1, x0:x1 + 1]
            upd = mask & (t > zb + 1e-4)
            zb[upd] = t[upd]
            fid[y0:y1 + 1, x0:x1 + 1][upd] = k
            shade = 0.62 + 0.38 * max(0.0, n @ L)
            rgb[y0:y1 + 1, x0:x1 + 1][upd] = np.clip(col * shade, 0, 255)
    edge = np.zeros((h, w), bool)
    edge[:, 1:] |= fid[:, 1:] != fid[:, :-1]
    edge[1:, :] |= fid[1:, :] != fid[:-1, :]
    edge[:, :-1] |= edge[:, 1:].copy()
    edge[:-1, :] |= edge[1:, :].copy()
    rgb[edge] = [40, 44, 50]
    img = Image.fromarray(rgb.astype(np.uint8)).resize((view.w, view.h), Image.LANCZOS)
    buf = io.BytesIO(); img.save(buf, "PNG", optimize=True)
    return base64.b64encode(buf.getvalue()).decode()


# ---------------------------------------------------------------- model + bookkeeping
M = frame_members()
JOINTS = {n: (c, n1, n2) for n, c, n1, n2, *_ in g.box_joints() + g.cart_joints()}
for _n in ("boom_under", "boom_over"):
    _c, _n1, _n2 = JOINTS[_n]
    JOINTS[_n] = ((_c[0], _c[1], _c[2] - BOOM_DROP), _n1, _n2)
BRACKETS = {n: bracket(n, *j) for n, j in JOINTS.items()}
SHELF, LAPTOP = shelf_panel(), laptop_panel()
LOCATORS, LOC_NUTS = locator_parts()
CASTERS = caster_parts()
CAMERA = camera_parts()
BOX_CAPS, CART_CAPS = end_caps("box"), end_caps("cart")
BOARD_PLATE = box_part("board", "board", (0, 0, g.TOP_Z), (g.BOARD, g.BOARD, g.BOARD_TOP))
ALL = {p.name: p for p in list(M.values()) + list(BRACKETS.values()) + [SHELF, LAPTOP] + LOCATORS + CASTERS
       + CAMERA + BOX_CAPS + CART_CAPS + [BOARD_PLATE]}


def nut_points():
    """[(point inside a member, face normal pointing out of that member, purpose)]"""
    pts = []
    for n, j in JOINTS.items():
        pts += [(p, nn, "4132") for p, nn in bracket_nuts(*j)]
    for key, z in (("shelf", g.SHELF_RAIL_TOP), ("laptop", g.CART_TOP)):
        pts += [(np.array([x, y, z - 0.2]), np.array([0, 0, 1.0]), "3321 panel") for (x, y), _ in g.PANEL_SCREWS[key]]
    pts += [(p, nn, "locator") for p, nn in LOC_NUTS]
    cx, cy = g.CAMERA_XY
    pts.append((np.array([cx, cy, g.BOOM_BOTTOM - BOOM_DROP + 0.2]), np.array([0, 0, -1.0]), "ball head"))
    return pts


FACE_NAME = {(0, 0, 1): "top", (0, 0, -1): "bottom", (0, -1, 0): "front", (0, 1, 0): "back",
             (-1, 0, 0): "laptop side", (1, 0, 0): "mast side"}


def face_name(member, n):
    """top / bottom, inside / outside (toward or away from the stand's middle), else front / back etc."""
    if abs(n[2]) > 0.5:
        return FACE_NAME[(0, 0, int(round(n[2])))]
    d = np.array([g.BOARD / 2, g.BOARD / 2]) - member.center[:2]
    k = n[:2] @ d / np.linalg.norm(d)
    return "inside" if k > 0.3 else "outside" if k < -0.3 else FACE_NAME[tuple(int(round(c)) for c in n)]


def nut_table():
    """{member: {face name: {purpose: count}}}"""
    tab = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))
    for p, n, why in nut_points():
        hits = [m for m in M.values() if np.all(p >= m.lo - 1e-6) and np.all(p <= m.hi + 1e-6)]
        assert len(hits) == 1, (p, [m.name for m in hits])
        tab[hits[0].name][face_name(hits[0], n)][why] += 1
    return tab


NUTS = nut_table()


def nuts_text(names):
    rows = []
    for n in names:
        if n in NUTS:
            faces = "; ".join(f"{f}: {sum(d.values())}" + (f" ({', '.join(k for k in d if k != '4132')})" if any(k != '4132' for k in d) else "")
                              for f, d in NUTS[n].items())
            rows.append((n, sum(sum(d.values()) for d in NUTS[n].values()), faces))
    return rows


# ---------------------------------------------------------------- content
CODES = {  # code: (member names, description)
    "A": (["top_front_1010", "top_back_1010"], "1010 x 24 in", "Box top frame, front + back"),
    "B": (["top_left_1010", "top_right_1010"], "1010 x 22 in", "Box top frame, sides"),
    "C": (["shelf_front_1010", "shelf_back_1010", "shelf_left_1010", "shelf_right_1010"], "1010 x 20 in", "Box shelf ring"),
    "D": ([f"post{i}_2020" for i in range(1, 5)], f"2020 x {g.POST_L:.2f} in", "Box posts"),
    "E": (["camera_mast_1010"], f"1010 x {g.MAST_L:.2f} in", "Camera mast"),
    "F": (["camera_boom_1010"], f"1010 x {g.BOOM_LEN:g} in", "Camera boom"),
    "G": (["cart_top_front_1010", "cart_top_back_1010"], f"1010 x {g.BOARD + g.LAPTOP_DEPTH:g} in", "Cart top, front + back (run on under the laptop table)"),
    "H": (["cart_top_left_1010", "cart_top_right_1010", "cart_top_laptop_end_1010"], "1010 x 22 in", "Cart top, sides + laptop end"),
    "J": (["cart_lower_front_1010", "cart_lower_back_1010", "cart_lower_left_1010", "cart_lower_right_1010"], "1010 x 20 in", "Cart lower ring"),
    "K": ([f"cart_leg{i}_2020" for i in range(1, 5)], f"2020 x {g.LEG_L:.2f} in", "Cart legs"),
}
CODE_OF = {n: c for c, (ns, *_) in CODES.items() for n in ns}

BOX_FRAME = [n for n in M if not n.startswith("cart") and not n.startswith("camera")]
FL = (-1, -1.25, 0.95)          # view from front, laptop side, above
FR = (1, -1.25, 0.95)           # view from front, mast side, above
LIFT = 14.0                     # step 10: box drawn this far above the cart


def names(prefix):
    return [n for n in ALL if n.startswith(prefix)]


def lbl(target, text, dx, dy):
    """Callout on a part name or a 3D point, text offset (dx, dy) px from the anchor."""
    return (target, text, dx, dy)


def vw(direction, fit=None, h=H, w=W, margin=50):
    """View spec; fit (lo, hi) defaults to the bounding box of the parts shown."""
    return dict(direction=direction, fit=fit, h=h, w=w, margin=margin)


def box_fit(lo, hi):
    return np.array(lo, float), np.array(hi, float)


STEPS = []


def step(title, unit, show, new, view, notes, labels=(), nuts=(), detail=None, lift=()):
    STEPS.append(dict(title=title, unit=unit, show=show, new=new, view=view, notes=notes, labels=labels, nuts=nuts,
                      detail=detail, lift=set(lift)))


box_posts = [f"post{i}_2020" for i in range(1, 5)]
shelf_rails = ["shelf_front_1010", "shelf_back_1010", "shelf_left_1010", "shelf_right_1010"]
top_rails = ["top_front_1010", "top_back_1010", "top_left_1010", "top_right_1010"]
shelf_br = names("shelf_post")
top_ring_br = [n for n in names("top_") if "leg" not in n and n in BRACKETS]
top_post_br = [n for n in names("top_leg") if n in BRACKETS]
mast_br = names("mast_") + names("boom_")
cart_legs = [f"cart_leg{i}_2020" for i in range(1, 5)]
cart_lower = [n for n in M if n.startswith("cart_lower_")]
cart_lower_br = [n for n in BRACKETS if n.startswith("cart_lower_leg")]
cart_top = [n for n in M if n.startswith("cart_top_")]
cart_ring_br = [n for n in BRACKETS if n.startswith("cart_top_") and "leg" not in n]
cart_leg_br = [n for n in BRACKETS if n.startswith("cart_top_leg")]
box_all = BOX_FRAME + [n for n in BRACKETS if not n.startswith("cart")] + ["shelf_panel", "camera_mast_1010", "camera_boom_1010", "ballhead", "camera"]
cart_all = [n for n in M if n.startswith("cart")] + [n for n in BRACKETS if n.startswith("cart")] + ["laptop_panel"] \
    + [p.name for p in LOCATORS + CASTERS] + [p.name for p in CART_CAPS]
casters = [p.name for p in CASTERS]

step("Shelf ring and posts", "Bench box",
     box_posts + shelf_rails + shelf_br, box_posts + shelf_rails + shelf_br, vw(FL, h=560),
     ["Work on a flat bench, right way up: the box bottom is the posts' bottoms and the shelf rails' bottoms, all flush.",
      "Fit the four <b>C</b> 20 in rails between the four <b>D</b> posts, flush with the post bottoms and with the outside faces.",
      "Join each rail end to its post with a 4132 on the <b>inside</b> face of the rail (8 brackets). Nothing sticks out below the box.",
      "Before the front and back rails are closed in, slide <b>2 extra T-nuts into each one's top slot</b> for the shelf-panel screws (step 2), roughly 3 in and 21 in from the laptop-side end."],
     [lbl("post1_2020", "D  post, 2020 x %.2f" % g.POST_L, -60, 70), lbl("shelf_front_1010", "C  1010 x 20", 80, 60),
      lbl("shelf_left_1010", "C", -90, 0), lbl("shelf_back_1010", "C", 40, -50), lbl("shelf_right_1010", "C", 60, -30)],
     box_posts + shelf_rails,
     detail=dict(show=box_posts[:1] + ["shelf_front_1010", "shelf_left_1010", "shelf_post1_x", "shelf_post1_y"],
                 new=["shelf_post1_x", "shelf_post1_y"], view=vw((1.0, 1.4, 1.4), box_fit((0, 0, g.BOX_BOTTOM), (4.5, 4.5, g.BOX_BOTTOM + 3)), 400, 520, 30),
                 caption="Inside corner, seen from the middle of the box: one 4132 on each rail's inside face, its other flange on the post."))

step("Shelf panel", "Bench box",
     box_posts + shelf_rails + shelf_br + ["shelf_panel"], ["shelf_panel"], vw(FL, h=560),
     ["Notch the 24 x 24 in black ACM panel 2 x 2 in at all four corners so it fits around the posts.",
      "Drill four 17/64 in clearance holes (no countersink: the 3321 is a flanged button head that sits on the panel) 0.5 in in from the front and back edges, at 3 in and 21 in from the laptop-side edge. (Mark them off the T-nuts if they've drifted.)",
      "Lay it on the shelf rails and fix with four 3321 flat-head screws into the T-nuts pre-loaded in the front and back rails.",
      "Do this now: once the top frame is on, the panel can't get past it."],
     [lbl("shelf_panel", "Shelf panel 24 x 24, corners notched 2 x 2", 120, -150),
      lbl(np.array([3, 0.5, g.SHELF_PANEL_TOP]), "3321 screw, 4 places", -40, 70)])

step("Top frame", "Bench box",
     top_rails + top_ring_br, top_rails + top_ring_br, vw(FL, h=520, margin=70),
     ["Lay out the two <b>A</b> 24 in rails (front and back) and fit the two <b>B</b> 22 in rails between their ends.",
      "Join each corner with a 4132 on the inside face of the <b>B</b> rail (4 brackets), flush top and bottom.",
      "Load the T-nuts for step 4 now (table): every rail needs 2 in its bottom slot, one per post bracket.",
      "Square it: the diagonals should match. This frame is the breadboard's footprint, flush with its edges."],
     [lbl("top_front_1010", "A  1010 x 24", 60, 70), lbl("top_back_1010", "A", -40, -60),
      lbl("top_left_1010", "B  1010 x 22", -70, 60), lbl("top_right_1010", "B", 60, -50),
      lbl("top_left_front", "4132, 4 places", 40, 90)],
     top_rails)

step("Top frame onto the posts", "Bench box",
     BOX_FRAME + shelf_br + top_ring_br + top_post_br + ["shelf_panel"], top_rails + top_ring_br + top_post_br, vw(FL, h=560),
     ["Set the top frame on the four posts, outside faces flush all round.",
      "Fix each post with two 4132s (8 in all): one flange under the rail above it, the other on the post's inside face.",
      "The post T-nuts for these brackets can still go in from the post bottoms (a different slot from the shelf brackets' nuts), but it's easier to load them first."],
     [lbl("top_front_1010", "Top frame from step 3", 90, -60), lbl("top_leg1_x", "2 x 4132 per post", -80, 90),
      lbl("shelf_panel", "Shelf panel", -330, -120)],
     box_posts,
     detail=dict(show=["post1_2020", "top_front_1010", "top_left_1010", "top_leg1_x", "top_leg1_y", "top_left_front"],
                 new=["top_leg1_x", "top_leg1_y"], view=vw((1.0, 1.4, -0.7), box_fit((0, 0, g.POST_TOP - 2.5), (4.5, 4.5, g.TOP_Z)), 400, 520, 30),
                 caption="Under the top frame, seen from inside: the brackets hang below the rails and bolt into the post's inside faces."))

step("Camera mast and boom", "Bench box",
     box_all + mast_br, ["camera_mast_1010", "camera_boom_1010", "ballhead", "camera"] + mast_br, vw(FR, h=900, margin=40),
     ["Stand the <b>E</b> mast against the <b>mast side</b> (the end without the laptop table) at mid-depth, bottom flush with the box bottom.",
      "Bolt it to the top-frame and shelf side rails with four 4132s, one each side of the mast at each rail.",
      "Fit the <b>F</b> boom to the mast's inside face with one 4132 under it and one on top; loosen both to slide the boom up or down.",
      "<b>Keep the boom's top at least 1 in below the mast top</b>, or the upper bracket has nothing to bolt to. (The CAD had them flush; 1 in down puts the lens 35 in above the board.)",
      "Put one T-nut in the boom's bottom slot ~1 in from its free end (over the board centre) for the ball-head stud, then fit the camera."],
     [lbl("camera_mast_1010", "E  mast, 1010 x %.2f" % g.MAST_L, 60, -60), lbl("camera_boom_1010", "F  boom, 1010 x 13", -60, -60),
      lbl("mast_top_a", "2 x 4132 at each rail", 60, 50), lbl("mast_shelf_a", "", 70, 0), lbl("camera", "ball head + camera", -80, 40)],
     ["camera_mast_1010", "camera_boom_1010", "top_right_1010", "shelf_right_1010"],
     detail=dict(show=["camera_mast_1010", "camera_boom_1010", "boom_under", "boom_over"], new=["boom_under", "boom_over"],
                 view=vw((0.6, -1.3, 0.5), box_fit((g.BOARD - 4, g.MAST_Y - 1, g.MAST_TOP - 4.5), (g.BOARD + 1, g.MAST_Y + 1, g.MAST_TOP)), 400, 520, 30),
                 caption="Boom to mast: one 4132 under the boom, one on top. The top one needs mast above it, so the boom top sits at least 1 in below the mast top."))

step("Legs and lower ring", "Wheeled cart",
     cart_legs + cart_lower + cart_lower_br, cart_legs + cart_lower + cart_lower_br, vw(FL, h=640),
     ["Stand the four <b>K</b> legs upright and fit the four <b>J</b> 20 in rails between them, outside faces flush.",
      "Rail <b>bottoms 3.0 in above the leg bottoms</b>: cut two 3.0 in spacer blocks and stand the rails on them while you tighten.",
      "Join with 4132s on the rails' inside faces (8 brackets), same as the box shelf ring.",
      "Tap two corner holes 1/4-20 in each leg's bottom end now (step 8 shows which), while the legs are easy to handle."],
     [lbl("cart_leg1_2020", "K  leg, 2020 x %.2f" % g.LEG_L, -110, 120), lbl("cart_lower_front_1010", "J  1010 x 20", 60, 70),
      lbl(np.array([g.BOARD, 0, g.LEG_BOTTOM + 1.5]), "3.0 in, leg bottom to rail bottom", 60, 50)],
     cart_legs + cart_lower)

step("Cart top frame", "Wheeled cart",
     cart_legs + cart_lower + cart_lower_br + cart_top + cart_ring_br + cart_leg_br, cart_top + cart_ring_br + cart_leg_br, vw(FL, h=600),
     ["Lay the two <b>G</b> 38 in rails front and back, and fit three <b>H</b> 22 in rails between them: one over each end of the leg square and one at the far end of the 14 in run-on (the laptop end).",
      "Join with 4132s on the inside faces of the <b>H</b> rails (6 brackets).",
      "Pre-load the T-nuts in the table, including 2 in each <b>G</b> rail's top slot under the laptop table (1.5 in and 12.5 in from the leg square) and the locator nuts in the outside slots of the rails and legs (step 9).",
      "Set the frame on the legs, run-on toward the laptop side, and fix each leg with two 4132s under the rails (8 brackets)."],
     [lbl("cart_top_front_1010", "G  1010 x 38", 40, 90), lbl("cart_top_laptop_end_1010", "H  laptop end", -110, 10),
      lbl("cart_top_left_1010", "H", 40, -110), lbl("cart_top_right_1010", "H", 70, -50), lbl("cart_top_leg2_x", "2 x 4132 per leg", 60, 70)],
     cart_top + cart_legs)

step("Casters", "Wheeled cart",
     cart_legs + cart_lower + cart_top + cart_lower_br + cart_ring_br + cart_leg_br + casters, casters, vw(FL, h=640),
     ["Turn the cart over (top frame down, on cardboard).",
      "Each 2419 plate sits <b>0.5 in inboard of the leg centre, toward the middle of the cart front-to-back</b>, so two of its counterbored holes land on two of the leg's corner holes. Tap those two holes 1/4-20 and fix with 1/4-20 x 3/4 SHCS, heads in the counterbores.",
      "Bolt each 2323 caster to its plate with four 5/16-18 x 5/8 SHCS, long side of the flange along the cart's length.",
      "Only two screws hold each plate to its leg: snug them firmly and re-check after the first few moves."],
     [lbl("plate1", "2419 plate", -170, 20), lbl("caster2", "2323 caster, 5 in", 80, 40)],
     detail=dict(show=["cart_leg1_2020", "cart_lower_front_1010", "cart_lower_left_1010", "plate1", "caster1"], new=["plate1", "caster1"],
                 view=vw((-1, -1.3, -0.8), box_fit((-2, -2, g.CASTER_H - 0.4), (4.0, 4.0, g.LEG_BOTTOM + 2)), 400, 520, 30),
                 caption="From below: the plate is shifted 0.5 in toward the middle of the cart front-to-back, so two of its holes meet two corner holes in the leg end."))

step("Locators, laptop table, end caps", "Wheeled cart",
     cart_all, [p.name for p in LOCATORS] + ["laptop_panel"] + [p.name for p in CART_CAPS], vw(FL, h=640),
     ["Stand the cart on its wheels and lock the brakes.",
      "Bolt a 4166 flat plate (3 x 2 in, 6 holes) upright on the outside of each leg top, flush with the corner and standing 1.0 in above the cart top, with a 1/32 in shim behind it (box clearance). The top row of holes stays empty; the middle row goes into the top rail's slot and the bottom row into the leg's two slots. Mast-side corners get a plate on both faces: front/back 4 bolts, end 3 (its middle-row corner hole is over the rail end, no slot). Laptop-side corners: front/back plate only, 4 bolts.",
      "Drill 17/64 in clearance holes (no countersink) in the 13.84 x 24 in laptop panel for four 3321 screws (0.5 in from the long edges, at 1.5 in and 12.5 in from its box end) and fix it to the <b>G</b> rails.",
      "Push 2015 end caps onto the two open <b>G</b> rail ends at the laptop end."],
     [lbl("locator2", "4166 locator plates, 6 places", 60, -50), lbl("laptop_panel", "Laptop table 13.84 x 24", -40, -90),
      lbl("cart_cap_f", "2015 end cap, 2 places", -40, 70)])

step("Lift the box onto the cart", "Final",
     cart_all + box_all + mast_br + [p.name for p in BOX_CAPS] + ["board"], box_all + mast_br + [p.name for p in BOX_CAPS] + ["board"],
     vw(FL, h=820, margin=30),
     ["Lock the casters. With two people, lower the bench box straight down into the four corner locators.",
      "Mount the breadboard + gantry on the top frame: the gantry legs' M4 nuts slide into the top slots of the <b>A</b> rails from their ends.",
      "Then push 2015 end caps onto the four <b>A</b> rail ends.",
      f"The board top ends up {g.BOARD_TOP:g} in above the floor; the laptop table is at {g.CART_TOP + g.PANEL_T:.2f} in."],
     [lbl("board", "Breadboard + gantry", -60, -80), lbl("laptop_panel", "Laptop table", -60, 50),
      lbl("locator2", "Posts drop into the locators", 60, 60),
      ("arrow", np.array([0.0, 0.0, g.BOX_BOTTOM + LIFT - 0.5]), np.array([0.0, 0.0, g.CART_TOP + 1.2])),
      ("arrow", np.array([g.BOARD, 0.0, g.BOX_BOTTOM + LIFT - 0.5]), np.array([g.BOARD, 0.0, g.CART_TOP + 1.2]))],
     lift=box_all + mast_br + [p.name for p in BOX_CAPS] + ["board"])


# ---------------------------------------------------------------- page
def moved(part, dz):
    faces = [(v + [0, 0, dz], n) for v, n in part.faces]
    return Part(part.name, part.kind, faces, (part.lo + [0, 0, dz], part.hi + [0, 0, dz]))


def make_view(spec, parts):
    if spec["fit"] is None:
        lo, hi = np.min([p.lo for p in parts], 0), np.max([p.hi for p in parts], 0)
    else:
        lo, hi = spec["fit"]
    return View(spec["direction"], lo, hi, spec["w"], spec["h"], spec["margin"])


def svg_overlay(view, labels, parts):
    out = []
    for item in labels:
        if isinstance(item[0], str) and item[0] == "arrow":
            (x1, y1), (x2, y2) = view.px(np.array([item[1], item[2]]))
            out.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" class="arrow" marker-end="url(#ah)"/>')
            continue
        target, text, dx, dy = item
        p = parts[target].center if isinstance(target, str) else np.asarray(target, float)
        (x, y), = view.px(p)
        tx, ty = x + dx, y + dy
        out.append(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{tx:.1f}" y2="{ty:.1f}" class="lead"/><circle cx="{x:.1f}" cy="{y:.1f}" r="4" class="dot"/>')
        if text:
            tw = 8.4 * len(text)                              # rough width at 15px semibold
            tx0 = min(max(tx - tw - 6 if dx < 0 else tx + 6, 4), view.w - tw - 4)
            ty0 = min(max(ty + 5, 16), view.h - 6)
            out.append(f'<text x="{tx0:.1f}" y="{ty0:.1f}" class="halo">{html.escape(text)}</text>'
                       f'<text x="{tx0:.1f}" y="{ty0:.1f}">{html.escape(text)}</text>')
    return "".join(out)


ARROW_DEF = ('<defs><marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" '
             'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#c8102e"/></marker></defs>')


def figure(show, new, spec, labels, lift=()):
    parts = {n: (moved(ALL[n], LIFT) if n in lift else ALL[n]) for n in dict.fromkeys(show)}
    view = make_view(spec, list(parts.values()))
    png = render(list(parts.values()), view, set(new))
    return (f'<div class="fig"><svg viewBox="0 0 {view.w} {view.h}">{ARROW_DEF}'
            f'<image href="data:image/png;base64,{png}" width="{view.w}" height="{view.h}"/>'
            f'{svg_overlay(view, labels, parts)}</svg></div>')


def cut_list_rows():
    rows = []
    for c, (ns, size, what) in CODES.items():
        rows.append(f"<tr><td class=code>{c}</td><td>{len(ns)}</td><td>{size}</td><td>{what}</td></tr>")
    return "".join(rows)


def nut_rows():
    rows = []
    for n in M:
        if n in NUTS:
            faces = ", ".join(f"{f} {sum(d.values())}" for f, d in NUTS[n].items())
            rows.append(f"<tr><td class=code>{CODE_OF.get(n, '')}</td><td>{html.escape(n.replace('_1010', '').replace('_2020', '').replace('_', ' '))}</td>"
                        f"<td>{sum(sum(d.values()) for d in NUTS[n].values())}</td><td>{faces}</td></tr>")
    return "".join(rows)


def build():
    total_nuts = sum(sum(sum(d.values()) for d in f.values()) for f in NUTS.values())
    overview = figure(cart_all + box_all + mast_br, [n for n in cart_all if n in M or n == "laptop_panel"], vw(FL, h=820, margin=30),
                      [("top_front_1010", "Bench box (grey)", -60, -110), ("cart_lower_front_1010", "Wheeled cart (orange)", 60, 70),
                       ("laptop_panel", "LAPTOP SIDE", -70, -60), ("camera_mast_1010", "MAST SIDE", 70, 0),
                       ("cart_lower_front_1010", "FRONT", -200, 120)])
    sections = []
    for i, s in enumerate(STEPS, 1):
        notes = "".join(f"<li>{t}</li>" for t in s["notes"])
        nut_html = ""
        if s["nuts"]:
            rows = "".join(f"<tr><td class=code>{CODE_OF.get(n, '')}</td><td>{html.escape(n.replace('_1010', '').replace('_2020', '').replace('_', ' '))}</td>"
                           f"<td>{k}</td><td>{f}</td></tr>" for n, k, f in nuts_text(s["nuts"]))
            nut_html = f'<details open><summary>T-nuts to slide in first</summary><table class=nuts><tr><th></th><th>Member</th><th>Qty</th><th>Slot (face it\'s on)</th></tr>{rows}</table></details>'
        det = ""
        if s["detail"]:
            d = s["detail"]
            det = f'<div class="detail">{figure(d["show"], d["new"], d["view"], [])}<p>{d["caption"]}</p></div>'
        sections.append(f'<section class=step><h2><span class=num>{i}</span>{html.escape(s["title"])}<span class=unit>{s["unit"]}</span></h2>'
                        f'{figure(s["show"], s["new"], s["view"], s["labels"], s["lift"])}<div class=cols><ol>{notes}</ol><div>{nut_html}{det}</div></div></section>')
    doc = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gantry Stand Assembly</title>
<style>
:root {{ --bg:#f6f6f4; --card:#fff; --ink:#1d1d1f; --mute:#62636a; --line:#e2e2e0; --accent:#e07b2a; --blue:#2f74d0; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#141416; --card:#1d1d20; --ink:#ececee; --mute:#9a9aa0; --line:#2e2e33; }} }}
:root[data-theme="dark"] {{ --bg:#141416; --card:#1d1d20; --ink:#ececee; --mute:#9a9aa0; --line:#2e2e33; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }}
main {{ max-width:1140px; margin:0 auto; padding:28px 16px 64px; }}
h1 {{ margin:0 0 4px; font-size:28px; }} .sub {{ color:var(--mute); margin-bottom:20px; }}
h2 {{ display:flex; align-items:center; gap:12px; font-size:20px; margin:0 0 10px; }}
h2 .num {{ background:var(--accent); color:#fff; border-radius:50%; width:34px; height:34px; display:grid; place-items:center; font-size:17px; flex:none; }}
h2 .unit {{ margin-left:auto; color:var(--mute); font-size:13px; font-weight:500; text-transform:uppercase; letter-spacing:.06em; }}
section {{ background:var(--card); border:1px solid var(--line); border-radius:12px; padding:18px; margin:18px 0; break-inside:avoid; page-break-inside:avoid; }}
.fig {{ background:#fff; border-radius:8px; }} .fig svg {{ display:block; width:100%; height:auto; }}
.fig text {{ font:600 15px system-ui,"Segoe UI",sans-serif; fill:#1d1d1f; }}
.fig text.halo {{ stroke:#fff; stroke-width:5px; stroke-linejoin:round; }}
.fig .lead {{ stroke:#1d1d1f; stroke-width:1.4; }} .fig .arrow {{ stroke:#c8102e; stroke-width:3.5; }} .fig .dot {{ fill:#1d1d1f; stroke:#fff; stroke-width:1.5; }}
.cols {{ display:grid; grid-template-columns:1.2fr 1fr; gap:20px; margin-top:12px; }}
@media (max-width:760px) {{ .cols {{ grid-template-columns:1fr; }} }}
ol {{ margin:0; padding-left:20px; }} li {{ margin:6px 0; }}
table {{ border-collapse:collapse; width:100%; font-size:13.5px; }}
th, td {{ text-align:left; padding:5px 8px; border-bottom:1px solid var(--line); vertical-align:top; }}
th {{ color:var(--mute); font-size:12px; text-transform:uppercase; letter-spacing:.04em; }}
td.code {{ font-weight:700; color:var(--accent); width:28px; }}
summary {{ font-weight:600; cursor:pointer; margin-bottom:6px; }}
.detail {{ margin-top:12px; max-width:420px; }} .detail p {{ color:var(--mute); font-size:13.5px; margin:6px 0 0; }}
.legend {{ display:flex; flex-wrap:wrap; gap:14px; font-size:13.5px; color:var(--mute); margin:10px 0 0; }}
.sw {{ display:inline-block; width:14px; height:14px; border-radius:3px; vertical-align:-2px; margin-right:6px; border:1px solid #0003; }}
.grid2 {{ display:grid; grid-template-columns:1fr 1fr; gap:18px; }} @media (max-width:760px) {{ .grid2 {{ grid-template-columns:1fr; }} }}
.warn {{ border-left:4px solid var(--blue); padding:8px 12px; background:color-mix(in srgb, var(--blue) 8%, transparent); border-radius:4px; margin:12px 0 0; }}
@media print {{ body {{ background:#fff; }} section {{ border-color:#ccc; }} details > summary {{ list-style:none; }} }}
</style></head><body><main>
<h1>Gantry Stand: Assembly</h1>
<div class=sub>Bench box + wheeled cart, 80/20 10-series. Generated from gantry_cart.py; all lengths in inches.</div>
<section><h2>The finished stand</h2>{overview}
<div class=legend><span><i class=sw style="background:{BASE['1010'][1]}"></i>Parts added in a step</span>
<span><i class=sw style="background:{BASE['4132'][1]}"></i>New 4132 brackets</span><span><i class=sw style="background:{BASE['1010'][0]}"></i>Already built</span></div>
<p class=warn><b>Before you start:</b> the 3393 bolt kits use <b>slide-in</b> economy T-nuts. They can only enter a slot from an open end, and many ends get closed off as the frame goes together. For each member, slide in every T-nut it will need (table below) before you join it, threading the bolt through the bracket a turn or two so the nut stays put. {total_nuts} T-nuts in all: {2 * len(JOINTS)} from the 3393 kits, {g.N_LOC_BOLTS} 3382s for the locators, 8 from the 3321 kits and 1 for the ball head.</p>
<p class=warn><b>Orientation used throughout:</b> <b>front</b> and <b>back</b> are the two long sides of the cart; the <b>laptop side</b> is the end with the laptop table (the gantry's leg-rail motor end); the <b>mast side</b> is the opposite end, where the camera mast goes.</p></section>
<section><h2>Sort the profiles</h2><div class=grid2><div><table><tr><th></th><th>Qty</th><th>Profile</th><th>Where</th></tr>{cut_list_rows()}</table>
<p class=sub style="margin-top:8px">B and H (22 in) are the same part, and so are C and J (20 in): 5 x 22 in and 8 x 20 in in the box.</p></div>
<div><table><tr><th>Hardware</th><th>Qty</th></tr>
<tr><td>4132 gusseted inside corner bracket</td><td>{len(JOINTS)} (box {len(g.box_joints())}, cart {len(g.cart_joints())})</td></tr>
<tr><td>3393 1/4-20 x 1/2 BHSCS + economy T-nut</td><td>{2 * len(JOINTS)}</td></tr>
<tr><td>4166 6-hole flat plate (locators) / 3061 BHSCS + 3382 T-nut</td><td>{g.N_LOC_PLATES} / {g.N_LOC_BOLTS}</td></tr>
<tr><td>3321 1/4-20 x 1/2 FBHSCS + T-nut (panels)</td><td>8</td></tr>
<tr><td>2015 end cap</td><td>6</td></tr><tr><td>2419 caster plate / 2323 caster</td><td>4 / 4</td></tr>
<tr><td>3110 5/16-18 x 5/8 SHCS / 3064 1/4-20 x 3/4 SHCS (casters)</td><td>16 / 8</td></tr>
<tr><td>Not from 80/20: ball head; 1/32 in shim stock</td><td>1; 6 pieces 3 x 2 in</td></tr></table>
<p class=sub style="margin-top:8px">Tools: 5/32 in hex key (1/4-20 BHSCS), 3/16 in (1/4-20 SHCS), 1/4 in (5/16 SHCS), 1/4-20 tap, square.</p></div></div></section>
<section><h2>T-nut pre-load list</h2><table><tr><th></th><th>Member</th><th>T-nuts</th><th>Slots (face they're on)</th></tr>{nut_rows()}</table></section>
{''.join(sections)}
</main></body></html>"""
    out = Path(__file__).with_name("stand_assembly_guide.html")
    out.write_text(doc, encoding="utf-8")
    return out


if __name__ == "__main__":
    print(build())
