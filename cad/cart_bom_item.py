"""Bill of materials and price quote for the gantry stand (gantry_cart.py) built from item (item24.com)
profiles instead of 80/20: bench box + wheeled cart, same layout and outside dimensions, in metric.

Profile mapping: 80/20 1010 (1 in) -> item Profile 6 30x30 light, 80/20 2020 (2 in) -> item Profile 8 40x40
light. Rails stay Line 6, posts and legs Line 8, so the rail-to-post joints use Line 6 angle bracket sets
with a Line 8 T-slot nut (and a longer screw) on the post side.

Geometry (mm), following gantry_cart.py:
  - Top frame: the rails' centre lines stay on the board's outer hole rows (12.7 mm in), so the Line 6 top
    slot sits under the gantry legs' M4 fixings as before; with a 30 mm section the frame's outer faces
    are 2.3 mm inside the board edge (frame 605.0 square). An M4 hex nut (7.0 AF, 8.1 AC) sits in the
    Line 6 8 mm groove opening and can't turn.
  - Posts/legs 40x40 at the corners, flush with the frame outside; shelf and lower-ring rails between them.
  - Shelf height, board top (30 in), camera height and laptop table depth as in gantry_cart.py.
  - Casters: item 0.0.418.10 (D125 swivel, double brake, 158 mm high, one 12.5 mm bolt hole) screwed into
    the leg's core bore, tapped M8, so the leg stands on the caster's top face.

Prices are item24.com US list prices read on PRICE_DATE (profiles: per metre + a cut charge per piece; volume
tiers applied on the order quantity). Items marked "est." are hardware-store estimates. Shipping and tax
are not included.

Run:  python cart_bom_item.py   -> prints the BOM and item24 order list, writes cart_bom_item.csv and
                                   cart_quote_item.html
"""
import csv
import datetime
import html
from collections import OrderedDict
from pathlib import Path
import gantry_cart as g

PRICE_DATE = "2026-10-16"
IN = 25.4

# --- metric geometry ---
P6, P8 = 30.0, 40.0                   # Line 6 rail and Line 8 post sections
BOARD = g.BOARD * IN                  # 609.6
BOARD_TOP = g.BOARD_TOP * IN
TOP_Z = BOARD_TOP - g.BOARD_T * IN
EDGE_ROW = g.EDGE_ROW * IN            # 12.7: rail centre lines on the outer hole rows
INSET = P6 / 2 - EDGE_ROW             # 2.3: frame outer faces inside the board edge
FRAME = BOARD - 2 * INSET             # 605.0
PANEL_T = 4.0                         # item compound material Al 4 mm
SHELF_CLEAR = 12.7
POST_TOP = TOP_Z - P6
SHELF_PANEL_TOP = POST_TOP - g.TALLEST * IN - SHELF_CLEAR
SHELF_RAIL_TOP = SHELF_PANEL_TOP - PANEL_T
BOX_BOTTOM = SHELF_RAIL_TOP - P6
POST_L = POST_TOP - BOX_BOTTOM
CASTER_H = 158.0                      # 0.0.418.10, floor to top face
CART_TOP = BOX_BOTTOM
LEG_L = CART_TOP - P6 - CASTER_H
LAPTOP_DEPTH = 355.0                  # 14 in, rounded
LOC_GAP, LOC_T = 0.8, 3.2             # locator shim and angle thickness
BOOM_BOTTOM = BOARD_TOP + (g.CAMERA_HEIGHT + g.CAMERA_H + g.BALLHEAD_H) * IN
MAST_L = BOOM_BOTTOM + P6 - BOX_BOTTOM
BOOM_LEN = (BOARD - INSET) - g.CAMERA_XY[0] * IN + IN   # mast face to 1 in past the camera

# --- item24 parts: id -> (description, unit, [(min qty, unit price)]) ---
# unit "m": price per metre, plus CUT per piece; "m2": quoted per panel below
CUT = 3.95
PARTS = {
    "41906": ("Profile 6 30x30 light, natural", "m", [(1, 34.16)]),
    "2633": ("Profile 8 40x40 light, natural", "m", [(1, 42.72)]),
    "41967": ("Angle Bracket Set 6 30x30 (bracket, cap, 2 M6x14, 2 washers, 2 T-slot nuts 6 M6)", "pc",
              [(1, 14.32), (10, 13.62), (100, 12.92)]),
    "48050": ("T-Slot Nut V 8 St M6, bright zinc-plated (drop-in, spring-held)", "pc", [(1, 1.10), (100, 0.96)]),
    "8000245": ("Button-Head Screw M6x18, bright zinc-plated", "pc", [(1, 0.70), (100, 0.62)]),
    "8000063": ("Button-Head Screw M6x16, bright zinc-plated", "pc", [(1, 0.66), (100, 0.54)]),
    "8000240": ("Button-Head Screw M6x12, bright zinc-plated", "pc", [(1, 0.66), (100, 0.54)]),
    "41940": ("T-Slot Nut 6 St M6, bright zinc-plated", "pc", [(1, 1.16), (100, 1.01)]),
    "41922": ("Cap 6 30x30, black", "pc", [(1, 1.51), (100, 1.35)]),
    "41810": ("Castor D125 swivel with double-brake, 100 kg", "pc", [(1, 94.15), (4, 87.86), (40, 83.42)]),
    "8000009": ("Button-Head Screw M8x30, bright zinc-plated", "pc", [(1, 1.14), (100, 0.98)]),
    "2673": ("Compound Material Al 4mm, natural anodized, cut to size", "panel", None),
}
PANEL_PRICES = {(FRAME, FRAME): 138.97, (LAPTOP_DEPTH - LOC_GAP - LOC_T, FRAME): 83.86}   # item24 quotes, incl. cut
OTHER = {   # not from item: (description, unit price)
    "washer-M8": ("M8 fender washer 8.4 x 24, under the caster bolt (est.)", 0.25),
    "ballhead": ("1/4-20 mini ball head for the camera (est.)", 15.00),
    "stud-M6": ("M6 to 1/4-20 adapter stud, ball head to boom (est.)", 2.00),
    "angle": ("Aluminium angle 50 x 50 x 3 mm, cut 89 mm, 3 holes (est.)", 6.00),
}

BOX, CART = "Bench box", "Wheeled cart"
RAILS = {BOX: [("Top front/back rail", 2, FRAME),
               ("Top side rail", 2, FRAME - 2 * P6),
               ("Shelf rail", 4, FRAME - 2 * P8),
               ("Camera mast", 1, MAST_L),
               ("Camera boom", 1, BOOM_LEN)],
         CART: [("Top front/back rail (cantilevers to laptop table)", 2, FRAME + LAPTOP_DEPTH),
                ("Top side + laptop end rail", 3, FRAME - 2 * P6),
                ("Lower ring rail", 4, FRAME - 2 * P8)]}
LEGS = {BOX: [("Post", 4, POST_L)], CART: [("Leg", 4, LEG_L)]}
PANELS = {BOX: [("Shelf panel (notch the 4 post corners 40 x 40)", FRAME, FRAME)],
          CART: [("Laptop table panel", LAPTOP_DEPTH - LOC_GAP - LOC_T, FRAME)]}
# joints from the 80/20 model, split by what they join: (Line 6 to Line 6, Line 6 rail to Line 8 post/leg)
JOINTS = {BOX: (4 + 2 + 2 * len(g.BOX_Y_RAILS), 8 + 8), CART: (2 * len(g.CART_Y_RAILS), 8 + 8)}
assert sum(JOINTS[BOX]) == len(g.box_joints()) and sum(JOINTS[CART]) == len(g.cart_joints())
PANEL_SCREWS = {BOX: len(g.PANEL_SCREWS["shelf"]), CART: len(g.PANEL_SCREWS["laptop"])}
END_CAPS = {BOX: 2 * len(g.END_CAPS["box"]), CART: 2 * len(g.END_CAPS["cart"])}
LOC_RAIL, LOC_LEG = 6, 6              # locator bolts into the cart's top rails (Line 6) and legs (Line 8)


def lines():
    """[(unit, section, part, qty, description, length mm or None)] before pricing."""
    out = []
    for u in (BOX, CART):
        for name, n, L in RAILS[u]:
            out.append((u, "Frame", "41906", n, name, round(L, 1)))
        for name, n, L in LEGS[u]:
            out.append((u, "Frame", "2633", n, name, round(L, 1)))
        same, mixed = JOINTS[u]
        out.append((u, "Joining", "41967", same + mixed, f"{same} rail-rail + {mixed} rail-post joints", None))
        out.append((u, "Joining", "48050", mixed, "post side of the rail-post joints (replaces the Line 6 nut)", None))
        out.append((u, "Joining", "8000245", mixed, "post side of the rail-post joints (the set's M6x14 is short for Line 8)", None))
        if u == CART:
            out.append((u, "Locating", "angle", 4, "box locators on the leg tops", None))
            out.append((u, "Locating", "8000240", LOC_RAIL, "locator angles to the top rails", None))
            out.append((u, "Locating", "41940", LOC_RAIL, "locator angles to the top rails", None))
            out.append((u, "Locating", "8000063", LOC_LEG, "locator angles to the legs", None))
            out.append((u, "Locating", "48050", LOC_LEG, "locator angles to the legs", None))
            out.append((u, "Mobility", "41810", 4, "one under each leg", None))
            out.append((u, "Mobility", "8000009", 4, "caster to the leg's core bore (tap M8)", None))
            out.append((u, "Mobility", "washer-M8", 4, "", None))
        for name, a, b in PANELS[u]:
            out.append((u, "Surfaces", "2673", 1, f"{name}, {a:g} x {b:g}", (a, b)))
        out.append((u, "Surfaces", "8000240", PANEL_SCREWS[u], "panel to rails", None))
        out.append((u, "Surfaces", "41940", PANEL_SCREWS[u], "panel to rails", None))
        if u == BOX:
            out.append((u, "Camera", "ballhead", 1, "", None))
            out.append((u, "Camera", "stud-M6", 1, "", None))
            out.append((u, "Camera", "41940", 1, "ball head stud in the boom's bottom slot", None))
        out.append((u, "Finish", "41922", END_CAPS[u], "exposed rail ends", None))
    return out


def tier(part, qty):
    return [p for q, p in PARTS[part][2] if qty >= q][-1]


def build_rows():
    """[(unit, section, part, qty, description, unit $, ext $)], volume tiers on the whole order's quantity."""
    raw = lines()
    order_qty = {}
    for _, _, part, n, _, L in raw:
        if part in PARTS and PARTS[part][1] == "pc":
            order_qty[part] = order_qty.get(part, 0) + n
    rows = []
    for u, sec, part, n, note, L in raw:
        if part in OTHER:
            desc, each = OTHER[part][0] + (f" - {note}" if note else ""), OTHER[part][1]
        elif PARTS[part][1] == "m":
            desc = f"{PARTS[part][0]}: {note}, {L:g} mm"
            each = tier(part, 1) * L / 1000 + CUT
        elif PARTS[part][1] == "panel":
            desc, each = f"{PARTS[part][0]}: {note} mm", PANEL_PRICES[L]
        else:
            desc, each = PARTS[part][0] + (f" - {note}" if note else ""), tier(part, order_qty[part])
        rows.append((u, sec, part, n, desc, round(each, 2), round(n * each, 2)))
    return rows


def order_list():
    """item24 cart lines: [(item number, qty, length/size, description)], identical pieces merged."""
    merged = OrderedDict()
    for _, _, part, n, _, L in lines():
        if part in OTHER:
            continue
        key = (part, L)
        merged[key] = merged.get(key, 0) + n
    fmt = lambda i: f"0.0.{i[:-2].zfill(3)}.{i[-2:]}" if len(i) <= 5 else f"{i[0]}.{i[1]}.{i[2:5]}.{i[5:]}"
    out = []
    for (part, L), n in merged.items():
        size = "" if L is None else (f"{L[0]:g} x {L[1]:g} mm" if isinstance(L, tuple) else f"{L:g} mm")
        out.append((fmt(part), part, n, size, PARTS[part][0]))
    return out


def money(x):
    return f"${x:,.2f}"


def write_quote(rows, path, total_8020):
    total = sum(r[6] for r in rows)
    sub = {u: sum(r[6] for r in rows if r[0] == u) for u in (BOX, CART)}
    vendor = sum(r[6] for r in rows if r[2] not in OTHER)
    other = total - vendor
    today = datetime.date.today().isoformat()

    def table(u):
        trs = "".join(
            f"<tr><td>{html.escape(r[1])}</td><td class=pn>{html.escape(r[2])}</td><td class=num>{r[3]}</td>"
            f"<td>{html.escape(r[4])}</td><td class=num>{money(r[5])}</td><td class=num>{money(r[6])}</td></tr>"
            for r in rows if r[0] == u)
        return (f"<h2>{u}<span>{money(sub[u])}</span></h2><div class=scroll><table><thead><tr><th>Section</th><th>Part</th>"
                f"<th class=num>Qty</th><th>Description</th><th class=num>Each</th><th class=num>Extended</th></tr></thead>"
                f"<tbody>{trs}</tbody><tfoot><tr><td colspan=5>{u} subtotal</td><td class=num>{money(sub[u])}</td></tr>"
                f"</tfoot></table></div>")

    orders = "".join(f"<tr><td class=pn>{html.escape(f)}</td><td class=num>{n}</td><td class=num>{html.escape(s)}</td>"
                     f"<td>{html.escape(d)}</td></tr>" for f, _, n, s, d in order_list())
    diff = total - total_8020
    doc = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gantry Stand Quote (item)</title>
<style>
:root {{ --bg:#f6f6f4; --card:#fff; --ink:#1d1d1f; --mute:#6b6b70; --line:#e2e2e0; --accent:#0a6ebd; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#141416; --card:#1d1d20; --ink:#ececee; --mute:#9a9aa0; --line:#2e2e33; --accent:#5aaef0; }} }}
:root[data-theme="dark"] {{ --bg:#141416; --card:#1d1d20; --ink:#ececee; --mute:#9a9aa0; --line:#2e2e33; --accent:#5aaef0; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }}
main {{ max-width:1040px; margin:0 auto; padding:32px 16px 64px; }}
header {{ display:flex; flex-wrap:wrap; justify-content:space-between; gap:16px; align-items:flex-end; border-bottom:2px solid var(--ink); padding-bottom:16px; }}
h1 {{ margin:0; font-size:28px; letter-spacing:-.01em; }}
.meta {{ color:var(--mute); font-size:13px; text-align:right; }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr)); gap:12px; margin:24px 0; }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:14px 16px; }}
.card .k {{ color:var(--mute); font-size:12px; text-transform:uppercase; letter-spacing:.06em; }}
.card .v {{ font-size:24px; font-weight:650; font-variant-numeric:tabular-nums; }}
.card.total {{ border-color:var(--accent); }} .card.total .v {{ color:var(--accent); }}
h2 {{ display:flex; justify-content:space-between; font-size:18px; margin:32px 0 8px; }}
h2 span {{ font-variant-numeric:tabular-nums; }}
.scroll {{ overflow-x:auto; background:var(--card); border:1px solid var(--line); border-radius:10px; }}
table {{ width:100%; border-collapse:collapse; font-size:13.5px; }}
th, td {{ padding:7px 10px; border-bottom:1px solid var(--line); text-align:left; vertical-align:top; }}
th {{ color:var(--mute); font-weight:600; font-size:12px; text-transform:uppercase; letter-spacing:.04em; }}
td.num, th.num {{ text-align:right; white-space:nowrap; font-variant-numeric:tabular-nums; }}
td.pn {{ white-space:nowrap; font-family:ui-monospace,Consolas,monospace; font-size:12.5px; }}
tfoot td {{ font-weight:650; border-bottom:none; }}
.notes {{ margin-top:32px; color:var(--mute); font-size:13.5px; }}
.notes li {{ margin:4px 0; }}
@media print {{ body {{ background:#fff; }} .scroll, .card {{ border-color:#ccc; }} main {{ padding-top:0; }} }}
</style></head><body><main>
<header><div><h1>Gantry Stand: Price Quote (item)</h1>
<div style="color:var(--mute)">Bench box + wheeled cart for the 24 &times; 24 in breadboard gantry, item Line 6 30&times;30 rails + Line 8 40&times;40 posts</div></div>
<div class=meta>Quote date {today}<br>item24.com US list prices read {PRICE_DATE}<br>Generated from cart_bom_item.py</div></header>
<div class=cards>
<div class=card><div class=k>{BOX}</div><div class=v>{money(sub[BOX])}</div></div>
<div class=card><div class=k>{CART}</div><div class=v>{money(sub[CART])}</div></div>
<div class=card><div class=k>item24 order</div><div class=v>{money(vendor)}</div></div>
<div class=card><div class=k>Other / est.</div><div class=v>{money(other)}</div></div>
<div class="card total"><div class=k>Total before shipping &amp; tax</div><div class=v>{money(total)}</div></div>
<div class=card><div class=k>vs 80/20 build</div><div class=v>{"+" if diff >= 0 else "&minus;"}{money(abs(diff))}</div></div>
</div>
{table(BOX)}
{table(CART)}
<h2>item24 order list</h2><div class=scroll><table><thead><tr><th>Item no.</th><th class=num>Qty</th><th class=num>Length / size</th>
<th>Description</th></tr></thead><tbody>{orders}</tbody></table></div>
<div class=notes><strong>Notes</strong><ul>
<li>item24.com US list prices, volume tiers applied on the order quantity. Profiles are priced per metre plus {money(CUT)} per cut piece; panels are item24's cut-to-size quotes. Shipping and tax are not included.</li>
<li>The 80/20 build of the same stand (cart_bom.py) totals {money(total_8020)}.</li>
<li>Rail-post joints: Line 6 angle bracket set, with a T-slot nut V 8 M6 and an M6x18 on the Line 8 post side; the set's spare Line 6 nut and M6x14 are left over.</li>
<li>Rail centre lines stay on the board's outer hole rows, so the frame is 605.0 mm square, 2.3 mm inside the board edge; the gantry legs' M4 nuts sit in the Line 6 groove opening.</li>
<li>Board top {BOARD_TOP:g} mm with the box on the cart; cart top / laptop table at {CART_TOP:.1f} mm; casters 158 mm.</li>
<li>Items not from item (estimated): {", ".join(sorted(OTHER))}.</li>
</ul></div>
</main></body></html>
"""
    path.write_text(doc, encoding="utf-8")


if __name__ == "__main__":
    import cart_bom
    total_8020 = sum(r[6] for r in cart_bom.build_rows())
    rows = build_rows()
    total = sum(r[6] for r in rows)
    here = Path(__file__).parent
    with (here / "cart_bom_item.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Unit", "Section", "Part", "Qty", "Description", "Unit $", "Ext $"])
        w.writerows(rows)
        w.writerow(["", "", "", "", "Total (before shipping/tax)", "", round(total, 2)])
    write_quote(rows, here / "cart_quote_item.html", total_8020)

    for u in (BOX, CART):
        print(u)
        for r in (r for r in rows if r[0] == u):
            print(f"  {r[1]:9s} {r[2]:9s} {r[3]:3d}  {r[4][:70]:70s} {r[5]:8.2f} {r[6]:9.2f}")
        print(f"  {'':9s} {'':9s} {'':3s}  {'subtotal':70s} {'':8s} {sum(r[6] for r in rows if r[0] == u):9.2f}")
    print(f"  {'':9s} {'':9s} {'':3s}  {'TOTAL':70s} {'':8s} {total:9.2f}   (80/20: {total_8020:.2f})")
    print("\nitem24 order list")
    for f, _, n, s, d in order_list():
        print(f"  {f:12s} {n:3d}  {s:>16s}  {d}")
