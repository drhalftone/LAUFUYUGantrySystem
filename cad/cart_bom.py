"""Master bill of materials for the gantry stand (gantry_cart.py): bench box + wheeled cart.

This is the single BOM to order a complete stand from. Every line is an 80/20 catalog part or an 80/20
machining service (cuts, end taps, panel notches), so the order arrives ready to assemble except for
the steps in USER_STEPS. Parts 80/20 doesn't sell are in NEEDS_APPROVAL and are NOT in the total until
the owner approves them. SCREW_CHECK / TAPPED_CHECK are the separate head-type + length check for
every joint.

Cut lengths and hardware counts come from gantry_cart.py, so the BOM tracks the model. Prices are
80/20 list prices read from 8020.net (profiles, brackets, casters 2026-10-06; plates, screws,
machining, 2650 panel 2026-10-22; single-piece pricing). Shipping and tax are not included.

Run:  python cart_bom.py   -> prints the BOM and screw check, writes cart_bom.csv and cart_quote.html
"""
import csv
import datetime
import html
from pathlib import Path
import gantry_cart as g

PRICE_DATE = "2026-10-22"
PRICES = {                                 # part: (unit price, description)
    "1010": (0.45, "1010 profile, 1.00 x 1.00 in, per inch"),
    "1010-cut": (3.00, "1010 cut charge, per cut"),
    "2020": (1.13, "2020 profile, 2.00 x 2.00 in, per inch"),
    "2020-cut": (3.18, "2020 cut charge, per cut"),
    "7064": (11.77, "Machining: four-hole end tap 1/4-20 x 1.125 deep, leg bottom end (for the 2419)"),
    "4132": (7.11, "10 series 2-hole gusseted inside corner bracket"),
    "3393": (0.73, "Bolt assembly: 1/4-20 x .5 BHSCS (3061) + slide-in economy T-nut (3382)"),
    "4166": (9.71, "10 series 6-hole rectangular flat plate, 3 x 2 x .188, clear"),
    "2323": (42.78, "Flange mount swivel caster, 5 in, top brake, 300 lb"),
    "2419": (28.46, "10 series flange mount caster base plate"),
    "3110": (0.41, "5/16-18 x .625 SHCS, caster flange to 2419"),
    "3064": (0.37, "1/4-20 x .750 SHCS, 2419 to leg end"),
    "2650": (19.68, "Aluminum composite panel (solid core), .236 in, platinum, per sq ft"),
    "2650-cut": (16.13, "Panel cut-to-size charge, per panel"),
    "7599": (5.70, "Machining: panel corner clearance notch 2.06 x 2.06 (10 series)"),
    "3321": (0.87, "Bolt assembly: 1/4-20 x .5 FBHSCS + slide-in economy T-nut, panel to rail"),
    "2015": (2.02, "10 series 1 x 1 end cap, push-in (exposed 1010 ends)"),
}
# Not sold by 80/20: kept out of the order and the total until the owner approves each one.
NEEDS_APPROVAL = [
    ("1/32 in shim stock", "6 pieces 3 x 2 in", "behind each 4166 locator plate, sets the box clearance; 80/20 sells no shim"),
    ("1/4-20 mini ball head + 1/4-20 stud", "1 + 1", "camera on the stand's 1010 boom (economy T-nut in the boom's bottom "
     "slot); skip if the camera rides only on the gantry's own mast (camera_mount.py)"),
]
USER_STEPS = [
    "Drill the panel screw holes: 17/64 in clearance, no countersink (the 3321 is a flanged button head); 80/20 "
    "offers no drilled holes on panels online. Shelf: 4 holes 0.5 in in from the front/back edges, 3 in and 21 in "
    "from the laptop-side edge. Laptop table: 4 holes 0.5 in in from the long edges, 1.5 in and 12.5 in from its box end.",
    "Pre-load every slide-in T-nut before closing a slot end (see stand_assembly_guide.html).",
]
OPEN_ITEMS = [
    "Nothing stops the box sliding toward the laptop end above the 1/4 in panel edge (no locator plate fits there).",
    "The breadboard rests on the box's top frame; it is not fastened.",
    "Drivers, controller and PSU rest on the shelf; their mounting isn't designed.",
]

# Head type + length for every joint. Slot joints: the 3382 nut spans 0.087-0.211 in below the face and
# the slot floor is 0.323 in down, so tip = length - clamp must be in [0.211, 0.323).
NUT_BOTTOM, SLOT_FLOOR = 0.211, 0.323
SCREW_CHECK = [   # (joint, part, head / key, clamped (in), length (in))
    ("4132 bracket to 1010/2020 slot", "3393 (3061)", "button head, 5/32 key", 0.188, 0.500),
    ("4166 locator to rail/leg slot", "3393 (3061)", "button head, 5/32 key; head outboard of the box",
     0.188 + g.LOC_GAP, 0.500),
    ("2650 panel to rail slot", "3321", "flanged button head, 5/32 key; sits 0.13 in proud, no countersink",
     0.236, 0.500),
]
TAPPED_CHECK = [  # (joint, part, head / key, verdict)
    ("2323 caster flange to 2419 (tapped 5/16-18)", "3110", "socket cap, 1/4 key",
     "80/20 lists 4 x 3110 for the 2323; 0.625 in through the flange into the 0.5 in plate"),
    ("2419 to 2020 leg end (7064 end tap, 1.125 deep)", "3064",
     "socket cap, 3/16 key; 0.370 head in the 0.405 counterbore",
     "80/20 lists 3064 for the 2419; the 0.75 in screw stays inside the 1.125 in tap"),
]

BOX, CART = "Bench box", "Wheeled cart"
RAILS_10 = {BOX: [("Top front/back rail", 2, g.BOARD),
                  ("Top side rail", 2, g.BOARD - 2 * g.P10),
                  ("Shelf rail", 4, g.BOARD - 2 * g.P20),
                  ("Camera mast", 1, round(g.MAST_L, 2)),
                  ("Camera boom", 1, round(g.BOOM_LEN, 2))],
            CART: [("Top front/back rail (cantilevers to laptop table)", 2, g.BOARD + g.LAPTOP_DEPTH),
                   ("Top side + laptop end rail", 3, g.BOARD - 2 * g.P10),
                   ("Lower ring rail", 4, g.BOARD - 2 * g.P20)]}
LEGS_20 = {BOX: [("Post", 4, round(g.POST_L, 2))], CART: [("Leg", 4, round(g.LEG_L, 2))]}
PANELS = {BOX: [("Shelf panel", g.BOARD, g.BOARD)],
          CART: [("Laptop table panel", g.LAPTOP_PANEL_L, g.BOARD)]}
JOINTS = {BOX: len(g.box_joints()), CART: len(g.cart_joints())}
PANEL_SCREWS = {BOX: len(g.PANEL_SCREWS["shelf"]), CART: len(g.PANEL_SCREWS["laptop"])}
END_CAPS = {BOX: 2 * len(g.END_CAPS["box"]), CART: 2 * len(g.END_CAPS["cart"])}


def screw_rows():
    """[(joint, part, head, clamp, length, tip, ok)] for the slot joints."""
    return [(j, pn, head, c, L, L - c, NUT_BOTTOM <= L - c < SLOT_FLOOR) for j, pn, head, c, L in SCREW_CHECK]


def build_rows():
    """[(unit, section, part, qty, description, unit $, ext $)]"""
    rows = []

    def add(unit, section, part, qty, desc, each):
        rows.append((unit, section, part, qty, desc, round(each, 2), round(qty * each, 2)))

    for u in (BOX, CART):
        for name, n, L in RAILS_10[u]:
            add(u, "Frame", "1010", n, f"{name}, {L:g} in", PRICES["1010"][0] * L)
        add(u, "Frame", "1010-cut", sum(n for _, n, _ in RAILS_10[u]), PRICES["1010-cut"][1], PRICES["1010-cut"][0])
        for name, n, L in LEGS_20[u]:
            add(u, "Frame", "2020", n, f"{name}, {L:g} in", PRICES["2020"][0] * L)
        add(u, "Frame", "2020-cut", sum(n for _, n, _ in LEGS_20[u]), PRICES["2020-cut"][1], PRICES["2020-cut"][0])
        if u == CART:
            add(u, "Frame", "7064", 4, PRICES["7064"][1], PRICES["7064"][0])
        add(u, "Joining", "4132", JOINTS[u], PRICES["4132"][1], PRICES["4132"][0])
        add(u, "Joining", "3393", 2 * JOINTS[u], PRICES["3393"][1] + ", 2 per 4132", PRICES["3393"][0])
        if u == CART:
            add(u, "Locating", "4166", g.N_LOC_PLATES, PRICES["4166"][1] + " - box locators on the leg tops",
                PRICES["4166"][0])
            add(u, "Locating", "3393", g.N_LOC_BOLTS, PRICES["3393"][1] + ", locator plates", PRICES["3393"][0])
            add(u, "Mobility", "2323", 4, PRICES["2323"][1], PRICES["2323"][0])
            add(u, "Mobility", "2419", 4, PRICES["2419"][1], PRICES["2419"][0])
            add(u, "Mobility", "3110", 16, PRICES["3110"][1], PRICES["3110"][0])
            add(u, "Mobility", "3064", 8, PRICES["3064"][1] + " (2 per plate land on the leg's corner holes)",
                PRICES["3064"][0])
        for name, a, b in PANELS[u]:
            add(u, "Surfaces", "2650", 1, f"{name}, {a:g} x {b:g} in", PRICES["2650"][0] * a * b / 144)
        add(u, "Surfaces", "2650-cut", len(PANELS[u]), PRICES["2650-cut"][1], PRICES["2650-cut"][0])
        if u == BOX:
            add(u, "Surfaces", "7599", 4, PRICES["7599"][1] + ", shelf corners around the 2020 posts", PRICES["7599"][0])
        add(u, "Surfaces", "3321", PANEL_SCREWS[u], PRICES["3321"][1], PRICES["3321"][0])
        add(u, "Finish", "2015", END_CAPS[u], PRICES["2015"][1], PRICES["2015"][0])
    return rows


def money(x):
    return f"${x:,.2f}"


def write_quote(rows, path):
    total = sum(r[6] for r in rows)
    sub = {u: sum(r[6] for r in rows if r[0] == u) for u in (BOX, CART)}
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

    def screw_table():
        trs = "".join(f"<tr><td>{html.escape(j)}</td><td class=pn>{pn}</td><td>{html.escape(h)}</td>"
                      f"<td class=num>{c:.3f}</td><td class=num>{L:.3f}</td><td class=num>{t:.3f}</td>"
                      f"<td>{'OK' if ok else 'FAIL'}</td></tr>" for j, pn, h, c, L, t, ok in screw_rows())
        trs += "".join(f"<tr><td>{html.escape(j)}</td><td class=pn>{pn}</td><td>{html.escape(h)}</td>"
                       f"<td colspan=3>{html.escape(v)}</td><td>OK</td></tr>" for j, pn, h, v in TAPPED_CHECK)
        return (f"<h2>Screw check: head type and length</h2><div class=scroll><table><thead><tr><th>Joint</th>"
                f"<th>Part</th><th>Head / key</th><th class=num>Clamped</th><th class=num>Length</th>"
                f"<th class=num>Tip in slot</th><th></th></tr></thead><tbody>{trs}</tbody></table></div>"
                f"<p class=notes>Slot joints: the tip must reach {NUT_BOTTOM} in (through the economy T-nut) and stay "
                f"under {SLOT_FLOOR} in (the slot floor).</p>")

    def listing(title, items):
        return f"<div class=notes><strong>{title}</strong><ul>" + "".join(f"<li>{x}</li>" for x in items) + "</ul></div>"

    approval = [f"<b>{html.escape(a)}</b> ({html.escape(q)}): {html.escape(w)}" for a, q, w in NEEDS_APPROVAL]
    cut = lambda u: ", ".join(f"{n} @ {L:g} in" for _, n, L in RAILS_10[u])
    doc = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gantry Stand BOM</title>
<style>
:root {{ --bg:#f6f6f4; --card:#fff; --ink:#1d1d1f; --mute:#6b6b70; --line:#e2e2e0; --accent:#c8102e; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#141416; --card:#1d1d20; --ink:#ececee; --mute:#9a9aa0; --line:#2e2e33; --accent:#ff5a6e; }} }}
:root[data-theme="dark"] {{ --bg:#141416; --card:#1d1d20; --ink:#ececee; --mute:#9a9aa0; --line:#2e2e33; --accent:#ff5a6e; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }}
main {{ max-width:1040px; margin:0 auto; padding:32px 16px 64px; }}
header {{ display:flex; flex-wrap:wrap; justify-content:space-between; gap:16px; align-items:flex-end; border-bottom:2px solid var(--ink); padding-bottom:16px; }}
h1 {{ margin:0; font-size:28px; letter-spacing:-.01em; }}
.meta {{ color:var(--mute); font-size:13px; text-align:right; }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:12px; margin:24px 0; }}
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
<header><div><h1>Gantry Stand: Master BOM</h1>
<div style="color:var(--mute)">Bench box + wheeled cart for the 24 &times; 24 in breadboard gantry, 80/20 10-series</div></div>
<div class=meta>Quote date {today}<br>80/20 list prices read {PRICE_DATE}<br>Generated from gantry_cart.py</div></header>
<div class=cards>
<div class=card><div class=k>{BOX}</div><div class=v>{money(sub[BOX])}</div></div>
<div class=card><div class=k>{CART}</div><div class=v>{money(sub[CART])}</div></div>
<div class="card total"><div class=k>80/20 order, before shipping &amp; tax</div><div class=v>{money(total)}</div></div>
</div>
{table(BOX)}
{table(CART)}
{screw_table()}
{listing("Not from 80/20: needs the owner's approval (not in the total)", approval)}
{listing("Owner does", [html.escape(x) for x in USER_STEPS])}
{listing("Open design items (decide before ordering)", [html.escape(x) for x in OPEN_ITEMS])}
<div class=notes><strong>Notes</strong><ul>
<li>80/20 list prices, single piece. Shipping and tax are not included.</li>
<li>Machining (7064 end taps on the leg bottoms, 7599 notches on the shelf corners) is added with the profile or panel's &ldquo;Add Machining Services&rdquo; when ordering.</li>
<li>Cut list. Box 1010: {cut(BOX)}; 2020 posts 4 @ {g.POST_L:.2f} in. Cart 1010: {cut(CART)}; 2020 legs 4 @ {g.LEG_L:.2f} in.</li>
<li>Board top sits at {g.BOARD_TOP:g} in with the box on the cart; the cart top and laptop table are at {g.CART_TOP:.2f} in.</li>
</ul></div>
</main></body></html>
"""
    path.write_text(doc, encoding="utf-8")


if __name__ == "__main__":
    rows = build_rows()
    total = sum(r[6] for r in rows)
    here = Path(__file__).parent
    with (here / "cart_bom.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Unit", "Section", "Part", "Qty", "Description", "Unit $", "Ext $"])
        w.writerows(rows)
        w.writerow(["", "", "", "", "Total (before shipping/tax)", "", round(total, 2)])
    write_quote(rows, here / "cart_quote.html")

    for u in (BOX, CART):
        print(u)
        for r in (r for r in rows if r[0] == u):
            print(f"  {r[1]:9s} {r[2]:11s} {r[3]:3d}  {r[4][:62]:62s} {r[5]:8.2f} {r[6]:9.2f}")
        print(f"  {'':9s} {'':11s} {'':3s}  {'subtotal':62s} {'':8s} {sum(r[6] for r in rows if r[0] == u):9.2f}")
    print(f"  {'':9s} {'':11s} {'':3s}  {'TOTAL (80/20)':62s} {'':8s} {total:9.2f}")
    print(f"Screw check (slot joints: tip in [{NUT_BOTTOM}, {SLOT_FLOOR})):")
    for j, pn, h, c, L, t, ok in screw_rows():
        print(f"  {'OK  ' if ok else 'FAIL'} {pn:12s} {j:34s} clamp {c:.3f} + tip {t:.3f} = {L:.3f}  {h}")
    for j, pn, h, v in TAPPED_CHECK:
        print(f"  OK   {pn:12s} {j:34s} {v}")
    print("Needs approval (not in total):", "; ".join(a for a, _, _ in NEEDS_APPROVAL))
