"""Bill of materials, cost estimate and price quote for the gantry stand (gantry_cart.py): bench box +
wheeled cart.

Cut lengths and hardware counts come from gantry_cart.py, so the BOM tracks the model. Prices are
80/20 list prices read from 8020.net on 2026-10-06 (single-piece pricing; pack pricing is lower);
items marked "est." are hardware-store estimates. Shipping and tax are not included.

Run:  python cart_bom.py   -> prints the BOM, writes cart_bom.csv and cart_quote.html
"""
import csv
import datetime
import html
from pathlib import Path
import gantry_cart as g

PRICE_DATE = "2026-10-06"
PRICES = {                                 # part: (unit price, description)
    "1010": (0.45, "1010 profile, 1.00 x 1.00 in, per inch"),
    "1010-cut": (3.00, "1010 cut charge, per cut"),
    "2020": (1.13, "2020 profile, 2.00 x 2.00 in, per inch"),
    "2020-cut": (3.18, "2020 cut charge, per cut"),
    "2323": (42.78, "Flange mount swivel caster, 5 in, top brake, 300 lb"),
    "2419": (28.46, "10 series flange mount caster base plate"),
    "4132": (7.11, "10 series 2-hole gusseted inside corner bracket"),
    "3416": (0.71, "Bolt assembly: 1/4-20 x .375 BHSCS + economy T-nut, 2 per 4132"),
    "2637": (13.83, "Lite aluminum composite panel, .236 in, black, per sq ft"),
    "2637-cut": (16.13, "Panel cut-to-size charge, per panel"),
    "2015": (2.02, "10 series 1 x 1 end cap (exposed 1010 ends)"),
    "SHCS-5/16": (0.35, "5/16-18 x 5/8 SHCS, caster flange to 2419 plate (est.)"),
    "leg-end": (3.00, "Tap 2020 end hole 1/4-20 + 1/4-20 x 3/4 SHCS, 2419 to leg (est.)"),
    "ballhead": (15.00, "1/4-20 mini ball head for the camera (est.)"),
    "3393-cam": (0.73, "1/4-20 stud + economy T-nut, ball head to boom (est.)"),
    "angle": (6.00, "Aluminium angle 2 x 2 x 1/8 in, cut 3.5 in, 3 holes (est.)"),
    "3321": (0.87, "Bolt assembly: 1/4-20 x .5 FBHSCS + economy T-nut, panel to rail"),
}
NOT_8020 = {"SHCS-5/16", "leg-end", "ballhead", "3393-cam", "angle"}   # bought elsewhere / estimated

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
PANELS = {BOX: [("Shelf panel (notch the 4 post corners 2 x 2 in)", g.BOARD, g.BOARD)],
          CART: [("Laptop table panel", round(g.LAPTOP_DEPTH - g.LOC_GAP - g.LOC_T, 2), g.BOARD)]}
JOINTS = {BOX: len(g.box_joints()), CART: len(g.cart_joints())}
PANEL_SCREWS = {BOX: len(g.PANEL_SCREWS["shelf"]), CART: len(g.PANEL_SCREWS["laptop"])}
END_CAPS = {BOX: 2 * len(g.END_CAPS["box"]), CART: 2 * len(g.END_CAPS["cart"])}


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
        add(u, "Joining", "4132", JOINTS[u], PRICES["4132"][1], PRICES["4132"][0])
        add(u, "Joining", "3416", 2 * JOINTS[u], PRICES["3416"][1], PRICES["3416"][0])
        if u == CART:
            add(u, "Locating", "angle", 4, PRICES["angle"][1] + " - box locators on the leg tops", PRICES["angle"][0])
            add(u, "Locating", "3416", 12, PRICES["3416"][1].split(",")[0] + ", locator angles (1/32 in shim behind each flange)",
                PRICES["3416"][0])
            add(u, "Mobility", "2323", 4, PRICES["2323"][1], PRICES["2323"][0])
            add(u, "Mobility", "2419", 4, PRICES["2419"][1], PRICES["2419"][0])
            add(u, "Mobility", "SHCS-5/16", 16, PRICES["SHCS-5/16"][1], PRICES["SHCS-5/16"][0])
            add(u, "Mobility", "leg-end", 8, PRICES["leg-end"][1] + " - 2 corner holes per leg", PRICES["leg-end"][0])
        for name, a, b in PANELS[u]:
            add(u, "Surfaces", "2637", 1, f"{name}, {a:g} x {b:g} in, Lite ACM .236 black", PRICES["2637"][0] * a * b / 144)
        add(u, "Surfaces", "2637-cut", len(PANELS[u]), PRICES["2637-cut"][1], PRICES["2637-cut"][0])
        add(u, "Surfaces", "3321", PANEL_SCREWS[u], PRICES["3321"][1] + " (countersink the panel)", PRICES["3321"][0])
        if u == BOX:
            add(u, "Camera", "ballhead", 1, PRICES["ballhead"][1], PRICES["ballhead"][0])
            add(u, "Camera", "3393-cam", 1, PRICES["3393-cam"][1], PRICES["3393-cam"][0])
        add(u, "Finish", "2015", END_CAPS[u], PRICES["2015"][1], PRICES["2015"][0])
    return rows


def money(x):
    return f"${x:,.2f}"


def write_quote(rows, path):
    total = sum(r[6] for r in rows)
    sub = {u: sum(r[6] for r in rows if r[0] == u) for u in (BOX, CART)}
    vendor = sum(r[6] for r in rows if r[2] not in NOT_8020)
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

    cut = lambda u: ", ".join(f"{n} @ {L:g} in" for _, n, L in RAILS_10[u])
    doc = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gantry Stand Quote</title>
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
<header><div><h1>Gantry Stand: Price Quote</h1>
<div style="color:var(--mute)">Bench box + wheeled cart for the 24 &times; 24 in breadboard gantry, 80/20 10-series</div></div>
<div class=meta>Quote date {today}<br>80/20 list prices read {PRICE_DATE}<br>Generated from gantry_cart.py</div></header>
<div class=cards>
<div class=card><div class=k>{BOX}</div><div class=v>{money(sub[BOX])}</div></div>
<div class=card><div class=k>{CART}</div><div class=v>{money(sub[CART])}</div></div>
<div class=card><div class=k>80/20 order</div><div class=v>{money(vendor)}</div></div>
<div class=card><div class=k>Other / est.</div><div class=v>{money(other)}</div></div>
<div class="card total"><div class=k>Total before shipping &amp; tax</div><div class=v>{money(total)}</div></div>
</div>
{table(BOX)}
{table(CART)}
<div class=notes><strong>Notes</strong><ul>
<li>80/20 list prices (pack sizes are the same price each). Shipping and tax are not included; for this order 80/20 estimated UPS Ground $40.39 and tax $75.72 to the account address.</li>
<li>Saved on 8020.net as the wish list &ldquo;Gantry stand (bench box + wheeled cart)&rdquo;; use Add All to Cart to order.</li>
<li>Items not from 80/20 (estimated): {", ".join(sorted(NOT_8020))}.</li>
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
    print(f"  {'':9s} {'':11s} {'':3s}  {'TOTAL':62s} {'':8s} {total:9.2f}")
