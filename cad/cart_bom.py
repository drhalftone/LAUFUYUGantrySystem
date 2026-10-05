"""Bill of materials and cost estimate for the gantry cart (gantry_cart.py).

Cut lengths come from gantry_cart.py, so the BOM tracks the model. Prices are 80/20 list prices
read from 8020.net on 2026-10-04 (single-piece pricing; pack pricing is lower); items marked
"est." are hardware-store estimates. Shipping and tax are not included.

Run:  python cart_bom.py   -> prints the BOM, writes cart_bom.csv
"""
import csv
from pathlib import Path
import gantry_cart as g

PRICES = {                                 # part: (unit price, description)
    "1010": (0.45, "1010 profile, 1.00 x 1.00 in, per inch"),
    "1010-cut": (3.00, "1010 cut charge, per cut"),
    "2020": (1.13, "2020 profile, 2.00 x 2.00 in, per inch"),
    "2020-cut": (3.18, "2020 cut charge, per cut"),
    "2323": (42.78, "Flange mount swivel caster, 5 in, top brake, 300 lb"),
    "2419": (28.46, "10 series flange mount caster base plate"),
    "4132": (7.11, "10 series 2-hole gusseted inside corner bracket"),
    "3393": (0.73, "Bolt assembly: 1/4-20 BHSCS + economy T-nut, 2 per 4132 (use the .375 long version)"),
    "2633": (12.47, "Lite aluminum composite panel, .236 in, per sq ft"),
    "2633-cut": (16.13, "Panel cut-to-size charge, per panel"),
    "2015-Plain": (2.29, "10 series 1 x 1 end cap (exposed 1010 ends)"),
    "SHCS-5/16": (0.35, "5/16-18 x 5/8 SHCS, caster flange to 2419 plate (est.)"),
    "leg-end": (3.00, "Tap 2020 end hole 1/4-20 + 1/4-20 x 3/4 SHCS, 2419 to leg (est.)"),
    "ballhead": (15.00, "1/4-20 mini ball head for the camera (est.)"),
    "3393-cam": (0.73, "1/4-20 stud + economy T-nut, ball head to boom (est. as 3393)"),
    "3321": (0.87, "Bolt assembly: 1/4-20 x .5 FBHSCS + economy T-nut, panel to rail"),
}

rails_10 = [("Top front/back rail (cantilevers to laptop table)", 2, g.BOARD + g.LAPTOP_DEPTH),
            ("Top side + laptop end rail", 3, g.BOARD - 2 * g.P10),
            ("Shelf rail", 4, g.BOARD - 2 * g.P20),
            ("Camera mast", 1, round(g.MAST_L, 2)),
            ("Camera boom", 1, round(g.BOOM_LEN, 2))]
legs_20 = [("Leg", 4, round(g.LEG_L, 2))]

# joints: Y top rails (left, right, laptop end) to the X rails = 6; each leg to both top rails = 8;
# shelf rails to legs = 8
gussets = len(g.joints())

rows = []
def add(section, part, qty, desc, unit, ext=None):
    rows.append((section, part, qty, desc, round(unit, 2), round(ext if ext is not None else qty * unit, 2)))

for name, n, L in rails_10:
    add("Frame", "1010", n, f"{name}, {L:g} in", PRICES["1010"][0] * L)
add("Frame", "1010-cut", sum(n for _, n, _ in rails_10), PRICES["1010-cut"][1], PRICES["1010-cut"][0])
for name, n, L in legs_20:
    add("Frame", "2020", n, f"{name}, {L:g} in", PRICES["2020"][0] * L)
add("Frame", "2020-cut", sum(n for _, n, _ in legs_20), PRICES["2020-cut"][1], PRICES["2020-cut"][0])
add("Joining", "4132", gussets, PRICES["4132"][1], PRICES["4132"][0])
add("Joining", "3393", 2 * gussets, PRICES["3393"][1], PRICES["3393"][0])
add("Mobility", "2323", 4, PRICES["2323"][1], PRICES["2323"][0])
add("Mobility", "2419", 4, PRICES["2419"][1], PRICES["2419"][0])
add("Mobility", "SHCS-5/16", 16, PRICES["SHCS-5/16"][1], PRICES["SHCS-5/16"][0])
add("Mobility", "leg-end", 8, PRICES["leg-end"][1] + " - 2 corner holes per leg", PRICES["leg-end"][0])
panels = [("Shelf panel (notch the 4 leg corners 2 x 2 in)", g.BOARD, g.BOARD),
          ("Laptop table panel", g.LAPTOP_DEPTH, g.BOARD)]
for name, a, b in panels:
    add("Surfaces", "2633", 1, f"{name}, {a:g} x {b:g} in, Lite ACM .236", PRICES["2633"][0] * a * b / 144)
add("Surfaces", "2633-cut", len(panels), PRICES["2633-cut"][1], PRICES["2633-cut"][0])
add("Surfaces", "3321", sum(len(v) for v in g.PANEL_SCREWS.values()), PRICES["3321"][1] + " (countersink the panel)",
    PRICES["3321"][0])
add("Camera", "ballhead", 1, PRICES["ballhead"][1], PRICES["ballhead"][0])
add("Camera", "3393-cam", 1, PRICES["3393-cam"][1], PRICES["3393-cam"][0])
add("Finish", "2015-Plain", 4, PRICES["2015-Plain"][1], PRICES["2015-Plain"][0])

total = sum(r[5] for r in rows)
out = Path(__file__).with_name("cart_bom.csv")
with out.open("w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["Section", "Part", "Qty", "Description", "Unit $", "Ext $"])
    w.writerows(rows)
    w.writerow(["", "", "", "Total (before shipping/tax)", "", round(total, 2)])

for r in rows:
    print(f"{r[0]:9s} {r[1]:13s} {r[2]:3d}  {r[3]:62s} {r[4]:8.2f} {r[5]:9.2f}")
print(f"{'':9s} {'':13s} {'':3s}  {'TOTAL':62s} {'':8s} {total:9.2f}")
print("profile:", ", ".join(f"1010 {n} @ {L:g}" for _, n, L in rails_10), "|",
      ", ".join(f"2020 {n} @ {L:g}" for _, n, L in legs_20))
