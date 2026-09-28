# Replacing the i1 iSis with a FUYU Gantry + i1Pro — Integration Design

**Status:** design sketch only (no code yet).
**Goal:** retire the sealed **i1 iSis** chart scanner and rebuild the same
chart-measurement workflow from two off-the-shelf pieces we already control:

- the **FUYU FMC4030 dual-X gantry** (`FUYUAutomation/FuyuRailController`) for **motion**, and
- the **X-Rite i1Pro** spectrophotometer (`LAUXRiteController`, SDK 4.2.9) for **measurement**.

The iSis combines motion and measurement in one box. We split those: the gantry
positions the i1Pro over each patch, and a thin orchestrator triggers a spot
measurement at every stop. Everything downstream of acquisition is reused
unchanged.

---

## 1. Why this is mostly glue, not a rewrite

### 1.1 The data structures already match

The i1Pro SDK and the iSis SDK share identical measurement primitives:

| Quantity | iSis | i1Pro |
|---|---|---|
| Spectral bands | `SPECTRUM_SIZE == 36` (380–730 nm @ 10 nm) | same |
| Tristimulus | `TRISTIMULUS_SIZE == 3` | same |
| Density | `DENSITY_SIZE == 4` | same |
| Trigger | `ISIS_TriggerMeasurement(...)` | `I1_TriggerMeasurement(dev)` |
| Read back | `ISIS_GetSpectrum/GetTriStimulus/GetDensities(buf, row, col)` | `I1_GetSpectrum/GetTriStimulus/GetDensities(dev, buf, index)` |

Because the shapes are identical, **`LAUi1IsisDialog::ColorMeasurement` is reused
verbatim**, and so are:

- `generateTestPattern()` — TIFF chart rendering,
- `saveMeasurementsToDisk()` / `readMeasurementsFromDisk()` — CSV,
- `exportToI1ProfilerCGATS()` and `exportToCxF3()`,
- `mergeMeasurementLists()`, `writeTI3File()`, and the Argyll `targen`/`colprof` flow.

We replace **only** the acquisition path (`scanTarget()`), not the data model or
the outputs.

### 1.2 The gantry already exposes the exact hook we need

`LAUMultiVelmexWidget::scanUserPath(QList<QVector4D> points)` accepts an arbitrary
list of coordinates, rasters through them non-blocking, and emits

```
emitTriggerScanner(float pos, int n, int N)   // n = current point, N = total
```

**at each point.** That signal is precisely where the i1Pro measurement fires. We
do not have to write the motion loop — it exists and is already poll-settled
(`FMC4030_Jog_Single_Axis` then poll `FMC4030_Check_Axis_Is_Stop` on a 500 ms
timer). The gantry is built with axes `{0, 2}` = **X gantry + Y beam**, which is
the 2-D plane we need.

### 1.3 The i1Pro measurement logic already exists

`LAUEyeOneDialog` (in `LAUXRiteController`) already wraps the i1Pro lifecycle:
`I1_GetDevices` → `I1_OpenDevice` → mode select (`I1_REFLECTANCE_SPOT`) →
`I1_Calibrate` → `I1_TriggerMeasurement` → `I1_GetSpectrum`. That code refactors
directly into a reusable, headless **measurement helper** (no dialog UI required).

---

## 2. The conceptual mapping

| iSis (today) | Gantry + i1Pro (proposed) |
|---|---|
| `scanTarget()` sets chart geometry in mm, waits for chart insert, pre-scans to register | Jog to two corners to register the chart, generate an XY path from the patch grid, hand it to `scanUserPath()` |
| `ISIS_TriggerMeasurement(false)` — 2 rows per trigger | `emitTriggerScanner` callback → one `I1_TriggerMeasurement` per patch |
| `ISIS_GetTriStimulus/Spectrum/Densities(row, col)` | `I1_Get*(device, buf, 0)` into the same `ColorMeasurement` |
| optical auto-registration via the alignment bar | **two-corner manual jog** (see §4) |
| device finds patches itself | host computes patch centres → gantry XY (mm) |

---

## 3. Proposed architecture

Recommended: a **new combined dialog**, e.g. `LAUGantryScannerDialog`, that leaves
the existing iSis path untouched and owns three collaborators:

```
LAUGantryScannerDialog
├── motion:       LAUMultiVelmexWidget({0, 2})      // reused from FuyuRailController
│                 (or just its LAUVelmexController)
├── measurement:  LAUi1ProMeasurer                  // headless helper refactored
│                 (open / calibrate / measure)       //   out of LAUEyeOneDialog
└── data + I/O:   LAUi1IsisDialog::ColorMeasurement  // reused verbatim,
                  + generateTestPattern / exporters  //   incl. all exports
```

The scan loop **replaces** `scanTarget()`:

```
1. Operator registers the chart (two-corner jog, §4).
2. Build QList<QVector4D> path = patchCentresToXY(grid, corners)   // §4.3
3. controller->scanUserPath(path)
4. On each emitTriggerScanner(pos, n, N):
     - confirm axes stopped + dwell (§5.2)
     - measurer.trigger(); measurer.readInto(measurement[n])
     - measurement[n].row/col = gridIndexOf(n)
     - append to scanMeasurements
5. On completion: hand scanMeasurements to the existing exporters.
```

Alternative structures considered (not recommended for the first cut):

- **Swap inside `LAUi1IsisDialog::scanTarget()`** — least new UI, but deletes the
  iSis option and entangles two device stacks in one method.
- **Merge the repos** — pull the Velmex classes into `LAUXRiteController`. Worth
  doing eventually so it is one app/one build, but it is a packaging decision that
  can follow a working prototype.

> Either way the two repos must be brought into one build. The Velmex classes
> (`lauvelmexwidget.h/.cpp`) and the i1Pro SDK linkage (`i1Pro64.lib`, runtime
> `FMC4030-Dll.dll`) both end up in the same `.pro`.

---

## 4. Registration — the two-corner jog

The gantry has no idea where the printed chart sits on its bed; unlike the iSis it
cannot find the chart optically. So the operator **jogs the i1Pro head to two
diagonally-opposite patch centres** and we derive the patch→XY map from those.

### 4.1 Procedure

1. Place and fix the chart on the bed (a registration corner/fence helps
   repeatability but is not required).
2. Calibrate the i1Pro on its white tile (§5.1).
3. Jog the head until the aperture is centred on patch **(row 0, col 0)** →
   record gantry position **P₀ = (x₀, y₀)** in mm.
4. Jog to the opposite corner patch **(row R−1, col C−1)** (for the default chart,
   row 29, col 28) → record **P₁ = (x₁, y₁)**.
5. Store P₀, P₁ for the run (and persist to `QSettings` for repeat runs on the
   same fixture).

The jog UI already exists: the Velmex position sliders / spin boxes move a single
axis to a setpoint. A small "Set corner 1 / Set corner 2" pair of buttons captures
the current `position(0)` / `position(2)` of each axis.

### 4.2 What two corners give you (and what they do not)

Two diagonally-opposite patch centres pin down **translation + independent
per-axis scale** under the assumption that the chart's rows/cols are
**axis-aligned** with the gantry (no in-plane rotation or skew):

```
colPitchX = (x₁ − x₀) / (C − 1)      // mm of X per column step
rowPitchY = (y₁ − y₀) / (R − 1)      // mm of Y per row step
originXY  = P₀                        // centre of patch (0,0)
```

This absorbs print scaling and bed placement automatically — we do **not**
hard-code the 6 mm patch pitch; we measure it from the two corners. (The iSis
artwork is ~6 mm patches over a 204 mm sheet, a useful sanity check: expect
`colPitchX ≈ rowPitchY ≈ 6 mm`.)

**Limitation:** two diagonal points cannot separate rotation from scale (3 unknowns,
2 equations). If the chart is slightly **rotated** on the bed, a diagonal pair
silently folds that rotation into the pitches and mis-targets interior patches.

Two robust options, both still "two corners":

- **Same-edge pair (handles rotation):** jog to **(0,0)** and **(0, C−1)** — both on
  the top row. Their difference is the pure **column axis vector** `u`, which gives
  column pitch **and** the chart's rotation angle. Take the row axis `v` ⟂ `u`
  (perpendicular), with row pitch from the artwork or a quick third tap. Best when
  the bed fixture cannot guarantee alignment.
- **Diagonal pair (simplest):** jog to **(0,0)** and **(R−1, C−1)**, assume
  axis-aligned. Fine when a fence/corner squares the chart to the gantry travel.

Recommendation: **diagonal pair + a mechanical fence** for the first build (assume
no rotation); upgrade to the same-edge pair (or a third corner → full affine) only
if bench tests show interior-patch drift.

### 4.3 Patch → XY (axis-aligned, diagonal corners)

```cpp
// grid index n (raster order) → patch (row, col) → gantry XY in mm
int row = n / C;
int col = n % C;
double x = originX + col * colPitchX;     // colPitchX = (x1-x0)/(C-1)
double y = originY + row * rowPitchY;     // rowPitchY = (y1-y0)/(R-1)
path << QVector4D(x, 0.0f, y, 0.0f);      // dim 0 = X, dim 2 = Y (Z,W unused)
```

`scanUserPath()` already clamps each coordinate to the calibrated axis travel, so a
bad corner capture cannot drive the head past a soft limit.

> **Full-affine upgrade (optional):** capturing a **third** corner (e.g. (R−1, 0))
> lets you solve a general 2-D affine map `XY = A·[col,row] + b` (handles rotation
> **and** skew). Same UI, one more "Set corner 3" tap. Worth keeping the math
> general so this is a config flag, not a rewrite.

---

## 5. Engineering concerns (the parts that are not glue)

### 5.1 No Z axis — fixed standoff
The gantry is X/Y only (`{0, 2}`); there is no Z. The i1Pro is a **contact spot**
instrument whose aperture wants to sit on the media. So the head mounts to the Y
carriage at a **fixed Z**, and reflectance accuracy depends on the media being
**flat and at a known height**. This is the largest physical risk. Mitigations:
spring-loaded/compliant head mount, a vacuum or platen to hold media flat,
verification against a known reference chart. (If standoff proves critical, a
future Z stage on the unused FMC4030 axis 1 is an option — but that axis currently
carries the shared X signal, so it would need rewiring.)

### 5.2 Settling dwell
Motion is non-blocking and "settled" means `FMC4030_Check_Axis_Is_Stop` returned
stopped. Add a short **dwell** (e.g. 200–500 ms, tunable) after stop and before
`I1_TriggerMeasurement` so carriage vibration does not smear the reading.

### 5.3 Calibration & expiry
The i1Pro must be **calibrated on its white tile** before a run, and calibration
expires (`I1_TIME_SINCE_LAST_CALIBRATION` is −1 / aged). Two patterns:

- **Manual:** operator calibrates by hand (head on tile) before the run, like
  `LAUEyeOneDialog::onCalibrateButtonClicked()`.
- **Automated:** mount the white tile at a **known XY** on the bed; the scan loop
  parks there and calls `I1_Calibrate()` at the start (and optionally every N
  patches if a long run risks expiry).

### 5.4 Throughput
~870 patches × (move + dwell + ~1–2 s measurement) ≈ **15–30 min** per chart,
versus seconds for the iSis. Acceptable, but set expectations and show a progress
dialog (the iSis path already used `QProgressDialog`).

### 5.5 Threading
`LAUVelmexController` lives in its own `QThread` and polls on a 500 ms timer. The
i1Pro SDK calls must be serialized against motion. Cleanest first cut: drive the
measurement from the **GUI thread** inside the `emitTriggerScanner` slot
(`Qt::QueuedConnection`), only after motion confirms stopped — no shared-state race
with the controller thread.

### 5.6 M0 / M1 / M2 (UV handling)
The iSis exposed an illumination mode (UV / No-UV). The i1Pro2 supports M0/M1/M2 via
`I1_RESULT_INDEX_KEY` on a dual measurement (subject to device variant — UV-cut
units expose only M2). Map the old UV toggle onto the i1Pro result index where the
hardware allows; otherwise drop to a single mode and note it in the export header
(`MEASUREMENT_MODE`).

### 5.7 Registration drift over a run
Thermal/mechanical drift over a 20-minute run can shift the origin. If bench tests
show it, re-tap a corner mid-run or add fiducials. Out of scope for v1.

---

## 6. File / change checklist (no code yet)

| Item | Action |
|---|---|
| `lauvelmexwidget.h/.cpp` | **Reuse** as the motion layer (verbatim from `FuyuRailController`). |
| i1Pro measurement helper | **New** `LAUi1ProMeasurer` — refactor open/calibrate/measure out of `LAUEyeOneDialog` (no UI). |
| `LAUGantryScannerDialog` | **New** orchestrator: registration UI (two-corner jog), path build, scan loop, progress. |
| `ColorMeasurement` + exporters | **Reuse** from `LAUi1IsisDialog` (CSV / CGATS / CxF3 / TIFF / Argyll). Consider lifting them into a shared header so the new dialog does not depend on the iSis dialog. |
| `patchCentresToXY()` | **New** small geometry function (§4.3); keep it affine-general so 2- or 3-corner is a flag. |
| Build (`.pro`) | **Merge** Velmex sources + i1Pro SDK link (`i1Pro64.lib`) + runtime `FMC4030-Dll.dll` copy step into one project. Keep the FMC4030 **non-blocking** DLL (poll-based control depends on it). |
| `main.cpp` | Add an entry path that launches `LAUGantryScannerDialog` (alongside, or instead of, the iSis dialog). |

---

## 7. Open questions to resolve before implementation

1. **Head mount & standoff:** how is the i1Pro fixtured to the Y carriage, and how
   is flat media / fixed height guaranteed? (Drives §5.1 — the main risk.)
2. **Calibration tile:** manual pre-run, or mounted at a known XY for automated
   calibration? (§5.3)
3. **Chart squareness:** is there a fence/corner to square the chart to gantry
   travel? Decides diagonal-pair vs same-edge-pair vs full-affine registration (§4).
4. **UV/OBA need:** is M1/M2 required, or is single-mode M0 acceptable? (§5.6)
5. **Packaging:** one merged app, or keep `FuyuRailController` and
   `LAUXRiteController` separate and share code via a small static lib?

---

© 2026 Dr. Daniel L. Lau. Design note; references `LAUXRiteController`
(i1 iSis / i1Pro controller) and `FUYUAutomation/FuyuRailController` (FMC4030 gantry).
