# FuyuRailController

A small Qt Widgets application for driving the **FUYU FMC4030** three-axis motion
controller that runs the dual-X gantry (two X rails + a Y cross-beam).

Its defining design choice: it **reuses the Velmex widget classes** from
`LAU3DVideoRecorder` *verbatim* and only **swaps the controller**. The Velmex GUI
(`LAUVelmexWidget`, `LAUMultiVelmexWidget`, `LAUVelmexSettingsDialog`, `LAUDoubleSpinBox`)
is unchanged; `LAUVelmexController` is reimplemented to talk to the FMC4030 over Ethernet
instead of a serial VXM. So you get the proven Velmex control panel, driven by FUYU hardware.

---

## Quick start (simulation — no hardware or SDK needed)

The default build runs against a built-in **software simulator**, so it compiles and runs
with nothing plugged in.

**Qt Creator:** open `FuyuRailController.pro`, select the Qt 6 / MSVC 2022 kit, and Build &
Run.

**Command line:**
```
qmake
nmake            # (Windows/MSVC)   or  make / jom
./FuyuRailController
```

Verified to build clean with **Qt 6.9.3, MSVC 2022 (64-bit)**.

You'll see one checkable group box per axis ("X gantry Controls:", "Y beam Controls:"),
each with Left-Limit / Slider-Position / Right-Limit rows and Calibrate / Center / Scan
buttons, plus a `?` settings dialog. In simulation each axis eases toward its commanded
target and the limit switches are emulated, so you can exercise the whole UI.

## Building for real hardware

The vendor library is loaded **at run time** (via `QLibrary`), so the build needs **no
`.lib` and no `.h`** — only the DLL at run time, and only if you actually have hardware.

1. Build with the `hardware` config:
   ```
   qmake CONFIG+=hardware
   nmake
   ```
   In Qt Creator: **Projects ▸ Build ▸ qmake ▸ Additional arguments → `CONFIG+=hardware`**.
   (This build links and runs even with **no** SDK present — it just reports *disconnected*.)
2. For real hardware, place the runtime library where it can be found:
   - Windows: **`FMC4030-Dll.dll`** next to the built `.exe` (or on the `PATH`)
   - Linux: **`libFMC4030-Lib.so`** on the library path
3. **Bitness must match** (use the 64-bit DLL with the MSVC 64-bit kit). On startup the app
   resolves the SDK symbols; if the DLL is missing or a symbol is absent it logs a warning
   and runs disconnected.

A copy of the DLL already lives in `sdk/` and is auto-copied next to the built `.exe` by the
`.pro` (win32 post-link step), so a hardware build finds it with no manual copying.

### Blocking vs non-blocking SDK (important)

FUYU ships **two** builds of the DLL — *blocking* and *non-blocking* — with identical function
names but different behaviour. **This project uses the non-blocking X64 build** (`sdk/FMC4030-Dll.dll`,
extracted from `国外FMC4030/FMC4030 API file (non blocking).rar`).

It must stay non-blocking, because `LAUVelmexController` is **poll-based**: it issues one
`FMC4030_Jog_Single_Axis` and then polls `FMC4030_Check_Axis_Is_Stop` from a 500 ms timer.

| | Non-blocking (used) | Blocking (do **not** use) |
|---|---|---|
| motion call returns | immediately | only when the move finishes |
| live position during a move | yes — the poll updates it | no — jumps at the end |
| Stop / abort mid-move | works | can't be processed until the move ends |
| controller thread during a move | responsive | frozen inside the call |

With the blocking build the code still *compiles and connects*, and simple point-to-point moves
complete — so it can look fine in casual testing — but you lose live position feedback and the
ability to abort a move, which is a safety concern on a gantry. If you ever re-extract the DLL,
take it from the **"non blocking"** archive (X64 subfolder). See `sdk/README.txt` for the path.

The controller defaults to `192.168.0.30:8088`. Set your PC's network adapter to the same
subnet (e.g. `192.168.0.35 / 255.255.255.0`).

---

## Axis mapping

The FMC4030 SDK numbers axes `0 = X (PU1)`, `1 = Y (PU2)`, `2 = Z (PU3)`. On this gantry both
X rails share **one** controller axis (a single PU/DR signal), so:

| App axis | SDK index | Physical |
|----------|-----------|----------|
| X gantry | 0 | both X rails (shared signal) |
| (unused) | 1 | — |
| Y beam   | 2 | cross-beam carriage |

`main.cpp` creates the panel with `LAUMultiVelmexWidget({0, 2})`.

## How it works

- **Units:** the widgets speak integer "counts"; here **1 count = 1 µm** (0.001 mm). The
  controller converts counts ↔ millimetres at the SDK boundary, and the display can show
  **mm** (default) or **inches** — switchable in the `?` dialog, where the **Speed** field
  also follows the unit (mm/s ↔ in/s). The unit choice is shared by every axis.
- **Motion is non-blocking.** A move issues `FMC4030_Jog_Single_Axis` once; the controller's
  500 ms `timerEvent` polls `FMC4030_Check_Axis_Is_Stop` until the axis settles. The position
  slider is a *setpoint* (Center/Scan jump straight to it — no scrolling). This depends on the
  **non-blocking** SDK build (see *Blocking vs non-blocking SDK* above).
- **Calibration discovers travel from the limit switches**, like Velmex: `calibrateLeft()`
  homes the **negative** switch (sets zero) and `calibrateRight()` seeks the **positive**
  switch (sets the travel). The sliders' range then reflects the real travel — there is no
  manual "full travel" field.
- **Settings dialog** (`?`): **Units · Home Toward · Speed · Scan Steps**. The Velmex-specific
  mechanical fields (rotary, motor clicks/rotation, travel/rotation) were removed because the
  FMC4030 abstracts them away.

## Files

| File | Purpose |
|------|---------|
| `lauvelmexwidget.h/.cpp` | Velmex widget classes (verbatim) **+** the FMC4030-backed `LAUVelmexController` and trimmed `LAUVelmexSettingsDialog` |
| `main.cpp` | Builds `LAUMultiVelmexWidget({0, 2})` |
| `FuyuRailController.pro` | qmake project; selects simulation vs hardware |
| `sdk/` | holds the **non-blocking X64** `FMC4030-Dll.dll` (auto-copied next to the `.exe`); see `sdk/README.txt` |

## FMC4030 vs Velmex (short version)

The Velmex VXM is an RS-232 controller you drive with ASCII commands in motor **step counts**,
with **blocking** moves (poll busy/ready). The FMC4030 is an **Ethernet** controller driven by
a C **SDK** in native **millimetres**, with **non-blocking** moves (poll is-stopped) and
hardware multi-axis interpolation. We kept `LAUVelmexController`'s public interface identical
and replaced only its internals (serial+steps → SDK+mm), so the GUI didn't change.

## Things to verify on the bench

- `calibrateRight` relies on the FMC4030 **halting the axis at the positive limit switch**
  (bounded to a 2000 mm safety ceiling if a switch is missing). Confirm the controller stops
  at `LP` before trusting auto-travel.
- Homing is assumed to **zero the axis at the switch** — standard for the FMC4030 but confirm.
- `FMC4030_Set_Output` polarity follows the SDK wording (status `0` = output HIGH); flip in the
  controller if your wiring is inverted.

---

© 2026 Dr. Daniel L. Lau. Reuses the LAU3DVideoRecorder Velmex module.
