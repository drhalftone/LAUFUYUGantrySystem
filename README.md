# LAU FUYU Gantry System

Documentation and control software for a FUYU FMC4030 dual-X gantry system.

## Contents

- **`FUYU_FMC4030_Assembly_Wiring_Manual.html`** — self-contained assembly & wiring
  manual, including the isometric system diagram.
- **`gantry_iso.svg`** — isometric diagram of the dual-X gantry (two X rails, Y
  cross-beam, motors, FSK40 limit switches, control cabinet, and the X-axis cable
  carrier).
- **`FuyuRailController/`** — Qt/C++ application for driving the gantry through the
  FMC4030 motion controller.

## Notes

Vendor-supplied FUYU datasheets, SDK binaries, and reference material are **not**
redistributed in this repository. Building `FuyuRailController` requires the FMC4030
SDK DLL from FUYU (see `FuyuRailController/sdk/README.txt`).
