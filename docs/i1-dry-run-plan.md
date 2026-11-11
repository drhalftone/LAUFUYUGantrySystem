# Plan: dry run through the empty i1 holder to fine-tune patch positions

Status: proposed, 2026-11-11. Nothing built yet.

## Why

Today, aiming the i1 at a patch chains three conversions, each with its own error:

| Step | What sets it | Error today |
|---|---|---|
| Picture to board mm | merged picture, after fine alignment (holes 0.50 mm RMS off the grid) | ~0.5 mm |
| Board mm to rail positions | aperture calibration `A0`, from only 3 of 12 frames on 2026-11-09 (spread 0.67 mm RMS) | ~0.7 mm |
| Rails actually arriving | not measured; the per-frame alignment corrections reached 1.4 mm | ~0.5 to 1 mm |

Together that is roughly 1 to 1.3 mm RMS, with occasional misses of 2 to 3 mm. The i1 Pro reads a spot about 4.5 mm across, so patches under about 10 mm are at risk.

The dry run measures the total error directly. With the i1 out of its holder, the camera sees the printed target through the holder's 9.8 mm hole. The hole's rim and the ink under it lie on the paper together, so how far the ink is from the hole's centre doesn't depend on the camera model, the merge or `A0`. It is the miss itself.

## The flow

Scan target already goes: scan, merge, find the patches (`locateTarget`, `laugantrymeasure.cpp:90`), "Target Found" check dialog, then `startI1()`. The dry run goes between the check dialog and `startI1()`:

1. **Check dialog.** Its OK button reads "Check positions through the holder" instead of going straight to the i1. The i1 is already off the carriage, since Scan target required that.
2. **Choose check points.** Pick 5 points on the target where four patches meet: one near each corner of the patch area, and one near the middle. Use crossings, not patch centres: through the hole, a crossing shows two edges that pin the position in both directions, while a patch centre shows flat colour.
   - Skip crossings whose four patches are too alike (for example light yellows next to white), because their edges won't show. Pick the nearest crossing with strong contrast instead. The target's CMYK values say which crossings are good.
   - All 5 must be in the i1's reach. Use `LAUPlatenGeometry::railsFor` and `reachable`.
3. **Visit each point.** Use the same order and approach direction the measurement will use, so backlash is the same in both passes. At each point, let the rails settle and take an 8-frame average, as the scan does.
4. **Measure the miss** (details under "Finding the crossing" below). The result is the miss in board mm, (dx, dy), between where the hole's centre landed and where the crossing is.
5. **Fit and check.**
   - Fit a 2D similarity (offset, rotation, scale: 4 numbers) mapping intended to actual positions, from the 5 misses.
   - Report each point's miss before the fit and its residual after.
   - If a residual is over 0.5 mm, say so in plain words, show the five through-hole views, and offer: measure anyway with the correction, rescan, or cancel.
   - If the misses are tiny (all under 0.3 mm), skip the correction and say so.
6. **Apply.** Correct every patch's rail target with the fit before building `patchStops`.
7. **Hand over to the i1.** The existing `startI1()` prompt, plus a reminder: "Put the i1 in its holder without moving the target sheet."

The dry run can also be skipped from the check dialog, to measure as before.

## Finding the crossing

Each through-hole view is a few hundred pixels around a fixed spot in the frame. The hole is fixed to the camera, so it is always in the same place.

1. **Flatten the view.** Undistort with the calibrated camera, then map onto the paper plane at 0.02 mm per pixel, the way `findDisc` already does (`lauplatenmosaic.cpp`, around 1040 to 1117). The paper sits 1 mm below the plate's rim (`APERTURE_FLOOR_MM`), so map the inside of the hole onto the paper plane and the rim onto the plate's top.
2. **Find the hole.** Fit a circle to the rim, where it changes from black plate to bright paper, using points along rays from the expected centre. Its centre is where the i1 measures, in this view. The rim is fixed to the camera, so its centre should barely change between points; a jump means the plate moved, and gets reported.
3. **Find the crossing.** Inside the circle (shrunk by 0.5 mm to stay off the rim), find the two edge lines:
   - Expected directions come from the target's layout turned by the merge's estimate. Search each line in a small angle range around that.
   - Fit each line to strong-gradient points (an edge detector plus robust line fitting, `cv::fitLine` with Huber). The crossing is where the two lines meet.
   - Check: each line must have enough points on both sides of the crossing, and the four quadrant colours must match the expected patches (light against dark in the right corners). This rejects a wrong crossing if the merge was off by a whole patch.
4. **Compute the miss.** Crossing position minus hole centre, in paper mm. This is local and independent of the camera model to about the local scale's accuracy, which is far better than 0.1 mm over 5 mm.

Accuracy goal: under 0.1 mm per point.

## Refining the aperture calibration as well

At each point, the hole's circle (step 2 above) is also a clean measurement of where the aperture is. Paper gives a high-contrast rim, unlike the platen, where only 3 of 12 frames worked. Average the 5 into a refined `A0` for this session, and report it against the stored one.

Whether to save it back to the platen calibration is a later decision. For now it only feeds this run's fit; the fit absorbs an offset anyway.

## Code layout

- **New `GantryController/lauholeview.h/.cpp` (`LAUHoleView`).** Pure image analysis, no Qt widgets: `measure(frame, camera, carriage position, expected crossing, expected directions)` returns hole centre, crossing, miss, quality, and a small debug picture. Testable from the command-line harness.
- **`laugantrymeasure.cpp`:** a dry-run state machine like the patch loop: list of check points, `moveToNextCheck()`, `onCheckMoveFinished()`, frame request, `onCheckFrame()`. It reuses the scan's settle timer and averaged-frame request.
- **`laugantrymainwindow.h`:** state for the dry run (check points, index, results, flag), plus the correction applied to patch stops.
- **Saved results, in the measurement folder (`<scan>/i1/`):**
  - `dry_run.json`: per point the intended and actual rail positions, hole centre, crossing, miss before and after, plus the fit.
  - `dry_run_<n>.png`: the five through-hole views with the fitted circle and lines drawn on them.
  - These let us track accuracy over time.

## Testing

1. **Offline, with the harness.** Grab five through-hole frames by hand (i1 off, carriage parked over crossings of a printed target, using Export frame). Run `LAUHoleView` on them and check the drawn circle and lines by eye.
2. **Repeatability.** Visit the same crossing 10 times from the same direction and from the opposite direction. The spread is the rails' repeatability; the difference between directions is backlash. These two numbers tell us whether a single fit is enough or whether each patch would need its own check.
3. **End to end.** Scan target, dry run, then measure with the i1. Repeat the dry run after measuring: the misses should match the first run's residuals. If they don't, the target or holder moved when the i1 went in.

## Open questions

- **Patch size.** The Nov 9 targets don't carry their patch layout in the TIFF, so check the next printed target's patch size in mm. Under about 7 mm, even 1 mm of leftover error is risky.
- **Holder sag with the i1 in.** The i1's weight could shift the holder on its hinge relative to the dry run. Step 3 of testing measures this; if it matters, put the i1 in for a second, shorter check.
- **More than 5 points?** If the repeatability test shows the rails wander, add points or check every patch through the surrounding view. That is the larger version and is not in this plan.
