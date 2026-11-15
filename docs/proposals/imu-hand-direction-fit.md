# Proposal: a robust hand-direction check for the IMU mounting wizard

Not built. Written 2026-11-14 from one rejected capture on the rig, a reading of the fit, and a synthetic curl run through a Python port of it. This document proposes replacing the centripetal correlation that decides which end of the forearm's long axis is the hand with a joint lever-arm fit judged by a confidence score, and having the wizard keep recording until that score is met.

## Goal

The mounting wizard rejected the right Joy-Con on a normal curl: 6 half-strokes of 60 to 70 degrees at about 1 s each, captured about 7 s into the curl stage. Every other gate passed with room (twist dominance 0.99, curl dominance 0.98, hinge spread 4.2 degrees, inter-axis 80.7 degrees). The one failure was the hand-direction fit correlation, 0.27 against a bar of 0.5, shown as `mountingWizard.reasonHandDirectionUnclear`. The left side was accepted at 0.51. Curling about twice as fast passed.

The check decides one bit. It should not be marginal on a normal curl, and it should not depend on the user curling fast.

## The current fit

`imuFitCentripetalRadius` (`src/Imu/ImuMountingMath.cpp`) carries a gravity direction through the curl by integrating the gyro, re-anchoring it toward the accelerometer on samples with a rate under 0.5 rad/s and a magnitude within 0.5 m/s^2 of gravity. It regresses the accelerometer residual along the long axis against the squared hinge rate. The sign of the slope says which end is the hand, the slope is the elbow-to-sensor radius, and `imuSolveMountingFromMotions` accepts the capture when the Pearson correlation reaches `k_minLengthFitCorrelation` (0.5).

The wizard captures by itself as soon as every participating side reaches the live gates, which for a curl is the 5 rad path minimum (`k_minTwistPathRadians`).

## Why it is marginal

Derived for a 65 degree stroke of 1 s at a 0.2 m radius:

- The hinge rate peaks at 1.78 rad/s. The centripetal term peaks at 0.63 m/s^2 and averages 0.32 m/s^2. A gravity tilt error of 1.9 degrees is the same size.

- The tangential term (angular acceleration times radius) peaks at 1.1 m/s^2, 1.8 times the centripetal peak. It flips with the lever arm, so it carries the same bit. The fit discards it by projecting onto the long axis.

- In displacement the sensor travels 21 cm across the forearm (the chord of the arc) and 3 cm along it (the sagitta). The fit reads only the 3 cm. An elbow that bobs 1.6 cm at twice the stroke rate cancels it, where cancelling the chord takes 11 cm of elbow travel against the stroke.

- Doubling the stroke speed quadruples both acceleration terms while gravity tilt error and sensor noise stay fixed. That is why curling faster passed.

Measured on a synthetic curl, 60 trials per row. The model: a sensor 0.2 m from the elbow and 3 cm off the bone axis, sinusoidal 65 degree strokes, 6 half-strokes, 200 Hz, 1 percent gyro scale error, 0.003 rad/s bias residual, white noise of 0.004 rad/s and 0.03 m/s^2, 0.08 m/s^2 accelerometer bias, 0.08 m/s^2 of 2 to 8 Hz tremor, elbow sway of 10 mm at the stroke rate and 4 mm at twice it in random directions, a 0.12 rad/s twist wobble, and 4 degrees of error on both measured axes.

| Half-stroke | Median r | 10th percentile r | Pass at 0.5 | Sign correct | Median radius (true 0.200 m) |
|---|---|---|---|---|---|
| 2.0 s | 0.58 | 0.44 | 75% | 100% | 0.242 |
| 1.5 s | 0.67 | 0.55 | 93% | 100% | 0.241 |
| 1.0 s | 0.71 | 0.61 | 100% | 100% | 0.233 |
| 0.75 s | 0.79 | 0.69 | 100% | 100% | 0.208 |
| 0.5 s | 0.85 | 0.74 | 100% | 100% | 0.200 |

What the runs show:

- The correlation is an effect size, not a confidence. It does not rise with stroke count: 0.71 at 6 half-strokes of 1 s, 0.64 at 12, 0.66 at 24. Recording longer cannot rescue a capture judged by it. The sign was correct in every synthetic trial the bar rejected at 65 degree strokes.

- The re-anchor hurts slow curls. A rate under 0.5 rad/s is the turnaround, where the tangential acceleration is largest and tilts the accelerometer away from gravity while barely changing its magnitude. With no noise at all, at 2 s strokes, the correlation is 0.77 with the re-anchor and 0.91 without, and the radius reads 0.237 m for a true 0.200. At 0.75 s and faster the turnaround is too brief to matter. Removing it alone is not a fix: under the full noise model at 2 s strokes the pass rate fell from 75 to 50 percent, because the gyro drift it was correcting then dominates.

- Small strokes are the weak case for the sign itself. At 35 degree strokes the current fit picks the wrong end in 13 to 17 percent of trials.

- The model does not reproduce the rig. It gives 0.6 to 0.8 at 1 s strokes where the rig gave 0.51 and 0.27, so the real curl holds an error the model lacks. Candidates are strap and soft-tissue motion, more elbow travel than modeled, and the right controller's Bluetooth gaps (45 sample gaps over 20 ms in the dump's 30 s of raw history against 4 on the left). No dump on disk holds a recorded curl, so piece 1 below comes first.

## Proposed design

### Piece 1: the capture is replayable

The diagnostic dump's `imuLastCapture.<side>` gains the twist and curl recordings the capture was solved from (rate, acceleration and dt per sample). A new self test, `--test-imumounting <dump.json>`, reads them, runs `imuSolveMountingFromMotions`, and logs every gate value with the old correlation and the new confidence side by side. It follows `src/Tests/TestImuPair.cpp`, which already reads a dump's raw IMU history.

Two dumps taken on the rig right after a capture, one slow curl of the failing kind and one fast, are the inputs that set the bar in piece 2.

### Piece 2: a joint lever-arm fit

One linear least squares over the curl recording replaces the regression. Per sample, in the sensor frame:

`accel = R(t)^T * g0 + alpha x r + w x (w x r) + bias`

- `R(t)` is the rotation since the first sample, integrated from the gyro
- `alpha` is the differentiated rate, with rate and acceleration box-smoothed over 9 samples first
- `r = a * longAxis + b * (longAxis x hingeAxis)`, the lever arm restricted to the plane across the hinge, since a hinge rotation says nothing about the component along it
- the unknowns are `g0` (3), `a`, `b` and `bias` (3)

The sign of `a` is the hand direction and its size is the elbow-to-sensor radius. The gravity start vector is solved with the lever arm, so the seed search and the re-anchor go away. The fit stays a pure function of the recording: a fixed-size solve from accumulated normal equations, no iteration to a tolerance, no clock.

Acceptance is `a` over its standard error, taken from sums of the score over 100-sample blocks because the residuals are autocorrelated and a per-sample error would overstate the confidence. `MountingCaptureResult` gains that score and loses `lengthFitCorrelation`, with its dump field and the wizard's literal 0.5.

On the same synthetic curls:

| Case | Sign correct | Median score | 10th percentile | Median radius |
|---|---|---|---|---|
| 2.0 s half-strokes | 100% | 10.3 | 5.7 | 0.198 |
| 1.0 s | 100% | 22.2 | 13.1 | 0.199 |
| 0.5 s | 100% | 28.9 | 16.1 | 0.199 |
| 35 degree strokes, 1.0 s | 100% | 8.8 | 2.9 | 0.205 |
| No rotation, elbow shaken 20 mm | no decision | 0.6 | 0.2 | near 0 |

The no-rotation control never scored above 1.2. A starting bar of 4 sits between it and the 2 s curls, to be set from the piece 1 dumps.

### Piece 3: the wizard records until confident

In the curl stage, when a capture comes back with a side failing only the hand-direction gate, the stage stays open, recording continues, that side shows a keep-curling line, and the capture is retried once a second. Finish Now ends the stage with the last capture's verdict, and `reasonHandDirectionUnclear` stays for that path. The retry needs `ImuService::captureMounting` to leave the recordings intact and to have no side effect a repeat would compound, which is to check when building it.

## Options set aside

- Keeping the regression and accepting on its slope over its standard error. It stops the bar rejecting right answers, and it still reads only the sagitta, so small strokes stay 13 to 17 percent wrong.

- Removing the re-anchor alone. Measured above: better on clean slow curls, worse under gyro drift.

- A live hint to curl faster. It works for the current fit by leaning on the user. With the lever-arm fit the speed stops mattering much, and the keep-curling line covers the rest.

- Taking the long-axis sign from the vision palm average or the previous mounting, as the palmar side does. The margin is 90 degrees and the effort is low. It needs the camera or a prior mounting, and it puts vision back into a decision the solve keeps out of it. Held in reserve for the case where real captures show the lever-arm fit still marginal.

## Tests

- `--test-imufilter` section (n) gains cases built from fixed sinusoids, with no random source:
	- a slow curl (2 s half-strokes, sensor 3 cm off the axis, elbow sway, gyro scale error) is accepted with the radius within 2 cm
	- the same curl with the twist axis handed in reversed solves to the same mounting
	- 35 degree strokes get the sign right
	- an elbow shaken with no rotation is rejected
	- the existing fast-curl assertions hold unchanged
- `--test-imumounting` on the two rig dumps: the slow curl the correlation rejects is accepted with its score well above the bar, and both dumps agree on the sign and on the radius within a few centimeters.
- `--test-dump` covers the new dump fields.
- Live: the wizard with a slow curl on both sides accepts, and the wrist bend readout in Review stays near zero through a twist.

Replay checksums are unaffected. The mounting solve is not a replayed stage.

## Docs to update when built

- `docs/reference/imu.md` and `calibration.md`: the fit and the gate list
- `docs/reference/debugging.md`: the dump fields
- `docs/reference/testing.md` and `commands.md`: the new self test
- `LEARNINGS.md`: the correlation verdict and the re-anchor measurement
- `docs/plan.md`: remove the entry

## Open

- The cause of the gap between the synthetic 0.7 and the rig's 0.27 is unidentified until a recorded curl is on disk.
- The lever-arm fit reads the arc the sensor actually swept, so an elbow that travels against the stroke shortens the radius it reports (0.137 m for a true 0.200 when the elbow cancels 30 percent of the sensor's travel). The current fit has the same bias. The radius overwrites `body.forearmLengthMeters`, so whether the 0.10 to 0.40 m window is enough of a guard is to judge on the rig dumps.
