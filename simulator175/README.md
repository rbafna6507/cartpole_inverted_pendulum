# 175 mm Pendulum Lab

Offline browser simulator for the 20T cart, 175 mm pendulum and 300 mm rail. Open `index.html` directly or serve the repository using `python3 -m http.server 8765 --bind 127.0.0.1`, then open `/simulator175/`.

## Current defaults

The startup preset follows v9: vmax=0.75 m/s, amax_s=amax_b=15 m/s², jmax=50 m/s³, 20T GT2 pulley and 1/16 microsteps (80,000 steps/m). The capture window is 0.60 rad, with direction, rate, velocity headroom, rail and acceleration-ramp checks. The governor anticipates speed gained during jerk ramp-down. JavaScript equations are checked against the compiled firmware headers.

- **Current firmware snapshot:** source configuration at the latest bundle build.
- **Recorded 20T:** parameters from the third automatic trial in `run_20260916_122300`: 0.75/15/15/50, firmware v7. Experiments use the current engine; this is a parameter preset, not exact historical firmware replay.
- **Historical 60T candidate:** preserved old search settings and geometry. Its old validation files describe the earlier engine and are not current validation.

Choose a scenario/input style, edit limits, and run or compare. Charts show angle, cart position, command speed, acceleration, jerk and STEP rate. Initial angular rate, cart speed and acceleration can be set under plant assumptions to reproduce a measured handoff state. Start angle is measured from hanging, except in balance recovery, where it is measured from upright. Export traces and settings before refreshing; experiments are in browser memory.

## Evidence

The three confirmed free-decay captures provide 33 cycles, effective length 0.165775568 m (firmware 0.166 m), and equivalent viscous damping 0.402279 s⁻¹. Physical length is 175 mm. Original calibration logs are unchanged and their hashes are checked on rebuild.

`recorded_20t_trial.json` freezes the exact 44.729-second preferred trial, including SWINGUP and BALANCE samples. It reached 179.908° lift from down, entered BALANCE 16 times, and ended with a software speed-limit fault. Longest BALANCE interval was 0.245 s. Original session snapshot hashes are retained in the file. Its points are measured angle plus pulse-derived position and commanded velocity/acceleration; this is not an independent cart-motion measurement.

`analysis/data_informed_cases.json` contains 31 observed handoff states and duration-weighted clipping statistics. `node analysis/validate_recorded.js` compares seeded recovery with archived v7 and current equations. See [v8 assessment](../analysis/v8_assessment.md) for results and limitations. The ideal model still predicts some recoveries that did not occur on hardware; no reliable physical balance claim is made.

## Model boundaries

The nonlinear acceleration-driven pendulum uses RK4 with 1 ms integration:

```
theta'' = (g sin(theta) - a cos(theta))/L - damping*omega - coulomb*tanh(omega/0.05)
```

It includes encoder quantization/sample holding, the tracking differentiator, jerk/speed/rail limiting, mean pulse counting and optional assumed tracking fraction, lag and delay. Motor torque, resonance, backlash, individual step impulses and actual missed steps are not identified. A smoother hardware run is useful user evidence but does not measure those quantities.

Replay sends the logged acceleration waveform through the selected governor/input style: it is an open-loop what-if, not a reconstruction of the measured trajectory. Calibration overlays reuse calibration data, not held-out measurements. Experiment success requires at least five final continuous seconds in BALANCE with angle <0.1 rad and physical cart position <0.1 m.

The four input styles are acceleration with jerk limiting, acceleration with instantaneous transitions, sampled velocity targets and streamed position targets. The latter two are experimental adapters, not emulations of a named driver library. STEP ceiling and geometry changes in this offline tool do not change firmware.

## Build and check

```
python3 simulator175/build_data.py
node simulator175/test.js
node analysis/validate_recorded.js
```

Optional browser test: `node simulator175/browser_test.js` with a local server running, or set `SIM_URL` to the `file://` URL of `index.html`. Playwright and Chrome are required. `NODE_PATH` and `CHROME_PATH` can locate them.

The older settling search/assessment files are retained as historical 60T experiments. Do not interpret their old pass counts as evidence for current v8 geometry and control logic.

## v9 rail recovery

Normal automatic travel is ±130 mm within the ±150 mm physical rail. At that
threshold, or earlier when the governor predicts inadequate stopping distance
to ±135 mm, automatic control pauses for braking and recentering. Recovery
returns toward center with a 0.15 m/s target and resumes swing-up only after
100 ms within ±3 mm with speed ≤0.01 m/s and acceleration ≤0.1 m/s². It does
not reset the position origin. Recovery timeout is 8 s; the hard fault remains
±140 mm. The simulator records recovery phases and durations. Historical
recovery comparisons/assessments describe their named engine revisions.
