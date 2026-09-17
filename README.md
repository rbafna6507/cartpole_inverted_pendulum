# Cart-pole: ESP32 firmware + Mac host console

## Manual cart frequency test

Use the [frequency-test GUI](frequency_test/README.md) to choose peak-to-peak
travel and frequency, preview position/velocity/acceleration/jerk, and run the
cart manually. It uses a separate ESP32 test sketch. Swing-up auto-tuning is
paused during these tests.

## Run the swing-up sketch (60T pulley, 300 mm travel)

**v21 requires reflashing for the 60T pulley and cart STEP25/DIR26 mapping.** `params` should report
`pulley_teeth 60`, `cart_mm_per_rev 120`, and `cart_steps_per_m 26666.67`.

`swingup/swingup.ino` runs on the ESP32; `cartpole.py` is its Mac console.
The swing-up cart pins are **STEP = GPIO25, DIR = GPIO26**.
Z1 uses **STEP = GPIO18, DIR = GPIO19**; Z2 uses **STEP = GPIO16 (RX2), DIR = GPIO17 (TX2)**.
The older sketches below retain their own wiring.
Place the cart at the **physical center** before power-up/reset, and let the
pendulum hang still while the encoder records its down reference. Startup assigns `x = 0` there;
there is **no homing movement**. The sketch boots idle with **all drivers disabled** and these cart limits:

| Setting | Default |
|---|---:|
| Total physical travel | 300 mm (±150 mm from startup center) |
| Maximum speed (`vmax`) | 0.8 m/s (400 RPM) |
| Swing-up acceleration limit (`amax_s`) | 12 m/s² (6,000 RPM/s) |
| Balance/braking acceleration limit (`amax_b`) | 12 m/s² (6,000 RPM/s) |
| Manual acceleration limit (`amax_m`) | 0.5 m/s² |
| Automatic jerk limit (`jmax`) | 60 m/s³ (30,000 RPM/s²) |
| Manual jerk limit (`jmax_m`) | 10 m/s³ |
| Keyboard jog speed (`vman`) | 0.05 m/s |

The 60T pulley and 2 mm belt pitch give 120 mm/revolution and 26.666667 steps/mm
at 1/16 microstepping. Automatic jerk is 60 m/s³: acceleration changes by
at most 0.06 m/s² per 1 ms tick, taking 200 ms from zero to 12 m/s².
A 7 µs pulse timer supports up to 71,429 steps/s (2.679 m/s pulse ceiling; configured limit remains 0.8 m/s); 0.8 m/s
requires 21,333 steps/s. DDS scaling uses the same timer period.
Manual jogging keeps its gentler acceleration and jerk limits.
`stop`, `off`, and faults stop pulses and **disable all three drivers**.
Startup drives the shared active-low ENABLE pin HIGH before serial delays,
I2C initialization or encoder zeroing. Support the height assembly while
motors are disabled. Firmware cannot control GPIO during the earlier reset/bootloader interval.
There are no `fast` or `slow` profiles; motion commands use the current limits.

### Measured upright reference (v14)

The recorded **180° / upright = 3416 raw counts** is now the fixed `theta = 0`
reference for balance, capture gating, swing-up and telemetry. Startup and `zero`
record the down reference for diagnostics; they do **not** replace this upright
reference. `params` reports `encoder_upright_raw 3416`. Flash the current v15 firmware to apply it.
The polarity remains `ENC_INVERT=0`; the linear scale remains 4096 counts/turn.
The 90° and 270° readings are shown in the
[calibration plot](encoder_calibration/encoder_20260916_135926_031977_linearity.png),
with [assessment](analysis/encoder_reference_v14.md). No quadrant correction is
applied: these single samples include serial-integrity concerns.

### Pendulum spin failsafe

**v18 trips immediately above |θ̇| = 25 rad/s and releases below 10 rad/s. Reflash to apply it.**
`params` reports `spin_trip_rad_s 25` and `spin_resume_rad_s 10`.

The filtered angular-speed magnitude **above 25 rad/s** latches
`SPIN_BRAKE → SPIN_CENTER → SPIN_WAIT` in either rotation direction.
It monitors automatic swing-up, automatic balance, and automatic rail recovery.
The upright-only `bal` trial retains its separate fall-return-stop behavior.
There is no full-turn requirement, pendulum-angle gate, or quiet-time delay.

Pendulum control pauses while the cart brakes, then returns to its original
center at up to **0.10 m/s**, with a **0.5 m/s²** return acceleration target and
the existing jerk limit. Braking still uses the normal motion/rail protections.
SWINGUP resumes only once the cart is within **±3 mm** of center,
|cart velocity| ≤ 0.005 m/s, |cart acceleration| ≤ 0.1 m/s², and a valid
encoder estimate has **|θ̇| strictly below 10 rad/s**. Exactly 10 rad/s keeps
waiting. The separate thresholds prevent repeated switching near the trip rate.
Waiting can continue indefinitely; failure to center within 10 seconds faults
and disables the motors. Ordinary edge recovery still only returns inside the
270 mm operating region; this rate failsafe specifically returns to center.

Drivers remain enabled to hold center during recovery. `stop`/`off` cancel it
and disable all drivers immediately. Repeated `auto`, `bal`, and jog commands
cannot bypass an active recovery. `stat` shows its phase, current rate, and
release threshold.
There is no homing or direct cart-position measurement; the center reference
still relies on commanded steps and can be wrong after physical slipping.

### Rail recovery

The physical ends are at ±150 mm from the startup center. Normal operation may
use the central **270 mm (±135 mm)**. Entering the last 15 mm at either end
starts `RAIL_BRAKE`: pendulum control pauses while outward motion decelerates.
Predictive braking can start earlier when velocity and acceleration require it;
it aims to stop before ±135 mm. At 0.8 m/s with zero acceleration and jerk
60 m/s³, stopping requires about **87.1 mm**, so braking cannot wait for the
last 15 mm at full speed.

After braking, `RAIL_RETURN` requests an inward speed up to 0.15 m/s and
acceleration up to 1.5 m/s² (or lower configured limits), still jerk limited.
Automatic operation resumes SWINGUP once back inside **±130 mm**, moving inward
or nearly stopped with inward acceleration. This 5 mm inset prevents repeated
boundary triggering; there is no center-seeking or dwell. If predictive braking
reverses the cart farther inside, control resumes there immediately.
Manual recovery resumes MANUAL with a zero velocity target. Held jog commands
are ignored during recovery; `stop` always cancels recovery and disables all
drivers. Recovery never resets the position reference. It times out to a
disabled fault after 8 s.

The independent fault boundary remains ±140 mm. Position is inferred from
commanded steps, not sensed at the ends; missed steps or manual movement can
invalidate it. These are software margins, not physical endstop detection.

Close other serial programs before uploading. Reset/upload releases the shared
driver enable, so support the height assembly. From Terminal:

```bash
cd /Users/sajivshah/Documents/GitHub/cartpole_inverted_pendulum
arduino-cli compile --fqbn esp32:esp32:esp32 swingup
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 swingup
python3 cartpole.py --port /dev/cu.usbserial-0001
```

If Python dependencies are missing, install `pyserial` and `matplotlib` in your
Python environment. The port may change after reconnecting; check `/dev/cu.*`.
The [Arduino CLI guide](https://docs.arduino.cc/arduino-cli/getting-started)
describes the compile/upload commands.

Wait for `# ready`, then at the `>>>` prompt:

```text
params
mag
auto
```

`auto` enables the drivers, starts swing-up and switches to balance when eligible.
`stat` reports the ENABLE output: LOW means enabled, HIGH means disabled.
v9 fixes v8 GPIO initialization that could report energized while ENABLE stayed HIGH.
While plotting, type **`stop` + Enter in the terminal**, click **STOP / disable**,
or press **Space/Escape in the plot**. In the terminal, Escape or Space at an
empty prompt also sends stop immediately. `q` + Enter exits the plot and stops.
Other terminal commands, including `set`, work while plotting. A/D/W/S jogging
is available in the `control` plot window; terminal letters form commands.
Restart `cartpole.py` to load this console change. Reflash v15 for the serial fixes, calibrated upright, rail
behavior and limits. No profile command is needed.
After moving the disabled cart by hand, place it at physical center and issue
`home` before starting again; commanded steps cannot observe manual movement.
Height-axis limits remain 2 mm/s and 20 mm/s².

To change limits, first send `stop`, then for example:

```text
set vmax 0.8
set amax_s 12
set amax_b 12
set jmax 60
set rail 0.15
```

`rail` is **half** the total physical travel, in metres. These serial settings
last until reset. `home` only
relabels the stopped cart's current position as zero; use it only at the physical
center. It does not search for a limit switch.

Both JSON files record 300 mm total travel; the calibration file also records
175 mm physical pendulum length and 14.2 g pendulum mass. JSON is reference
metadata, not automatically loaded by this sketch. The current model uses
**0.166 m effective length**, from 33 same-side cycles at 5–20° across the three
175 mm captures. [Fit provenance and controller validation](analysis/swingup_validation.md)
explain the measurement and its limits.

The revised `swingup-175mm-20t-v15` firmware uses smooth phase feedback for energy
pumping, cart centering, a gentle startup bias, and a balance handoff that also
checks cart position, speed headroom, approach direction, and acceleration ramp time.
The capture window opens at 0.60 rad (34.4°), with angular rate below 3 rad/s;
balance acceleration applies on the handoff tick. The dropout angle is 0.80 rad.
The speed limiter anticipates velocity gained during acceleration ramp-down. It stops startup if it
commands over 30 mm cumulative travel for over 2 seconds without a 0.03 rad
pendulum response. This detects gross non-response; it does **not** detect every
missed step. Buzzing with little physical movement still needs a motor/driver
check and a fresh physical center reference. Verify manual motion first. Motion-limit regressions pass for this revision, but
recorded-state simulations remain imperfect: some cases settle in the ideal
model despite failing physically. See [current change assessment](analysis/rail_recovery_v10.md) and the earlier [v8 assessment](analysis/v8_assessment.md).

The console now saves `logs/run_<timestamp>_events.jsonl` alongside each CSV.
It records outgoing commands, firmware identity, parameter replies, diagnostic
messages and faults; `auto` and `bal` request a parameter snapshot before starting.
Cart position is inferred from pulses, and velocity/acceleration are commands.

### Ongoing log review

The five-minute Codex review is currently **paused for manual frequency tests**.
When enabled, it reviews new completed runs and updates
and tests repository candidates; applying settings and starting trials remain
manual. The controller does not learn online. See [tuning workflow and current
candidate](analysis/continuous_tuning.md). Keep Codex open and this Mac awake.

Host-side motion regression checks (no hardware movement):

```bash
clang++ -std=c++11 -Wall -Wextra -Werror tests/cart_motion_test.cpp -o /tmp/cart_motion_test
/tmp/cart_motion_test
clang++ -std=c++11 -O2 -Wall -Wextra -Werror tests/swing_controller_test.cpp -o /tmp/swing_controller_test
/tmp/swing_controller_test
python3 tests/test_console_events.py
```

## Free-swing recording: automatic stop

Recorded datasets are indexed in [pendulum_characterization/README.md](pendulum_characterization/README.md), grouped by physical configuration and date.

The `characterize` sketch ends with `END quiet` when the encoder stays within
**1° peak-to-peak for 3 seconds**, after at least 7 seconds of recording. It uses
unwrapped angle counts, so one-count encoder noise does not restart the quiet
timer. Motion above that band restarts the timer; missing samples or sensor
errors also restart it. The 45-second maximum capture remains a fallback.

Recompile and upload `characterize` to apply this change, then rerun the host:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 characterize
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 characterize
python3 characterize_pendulum.py --port /dev/cu.usbserial-0001
```

Close other serial consoles, support the height assembly, and keep the cart
fixed with motors off for the free-swing test. Start hanging, displace the
pendulum about 10–20°, and release. The Python recorder also sends `STOP` when
its own timeout expires or you press Ctrl-C.

Replay the two recordings that previously reached the 45-second timeout:

```bash
clang++ -std=c++11 -Wall -Wextra -Werror tests/quiet_motion_test.cpp -o /tmp/quiet_motion_test
/tmp/quiet_motion_test pendulum_characterization/175mm_14p2g/2026-09-15/pendulum_20260915_184743.csv pendulum_characterization/175mm_14p2g/2026-09-15/pendulum_20260915_184905.csv
```

With the new detector, those captures qualify as quiet at approximately
19.1 seconds and 25.1 seconds respectively. This is an offline replay, not a
new hardware measurement.

## Serial integrity and encoder recording (v15)

**Flash v15 and restart the updated console together.** They use **115200 baud**
and **25 Hz telemetry** (maximum 100 Hz). Replies carry a CRC-16 checksum,
including telemetry, raw encoder readings, parameters and STOP acknowledgments.
Dropped digits and minus signs are rejected even when the remaining text looks
like a valid number. Host commands remain newline-delimited text.

The console sends keepalives every 250 ms without depending on parameter replies.
Firmware disables active motion after 1.5 seconds without host commands. The
console sends STOP after a one-second telemetry gap, retries STOP/rate commands
once per second as needed, and requires a valid STOP acknowledgment and three
consecutive valid telemetry samples before permitting a new motion command.
It never replays a previous run. Startup follows the same STOP acknowledgment
check. A blocked `auto`/`bal` stays at the prompt instead of opening a dashboard.

Type **`link`** in the terminal (also while plotting) for the exact blocking
reason, valid sample rate, and checked/rejected frame counts. Malformed frames
and blocked requests are logged. Automatic recovery retries the same serial
path; a different port name needs a console restart with the new `--port`.
Check physical cart center after a USB reconnect/reset before restarting motion.

The 14:06–14:09 sessions show dropped characters, stale telemetry and one
`Device not configured` disconnect. Lower baud/traffic and chunked host reads
reduce serial load; checksums detect corruption rather than reconstruct missing
bytes. Physical USB reliability still requires a run on the hardware. See
[serial investigation](analysis/serial_integrity_v15.md).

For explicit diagnostics with old firmware only, the console accepts
`--legacy-serial --baud 230400` for v13/v14 (`921600` for older swing-up versions).
Unchecked legacy data cannot reliably detect dropped digits. The calibration
recorder now requires v15 checksummed replies, preserving the original captured
files and the current 3416-count balance target.

After flashing v15, close the console and run:

```bash
python3 calibrate_encoder.py --port /dev/cu.usbserial-0001
```

Hold the pendulum manually at each prompt and press Enter:

1. **0°** — hanging down.
2. **90°** — horizontal toward positive cart travel.
3. **180°** — upright.
4. **270°** — horizontal toward negative cart travel.

All motor drivers remain disabled. Each Enter records **one raw encoder reading**
and immediately saves it under `encoder_calibration/`. The JSON contains the
four reference angles and their raw counts, plus the original readings and
sensor diagnostic flags. Use `--samples 20` only if you want multiple readings
and a circular mean at each position.

The recorder does not reject motion, magnet flags, offsets, or quadrant spacing.
Unreadable serial replies are retried five times, then the same position prompt
remains available to retry. Earlier readings remain saved. These values form the
recorded calibration table; firmware correction is not automatically applied by the recorder. The current
v15 firmware retains the saved 3416-count upright reference described above.
This serial update requires v15 firmware; older replies are rejected by default.

## Original cartpole_esp32 sketch

The remaining instructions describe the separate `cartpole_esp32` sketch,
including its older wiring and slow boot defaults. Its pulley conversion is also
60T; its linear limits are preserved. For the current 60T / 300 mm setup,
use the `swingup` commands above.

```
cartpole_esp32/cartpole_esp32.ino   flash this
cartpole.py                         run this
```

---

## 1. Wiring

Every STEP pin is deliberately in GPIO 0–31, because the firmware pulses all
three axes with a single register write. Don't relocate them above GPIO 31.

| ESP32 | Connects to | Notes |
|---|---|---|
| GPIO 25 | Cart driver — STEP | |
| GPIO 26 | Cart driver — DIR | |
| GPIO 16 | Z-left driver — STEP | |
| GPIO 17 | Z-left driver — DIR | |
| GPIO 18 | Z-right driver — STEP | |
| GPIO 19 | Z-right driver — DIR | |
| GPIO 27 | **ENABLE — all three drivers** | active LOW, wired in parallel |
| GPIO 21 | AS5600 SDA | |
| GPIO 22 | AS5600 SCL | |
| 3V3 | AS5600 VCC | module is a 3.3V part |
| GND | AS5600 GND **and its DIR pin** | DIR must not float |

Leave MS1/MS2/MS3 unconnected on **all three** drivers. The Big Easy Driver
pulls them high for its 1/16 default: 26.667 steps/mm on a 60T GT2 belt, 640
steps/mm on a 5 mm ball-screw lead.

The legacy sketch uses a 120 kHz configured timer and a 60T pulley. Its
configured fast profile remains 0.72 m/s and slow profile remains 0.05 m/s.
Use the swing-up sketch above for the current controller and safety settings.

All driver grounds tie to ESP32 ground. Motor power (24V) goes to **M+ only** —
the BED's VCC pin is a regulator *output*, not an input.

### Three things to do to the hardware first

**Solder the 3/5V jumper closed on all three Big Easy Drivers.** The A4988
needs V_IH ≥ 0.7×VDD, which is 3.5V when the board's logic rail is at its
default 5V. Your ESP32 drives 3.3V. It will half-work and drop steps
unpredictably, which is indistinguishable from a tuning problem.

**Set the current.** The BED uses 0.11 Ω sense resistors, so
`I = Vref / (8 × 0.11) = Vref / 0.88`. The ball-screw motors are 1.5A →
Vref = 1.32 V, measured at TP1 with the motor unplugged.

**Mind the air gap on the encoder magnet.** Aim for 1–2 mm, magnet rotation
axis exactly on the pivot axis. A 0.5 mm offset becomes a once-per-revolution
angle error the balancer will chase forever. The `mag` command reports AGC;
mid-range (~64) is good, railed at either end means bad gap or wrong magnet.

---

## 2. Flashing from a Mac, no Arduino IDE

```bash
brew install arduino-cli

arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

Find the board. The sketch folder name has to match the `.ino` name, which it
already does:

```bash
ls /dev/cu.*          # look for cu.usbserial-* / cu.wchusbserial-* / cu.SLAB_USBtoUART
```

Nothing there? You need the USB-UART driver for your dev board's bridge chip —
Silicon Labs CP210x or WCH CH34x, depending on which one it has. Check the chip
next to the USB connector.

Compile and upload:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 cartpole_esp32
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 cartpole_esp32
```

If upload fails to sync, hold **BOOT** on the dev board, tap **EN**, release
BOOT, and rerun. Some boards need this every time; most auto-reset fine.

Watch boot output directly if you want:

```bash
arduino-cli monitor -p /dev/cu.usbserial-0001 --config baudrate=921600
```

---

## 3. Host console

```bash
pip3 install pyserial matplotlib
python3 cartpole.py
```

It finds the port itself. At the prompt:

| Command | What it does |
|---|---|
| `data` | live dashboard — angle, angular rate, cart position + velocity, commanded acceleration, pendulum energy, plus AGC / loop time / stream rate |
| `control` | **A** / **D** drive the cart left and right, **W** / **S** raise and lower the rail, **SPACE** stops, **Q** returns to the prompt |
| `auto` | swing-up, hands off to the balancer, dashboard open the whole time |
| `bal` | balance only — hold the pole upright and send this |
| `stop` | emergency stop |
| `slow` | crawl speeds — **the boot default** |
| `fast` | full speeds, needed for `bal` and `auto` |

Spacebar is a stop from inside any window, and it works whether the plot or the
terminal has focus. Ctrl-C at the prompt stops the board on the way out.

Every sample is written to `logs/run_<timestamp>.csv` automatically. `rate 250`
gives you a denser capture if you want to analyze a specific fall.

### How the keyboard driving is safe

Holding **A** doesn't latch a velocity. Each key repeat sends a fresh `v`
command, and the firmware zeroes the cart if it hasn't heard one in 250 ms. If
the host crashes, the USB cable pops out, or you just let go, the cart stops on
its own rather than driving into the end of the rail.

### Emergency stop keeps the motors energized

`stop` zeroes every velocity but leaves the drivers on. That's deliberate: the
SFU1605 screws are efficient enough to back-drive, so cutting ENABLE with the
rail raised drops it. Since all three ENABLE lines share GPIO 27, de-energizing
is a separate command — `off` — and you should support the rail before using it.

---

### Speed profiles

The board boots into `slow`: 0.05 m/s top speed, 0.02 m/s jog, 1 m/s² accel.
Everything creeps, and a direction error nudges the end of the rail instead of
hitting it. Do all of your wiring and calibration checks here.

Balancing will not work on the slow profile, and that isn't a tuning problem —
the cart has to accelerate out from under a falling pole, and 1 m/s² isn't
close. Send `fast` before `bal` or `auto`, and stand clear when you do.

Both commands e-stop first, so `slow` doubles as a panic button that also
prevents the next command from being quick.

## 4. First run, in order

1. **Current down, motors nowhere near the rail ends.** `python3 cartpole.py`,
   then `mag` — confirm the magnet reads healthy before anything moves.
2. `control`, tap **D**. Note which way the cart physically goes; that direction
   is +x. Backwards? Set `CART_INVERT 1` and reflash.
3. `home`, then drive the cart a known distance and compare the reported `x`
   against a ruler. Disagreement means your pulley or microstep constants are
   wrong — fix the constants, don't invent a scale factor.
4. **Measure L_eff.** Let the pole hang, give it a small push, time 10 swings.
   `L_eff = g·(T/2π)²`. Then `set leff <value>`. Every balance gain is derived
   from this one number, so measuring beats guessing.
5. Tilt the pole toward +x and check in `data` that theta goes **positive**.
   If not, `ENC_INVERT 1` and reflash.
6. Send `fast`. Everything up to here was on the crawl profile; balancing
   needs real acceleration. Then hold the pole upright and send `bal`. It should fight you. Tune live, no
   reflashing: `set pw 9` for a stiffer pendulum loop, `set pc1 -2` if the cart
   wanders into the rail, `set bw 12` if it buzzes.
7. `auto`.

## 5. When it misbehaves

**Lost steps look exactly like bad tuning.** After any crash, check whether the
reported `x = 0` is still the physical centre of the rail. If it drifted, you're
losing steps: lower `amax_s`/`vmax`, raise Vref, or raise the motor supply
voltage as appropriate for the driver and motor. The current 60T swing-up
default of 0.8 m/s corresponds to 400 RPM; the motor supply is 24V.

**Rotor inertia also matters.** For example, 87 g·cm² through the 60T pulley's
19.10 mm pitch radius reflects to about 0.024 kg of apparent cart mass. This
is an illustration; the actual motor and pulley inertia have not been measured.

**Swing-up stalls at a fixed amplitude** → check `v` in the dashboard before
touching gains. The energy pump does work at a rate proportional to the cart's
*acceleration*, so the instant velocity saturates at `vmax`, acceleration goes
to zero and pumping stops for the rest of that half-swing. A flat-topped `v`
trace means you're speed-limited and no gain will fix it.

**Swing-up never gets going at all** → raise `amax_s`, or lower `kpx`; the
centring term steals energy from the pump. **Flies past upright too fast to catch** →
lower `ke` so the pump eases off near the top, or widen `catch_r`.


## Interactive 175 mm simulator

Open [Pendulum Lab](simulator175/index.html) in a browser to test acceleration, jerk, speed ceilings, and four stepper input styles against the measured 175 mm pendulum model. Includes calibration overlays, recorded command-waveform replay, comparisons, parameter sweeps, and CSV/JSON exports.

See [simulator documentation](simulator175/README.md) for local serving, model assumptions, source provenance, and validation. Run `node simulator175/test.js` to check the model and firmware-equation parity.

## Manual upright tuning (v17)

Use **`bal` / START upright** after manually raising the pendulum. The new upright-only session accepts a stopped start near upright, aborts beyond **±50° from calibrated upright**, brakes, returns to center and disables. It never switches to swing-up or automatically restarts. Rail protection also ends the trial. Existing motion limits remain **0.8 / 12 / 12 / 60** (speed / swing acceleration / balance acceleration / jerk).

The controller is four-state feedback, not PID. `bal_pw=8` is the simulation-screened first candidate; `set bal_pw 7` restores the old angular response for comparison while stopped. See [upright tuning and hardware procedure](analysis/upright_tuning.md). Compile/upload v17 and restart the Python console before testing; no physical trial has been performed.

## v19 direction trial

After seven v18 upright trials aborted on predictive rail protection and the user observed motion away from the falling side, **v19 sets `CART_INVERT=0` (previously 1)**. `ENC_INVERT=0`, upright raw 3416, `bal_pw=8`, motion limits .8/12/12/60 and ±50° upright abort/centering are retained. `params` now reports `cart_invert` so logs identify the physical direction mapping. The driver writes DIR on the first command as well as reversals; otherwise a first positive command could retain the boot LOW pin level after this polarity change.

Place the cart at physical center before reset/upload; after upload confirm firmware v19 and `cart_invert=0`, then use `data` and **START upright** / `bal` for a near-vertical release. Expected corrective cart motion is toward the falling side. The separate possible 2.9° reference discrepancy is not applied in this trial. [Physical review](analysis/upright_review_20260916/review.md).
