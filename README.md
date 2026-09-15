# Cart-pole: ESP32 firmware + Mac host console

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
pulls them high for its 1/16 default: 80 steps/mm on a 20T GT2 belt, 640
steps/mm on a 5 mm ball-screw lead.

That default caps cart speed at 0.5 m/s, because the step generator needs two
timer ticks per pulse to meet the A4988's minimum high and low times, and the
timer runs at 80 kHz. If you want a faster cart later, a 40T pulley halves the
steps/mm and doubles the ceiling — cheaper than wiring three more signals.

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
voltage. A NEMA 17 at the ~1200 RPM this thing asks for has very little torque
left on 12V, which is why 24V is the recommendation.

**The rotor isn't free.** 87 g·cm² through a 6.37 mm pitch radius reflects to
roughly 0.22 kg of apparent cart mass. On a light cart that's comparable to the
cart itself, and it's why raising acceleration limits hits a wall sooner than
the torque numbers suggest.

**Swing-up stalls at a fixed amplitude** → check `v` in the dashboard before
touching gains. The energy pump does work at a rate proportional to the cart's
*acceleration*, so the instant velocity saturates at `vmax`, acceleration goes
to zero and pumping stops for the rest of that half-swing. A flat-topped `v`
trace means you're speed-limited and no gain will fix it.

**Swing-up never gets going at all** → raise `amax_s`, or lower `kpx`; the
centring term steals energy from the pump. **Flies past upright too fast to catch** →
lower `ke` so the pump eases off near the top, or widen `catch_r`.
