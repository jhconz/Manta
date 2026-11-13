# Belt-slip test rig: one motor on a Raspberry Pi

One Pololu #5724 gearmotor (34:1, 48 CPR motor encoder) on a G2 24v13 driver, driven
from a Raspberry Pi 4 or 5 and powered by a bench supply. `slip_rig.py` drives the
motor against a **locked output** at increasing PWM duty and finds the duty where
the belt slips.

## How slip is detected

The encoder sits on the motor shaft, before the belt, so the drive pulley must be
held still: clamp the driven pulley, or put the actuator against a hard stop.

- **Belt holds:** the motor takes up slack, stretches the belt a few degrees and
  stalls. The encoder stops.
- **Belt slips:** the teeth ratchet over and the encoder jumps forward about one tooth
  at a time. With the 34:1 gearbox, one tooth of a Z-tooth drive pulley is
  48 × 34.014 / Z counts (82 counts for 20 teeth).

The script stops driving the moment it sees slip, so the belt skips as few teeth as
possible (usually under one).

## Wiring

| From | To | Notes |
|---|---|---|
| Bench supply + | G2 **VIN** | 16 V, see supply settings below |
| Bench supply − | G2 **GND** (power side) | |
| 100 µF (+ 100 nF) | across G2 VIN–GND | same as on the robot |
| Motor red / black | G2 **OUTA / OUTB** | swap, or use `--invert-dir`, if "fwd" runs the wrong way |
| Pi pin 12 (GPIO18) | G2 **PWM** | hardware PWM, 20 kHz |
| Pi pin 16 (GPIO23) | G2 **DIR** | |
| Pi pin 15 (GPIO22) | G2 **FLT** | optional; Pi pull-up enabled, low = fault and the test stops |
| Pi pin 14 (GND) | G2 **GND** (logic side) | the one ground link between the Pi and the driver |
| Pi pin 2 (5 V) | encoder **Vcc** (blue) | |
| Pi pin 6 (GND) | encoder **GND** (green) | |
| Encoder **A** (yellow) | 20 k → Pi pin 11 (GPIO17), 22 k to GND | same divider as the signal board: ~2.6 V high |
| Encoder **B** (white) | 20 k → Pi pin 13 (GPIO27), 22 k to GND | |
| G2 SLP, CS, VREF, 3V3 | unconnected | as on the robot (the board keeps the driver awake) |

Wire colours are Pololu's usual ones for these motors: check them against your
motor's product page before powering up. Power the Pi from its own USB supply.

## Pi setup (once)

1. Hardware PWM on GPIO18: add one line to `/boot/firmware/config.txt`, then reboot.
   ```
   dtoverlay=pwm
   ```
   On both the Pi 4 and the Pi 5 this puts PWM on GPIO18. The script finds the right
   PWM chip and channel itself: channel 0 on a Pi 4, channel 2 on a Pi 5.
   Check after the reboot: `pinctrl get 18` should show a PWM function (`PWM0_0` on a
   Pi 4, `PWM0_CHAN2` on a Pi 5).
2. Packages:
   ```
   sudo apt install python3-lgpio python3-matplotlib
   ```
3. Copy `slip_rig.py` and `slip_plot.py` to the Pi.
4. Dry run with the simulated motor and belt, without the hardware:
   ```
   python3 slip_rig.py --sim --pulley-teeth 20
   ```

If you get a `PermissionError` on `/sys/class/pwm`, run the commands with `sudo`.

## Bench supply

- **Voltage:** 16.0 V (the robot's rail), constant-voltage mode.
- **Current limit:** about 2.5 A. A stalled motor at 100 % duty takes about 1.8 A.
  If the supply drops into constant-current mode, the voltage falls and the torque
  estimate is wrong, so watch for the CC indicator.
- **Torque check:** the supply ammeter reads about duty × motor current. The script
  prints the expected value for each test, so you can compare.

## Procedure

1. **Measure the winding resistance** (optional, improves the torque estimate). Read a
   multimeter across the motor leads at a few shaft positions and take the lowest
   reading. Pass it as `--r-ohm 8.6`, for example. The spec-sheet value is 8.89 Ω.
2. **Wiring check, supply off or on with the PWM held at 0:**
   ```
   python3 slip_rig.py --check
   ```
   Turn the motor by hand: the counts should change smoothly, with "missed edges" at
   0 and the fault line "ok". Use `--invert-enc` if you want "fwd" to count up.
3. **Lock the output.**
4. **Coarse sweep**, 20 % to 100 % in 5 % steps, forward:
   ```
   python3 slip_rig.py --duties 20:100:5 --pulley-teeth 20 --volts 16
   ```
   Each duty: an 80 ms smooth step up (as the robot firmware does), 2 s hold, smooth
   release, 4 s rest to keep the motor cool. The sweep stops one duty after the first
   slip, to confirm it. Type `GO` at the prompt. Ctrl-C stops at once with the motor off.
5. **Fine sweep** around the threshold, both directions, repeated:
   ```
   python3 slip_rig.py --duties 50:65:1 --repeats 3 --dir both --pulley-teeth 20
   ```
6. **Ramp mode** for a quick single number: the duty climbs slowly and stops at the
   first slip.
   ```
   python3 slip_rig.py --mode ramp --start 20 --rate 2 --repeats 3
   ```
7. **Plot:**
   ```
   python3 slip_plot.py slip_runs/<run folder>
   ```

## Output

Each run writes `slip_runs/<date-time>/`:

- `config.json`: every setting, the pins, and the motor constants used.
- `raw.csv`: every 2 ms. Test number, direction, commanded duty, encoder counts,
  output degrees.
- `summary.csv`: one row per test.
  - duty, and the wind-up (slack + stretch) in counts and output degrees
  - motion after settling, slipped yes / no, why, and when it stopped
  - teeth skipped (with `--pulley-teeth`)
  - estimated motor and supply current, estimated output torque
- `slip_plot.png`, from `slip_plot.py`.

## Settings worth knowing

| Option | Default | What it does |
|---|---|---|
| `--hold` | 2.0 s | time at each duty |
| `--settle` | 0.4 s | wind-up allowed after the ramp before slip is judged |
| `--slip-counts` | 30 | motion after settling that counts as slip (6.6° at the output) |
| `--max-windup` | 150 | wind-up that counts as slip at any time. After a few held duties the script tightens this to 2.5 × the largest wind-up that held |
| `--ramp-ms` | 80 | smooth-step edge, as the robot. 0 = instant step |
| `--max-duty` | 100 | never exceed this duty |
| `--continue-after-slip` | 1 | duties tested after the first slip |

## Torque estimate

With the motor stalled, the average winding current is duty × V / R. The output
torque is then (Kt × I − friction) × 34.014 × gearbox efficiency. These are spec-sheet
values: Kt 0.0216 N·m/A, efficiency 0.59, friction from the 0.1 A no-load current.
Treat it as about ±15 %. The duty at which the belt slips is the measured result;
the torque is derived from it.

## Files

- `slip_rig.py`: the rig, including `--sim` and `--check`.
- `slip_plot.py`: plots a run folder.
- `tests/`: checks that need no Pi. `test_hw_stub.py` covers PWM chip/channel
  detection for Pi 4 and Pi 5, PWM writes, DIR, the fault input and quadrature
  decoding. `run_hw_path.py` runs a full test through the Pi code path against
  stand-ins.
