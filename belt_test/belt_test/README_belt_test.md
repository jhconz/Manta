# Constant-PWM belt test (one motor on a Raspberry Pi)

Runs one Pololu #5724 gearmotor on a G2 24v13 at a constant PWM duty and records
the motor encoder. Use it to see how shaft speed affects belt slip angle.

**No boot or `config.txt` changes.** PWM is software-timed by `lgpio`, which comes
with Raspberry Pi OS Bookworm. If it's missing: `sudo apt install python3-lgpio`.

## Wiring (unchanged)

| Signal | Physical pin | BCM |
|---|---|---|
| PWM → G2 PWM | 12 | GPIO18 |
| DIR → G2 DIR | 16 | GPIO23 |
| FLT ← G2 FLT (optional; use `--no-flt` if not wired) | 15 | GPIO22 |
| Encoder A (yellow), through the divider | 11 | GPIO17 |
| Encoder B (white), through the divider | 13 | GPIO27 |
| GND → G2 logic GND | 14 | — |
| 5 V → encoder Vcc (blue) | 2 | — |
| GND → encoder GND (green) | 6 | — |

- Motor red / black go to G2 OUTA / OUTB.
- The bench supply (16 V) goes to G2 VIN / GND.
- G2 SLP, CS, VREF and 3V3 stay unconnected.
- Encoder A and B must reach the Pi only through the dividers: the encoder runs on 5 V.

## Use

```
python3 belt_speed_test.py --check                         # motor off: turn it by hand, counts should follow
python3 belt_speed_test.py --duties 20,40,60,80 --time 3   # 3 s at each duty
python3 belt_speed_test.py --duties 20:100:10 --angle 360  # each run: one output turn by the encoder
python3 belt_speed_test.py --duties 30,60 --dir both --repeats 2
python3 belt_plot.py belt_runs/<run folder>
```

**Each run:**
- An 80 ms smooth step up (as the robot), constant duty, then a smooth step down.
- With `--angle`, the ramp-down starts early so the run ends near the target. In the
  simulator a 360° run landed within 1–7°.

**Between runs** the script waits for Enter, so you can mark the output position.

**After each run** it asks for the output angle you measured. It logs
**slip = encoder angle − measured angle**, in degrees and as a % of the encoder
angle. Press Enter to skip. `--no-prompt` runs straight through without asking.

**Stopping:** Ctrl-C stops at once with the motor off.

## Output

`belt_runs/<date-time>/` holds:

- `raw.csv`: every 2 ms. Run, duty, direction, time, commanded duty, counts, output
  degrees, motor and output rpm over a 50 ms window.
- `summary.csv`: one row per run. Duty, hold time, steady motor and output rpm,
  encoder output angle, measured angle, slip in ° and %, and missed encoder edges.
- `config.json`: the settings used.
- `belt_plot.png`, from `belt_plot.py`.

## Notes

- **Steady speed** is averaged over the constant-duty part, skipping its first 0.3 s
  (or its first half, if the hold is shorter than 0.6 s).
- **Software PWM** is 1 kHz by default (`--pwm-hz`, up to 10 kHz). The robot uses
  20 kHz hardware PWM. Average speed for a given duty is about the same, but the
  motor whines and its current ripples more. 0 % and 100 % are exact: the pin is
  held low or high.
- **Missed edges** counts encoder transitions the Pi didn't catch. At high speed
  (around 6,000 motor rpm = 4,800 edges/s) a busy Pi can drop some. If the count
  isn't 0, the encoder angle for that run is a little low: close other programs or
  test lower duties.
- **Simulator:** `--sim` runs the script against a simulated motor, to check the
  procedure without hardware.

## Files

- `belt_speed_test.py`: the test.
- `belt_plot.py`: plots a run folder.
- `tests/test_pi_path.py`: runs the Pi code path against a stand-in `lgpio`.
  It covers software-PWM calls, DIR, the fault input, encoder decoding, the
  measured-angle prompt and a clean stop.
