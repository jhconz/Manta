#!/usr/bin/env python3
"""Constant-PWM belt test: one Pololu #5724 gearmotor + G2 24v13 on a
Raspberry Pi, bench-supply powered. Runs the motor at a constant PWM duty,
records the motor encoder, and (optionally) logs the output angle you
measured, so belt slip can be compared against shaft speed.

No boot / config.txt changes: PWM is software-timed by lgpio (preinstalled on
Raspberry Pi OS Bookworm; otherwise: sudo apt install python3-lgpio).

Each run: smooth-step up to the duty (80 ms, as the robot), hold for --time
seconds (or until the encoder says the output turned --angle degrees),
smooth-step down, stop. Then it asks for the output angle you measured
(Enter skips) and moves on to the next duty when you press Enter.

Examples
  python3 belt_speed_test.py --duties 20,40,60,80 --time 3
  python3 belt_speed_test.py --duties 20:80:10 --angle 360      # one output turn per run
  python3 belt_speed_test.py --duties 30,60 --dir both --repeats 2
  python3 belt_speed_test.py --check                             # encoder wiring check, motor off
  python3 belt_speed_test.py --sim --duties 20,60 --no-prompt    # dry run, no hardware

Output: belt_runs/<date-time>/ with raw.csv (every 2 ms), summary.csv, config.json.
Plot: python3 belt_plot.py belt_runs/<date-time>
"""
import argparse, csv, json, math, os, signal, sys, time
from datetime import datetime

# ---------------------------------------------------------------- motor
GEAR = 34.014            # Pololu #5724 gearbox
CPR = 48                 # quadrature counts per motor revolution (x4)
KT = (24.0 - 0.1 * 24.0 / 2.7) / (300 * GEAR * 2 * math.pi / 60)    # for the simulator only

# ---------------------------------------------------------------- pins (BCM numbers)
PIN_PWM = 18             # physical pin 12
PIN_DIR = 23             # physical pin 16
PIN_ENC_A = 17           # physical pin 11 (through the divider)
PIN_ENC_B = 27           # physical pin 13 (through the divider)
PIN_FLT = 22             # physical pin 15, optional

SAMPLE_S = 0.002


def smooth_step(x):
    """Abaqus SMOOTH STEP, as in the robot firmware."""
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    return x * x * x * (10 + x * (-15 + 6 * x))


def out_deg(counts):
    return counts / CPR / GEAR * 360.0


# ================================================================ hardware
class PiRig:
    def __init__(self, pwm_hz, invert_dir=False, invert_enc=False, use_flt=True):
        import lgpio
        self.lg = lgpio
        self.pwm_hz = pwm_hz
        self.invert_dir = invert_dir
        self.use_flt = use_flt
        self.h = self._open_chip()
        lg = lgpio
        lg.gpio_claim_output(self.h, PIN_PWM, 0)
        lg.gpio_claim_output(self.h, PIN_DIR, 0)
        if use_flt:
            lg.gpio_claim_input(self.h, PIN_FLT, lg.SET_PULL_UP)
        lg.gpio_claim_alert(self.h, PIN_ENC_A, lg.BOTH_EDGES)
        lg.gpio_claim_alert(self.h, PIN_ENC_B, lg.BOTH_EDGES)
        self.count = 0
        self.bad = 0                                  # impossible transitions = missed edges
        self._a = lg.gpio_read(self.h, PIN_ENC_A)
        self._b = lg.gpio_read(self.h, PIN_ENC_B)
        self._state = (self._a << 1) | self._b
        t = [0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0]    # (old << 2) | new; 2 = invalid
        self._table = [-x if (invert_enc and x != 2) else x for x in t]
        self._cbs = [lg.callback(self.h, p, lg.BOTH_EDGES, self._edge) for p in (PIN_ENC_A, PIN_ENC_B)]
        self._duty = 0.0
        self.t0 = time.perf_counter()

    def _open_chip(self):
        """Header GPIO is gpiochip0 on a Pi 4 (and recent Pi 5 kernels), gpiochip4 on older Pi 5 kernels."""
        lg = self.lg
        for n in (0, 4):
            try:
                h = lg.gpiochip_open(n)
            except Exception:
                continue
            try:
                info = ' '.join(str(x) for x in lg.gpio_get_chip_info(h)[2:])
            except Exception:
                info = ''
            if 'rp1' in info or 'bcm' in info or 'brcm' in info:
                self.gpiochip = n
                return h
            lg.gpiochip_close(h)
        raise RuntimeError('No Raspberry Pi GPIO chip found')

    def _edge(self, chip, gpio, level, tick):
        if level > 1:                                 # watchdog report, not an edge
            return
        if gpio == PIN_ENC_A:
            self._a = level
        else:
            self._b = level
        new = (self._a << 1) | self._b
        step = self._table[(self._state << 2) | new]
        if step == 2:
            self.bad += 1
        else:
            self.count += step
        self._state = new

    def now(self):
        return time.perf_counter() - self.t0

    def wait_until(self, t):
        d = t - self.now()
        if d > 0:
            time.sleep(d)

    def set_dir(self, forward):
        self.lg.gpio_write(self.h, PIN_DIR, 0 if forward != self.invert_dir else 1)

    def set_duty(self, duty):
        duty = max(0.0, min(1.0, duty))
        if duty < 1e-4:
            duty = 0.0                                # below one PWM step: off
        if duty == self._duty or (duty and abs(duty - self._duty) < 1e-4):
            return                                    # skip tiny updates, but 0 is always applied
        if duty <= 0.0:
            self.lg.tx_pwm(self.h, PIN_PWM, 0, 0)     # stop PWM, then hold the pin low (brake)
            self.lg.gpio_write(self.h, PIN_PWM, 0)
        elif duty >= 1.0:
            self.lg.tx_pwm(self.h, PIN_PWM, 0, 0)
            self.lg.gpio_write(self.h, PIN_PWM, 1)
        else:
            self.lg.tx_pwm(self.h, PIN_PWM, self.pwm_hz, duty * 100.0)
        self._duty = duty

    def fault(self):
        return self.use_flt and self.lg.gpio_read(self.h, PIN_FLT) == 0

    def close(self):
        try:
            self.lg.tx_pwm(self.h, PIN_PWM, 0, 0)
            self.lg.gpio_write(self.h, PIN_PWM, 0)
        finally:
            for cb in self._cbs:
                cb.cancel()
            self.lg.gpiochip_close(self.h)


# ================================================================ simulator (code check only)
class SimRig:
    """Motor with a light load, for exercising the script without hardware."""

    def __init__(self, volts=16.0):
        self.V, self.R = volts, 24.0 / 2.7
        self.J = 3e-7
        self.th = self.w = self.t = 0.0
        self.duty, self.fwd = 0.0, True
        self.count = self.bad = 0
        self.gpiochip = -1

    def now(self):
        return self.t

    def wait_until(self, t, dt=20e-6):
        while self.t < t:
            if self.duty == 0 and abs(self.w) < 0.05:
                self.w, self.t = 0.0, t
                break
            i = ((1 if self.fwd else -1) * self.duty * self.V - KT * self.w) / self.R
            load = 0.0025 * math.tanh(self.w / 5) + 4e-9 * self.w * abs(self.w)
            self.w += (KT * i - load) / self.J * dt
            self.th += self.w * dt
            self.t += dt
        self.count = int(self.th / (2 * math.pi) * CPR)

    def set_dir(self, forward):
        self.fwd = forward

    def set_duty(self, duty):
        self.duty = max(0.0, min(1.0, duty))

    def fault(self):
        return False

    def close(self):
        self.duty = 0.0


# ================================================================ test
class Stop(Exception):
    pass


class Test:
    def __init__(self, rig, a, out):
        self.rig, self.a, self.out = rig, a, out
        self.raw_f = open(os.path.join(out, 'raw.csv'), 'w', newline='')
        self.raw = csv.writer(self.raw_f)
        self.raw.writerow(['run', 'duty_pct', 'direction', 't_s', 'duty_cmd', 'counts', 'output_deg',
                           'motor_rpm', 'output_rpm'])
        self.rows = []
        self.run_no = 0
        self.abort = False

    def run(self, duty, fwd):
        a, r = self.a, self.rig
        self.run_no += 1
        label = 'fwd' if fwd else 'rev'
        ramp = a.ramp_ms / 1000
        r.set_duty(0.0)
        r.set_dir(fwd)
        r.wait_until(r.now() + 0.05)
        c0, bad0 = r.count, r.bad
        t0 = r.now()
        target = a.angle / 360 * GEAR * CPR if a.angle else None
        hold_end = ramp + (a.time if not a.angle else a.max_time)
        phase, t_down = 'up', None
        samples = []                                 # (t, duty, counts)
        win = int(0.050 / SAMPLE_S)                  # speed over a 50 ms window
        k = 0
        while True:
            t = k * SAMPLE_S
            r.wait_until(t0 + t)
            if self.abort:
                raise Stop('stopped by user')
            if r.fault():
                raise Stop('driver fault (FLT low)')
            c = r.count - c0
            if phase == 'up' and t >= ramp:
                phase = 'hold'
            if phase == 'hold':
                # a smooth-step ramp-down covers half the distance it would at full speed,
                # so with --angle start it that much early to finish near the target
                speed = abs(c - samples[-win][2]) / (t - samples[-win][0]) if len(samples) >= win else 0.0
                if t >= hold_end or (target and abs(c) + speed * ramp / 2 >= target):
                    phase, t_down = 'down', t
            if phase == 'up':
                d = duty * smooth_step(t / ramp) if ramp else duty
            elif phase == 'hold':
                d = duty
            else:
                d = duty * (1 - smooth_step((t - t_down) / ramp)) if ramp else 0.0
            r.set_duty(d)
            if len(samples) >= win:
                dc = c - samples[-win][2]
                rpm = dc / CPR / (t - samples[-win][0]) * 60
            else:
                rpm = 0.0
            samples.append((t, d, c))
            self.raw.writerow([self.run_no, round(duty * 100, 2), label, '%.4f' % t, '%.4f' % d, c,
                               '%.3f' % out_deg(c), '%.1f' % rpm, '%.2f' % (rpm / GEAR)])
            if phase == 'down' and t - t_down >= ramp:
                break
            k += 1
        r.set_duty(0.0)
        # let it coast / brake to a stop and include that motion
        last, still, waited = r.count, 0, 0
        while still < 25 and waited < 1000:          # 50 ms without a count, at most 2 s
            waited += 1
            r.wait_until(r.now() + SAMPLE_S)
            still = still + 1 if r.count == last else 0
            last = r.count
        total = r.count - c0
        # steady speed: the hold, minus its first 0.3 s (or its first half, if it is short)
        skip = min(0.3, ((t_down or ramp) - ramp) / 2)
        steady = [(t, c) for t, d, c in samples if t_down and ramp + skip <= t <= t_down]
        if len(steady) >= 2:
            (ta, ca), (tb, cb) = steady[0], steady[-1]
            rpm = (cb - ca) / CPR / (tb - ta) * 60
        else:
            rpm = float('nan')
        row = dict(run=self.run_no, direction=label, duty_pct=round(duty * 100, 2),
                   hold_s=round((t_down or 0) - ramp, 3), steady_motor_rpm=round(abs(rpm), 1),
                   steady_output_rpm=round(abs(rpm) / GEAR, 2),
                   encoder_output_deg=round(out_deg(total), 2), measured_output_deg='',
                   slip_deg='', slip_pct='', missed_edges=r.bad - bad0)
        print('  run %2d  %-3s  duty %5.1f %%  steady %6.0f rpm motor = %5.1f rpm output   '
              'encoder says the output turned %8.1f°   missed edges %d'
              % (self.run_no, label, duty * 100, abs(rpm), abs(rpm) / GEAR, abs(out_deg(total)), r.bad - bad0))
        if a.prompt:
            m = input('        measured output angle in degrees (Enter to skip): ').strip()
            if m:
                try:
                    meas = abs(float(m))
                    enc = abs(out_deg(total))
                    row.update(measured_output_deg=meas, slip_deg=round(enc - meas, 2),
                               slip_pct=round(100 * (enc - meas) / enc, 2) if enc else '')
                    print('        slip %.1f° (%.1f %% of the encoder angle)' % (enc - meas, row['slip_pct'] or 0))
                except ValueError:
                    print('        not a number; skipped')
        self.rows.append(row)
        self.raw_f.flush()

    def save(self):
        self.raw_f.close()
        if self.rows:
            with open(os.path.join(self.out, 'summary.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)


def parse_duties(s):
    if ':' in s:
        a, b, c = (float(x) for x in s.split(':'))
        return [round(a + k * c, 4) for k in range(int(round((b - a) / c)) + 1)]
    return [float(x) for x in s.split(',')]


def wiring_check(rig, seconds=30):
    print('Motor off. Turn the motor by hand: one motor turn = %d counts, one output turn = %.0f counts. '
          'Ctrl-C to end.' % (CPR, CPR * GEAR))
    try:
        end = rig.now() + seconds
        while rig.now() < end:
            rig.wait_until(rig.now() + 0.25)
            print('\r  counts %+8d  (%+8.1f° at the output)  missed edges %d  fault %s   '
                  % (rig.count, out_deg(rig.count), rig.bad, 'LOW' if rig.fault() else 'ok'), end='', flush=True)
    except KeyboardInterrupt:
        pass
    print()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duties', default='20,40,60,80', help='PWM duties in %%: list or start:stop:step')
    p.add_argument('--time', type=float, default=3.0, help='seconds at constant duty (default 3)')
    p.add_argument('--angle', type=float, default=0, help='instead of --time: stop after this many output '
                                                          'degrees by the encoder')
    p.add_argument('--max-time', type=float, default=20, help='time cap with --angle (default 20 s)')
    p.add_argument('--dir', choices=('fwd', 'rev', 'both'), default='fwd')
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--ramp-ms', type=float, default=80, help='smooth-step edge (default 80 ms, as the robot)')
    p.add_argument('--pwm-hz', type=float, default=1000, help='software PWM frequency (default 1000, max 10000)')
    p.add_argument('--max-duty', type=float, default=100)
    p.add_argument('--invert-dir', action='store_true')
    p.add_argument('--invert-enc', action='store_true')
    p.add_argument('--no-flt', action='store_true', help='FLT not wired')
    p.add_argument('--no-prompt', action='store_true', help="don't ask for measured angles or wait between runs")
    p.add_argument('--check', action='store_true', help='encoder wiring check, motor off')
    p.add_argument('--sim', action='store_true', help='simulated motor (no hardware)')
    p.add_argument('--out', default='belt_runs')
    a = p.parse_args()
    a.prompt = not a.no_prompt and not a.sim

    rig = SimRig() if a.sim else PiRig(a.pwm_hz, a.invert_dir, a.invert_enc, not a.no_flt)
    if a.check:
        try:
            wiring_check(rig)
        finally:
            rig.close()
        return 0

    out = os.path.join(a.out, datetime.now().strftime('%Y%m%d-%H%M%S') + ('-sim' if a.sim else ''))
    os.makedirs(out, exist_ok=True)
    json.dump(dict(vars(a), gear=GEAR, cpr=CPR, pins=dict(pwm=PIN_PWM, dir=PIN_DIR, enc_a=PIN_ENC_A,
              enc_b=PIN_ENC_B, flt=PIN_FLT)), open(os.path.join(out, 'config.json'), 'w'), indent=1)
    test = Test(rig, a, out)

    def on_sigint(sig, frm):
        test.abort = True
        rig.set_duty(0.0)
    signal.signal(signal.SIGINT, on_sigint)

    duties = [d / 100 for d in parse_duties(a.duties) if 0 < d <= a.max_duty]
    dirs = {'fwd': [True], 'rev': [False], 'both': [True, False]}[a.dir]
    print('Run folder: %s   (software PWM %.0f Hz on GPIO%d)' % (out, a.pwm_hz, PIN_PWM))
    status = 0
    try:
        for duty in duties:
            for fwd in dirs:
                for _ in range(a.repeats):
                    if a.prompt:
                        input('Next: %.0f %% %s. Mark the output position, then press Enter to run... '
                              % (duty * 100, 'forward' if fwd else 'reverse'))
                    test.run(duty, fwd)
    except Stop as e:
        print('\nSTOPPED: %s' % e)
        status = 2
    except (KeyboardInterrupt, EOFError):
        print('\nSTOPPED')
        status = 2
    finally:
        rig.set_duty(0.0)
        rig.close()
        test.save()
        print('Saved %s. Plot: python3 belt_plot.py %s' % (out, out))
    return status


if __name__ == '__main__':
    sys.exit(main())
