#!/usr/bin/env python3
"""Belt-slip test rig: one Pololu #5724 gearmotor + G2 24v13 driver on a
Raspberry Pi (4 or 5), powered from a bench supply.

Method: lock the driven pulley (or hold the actuator against a hard stop),
then drive the motor at increasing PWM duty. While the belt holds, the motor
winds up the belt stretch and stalls. When the belt slips, the teeth ratchet
and the motor encoder jumps forward in bursts of about one tooth each. The
encoder is on the motor shaft, so one tooth of a Z-tooth drive pulley is
48 x 34.014 / Z counts (82 counts for Z = 20).

Modes
  step   for each duty: smooth-step up (as the robot firmware does), hold,
         release, rest. Slip = encoder motion after the wind-up has settled.
  ramp   duty rises slowly from --start; stops at the first slip and
         reports the duty it happened at.

Examples
  python3 slip_rig.py --sim                          # dry run, simulated motor + belt
  python3 slip_rig.py --duties 20:100:5              # step 20 %, 25 %, ... 100 %
  python3 slip_rig.py --mode ramp --start 20 --rate 2
  python3 slip_rig.py --duties 40,50,60 --repeats 3 --dir both

Every run writes slip_runs/<time>/ : config.json, raw.csv (2 ms samples),
summary.csv. Plot with slip_plot.py.
"""
import argparse, csv, glob, json, math, os, signal, sys, time
from datetime import datetime

# ---------------------------------------------------------------- motor constants
# Pololu #5724 (25D HP 24 V, 34.014:1, 48 CPR encoder on the motor shaft).
# From the spec sheet at 24 V: 300 rpm / 0.1 A no load, 2.7 A / 12 kg.cm stall.
GEAR = 34.014
CPR = 48                                     # quadrature counts per motor revolution
R_SPEC = 24.0 / 2.7                          # winding resistance, ohm (measure yours: --r-ohm)
KT = (24.0 - 0.1 * R_SPEC) / (300 * GEAR * 2 * math.pi / 60)   # N.m/A at the motor
T_FRIC = KT * 0.1                            # N.m at the motor (no-load current)
ETA = 12 * 0.0980665 / (KT * 2.7 * GEAR)     # gearbox efficiency at stall (~0.59)
PWM_HZ = 20_000

# ---------------------------------------------------------------- pins (BCM numbering)
PIN_PWM = 18          # header pin 12, hardware PWM
PIN_DIR = 23          # header pin 16
PIN_ENC_A = 17        # header pin 11
PIN_ENC_B = 27        # header pin 13
PIN_FLT = 22          # header pin 15, G2 fault (open drain, active low), optional


def smooth_step(x):
    """Abaqus SMOOTH STEP, same as the robot firmware: x^3 (10 - 15x + 6x^2)."""
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    return x * x * x * (10 + x * (-15 + 6 * x))


def stall_torque_est(duty, volts, r_ohm):
    """Estimated torque at the gearbox output with the motor stalled: average
    winding current = duty x V / R (PWM drive/brake, no back-EMF at stall)."""
    i = duty * volts / r_ohm
    return max(0.0, KT * i - T_FRIC) * GEAR * ETA, i


# ================================================================ hardware
class HardwareRig:
    """Raspberry Pi: hardware PWM through sysfs, DIR / FLT / encoder through lgpio."""

    def __init__(self, pwm_chip=None, pwm_channel=None, sysfs='/sys/class/pwm', invert_dir=False,
                 invert_enc=False):
        import lgpio                                    # Pi OS: sudo apt install python3-lgpio
        self.lg = lgpio
        self.invert_dir = invert_dir
        self._open_gpio()
        self._open_pwm(sysfs, pwm_chip, pwm_channel)
        self.count = 0
        self.bad = 0                                    # impossible quadrature transitions (missed edges)
        a, b = lgpio.gpio_read(self.h, PIN_ENC_A), lgpio.gpio_read(self.h, PIN_ENC_B)
        self._state = (a << 1) | b
        # x4 decoding: index = (old << 2) | new
        self._table = [0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0]
        if invert_enc:
            self._table = [-x if x != 2 else 2 for x in self._table]
        self._cbs = [lgpio.callback(self.h, p, lgpio.BOTH_EDGES, self._edge) for p in (PIN_ENC_A, PIN_ENC_B)]
        self.t0 = time.perf_counter()

    def _open_gpio(self):
        lg = self.lg
        # The header is gpiochip0 on a Pi 4 and on recent Pi 5 kernels, gpiochip4 on older Pi 5 kernels.
        last = None
        for chip in (0, 4, 1, 2, 3):
            try:
                h = lg.gpiochip_open(chip)
            except Exception as e:
                last = e
                continue
            try:
                info = lg.gpio_get_chip_info(h)          # (status, lines, name, label)
                label = ' '.join(str(x) for x in info[2:])
            except Exception:
                label = ''
            if 'rp1' in label or 'bcm' in label or 'brcm' in label:
                self.h, self.gpiochip = h, chip
                break
            lg.gpiochip_close(h)
        else:
            raise RuntimeError('No Raspberry Pi GPIO chip found (%s)' % last)
        lg.gpio_claim_output(self.h, PIN_DIR, 0)
        lg.gpio_claim_alert(self.h, PIN_ENC_A, lg.BOTH_EDGES)
        lg.gpio_claim_alert(self.h, PIN_ENC_B, lg.BOTH_EDGES)
        lg.gpio_claim_input(self.h, PIN_FLT, lg.SET_PULL_UP)

    def _open_pwm(self, sysfs, chip, channel):
        """Pi 4: pwmchip0 channel 0 for GPIO18. Pi 5: the 4-channel RP1 chip, channel 2."""
        if chip is None:
            chips = sorted(glob.glob(os.path.join(sysfs, 'pwmchip*')))
            if not chips:
                raise RuntimeError('No PWM chip in %s: add dtoverlay=pwm-2chan (Pi 4) or dtoverlay=pwm '
                                   '(Pi 5) to /boot/firmware/config.txt and reboot' % sysfs)
            best = None
            for c in chips:
                n = int(open(os.path.join(c, 'npwm')).read())
                if n == 4:                      # Pi 5 RP1 PWM0: GPIO18 = channel 2
                    best = (c, 2)
                    break
                if n == 2 and best is None:     # Pi 4 PWM0: GPIO18 = channel 0
                    best = (c, 0)
            if best is None:
                raise RuntimeError('No usable PWM chip among %s' % chips)
            path, ch = best
        else:
            path, ch = os.path.join(sysfs, 'pwmchip%d' % chip), channel if channel is not None else 0
        if channel is not None:
            ch = channel
        self.pwm_path = os.path.join(path, 'pwm%d' % ch)
        if not os.path.isdir(self.pwm_path):
            with open(os.path.join(path, 'export'), 'w') as f:
                f.write(str(ch))
            for _ in range(50):                 # udev may take a moment to set permissions
                if os.access(os.path.join(self.pwm_path, 'duty_cycle'), os.W_OK):
                    break
                time.sleep(0.02)
        self.period_ns = int(1e9 / PWM_HZ)
        self._w('enable', 0)
        self._w('duty_cycle', 0)
        self._w('period', self.period_ns)
        self._w('duty_cycle', 0)
        self._w('enable', 1)
        self._duty = 0.0

    def _w(self, name, value):
        with open(os.path.join(self.pwm_path, name), 'w') as f:
            f.write(str(int(value)))

    def _edge(self, chip, gpio, level, tick):
        a = self.lg.gpio_read(self.h, PIN_ENC_A) if gpio != PIN_ENC_A else level
        b = self.lg.gpio_read(self.h, PIN_ENC_B) if gpio != PIN_ENC_B else level
        new = (a << 1) | b
        step = self._table[(self._state << 2) | new]
        if step == 2:
            self.bad += 1
        else:
            self.count += step
        self._state = new

    # ---- rig interface
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
        if duty != self._duty:
            self._w('duty_cycle', round(duty * self.period_ns))
            self._duty = duty

    def fault(self):
        return self.lg.gpio_read(self.h, PIN_FLT) == 0

    def close(self):
        try:
            self.set_duty(0.0)
            self._w('enable', 0)
        finally:
            for cb in self._cbs:
                cb.cancel()
            self.lg.gpiochip_close(self.h)


# ================================================================ simulator
class SimRig:
    """Motor + belt with a locked output pulley, for dry runs. The belt is a
    spring at the drive pulley; above --sim-slip-nm a tooth jumps."""

    def __init__(self, volts, slip_nm=0.45, teeth=20, k_belt=8.0, r_ohm=R_SPEC, seed=1):
        import random
        self.rnd = random.Random(seed)
        self.V, self.R = volts, r_ohm
        self.slip_nm, self.k = slip_nm, k_belt          # N.m, N.m/rad at the output
        self.tooth = 2 * math.pi / teeth
        self.J = 1.7e-7 + 2e-5 / GEAR ** 2               # rotor + pulley, at the motor
        self.th = 0.0; self.w = 0.0; self.off = 0.0      # motor angle, speed; belt slip offset (output rad)
        self.t = 0.0; self.duty = 0.0; self.fwd = True
        self.count = 0; self.bad = 0
        self.slipping = 0                                # +1 / -1 while riding over a tooth
        self.slip_from = 0.0
        self.limit = slip_nm

    def now(self):
        return self.t

    def wait_until(self, t, dt=20e-6):
        while self.t < t:
            if self.duty == 0 and abs(self.w) < 0.05 and abs(self.k * (self.th / GEAR - self.off)) < 1e-3:
                self.t = t                                      # at rest: nothing changes
                break
            sgn = 1 if self.fwd else -1
            i = (sgn * self.duty * self.V - KT * self.w) / self.R
            out = self.th / GEAR                                # drive-pulley angle, rad
            if self.slipping:
                # riding over a tooth: the belt drags at a fraction of the slip torque
                # until the pulley has moved one tooth, then it re-engages there
                t_belt = self.slipping * 0.3 * self.slip_nm
                if self.slipping * (out - self.slip_from) < 0:
                    self.slipping = 0                           # backed off: drops into the same tooth
                    t_belt = self.k * (out - self.off)
                elif self.slipping * (out - self.slip_from) >= self.tooth:
                    self.off += self.slipping * self.tooth
                    self.slipping = 0
                    self.limit = self.slip_nm * (1 + 0.03 * self.rnd.uniform(-1, 1))  # varies tooth to tooth
                    t_belt = self.k * (out - self.off)
            else:
                t_belt = self.k * (out - self.off)              # belt stretch as a spring
                if abs(t_belt) > self.limit:
                    self.slipping = 1 if t_belt > 0 else -1
                    self.slip_from = out
            fric = T_FRIC * math.tanh(self.w / 5) + 2e-6 * self.w
            a = (KT * i - fric - t_belt / (GEAR * ETA)) / self.J
            self.w += a * dt
            self.th += self.w * dt
            self.t += dt
        self.count = int(round(self.th / (2 * math.pi) * CPR))

    def set_dir(self, forward):
        self.fwd = forward

    def set_duty(self, duty):
        self.duty = max(0.0, min(1.0, duty))

    def fault(self):
        return False

    def close(self):
        self.duty = 0.0


# ================================================================ test sequences
class Stop(Exception):
    pass


class SlipTest:
    SAMPLE_S = 0.002

    def __init__(self, rig, args, out):
        self.rig, self.a, self.out = rig, args, out
        self.raw = csv.writer(open(os.path.join(out, 'raw.csv'), 'w', newline=''))
        self.raw.writerow(['t_s', 'test', 'direction', 'duty_cmd', 'counts', 'output_deg'])
        self.rows = []
        self.test_no = 0
        self.abort = False
        self.max_held = {}                       # direction -> largest wind-up that held

    def windup_limit(self, label):
        """Wind-up beyond this is slip: 2.5 x the largest wind-up that held so far in
        this direction (at least --slip-counts), never more than --max-windup."""
        held = self.max_held.get(label)
        if not held:
            return self.a.max_windup
        return min(self.a.max_windup, max(self.a.slip_counts, int(2.5 * held)))

    def deg(self, counts):
        return counts / CPR / GEAR * 360

    def run_profile(self, label, fwd, duty_fn, t_end, early_stop=None):
        """Drive duty_fn(t) for t in [0, t_end); record every 2 ms.
        early_stop(t, counts) -> True ends it. Returns [(t, duty, counts)]."""
        r = self.rig
        r.set_duty(0.0)
        r.set_dir(fwd)
        c0 = r.count
        t0 = r.now()
        trace = []
        k = 0
        while True:
            t = k * self.SAMPLE_S
            r.wait_until(t0 + t)
            if self.abort:
                raise Stop('stopped by user')
            if r.fault():
                r.set_duty(0.0)
                raise Stop('driver fault (FLT low): check the supply current limit and wiring')
            c = r.count - c0
            d = duty_fn(t) if t < t_end else 0.0
            r.set_duty(d)
            trace.append((t, d, c))
            self.raw.writerow(['%.4f' % t, self.test_no, label, '%.4f' % d, c, '%.3f' % self.deg(c)])
            if t >= t_end or (early_stop and early_stop(t, c)):
                break
            k += 1
        r.set_duty(0.0)
        return trace

    def release(self, fwd, from_duty):
        """Smooth step back to 0 (belt unloads gently) and let it spring back."""
        ramp = self.a.ramp_ms / 1000
        self.run_profile('release', fwd, lambda t: from_duty * (1 - smooth_step(t / ramp)) if ramp else 0.0,
                         max(ramp, self.SAMPLE_S) + 0.2)

    def analyse(self, trace, duty):
        """Wind-up = motion until the settle point; slip = motion after it."""
        a = self.a
        t_settle = a.ramp_ms / 1000 + a.settle
        windup = next((c for t, d, c in trace if t >= t_settle), trace[-1][2])
        after = [c for t, d, c in trace if t >= t_settle and d > 0]
        post = (max(after) - min(after)) if after else 0
        # jumps: >= half the threshold within 20 ms
        n = int(0.020 / self.SAMPLE_S)
        jumps, k = 0, 0
        cs = [c for t, d, c in trace if t >= t_settle]
        while k + n < len(cs):
            if abs(cs[k + n] - cs[k]) >= a.slip_counts / 2:
                jumps += 1
                k += n
            else:
                k += 1
        return windup, post, jumps

    def step_test(self, duty, fwd):
        a = self.a
        ramp = a.ramp_ms / 1000
        self.test_no += 1
        label = 'fwd' if fwd else 'rev'
        t_settle = ramp + a.settle
        mark = {}
        lim = self.windup_limit(label)
        def early(t, c):
            # stop at the first sign of slip, so the belt skips as few teeth as possible
            if abs(c) > lim:
                mark['why'] = 'wind-up over %d counts' % lim
                return True
            if t >= t_settle:
                base = mark.setdefault('base', c)
                if abs(c - base) > a.slip_counts:
                    mark['why'] = 'moved after settling'
                    return True
            return False
        trace = self.run_profile(label, fwd, lambda t: duty * smooth_step(t / ramp) if ramp else duty,
                                 ramp + a.hold, early_stop=early)
        self.release(fwd, trace[-1][1])
        windup, post, jumps = self.analyse(trace, duty)
        slipped = 'why' in mark or post > a.slip_counts
        if not slipped:
            self.max_held[label] = max(self.max_held.get(label, 0), abs(windup))
        tq, i = stall_torque_est(duty, a.volts, a.r_ohm)
        row = dict(test=self.test_no, mode='step', direction=label, duty_pct=round(duty * 100, 2),
                   windup_counts=windup, windup_output_deg=round(self.deg(windup), 2),
                   post_settle_counts=post, slip_jumps=jumps, slipped=slipped,
                   teeth_skipped=round(post / self.tooth_counts(), 2) if a.pulley_teeth else '',
                   est_motor_current_A=round(i, 3), est_supply_current_A=round(i * duty, 3),
                   est_output_torque_Nm=round(tq, 3))
        row['stopped_after_s'] = round(trace[-1][0], 3)
        row['slip_reason'] = mark.get('why', '')
        self.rows.append(row)
        print('  %3d  %-3s  duty %5.1f %%  wind-up %+6d counts (%+6.1f° out)  after settle %5d counts  '
              '-> %s   [est. %.2f N.m out, supply ~%.2f A]'
              % (self.test_no, label, duty * 100, windup, self.deg(windup), post,
                 ('SLIP (%s, released at %.2f s)' % (mark['why'], trace[-1][0])) if slipped else 'held', tq, i * duty))
        return slipped

    def ramp_test(self, fwd):
        a = self.a
        self.test_no += 1
        label = 'fwd' if fwd else 'rev'
        start, stop, rate = a.start / 100, a.stop / 100, a.rate / 100
        ramp = a.ramp_ms / 1000
        t_settle = ramp + a.settle
        # duty: smooth step to `start`, settle, then a linear climb
        def duty_fn(t):
            if t < ramp:
                return start * smooth_step(t / ramp)
            if t < t_settle:
                return start
            return min(stop, start + rate * (t - t_settle))
        t_end = t_settle + (stop - start) / rate + a.hold
        win = int(0.050 / self.SAMPLE_S)
        hist = []
        mark = {}
        lim = self.windup_limit(label)
        def early(t, c):
            hist.append((t, c))
            if abs(c) > lim:                            # slipped while winding up
                mark['t'] = hist[0][0] if len(hist) <= win else hist[-1 - win][0]
                return True
            # after settling: more than --slip-counts of motion within 50 ms
            if t > t_settle + 0.05 and len(hist) > win and abs(hist[-1][1] - hist[-1 - win][1]) > a.slip_counts:
                mark['t'] = hist[-1 - win][0]           # when the jump began
                return True
            return False
        trace = self.run_profile(label, fwd, duty_fn, t_end, early_stop=early)
        last_d = trace[-1][1]
        self.release(fwd, last_d)
        slipped = 't' in mark
        d_slip = duty_fn(mark['t']) if slipped else None
        windup = next((c for t, d, c in trace if t >= t_settle), trace[-1][2])
        tq, i = stall_torque_est(d_slip or last_d, a.volts, a.r_ohm)
        row = dict(test=self.test_no, mode='ramp', direction=label,
                   duty_pct=round((d_slip if slipped else last_d) * 100, 2),
                   windup_counts=windup, windup_output_deg=round(self.deg(windup), 2),
                   post_settle_counts=trace[-1][2] - windup, slip_jumps='', slipped=slipped, teeth_skipped='',
                   est_motor_current_A=round(i, 3), est_supply_current_A=round(i * (d_slip or last_d), 3),
                   est_output_torque_Nm=round(tq, 3), stopped_after_s=round(trace[-1][0], 3),
                   slip_reason='moved during ramp' if slipped else '')
        self.rows.append(row)
        if slipped:
            print('  %3d  %-3s  ramp: SLIP at %.1f %% duty   [est. %.2f N.m out, motor ~%.2f A]'
                  % (self.test_no, label, d_slip * 100, tq, i))
        else:
            print('  %3d  %-3s  ramp: held up to %.1f %% duty' % (self.test_no, label, last_d * 100))
        return slipped

    def tooth_counts(self):
        return CPR * GEAR / self.a.pulley_teeth

    def rest(self):
        self.rig.set_duty(0.0)
        self.rig.wait_until(self.rig.now() + self.a.rest)

    def save_summary(self):
        if not self.rows:
            return
        with open(os.path.join(self.out, 'summary.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)


def wiring_check(a, seconds=20.0):
    rig = SimRig(a.volts) if a.sim else HardwareRig(a.pwm_chip, a.pwm_channel, a.sysfs, a.invert_dir, a.invert_enc)
    try:
        rig.set_duty(0.0)
        if not a.sim:
            print('PWM %s (held at 0), GPIO chip %d' % (rig.pwm_path, rig.gpiochip))
        print('Turn the motor shaft (or the drive pulley) by hand. One motor turn = %d counts; '
              'one output turn = %.0f counts.' % (CPR, CPR * GEAR))
        t_end = rig.now() + seconds
        while rig.now() < t_end:
            rig.wait_until(rig.now() + 0.25)
            print('\r  counts %+8d  (%+8.1f° at the output)  missed edges %d  fault line %s   '
                  % (rig.count, rig.count / CPR / GEAR * 360, rig.bad, 'LOW (fault)' if rig.fault() else 'ok'),
                  end='', flush=True)
        print()
    except KeyboardInterrupt:
        print()
    finally:
        rig.close()
    return 0


def parse_duties(s):
    if ':' in s:
        a, b, c = (float(x) for x in s.split(':'))
        n = int(round((b - a) / c)) + 1
        return [round(a + k * c, 4) for k in range(n)]
    return [float(x) for x in s.split(',')]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mode', choices=('step', 'ramp'), default='step')
    p.add_argument('--duties', default='20:100:5', help='step mode: start:stop:step or a list, in %% (default 20:100:5)')
    p.add_argument('--repeats', type=int, default=1, help='step mode: tests per duty (default 1)')
    p.add_argument('--continue-after-slip', type=int, default=1, metavar='N',
                   help='step mode: duties to keep testing after the first slip (default 1, to confirm it)')
    p.add_argument('--start', type=float, default=20, help='ramp mode: starting duty %% (default 20)')
    p.add_argument('--stop', type=float, default=100, help='ramp mode: highest duty %% (default 100)')
    p.add_argument('--rate', type=float, default=2.0, help='ramp mode: %% duty per second (default 2)')
    p.add_argument('--dir', choices=('fwd', 'rev', 'both'), default='fwd')
    p.add_argument('--hold', type=float, default=2.0, help='seconds at each duty (default 2)')
    p.add_argument('--ramp-ms', type=float, default=80, help='smooth-step edge length, ms (default 80, as the robot)')
    p.add_argument('--settle', type=float, default=0.4, help='wind-up time before slip is judged, s (default 0.4)')
    p.add_argument('--rest', type=float, default=4.0, help='seconds off between tests, to cool the motor (default 4)')
    p.add_argument('--slip-counts', type=int, default=30,
                   help='encoder counts of motion after settling that mean slip (default 30 = 6.6° at the output)')
    p.add_argument('--max-windup', type=int, default=150,
                   help='counts of wind-up (slack + stretch) treated as slip at any time (default 150 = 33° out)')
    p.add_argument('--pulley-teeth', type=int, default=0, help='drive pulley teeth, to report teeth skipped')
    p.add_argument('--volts', type=float, default=16.0, help='bench supply voltage, for the torque estimate')
    p.add_argument('--r-ohm', type=float, default=R_SPEC, help='measured winding resistance (default spec 8.89)')
    p.add_argument('--max-duty', type=float, default=100, help='never exceed this duty %% (default 100)')
    p.add_argument('--invert-dir', action='store_true', help='swap which DIR level is "fwd"')
    p.add_argument('--invert-enc', action='store_true', help='flip the encoder sign so "fwd" counts up')
    p.add_argument('--pwm-chip', type=int, help='override the PWM chip number')
    p.add_argument('--pwm-channel', type=int, help='override the PWM channel')
    p.add_argument('--sysfs', default='/sys/class/pwm', help=argparse.SUPPRESS)
    p.add_argument('--check', action='store_true',
                   help='wiring check: PWM off, print encoder counts and the fault line for 20 s '
                        '(turn the motor by hand)')
    p.add_argument('--sim', action='store_true', help='simulated motor and belt (no hardware)')
    p.add_argument('--sim-slip-nm', type=float, default=0.45, help='simulated belt slip torque at the output')
    p.add_argument('--yes', action='store_true', help='skip the "output locked?" prompt')
    p.add_argument('--out', default='slip_runs')
    a = p.parse_args()

    if a.check:
        return wiring_check(a)

    out = os.path.join(a.out, datetime.now().strftime('%Y%m%d-%H%M%S') + ('-sim' if a.sim else ''))
    os.makedirs(out, exist_ok=True)
    json.dump(dict(vars(a), motor=dict(gear=GEAR, cpr=CPR, kt=KT, eta=ETA, pwm_hz=PWM_HZ),
                   pins=dict(pwm=PIN_PWM, dir=PIN_DIR, enc_a=PIN_ENC_A, enc_b=PIN_ENC_B, flt=PIN_FLT)),
              open(os.path.join(out, 'config.json'), 'w'), indent=1)

    if not a.sim and not a.yes:
        print('Before starting:\n  - the driven pulley is locked (or the actuator is against a hard stop)\n'
              '  - the bench supply is at %.1f V, current limit set (2.5 A suggested), output on\n'
              '  - hands and loose items clear of the belt' % a.volts)
        if input('Type GO to start: ').strip().upper() != 'GO':
            print('Not started.')
            return 1

    rig = SimRig(a.volts, a.sim_slip_nm, a.pulley_teeth or 20, r_ohm=a.r_ohm) if a.sim else \
        HardwareRig(a.pwm_chip, a.pwm_channel, a.sysfs, a.invert_dir, a.invert_enc)
    test = SlipTest(rig, a, out)

    def on_sigint(sig, frm):
        test.abort = True
        rig.set_duty(0.0)
    signal.signal(signal.SIGINT, on_sigint)

    dirs = {'fwd': [True], 'rev': [False], 'both': [True, False]}[a.dir]
    cap = a.max_duty / 100
    print('Run folder: %s' % out)
    if not a.sim:
        print('PWM %s, GPIO chip %d' % (rig.pwm_path, rig.gpiochip))
    status = 0
    try:
        if a.mode == 'step':
            duties = [d / 100 for d in parse_duties(a.duties) if 0 < d / 100 <= cap]
            for fwd in dirs:
                print('%s:' % ('Forward' if fwd else 'Reverse'))
                extra = None
                for duty in duties:
                    any_slip = False
                    for _ in range(a.repeats):
                        any_slip |= test.step_test(duty, fwd)
                        test.rest()
                    if any_slip and extra is None:
                        extra = a.continue_after_slip
                    elif extra is not None:
                        extra -= 1
                    if extra is not None and extra <= 0 and any_slip:
                        break
                first = next((r for r in test.rows if r['slipped'] and r['direction'] == ('fwd' if fwd else 'rev')), None)
                held = [r for r in test.rows if not r['slipped'] and r['direction'] == ('fwd' if fwd else 'rev')]
                if first:
                    print('  -> first slip at %.1f %% duty (est. %.2f N.m at the output); held up to %.1f %%.'
                          % (first['duty_pct'], first['est_output_torque_Nm'],
                             max([r['duty_pct'] for r in held if r['duty_pct'] < first['duty_pct']] or [0])))
                else:
                    print('  -> no slip up to %.1f %% duty.' % (duties[-1] * 100 if duties else 0))
        else:
            a.stop = min(a.stop, a.max_duty)
            for fwd in dirs:
                for _ in range(a.repeats):
                    test.ramp_test(fwd)
                    test.rest()
    except Stop as e:
        print('\nSTOPPED: %s' % e)
        status = 2
    finally:
        rig.set_duty(0.0)
        rig.close()
        test.save_summary()
        print('Saved %s (summary.csv, raw.csv). Plot: python3 slip_plot.py %s' % (out, out))
    return status


if __name__ == '__main__':
    sys.exit(main())
