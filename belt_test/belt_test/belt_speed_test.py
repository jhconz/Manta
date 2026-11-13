#!/usr/bin/env python3
"""Constant-PWM belt slip test: one Pololu #5724 gearmotor + G2 24v13 on a
Raspberry Pi, bench-supply powered.

Each run drives the actuator at a constant PWM duty to the end of its travel.
When the belt slips there, the load torque drops away and the motor surges:
a sharp uptick in shaft speed. The script logs the cumulative encoder angle
every 2 ms, estimates speed by window-averaging those counts, and looks for
upticks in the last part of each run (--end-window). Running a range of
duties shows how shaft speed affects slip.

Encoder: 48 counts per motor turn (x4 quadrature) x 34.014 gearbox =
1632.67 counts per output turn. At 500 Hz sampling, slow runs see only 0-2
counts per sample, hence the window average (--avg-ms, default 10 ms).

Slip measures, most reliable first:
  beyond_travel_deg  total encoder angle minus the actuator's travel
                     (--travel-deg). The actuator can't go further, so all of
                     it is belt slip.
  upticks            sharp rises in speed in the last part of the run
                     (--end-window): when slip starts, and occasional slips.
                     Back-to-back ratcheting faster than ~3 averaging windows
                     per tooth can't be counted one by one; use beyond_travel_deg.

No boot / config.txt changes: PWM is software-timed by lgpio (preinstalled
on Raspberry Pi OS Bookworm; otherwise: sudo apt install python3-lgpio).

Before each run it waits for Enter so the actuator can be reset
(--no-pause runs straight through). Ctrl-C at that prompt ends the session;
during a run it stops the motor at once.

Examples
  python3 belt_speed_test.py --duties 20:100:10 --time 3
  python3 belt_speed_test.py --duties 30,60,90 --dir both --repeats 2 --end-window 1.5
  python3 belt_speed_test.py --check                   # encoder wiring check, motor off
  python3 belt_speed_test.py --sim --no-pause          # dry run: simulated actuator with an end stop

Output: belt_runs/<date-time>/ with raw.csv (every 2 ms), summary.csv (one row
per run), events.csv (one row per slip), config.json.
Plot: python3 belt_plot.py belt_runs/<date-time>
"""
import argparse, bisect, csv, json, math, os, signal, sys, time
from datetime import datetime

# ---------------------------------------------------------------- motor
GEAR = 34.014            # Pololu #5724 gearbox
CPR = 48                 # quadrature counts per motor revolution (x4)
CPR_OUT = CPR * GEAR     # 1632.67 counts per output revolution

# ---------------------------------------------------------------- pins (BCM numbers)
PIN_PWM = 18             # physical pin 12
PIN_DIR = 23             # physical pin 16
PIN_ENC_A = 17           # physical pin 11 (through the divider)
PIN_ENC_B = 27           # physical pin 13 (through the divider)
PIN_FLT = 22             # physical pin 15, optional

SAMPLE_S = 0.002         # 500 Hz


def smooth_step(x):
    """Abaqus SMOOTH STEP, as in the robot firmware."""
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    return x * x * x * (10 + x * (-15 + 6 * x))


def out_deg(counts):
    return counts / CPR_OUT * 360.0


def window_speed(ts, cs, avg_s):
    """Centred window average: (counts[i+h] - counts[i-h]) / time between them,
    in counts/s. Near the ends the window shrinks to what is available."""
    n = len(cs)
    h = max(1, int(round(avg_s / SAMPLE_S / 2)))
    out = []
    for i in range(n):
        a, b = max(0, i - h), min(n - 1, i + h)
        out.append((cs[b] - cs[a]) / (ts[b] - ts[a]) if ts[b] > ts[a] else 0.0)
    return out


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
        self._duty = 0.0
        self.t0 = time.perf_counter()
        # Every edge's kernel timestamp (lgpio: ns since the epoch) on this script's clock, so the
        # 500 Hz count series can be rebuilt from when edges happened, not when Python got them
        # (lgpio hands callbacks over in batches).
        self.edge_log = []                            # (t s, count)
        self._epoch_off = time.time() - time.perf_counter()
        self._tick_map = None
        self._cbs = [lg.callback(self.h, p, lg.BOTH_EDGES, self._edge) for p in (PIN_ENC_A, PIN_ENC_B)]

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
        if self._tick_map is None:                    # epoch-based ticks, or else map from this first edge
            t_epoch = tick * 1e-9 - self._epoch_off - self.t0
            self._tick_map = 'epoch' if abs(t_epoch - self.now()) < 1.0 else (tick, self.now())
        if self._tick_map == 'epoch':
            t = tick * 1e-9 - self._epoch_off - self.t0
        else:
            t = self._tick_map[1] + (tick - self._tick_map[0]) * 1e-9
        self.edge_log.append((t, self.count))

    def counts_at(self, times, c_start):
        """Cumulative count at each time (script clock), from the edge timestamps."""
        log = self.edge_log[:]                        # a snapshot; callbacks may still append
        log.sort()
        lt = [t for t, _ in log]
        out = []
        for t in times:
            k = bisect.bisect_right(lt, t)
            out.append(log[k - 1][1] if k else c_start)
        return out

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


# ================================================================ simulator
class SimRig:
    """Simulated actuator, to check the script and its slip detector.
    Free travel: the motor drives the load through the belt against a light
    speed-dependent drag. At the end of travel (stop_deg) the load is pinned:
    the belt then acts as a spring between the drive pulley and the pinned
    load. The motor decelerates as the belt stretches; when the tension beats
    the belt's grip, the belt skips one tooth, goes slack, and the motor surges
    until the belt re-tensions. Every true skip is recorded."""
    KT = (24.0 - 0.1 * 24.0 / 2.7) / (300 * GEAR * 2 * math.pi / 60)

    def __init__(self, volts=16.0, slip_nm=0.25, teeth=20, stop_deg=400.0, k_belt=3.0, seed=2):
        import random
        self.rnd = random.Random(seed)
        self.V, self.R = volts, 24.0 / 2.7
        self.J = 1.7e-7 + 1e-5 / GEAR ** 2           # rotor + drive pulley, at the motor
        self.c = 5e-4                                # free-travel drag c x w_out^2 (N.m)
        self.k_belt = k_belt                         # belt stiffness, N.m/rad at the pulley
        self.stop = math.radians(stop_deg)
        self.slip_nm, self.tooth = slip_nm, 2 * math.pi / teeth
        self.gpiochip = -1
        self.duty, self.fwd = 0.0, True
        self.t = 0.0
        self.bad = 0
        self.true_slips = []                         # (time, tooth angle)
        self.reset()

    def reset(self):
        """Put the actuator back at the start of travel (what you do between runs)."""
        self.th = self.w = 0.0
        self.off = 0.0                               # teeth skipped (as pulley angle)
        self.pinned = None                           # load angle once at the end stop
        self.limit = self.slip_nm
        self.count = 0
        self.edge_log = []

    def now(self):
        return self.t

    def wait_until(self, t, dt=20e-6):
        eta = 0.59
        while self.t < t:
            if self.duty == 0 and abs(self.w) < 0.05:
                self.w = 0.0
                self.t = t
                break
            sgn = 1 if self.fwd else -1
            i = (sgn * self.duty * self.V - self.KT * self.w) / self.R
            out, w_out = self.th / GEAR, self.w / GEAR
            if self.pinned is None:
                t_belt = self.c * w_out * abs(w_out)
                if sgn * (out - self.off) >= self.stop:
                    self.pinned = sgn * self.stop      # reached the end of travel
            if self.pinned is not None:
                stretch = (out - self.off) - self.pinned
                t_belt = self.k_belt * stretch if stretch * sgn > 0 else 0.0   # slack: no torque
                if abs(t_belt) > self.limit:           # tension beats the grip: skip a tooth
                    self.off += sgn * self.tooth
                    self.true_slips.append((self.t, self.tooth))
                    self.limit = self.slip_nm * (1 + 0.04 * self.rnd.uniform(-1, 1))
                    stretch = (out - self.off) - self.pinned
                    t_belt = self.k_belt * stretch if stretch * sgn > 0 else 0.0
            fric = 0.0022 * math.tanh(self.w / 5)
            self.w += (self.KT * i - fric - t_belt / (GEAR * eta)) / self.J * dt
            self.th += self.w * dt
            self.t += dt
            c = int(math.floor(self.th / (2 * math.pi) * CPR))
            if c != self.count:
                self.count = c
                self.edge_log.append((self.t, c))

    def counts_at(self, times, c_start):
        lt = [t for t, _ in self.edge_log]
        out = []
        for t in times:
            k = bisect.bisect_right(lt, t)
            out.append(self.edge_log[k - 1][1] if k else c_start)
        return out

    def set_dir(self, forward):
        self.fwd = forward

    def set_duty(self, duty):
        self.duty = max(0.0, min(1.0, duty))

    def fault(self):
        return False

    def close(self):
        self.duty = 0.0


# ================================================================ slip detection
def speed_noise(ts, v, t_from, t_to, block_s=0.1, floor=0.0):
    """Measurement noise of speed v over [t_from, t_to]: the sample-to-sample
    jitter (MAD of first differences) of the quietest 0.1 s block, so stretches
    full of slips don't count as noise. Never below `floor`."""
    x = [abs(a) for t, a in zip(ts, v) if t_from <= t <= t_to]
    nb = max(5, int(round(block_s / SAMPLE_S)))
    best = None
    for b in range(0, len(x) - nb + 1, nb // 2):
        blk = x[b:b + nb]
        d = sorted(abs(blk[i + 1] - blk[i]) for i in range(len(blk) - 1))
        val = 1.4826 * d[len(d) // 2] / math.sqrt(2)
        best = val if best is None else min(best, val)
    return None if best is None else max(best, floor)


def find_upticks(ts, v, cps_quant, t_from, t_to, k_noise=6.0, min_rise=0.10, onset_s=0.030, max_event_s=0.3,
                 noise=None):
    """Sharp upticks in window-averaged speed v (counts/s) within [t_from, t_to].

      noise  = sample-to-sample jitter of v (MAD of first differences), from
               the free-running part of the run when given (a window full of
               slips would otherwise count the slips as noise);
      onset  = v rises, within onset_s, by more than the largest of
               k_noise x noise, min_rise x speed, and 3 counts across the
               averaging window (cps_quant), so quantization can't trigger it;
      before = the lowest speed in the 2 x onset_s before (the trough the
               surge rises from);
      end    = speed back down to `before` (through any dip as the belt
               re-engages, and splitting at a sharp drop straight into a new
               surge), or max_event_s;
      slip angle = extra rotation above `before` over the event.
    Returns (events, engaged_cps, noise_cps)."""
    idx = [i for i, t in enumerate(ts) if t_from <= t <= t_to]
    if len(idx) < 10:
        return [], float('nan'), float('nan')
    lo, hi = idx[0], idx[-1] + 1
    t, x = ts[lo:hi], [abs(a) for a in v[lo:hi]]
    n = len(x)
    if noise is None:
        d = sorted(abs(x[i + 1] - x[i]) for i in range(n - 1))
        noise = 1.4826 * d[len(d) // 2] / math.sqrt(2)
    m = max(1, int(round(onset_s / SAMPLE_S)))
    events, in_event = [], [False] * n
    i = m
    while i < n:
        lo_i = min(range(max(0, i - 2 * m), i), key=lambda q: x[q])   # trough in the last 2 x onset_s
        before = x[lo_i]
        thr = max(k_noise * noise, min_rise * before, 3 * cps_quant)
        if x[i] - before > thr:
            j = lo_i
            while True:                              # one slip, or a chain of back-to-back slips
                k, peak, dipped, split = j, j, False, False
                while k + 1 < n and t[k + 1] - t[j] < max_event_s:
                    k += 1
                    if x[k] > x[peak]:
                        peak = k
                    if k <= peak:
                        continue
                    if (x[k] < x[peak] - 0.5 * (x[peak] - before) and x[k] > before + 2 * noise
                            and k + 1 < n and x[k + 1] >= x[k]
                            and any(x[q] - x[k] > thr for q in range(k + 1, min(n, k + 1 + m)))):
                        split = True
                        break
                    if x[k] < before - max(2 * noise, cps_quant):
                        dipped = True
                    if (not dipped and x[k] <= before + max(2 * noise, cps_quant)) or \
                            (dipped and x[k] >= before - noise):
                        if not dipped and k + 1 < n and x[k + 1] < before - max(2 * noise, cps_quant):
                            continue
                        break
                extra = sum((x[q] - before) * (t[q + 1] - t[q]) for q in range(j, min(k, n - 1)))
                events.append(dict(t_start=round(t[j], 4), t_peak=round(t[peak], 4),
                                   duration_ms=round((t[k] - t[j]) * 1000, 1),
                                   speed_before_rpm=round(before / CPR_OUT * 60, 2),
                                   peak_rpm=round(x[peak] / CPR_OUT * 60, 2),
                                   rise_pct=round(100 * (x[peak] - before) / before, 1) if before else '',
                                   slip_output_deg=round(out_deg(extra), 2),
                                   chained=bool(events and events[-1].get('_split'))))
                events[-1]['_split'] = split
                for q in range(j, k + 1):
                    in_event[q] = True
                if not split:
                    break
                j = k
            i = max(i + 1, k + m + 1)                    # always move forward
        else:
            i += 1
    for e in events:
        e.pop('_split', None)
    events = [e for e in events if e['slip_output_deg'] > 0]
    rest = sorted(a for a, e in zip(x, in_event) if not e) or sorted(x)
    return events, rest[len(rest) // 2], noise


# ================================================================ test
class Stop(Exception):
    pass


class Test:
    def __init__(self, rig, a, out):
        self.rig, self.a, self.out = rig, a, out
        self.raw_f = open(os.path.join(out, 'raw.csv'), 'w', newline='')
        self.raw = csv.writer(self.raw_f)
        self.raw.writerow(['run', 'duty_pct', 'direction', 't_s', 'duty_cmd', 'counts', 'output_deg',
                           'output_rpm_avg', 'in_end_window'])
        self.rows, self.events = [], []
        self.run_no = 0
        self.abort = False
        self.in_run = False

    def run(self, duty, fwd):
        a, r = self.a, self.rig
        self.run_no += 1
        label = 'fwd' if fwd else 'rev'
        ramp = a.ramp_ms / 1000
        if hasattr(r, 'reset'):
            r.reset()                                # simulator: actuator back to the start
        r.set_duty(0.0)
        r.set_dir(fwd)
        self.in_run = True
        r.wait_until(r.now() + 0.05)
        c0, bad0 = r.count, r.bad
        n_true0 = len(getattr(r, 'true_slips', []))
        t0 = r.now()
        t_down = ramp + a.time
        t_end = t_down + ramp
        ts, ds, cs = [], [], []
        k = 0
        while True:                                  # 500 Hz: command duty, record counts
            t = k * SAMPLE_S
            r.wait_until(t0 + t)
            if self.abort:
                raise Stop('stopped by user')
            if r.fault():
                raise Stop('driver fault (FLT low)')
            if t < ramp:
                d = duty * smooth_step(t / ramp) if ramp else duty
            elif t < t_down:
                d = duty
            else:
                d = duty * (1 - smooth_step((t - t_down) / ramp)) if ramp else 0.0
            r.set_duty(d)
            ts.append(t); ds.append(d); cs.append(r.count - c0)   # live count (for the run itself)
            if t >= t_end:
                break
            k += 1
        r.set_duty(0.0)
        last, still, waited = r.count, 0, 0
        while still < 25 and waited < 1000:          # wait for a stop: 50 ms without a count, at most 2 s
            waited += 1
            r.wait_until(r.now() + SAMPLE_S)
            still = still + 1 if r.count == last else 0
            last = r.count
        self.in_run = False

        # ---- analysis: counts at each 2 ms sample, from when the edges actually happened
        if hasattr(r, 'counts_at'):
            cs = [c - c0 for c in r.counts_at([t0 + t for t in ts], c0)]
        sgn = 1 if (cs[-1] >= 0) else -1
        v = window_speed(ts, [sgn * c for c in cs], a.avg_ms / 1000)
        cps_quant = 1.0 / (2 * max(1, int(round(a.avg_ms / 1000 / SAMPLE_S / 2))) * SAMPLE_S)  # 1 count per window
        settle = min(a.settle, a.time / 2)
        w_from = ramp + settle if not a.end_window else max(ramp + settle, t_down - a.end_window)
        ref = speed_noise(ts, v, ramp + settle, t_down, floor=0.5 * cps_quant)
        events, eng, noise = find_upticks(ts, v, cps_quant, w_from, t_down,
                                          k_noise=a.k_noise, min_rise=a.min_rise / 100,
                                          onset_s=max(0.010, a.avg_ms / 1000), noise=ref)
        # free-running speed: median over the hold before the end window, or before the first slip
        t_free_end = w_from if a.end_window else (events[0]['t_start'] if events else t_down)
        early = sorted(x for t, x in zip(ts, v) if ramp + settle <= t < t_free_end)
        # 75th percentile: robust to the actuator sitting stalled at its end for part of the hold
        free_rpm = early[int(0.75 * (len(early) - 1))] / CPR_OUT * 60 if early else float('nan')
        last = sorted(x for t, x in zip(ts, v) if t_down - 0.3 <= t <= t_down)
        final_rpm = last[len(last) // 2] / CPR_OUT * 60 if last else float('nan')
        for e in events:
            e['output_deg_at_start'] = round(out_deg(abs(cs[min(range(len(ts)), key=lambda q: abs(ts[q] - e['t_start']))])), 1)
            e.update(run=self.run_no, direction=label, duty_pct=round(duty * 100, 2))
            self.events.append(e)
        slip_total = sum(e['slip_output_deg'] for e in events)
        first = events[0] if events else None
        row = dict(run=self.run_no, direction=label, duty_pct=round(duty * 100, 2),
                   free_output_rpm=round(free_rpm, 2), final_output_rpm=round(final_rpm, 2),
                   end_window_output_rpm=round(eng / CPR_OUT * 60, 2) if eng == eng else '',
                   speed_noise_rpm=round(noise / CPR_OUT * 60, 3) if noise == noise else '',
                   window_from_s=round(w_from, 3), window_to_s=round(t_down, 3),
                   slips=len(events), slip_output_deg_total=round(slip_total, 2),
                   slip_output_deg_per_slip=round(slip_total / len(events), 2) if events else '',
                   first_slip_s=first['t_start'] if first else '',
                   first_slip_output_deg=first['output_deg_at_start'] if first else '',
                   max_rise_pct=max([e['rise_pct'] for e in events if e['rise_pct'] != ''] or ['']),
                   total_output_deg=round(out_deg(abs(cs[-1])), 2),
                   beyond_travel_deg=round(max(0.0, out_deg(abs(cs[-1])) - a.travel_deg), 2) if a.travel_deg else '',
                   end_window_travel_deg=round(out_deg(abs(cs[min(range(len(ts)), key=lambda q: abs(ts[q] - t_down))]
                                                     - cs[min(range(len(ts)), key=lambda q: abs(ts[q] - w_from))])), 2),
                   missed_edges=r.bad - bad0)
        if hasattr(r, 'true_slips'):
            row['sim_true_slips_in_window'] = sum(1 for tt, _ in r.true_slips[n_true0:]
                                                  if t0 + w_from <= tt <= t0 + t_down)
        self.rows.append(row)
        for t, d, c, x in zip(ts, ds, cs, v):
            self.raw.writerow([self.run_no, round(duty * 100, 2), label, '%.3f' % t, '%.4f' % d, c,
                               '%.3f' % out_deg(c), '%.3f' % (x / CPR_OUT * 60), int(w_from <= t <= t_down)])
        beyond = ('   beyond travel %6.1f°' % row['beyond_travel_deg']) if a.travel_deg else ''
        truth = ''
        if hasattr(r, 'true_slips'):
            ts_true = [tt - t0 for tt, _ in r.true_slips[n_true0:]]
            truth = '   (sim truth: first slip %s, %d slips, %.0f° beyond travel)' % (
                ('%.2f s' % ts_true[0]) if ts_true else 'none', len(ts_true),
                math.degrees(sum(x for _, x in r.true_slips[n_true0:])))
        onset = ('slip starts at %.2f s, %.0f°' % (first['t_start'], first['output_deg_at_start'])) if first \
            else 'no slip'
        print('  run %2d  %-3s  duty %5.1f %%  free %6.1f rpm out  total %7.1f°%s   %-31s upticks %3d%s'
              '   missed edges %d'
              % (self.run_no, label, duty * 100, free_rpm, out_deg(abs(cs[-1])), beyond, onset,
                 len(events), truth, r.bad - bad0))
        self.raw_f.flush()
        self.save_tables()                           # after every run, so they survive a kill

    def save_tables(self):
        for name, rows, keys in (('summary.csv', self.rows, None),
                                 ('events.csv', self.events,
                                  ['run', 'direction', 'duty_pct', 't_start', 't_peak', 'duration_ms',
                                   'output_deg_at_start', 'speed_before_rpm', 'peak_rpm', 'rise_pct',
                                   'slip_output_deg', 'chained'])):
            if not rows and name == 'summary.csv':
                continue
            tmp = os.path.join(self.out, name + '.tmp')
            with open(tmp, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=keys or list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            os.replace(tmp, os.path.join(self.out, name))

    def save(self):
        if not self.raw_f.closed:
            self.raw_f.close()
        self.save_tables()


def parse_duties(s):
    if ':' in s:
        a, b, c = (float(x) for x in s.split(':'))
        return [round(a + k * c, 4) for k in range(int(round((b - a) / c)) + 1)]
    return [float(x) for x in s.split(',')]


def wiring_check(rig, seconds=30):
    print('Motor off. Turn the motor by hand: one motor turn = %d counts, one output turn = %.2f counts. '
          'Ctrl-C to end.' % (CPR, CPR_OUT))
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
    p.add_argument('--duties', default='20:100:10', help='PWM duties in %%: list or start:stop:step (default 20:100:10)')
    p.add_argument('--time', type=float, default=3.0, help='seconds at constant duty per run (default 3)')
    p.add_argument('--end-window', type=float, default=0,
                   help='only look for slip in the last this-many seconds of the hold (default 0 = whole hold, '
                        'so the first uptick marks where slipping starts at the end of travel)')
    p.add_argument('--avg-ms', type=float, default=10, help='speed averaging window, ms (default 10)')
    p.add_argument('--travel-deg', type=float, default=0,
                   help="actuator's travel from its reset position, output degrees: rotation beyond it is "
                        "reported as slip (default 0 = not used)")
    p.add_argument('--dir', choices=('fwd', 'rev', 'both'), default='fwd')
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--no-pause', action='store_true',
                   help="don't wait for Enter before each run (default: wait, so the actuator can be reset)")
    p.add_argument('--rest', type=float, default=1.0, help='with --no-pause: seconds stopped between runs (default 1)')
    p.add_argument('--ramp-ms', type=float, default=80, help='smooth-step edge (default 80 ms, as the robot)')
    p.add_argument('--settle', type=float, default=0.3, help='seconds after the ramp ignored (default 0.3)')
    p.add_argument('--k-noise', type=float, default=6, help='uptick threshold, x speed noise (default 6)')
    p.add_argument('--min-rise', type=float, default=10, help='uptick threshold, %% of speed (default 10)')
    p.add_argument('--pulley-teeth', type=int, default=0, help='drive pulley teeth (marks one tooth on the plot)')
    p.add_argument('--pwm-hz', type=float, default=1000, help='software PWM frequency (default 1000, max 10000)')
    p.add_argument('--max-duty', type=float, default=100)
    p.add_argument('--invert-dir', action='store_true')
    p.add_argument('--invert-enc', action='store_true')
    p.add_argument('--no-flt', action='store_true', help='FLT not wired')
    p.add_argument('--check', action='store_true', help='encoder wiring check, motor off')
    p.add_argument('--sim', action='store_true', help='simulated actuator with an end stop (no hardware)')
    p.add_argument('--sim-stop-deg', type=float, default=400, help='simulated end of travel, output degrees')
    p.add_argument('--out', default='belt_runs')
    a = p.parse_args()

    rig = SimRig(teeth=a.pulley_teeth or 20, stop_deg=a.sim_stop_deg) if a.sim else \
        PiRig(a.pwm_hz, a.invert_dir, a.invert_enc, not a.no_flt)
    if a.check:
        try:
            wiring_check(rig)
        finally:
            rig.close()
        return 0

    out = os.path.join(a.out, datetime.now().strftime('%Y%m%d-%H%M%S') + ('-sim' if a.sim else ''))
    os.makedirs(out, exist_ok=True)
    json.dump(dict(vars(a), gear=GEAR, cpr=CPR, cpr_out=CPR_OUT, sample_hz=1 / SAMPLE_S,
                   pins=dict(pwm=PIN_PWM, dir=PIN_DIR, enc_a=PIN_ENC_A, enc_b=PIN_ENC_B, flt=PIN_FLT)),
              open(os.path.join(out, 'config.json'), 'w'), indent=1)
    test = Test(rig, a, out)

    def on_sigint(sig, frm):
        rig.set_duty(0.0)
        if test.in_run:
            test.abort = True                        # the run loop stops at its next 2 ms step
        else:
            raise KeyboardInterrupt                  # at the prompt: end the session
    signal.signal(signal.SIGINT, on_sigint)

    duties = [d / 100 for d in parse_duties(a.duties) if 0 < d <= a.max_duty]
    dirs = {'fwd': [True], 'rev': [False], 'both': [True, False]}[a.dir]
    print('Run folder: %s   (software PWM %.0f Hz on GPIO%d, speed = %.0f ms window average)'
          % (out, a.pwm_hz, PIN_PWM, a.avg_ms))
    pause = not a.no_pause and not a.sim
    status = 0
    try:
        first = True
        for duty in duties:
            for fwd in dirs:
                for _ in range(a.repeats):
                    if pause:
                        input('Next: %.0f %% %s. Reset the actuator, then press Enter to run '
                              '(Ctrl-C to finish)... ' % (duty * 100, 'forward' if fwd else 'reverse'))
                    elif not first:
                        rig.wait_until(rig.now() + a.rest)
                    first = False
                    test.run(duty, fwd)
    except Stop as e:
        print('\nSTOPPED: %s' % e)
        status = 2
    except (KeyboardInterrupt, EOFError):
        print('\nSTOPPED')
        status = 2
    finally:
        try:
            rig.set_duty(0.0)
        finally:
            test.save()
            rig.close()
        print('Saved %s (summary.csv, events.csv, raw.csv). Plot: python3 belt_plot.py %s' % (out, out))
    return status


if __name__ == '__main__':
    sys.exit(main())
