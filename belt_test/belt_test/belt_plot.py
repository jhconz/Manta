#!/usr/bin/env python3
"""Plot a belt_speed_test.py run folder.
  1. Cumulative output angle vs time for every run (travel line with --travel-deg).
  2. Window-averaged output speed vs time; detected upticks marked; end window shaded.
  3. Slip vs free-running output speed: angle beyond travel (or travel in the end
     window) and upticks in the end window.
Usage: python3 belt_plot.py belt_runs/<run>   (needs matplotlib)"""
import csv, json, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm

INK, INK2, GRID, RED = '#1f1f1e', '#5f5e58', '#e4e3df', '#c0392b'
COL = {'fwd': '#2a78d6', 'rev': '#eb6834'}


def main(run):
    cfg = json.load(open(os.path.join(run, 'config.json')))
    rows = list(csv.DictReader(open(os.path.join(run, 'summary.csv'))))
    ep = os.path.join(run, 'events.csv')
    events = list(csv.DictReader(open(ep))) if os.path.exists(ep) else []
    raw = {}
    for r in csv.DictReader(open(os.path.join(run, 'raw.csv'))):
        raw.setdefault(int(r['run']), []).append((float(r['t_s']), abs(float(r['output_deg'])),
                                                   abs(float(r['output_rpm_avg']))))
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.edgecolor': GRID,
                         'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
                         'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6})
    fig = plt.figure(figsize=(12.5, 10.5))
    gs = fig.add_gridspec(3, 2, height_ratios=(1, 1, 0.9))
    a1 = fig.add_subplot(gs[0, :])
    a2 = fig.add_subplot(gs[1, :], sharex=a1)
    a3, a4 = fig.add_subplot(gs[2, 0]), fig.add_subplot(gs[2, 1])
    duties = [float(r['duty_pct']) for r in rows]
    lo, hi = min(duties), max(duties)
    for r in rows:
        n = int(r['run'])
        tr = raw.get(n, [])
        frac = 0.35 + 0.65 * (float(r['duty_pct']) - lo) / max(1e-9, hi - lo)
        col = (cm.Blues if r['direction'] == 'fwd' else cm.Oranges)(frac)
        lab = '%s %% %s' % (r['duty_pct'].rstrip('0').rstrip('.'), r['direction'])
        a1.plot([t for t, _, _ in tr], [d for _, d, _ in tr], color=col, lw=1.2, label=lab)
        a2.plot([t for t, _, _ in tr], [w for _, _, w in tr], color=col, lw=0.8)
        ev = [e for e in events if int(e['run']) == n]
        if ev:
            tt = [float(e['t_peak']) for e in ev]
            a2.plot(tt, [float(e['peak_rpm']) for e in ev], 'v', color=RED, ms=4, mec='white', mew=0.5, zorder=4)
            a1.plot([float(e['t_start']) for e in ev], [float(e['output_deg_at_start']) for e in ev], '.',
                    color=RED, ms=4, zorder=4)
    if rows:
        w0, w1 = float(rows[0]['window_from_s']), float(rows[0]['window_to_s'])
        if cfg.get('end_window'):
            for ax in (a1, a2):
                ax.axvspan(w0, w1, color='#f6e7c8', alpha=0.6, zorder=0)
        if cfg.get('end_window'):
            a2.annotate('slip search window', (w0, 1.0), xycoords=('data', 'axes fraction'), xytext=(3, -12),
                        textcoords='offset points', fontsize=7.5, color=INK2)
    if cfg.get('travel_deg'):
        a1.axhline(cfg['travel_deg'], color=INK2, lw=1, ls='--')
        a1.annotate('actuator travel %.0f°: anything above is belt slip' % cfg['travel_deg'],
                    (0.01, cfg['travel_deg']), xycoords=('axes fraction', 'data'), xytext=(0, 4),
                    textcoords='offset points', fontsize=8, color=INK2)
    a2.plot([], [], 'v', color=RED, ms=4, label='uptick (slip)')
    a1.set_ylabel('cumulative output angle (°)')
    a1.set_title('Cumulative output angle (motor encoder ÷ 34.014)', loc='left', fontsize=10,
                 fontweight='bold', color=INK)
    a1.legend(fontsize=7, frameon=False, loc='upper left', bbox_to_anchor=(1.005, 1.0))
    a2.set_ylabel('output speed (rpm, %.0f ms window)' % cfg['avg_ms'])
    a2.set_xlabel('time from run start (s)')
    a2.set_title('Window-averaged speed: a sharp uptick is a slip', loc='left', fontsize=10,
                 fontweight='bold', color=INK)
    a2.legend(fontsize=7.5, frameon=False, loc='upper left', bbox_to_anchor=(1.005, 1.0))

    use_travel = bool(cfg.get('travel_deg'))
    key = 'beyond_travel_deg' if use_travel else 'end_window_travel_deg'
    for d in ('fwd', 'rev'):
        rs = sorted((float(r['free_output_rpm']), float(r[key] or 0), r['first_slip_s'])
                    for r in rows if r['direction'] == d and r['free_output_rpm'] not in ('', 'nan'))
        if rs:
            kw = dict(marker='o', ms=6, mec='white', mew=1.1, lw=1.4, color=COL[d],
                      label='forward' if d == 'fwd' else 'reverse')
            a3.plot([x for x, _, _ in rs], [y for _, y, _ in rs], **kw)
            on = [(x, float(z)) for x, _, z in rs if z not in ('', None)]
            if on:
                a4.plot([x for x, _ in on], [z for _, z in on], **kw)
    a3.set_xlabel('free-running output speed (rpm)')
    a3.set_ylabel('belt slip: angle beyond travel (°)' if use_travel else 'output angle in the search window (°)')
    a3.set_title('Slip angle vs speed' if use_travel else 'Motion in the search window vs speed',
                 loc='left', fontsize=10, fontweight='bold', color=INK)
    a4.set_xlabel('free-running output speed (rpm)')
    a4.set_ylabel('time from run start to the first slip (s)')
    a4.set_title('Slip onset vs speed (first uptick)', loc='left', fontsize=10, fontweight='bold', color=INK)
    for ax in (a3, a4):
        ax.legend(fontsize=8, frameon=False)
    fig.suptitle('Belt test %s  (%s, %.1f s per run, PWM %.0f Hz, sampled at 500 Hz)' % (
        os.path.basename(run.rstrip('/')), 'simulated' if cfg.get('sim') else 'hardware', cfg['time'], cfg['pwm_hz']),
        x=0.01, ha='left', fontsize=11, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(run, 'belt_plot.png')
    fig.savefig(out, dpi=105)
    print(out)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
