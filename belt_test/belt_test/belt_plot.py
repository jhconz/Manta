#!/usr/bin/env python3
"""Plot a belt_speed_test.py run folder.
Top: output speed vs time for every run. Bottom: belt slip vs steady output
speed (runs where you entered a measured angle), or the encoder angle per run
if none were measured. Usage: python3 belt_plot.py belt_runs/<run>
Needs matplotlib (sudo apt install python3-matplotlib)."""
import csv, json, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm

INK, INK2, GRID = '#1f1f1e', '#5f5e58', '#e4e3df'
COL = {'fwd': '#2a78d6', 'rev': '#eb6834'}


def main(run):
    cfg = json.load(open(os.path.join(run, 'config.json')))
    rows = list(csv.DictReader(open(os.path.join(run, 'summary.csv'))))
    raw = {}
    for r in csv.DictReader(open(os.path.join(run, 'raw.csv'))):
        raw.setdefault(int(r['run']), []).append((float(r['t_s']), abs(float(r['output_rpm']))))
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.edgecolor': GRID,
                         'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
                         'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6})
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7.5))
    duties = [float(r['duty_pct']) for r in rows]
    lo, hi = min(duties), max(duties)
    for r in rows:
        tr = raw.get(int(r['run']), [])
        frac = 0.35 + 0.65 * (float(r['duty_pct']) - lo) / max(1e-9, hi - lo)
        cmap = cm.Blues if r['direction'] == 'fwd' else cm.Oranges
        a1.plot([t for t, _ in tr], [w for _, w in tr], color=cmap(frac), lw=1.3,
                label='run %s: %s %% %s' % (r['run'], r['duty_pct'].rstrip('0').rstrip('.'), r['direction']))
    a1.set_xlabel('time from run start (s)')
    a1.set_ylabel('output speed (rpm, 50 ms average)')
    a1.set_title('Output shaft speed during each run (from the motor encoder)', loc='left',
                 fontsize=10, fontweight='bold', color=INK)
    if len(rows) <= 14:
        a1.legend(fontsize=7.5, frameon=False, ncol=2, loc='upper right')

    meas = [r for r in rows if r['slip_deg'] not in ('', None)]
    if meas:
        for d in ('fwd', 'rev'):
            pts = sorted((float(r['steady_output_rpm']), float(r['slip_deg'])) for r in meas if r['direction'] == d)
            if pts:
                a2.plot([p[0] for p in pts], [p[1] for p in pts], marker='o', ms=7, mec='white', mew=1.2,
                        lw=1.5, color=COL[d], label='forward' if d == 'fwd' else 'reverse')
        a2.axhline(0, color=INK2, lw=0.8)
        a2.set_ylabel('belt slip (°) = encoder angle − measured angle')
        a2.set_title('Belt slip vs steady output speed', loc='left', fontsize=10, fontweight='bold', color=INK)
        a2.legend(fontsize=8, frameon=False)
    else:
        for d in ('fwd', 'rev'):
            pts = sorted((float(r['steady_output_rpm']), abs(float(r['encoder_output_deg'])))
                         for r in rows if r['direction'] == d)
            if pts:
                a2.plot([p[0] for p in pts], [p[1] for p in pts], marker='o', ms=7, mec='white', mew=1.2,
                        lw=1.5, color=COL[d], label='forward' if d == 'fwd' else 'reverse')
        a2.set_ylabel('output angle by the encoder (°)')
        a2.set_title('Encoder output angle per run (no measured angles entered, so no slip yet)',
                     loc='left', fontsize=10, fontweight='bold', color=INK)
        a2.legend(fontsize=8, frameon=False)
    a2.set_xlabel('steady output speed (rpm)')
    fig.suptitle('Belt test %s  (%s, software PWM %.0f Hz%s)' % (
        os.path.basename(run.rstrip('/')), 'simulated' if cfg.get('sim') else 'hardware', cfg['pwm_hz'],
        (', %.0f° per run' % cfg['angle']) if cfg.get('angle') else (', %.1f s hold' % cfg['time'])),
        x=0.01, ha='left', fontsize=11, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(run, 'belt_plot.png')
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
