#!/usr/bin/env python3
"""Plot a belt_speed_test.py run folder.
Top: motor speed vs time for every run, detected slips marked.
Bottom left: slips per second vs engaged output speed.
Bottom right: slip angle per slip (at the drive pulley) vs engaged output speed.
Usage: python3 belt_plot.py belt_runs/<run>   (needs matplotlib)"""
import csv, json, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm

INK, INK2, GRID, RED = '#1f1f1e', '#5f5e58', '#e4e3df', '#c0392b'
COL = {'fwd': '#2a78d6', 'rev': '#eb6834'}


def load(run):
    rows = list(csv.DictReader(open(os.path.join(run, 'summary.csv'))))
    ep = os.path.join(run, 'events.csv')
    events = list(csv.DictReader(open(ep))) if os.path.exists(ep) else []
    raw = {}
    for r in csv.DictReader(open(os.path.join(run, 'raw.csv'))):
        raw.setdefault(int(r['run']), []).append((float(r['t_s']), abs(float(r['motor_rpm']))))
    return rows, events, raw


def main(run):
    cfg = json.load(open(os.path.join(run, 'config.json')))
    rows, events, raw = load(run)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.edgecolor': GRID,
                         'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
                         'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6})
    fig = plt.figure(figsize=(12.5, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=(1.15, 1))
    a1 = fig.add_subplot(gs[0, :])
    a2, a3 = fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])
    duties = [float(r['duty_pct']) for r in rows]
    lo, hi = min(duties), max(duties)
    for r in rows:
        n = int(r['run'])
        tr = raw.get(n, [])
        frac = 0.35 + 0.65 * (float(r['duty_pct']) - lo) / max(1e-9, hi - lo)
        cmap = cm.Blues if r['direction'] == 'fwd' else cm.Oranges
        a1.plot([t for t, _ in tr], [w for _, w in tr], color=cmap(frac), lw=0.9,
                label='%s %% %s: %s slips' % (r['duty_pct'].rstrip('0').rstrip('.'), r['direction'], r['slip_events']))
        ev = [e for e in events if int(e['run']) == n]
        a1.plot([float(e['t_peak']) for e in ev], [float(e['peak_rpm']) for e in ev], 'v', color=RED, ms=5,
                mec='white', mew=0.6, zorder=4)
    a1.plot([], [], 'v', color=RED, ms=5, label='detected slip')
    a1.set_xlabel('time from run start (s)')
    a1.set_ylabel('motor speed (rpm)')
    a1.set_title('Motor speed at constant PWM (each slip is a sudden surge)', loc='left',
                 fontsize=10, fontweight='bold', color=INK)
    if len(rows) <= 16:
        a1.legend(fontsize=7, frameon=False, ncol=1, loc='upper left', bbox_to_anchor=(1.005, 1.0))

    for d in ('fwd', 'rev'):
        rs = sorted((float(r['engaged_output_rpm']), float(r['slips_per_s'] or 0)) for r in rows if r['direction'] == d)
        if rs:
            a2.plot([x for x, _ in rs], [y for _, y in rs], marker='o', ms=6, mec='white', mew=1.1, lw=1.4,
                    color=COL[d], label='forward' if d == 'fwd' else 'reverse')
        ev = [e for e in events if e['direction'] == d]
        if ev:
            spd = {r['run']: float(r['engaged_output_rpm']) for r in rows}
            a3.plot([spd[e['run']] for e in ev], [float(e['slip_output_deg']) for e in ev], 'o', ms=4, alpha=0.6,
                    color=COL[d], mec='none', label='forward' if d == 'fwd' else 'reverse')
    a2.set_xlabel('engaged output speed (rpm)')
    a2.set_ylabel('slips per second')
    a2.set_title('Slip rate vs speed', loc='left', fontsize=10, fontweight='bold', color=INK)
    a2.legend(fontsize=8, frameon=False)
    a3.set_xlabel('engaged output speed (rpm)')
    a3.set_ylabel('slip angle per slip, at the drive pulley (°)')
    a3.set_title('Slip angle per slip vs speed', loc='left', fontsize=10, fontweight='bold', color=INK)
    if cfg.get('pulley_teeth'):
        tooth = 360 / cfg['pulley_teeth']
        a3.axhline(tooth, color=INK2, lw=0.9, ls=':')
        a3.annotate('one tooth (%.0f°)' % tooth, (0.01, tooth), xycoords=('axes fraction', 'data'),
                    xytext=(0, 3), textcoords='offset points', fontsize=7.5, color=INK2)
    if events:
        a3.legend(fontsize=8, frameon=False)
    fig.suptitle('Belt test %s  (%s, %.1f s per run, software PWM %.0f Hz)' % (
        os.path.basename(run.rstrip('/')), 'simulated' if cfg.get('sim') else 'hardware', cfg['time'], cfg['pwm_hz']),
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
