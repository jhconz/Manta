#!/usr/bin/env python3
"""Plot a slip_rig.py run folder: output angle vs time for every test, and the
wind-up / slip result per duty. Usage: python3 slip_plot.py slip_runs/<run>
Needs matplotlib (sudo apt install python3-matplotlib)."""
import csv, json, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm

INK, INK2, GRID, RED = '#1f1f1e', '#5f5e58', '#e4e3df', '#c0392b'


def main(run):
    cfg = json.load(open(os.path.join(run, 'config.json')))
    rows = list(csv.DictReader(open(os.path.join(run, 'summary.csv'))))
    raw = {}
    for r in csv.DictReader(open(os.path.join(run, 'raw.csv'))):
        if r['direction'] == 'release':
            continue
        raw.setdefault(int(r['test']), []).append((float(r['t_s']), float(r['output_deg']), float(r['duty_cmd'])))
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.edgecolor': GRID,
                         'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
                         'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6})
    dirs = sorted({r['direction'] for r in rows}, key=lambda d: d != 'fwd')
    fig, axes = plt.subplots(2, len(dirs), figsize=(6.2 * len(dirs), 7.2), squeeze=False)
    ramp = cfg.get('mode', 'step') == 'ramp'
    duties = [float(r['duty_pct']) for r in rows]
    lo, hi = min(duties), max(duties)
    for c, d in enumerate(dirs):
        rs = [r for r in rows if r['direction'] == d]
        ax = axes[0, c]
        for r in rs:
            tr = raw.get(int(r['test']), [])
            if not tr:
                continue
            slipped = r['slipped'] == 'True'
            frac = 0.35 + 0.65 * (float(r['duty_pct']) - lo) / max(1e-9, hi - lo)
            col = RED if slipped else cm.Blues(frac)
            xs = [dc * 100 for _, _, dc in tr] if ramp else [t for t, _, _ in tr]
            ax.plot(xs, [abs(a) for _, a, _ in tr], color=col, lw=1.6 if slipped else 1.1,
                    label=(('test %s: slip at %.1f %%' % (r['test'], float(r['duty_pct']))) if ramp else
                           ('%.0f %% %s' % (float(r['duty_pct']), 'SLIP' if slipped else '')).strip()))
        ax.set_title('%s: output angle during each test' % ('Forward' if d == 'fwd' else 'Reverse'),
                     loc='left', fontsize=10, fontweight='bold', color=INK)
        ax.set_xlabel('commanded duty (%)' if ramp else 'time from test start (s)')
        if ramp:
            ax.set_xlim(cfg['start'] - 2, max(duties) + 4)
        ax.set_ylabel('motor encoder, as output degrees')
        if cfg.get('mode', 'step') == 'step':
            ax.axvline(cfg['ramp_ms'] / 1000 + cfg['settle'], color=INK2, lw=1, ls=':')
            ax.annotate('slip judged from here', (cfg['ramp_ms'] / 1000 + cfg['settle'], ax.get_ylim()[1]),
                        xytext=(4, -10), textcoords='offset points', fontsize=7.5, color=INK2)
        h, l = ax.get_legend_handles_labels()
        if len(h) <= 12:
            ax.legend(h, l, fontsize=7, frameon=False, ncol=2, loc='upper right')

        ax = axes[1, c]
        if ramp:
            ys = [float(r['duty_pct']) for r in rs]
            cols = [RED if r['slipped'] == 'True' else '#2a78d6' for r in rs]
            ax.scatter(range(1, len(ys) + 1), ys, c=cols, s=60, edgecolors='white', linewidths=1.2, zorder=3)
            sl = [y for y, r in zip(ys, rs) if r['slipped'] == 'True']
            if sl:
                m = sum(sl) / len(sl)
                ax.axhline(m, color=RED, lw=1, ls=':')
                tq = sum(float(r['est_output_torque_Nm']) for r in rs if r['slipped'] == 'True') / len(sl)
                ax.annotate('mean %.1f %% (range %.1f–%.1f)\n≈ %.2f N·m at the output (estimate)'
                            % (m, min(sl), max(sl), tq), (0.02, m), xycoords=('axes fraction', 'data'),
                            xytext=(0, 6), textcoords='offset points', fontsize=8, color=RED)
                ax.set_ylim(min(sl) - 3, max(sl) + 3)
            ax.set_xticks(range(1, len(ys) + 1))
            ax.set_xlabel('repeat')
            ax.set_ylabel('duty at slip (%)')
            ax.set_title('Slip duty per repeat (red = slipped, blue = held to the top)', loc='left',
                         fontsize=10, fontweight='bold', color=INK)
            continue
        x = [float(r['duty_pct']) for r in rs]
        y = [abs(float(r['windup_output_deg'])) for r in rs]
        ok = [r['slipped'] != 'True' for r in rs]
        ax.plot([a for a, k in zip(x, ok) if k], [b for b, k in zip(y, ok) if k], 'o', color='#2a78d6',
                ms=7, mec='white', mew=1.2, label='held')
        ax.plot([a for a, k in zip(x, ok) if not k], [b for b, k in zip(y, ok) if not k], 'X', color=RED,
                ms=9, mec='white', mew=1.0, label='slipped')
        first = next((r for r in rs if r['slipped'] == 'True'), None)
        if first:
            ax.axvline(float(first['duty_pct']), color=RED, lw=1, ls=':')
            ax.annotate('first slip %.1f %%\n≈ %.2f N·m at the output (estimate)'
                        % (float(first['duty_pct']), float(first['est_output_torque_Nm'])),
                        (float(first['duty_pct']), 0.04), xycoords=('data', 'axes fraction'),
                        xytext=(6, 0), textcoords='offset points', fontsize=8, color=RED, ha='left')
        ax.set_xlabel('PWM duty (%)')
        ax.set_ylabel('wind-up before slip was judged (output °)')
        ax.set_title('Wind-up and slip per duty', loc='left', fontsize=10, fontweight='bold', color=INK)
        ax.legend(fontsize=8, frameon=False, loc='upper left')
    fig.suptitle('Belt slip test %s  (%s mode, %.1f V, %s)' % (os.path.basename(run.rstrip('/')),
                 cfg.get('mode', 'step'), cfg['volts'], 'simulated' if cfg.get('sim') else 'hardware'),
                 x=0.01, ha='left', fontsize=11, fontweight='bold', color=INK)
    fig.text(0.01, 0.005, 'Torque estimate assumes a stalled motor: average current = duty × V / R '
             '(R %.2f Ω), spec-sheet Kt and gearbox efficiency. Check it against the bench supply ammeter.'
             % cfg['r_ohm'], fontsize=7.5, color=INK2)
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    out = os.path.join(run, 'slip_plot.png')
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
