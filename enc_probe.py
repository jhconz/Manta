#!/usr/bin/env python3
"""Encoder probe: is the problem the read circuit, or the decoding?

Watches encoder A and B two independent ways at once and compares them:
  polling    reads both pins directly in a tight loop (no callbacks, no decode table)
  callbacks  lgpio edge callbacks, decoded exactly as belt_speed_test.py does

Motor off by default: turn the motor shaft slowly by hand. With --duty the
motor runs slowly instead (keep it low; nothing is stopping the actuator).

  python3 enc_probe.py                      # turn by hand for 20 s
  python3 enc_probe.py --duty 8 --seconds 3 # motor at 8 %
  python3 enc_probe.py --pins 5 6           # encoder moved to GPIO5 / GPIO6 (pins 29 / 31)

At the end it prints a verdict. Healthy quadrature: both channels switch about
equally often, each is high about half the time, all four AB states turn up,
and the count climbs steadily (48 per motor turn, 1632.67 per output turn).
"""
import argparse, sys, time

PIN_PWM, PIN_DIR = 18, 23
TABLE = [0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0]   # (old << 2) | new; 2 = invalid
PHYS = {17: 11, 27: 13, 22: 15, 5: 29, 6: 31, 13: 33, 19: 35, 26: 37, 16: 36, 20: 38, 21: 40, 24: 18, 25: 22}


def open_chip(lg):
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
            return h
        lg.gpiochip_close(h)
    raise RuntimeError('No Raspberry Pi GPIO chip found')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--duty', type=float, default=0, help='run the motor at this %% (default 0: turn by hand)')
    p.add_argument('--pins', type=int, nargs=2, default=(17, 27), metavar=('A', 'B'), help='BCM pins (default 17 27)')
    a = p.parse_args()
    import lgpio as lg
    pa, pb = a.pins
    h = open_chip(lg)
    lg.gpio_claim_output(h, PIN_PWM, 0)
    lg.gpio_claim_output(h, PIN_DIR, 0)
    lg.gpio_claim_alert(h, pa, lg.BOTH_EDGES)
    lg.gpio_claim_alert(h, pb, lg.BOTH_EDGES)

    # ---- callback path (same decode as belt_speed_test.py)
    cb = dict(a=lg.gpio_read(h, pa), b=lg.gpio_read(h, pb), count=0, bad=0, ea=0, eb=0, other=0, repeat=0)
    cb['state'] = (cb['a'] << 1) | cb['b']

    def edge(chip, gpio, level, tick):
        if level > 1:
            cb['other'] += 1
            return
        if gpio == pa:
            if level == cb['a']:
                cb['repeat'] += 1                    # an "edge" to the level it already had
            cb['a'] = level; cb['ea'] += 1
        else:
            if level == cb['b']:
                cb['repeat'] += 1
            cb['b'] = level; cb['eb'] += 1
        new = (cb['a'] << 1) | cb['b']
        s = TABLE[(cb['state'] << 2) | new]
        if s == 2:
            cb['bad'] += 1
        else:
            cb['count'] += s
        cb['state'] = new
    cbs = [lg.callback(h, pa, lg.BOTH_EDGES, edge), lg.callback(h, pb, lg.BOTH_EDGES, edge)]

    # ---- polling path
    la, lb = lg.gpio_read(h, pa), lg.gpio_read(h, pb)
    st = (la << 1) | lb
    pc = dict(count=0, bad=0, ta=0, tb=0, n=0, ha=0, hb=0)
    states = [0, 0, 0, 0]

    print('A = GPIO%d (pin %s), B = GPIO%d (pin %s).  %s for %.0f s. Ctrl-C to stop early.'
          % (pa, PHYS.get(pa, '?'), pb, PHYS.get(pb, '?'),
             'Motor at %.0f %%' % a.duty if a.duty else 'Motor off: turn the shaft slowly by hand', a.seconds))
    print('           live A B | polling: switches A    B   count | callbacks: edges A    B   count  missed')
    t0 = time.perf_counter()
    nxt = t0
    cb0 = dict(cb)                                    # compare both paths over the same interval
    try:
        if a.duty:
            lg.tx_pwm(h, PIN_PWM, 1000, max(0.0, min(30.0, a.duty)))
        while True:
            now = time.perf_counter()
            if now - t0 > a.seconds:
                break
            va, vb = lg.gpio_read(h, pa), lg.gpio_read(h, pb)
            pc['n'] += 1; pc['ha'] += va; pc['hb'] += vb
            if va != la: pc['ta'] += 1
            if vb != lb: pc['tb'] += 1
            if va != la or vb != lb:
                new = (va << 1) | vb
                s = TABLE[(st << 2) | new]
                if s == 2: pc['bad'] += 1
                else: pc['count'] += s
                st, la, lb = new, va, vb
            states[st] += 1
            if now >= nxt:
                nxt = now + 0.2
                print('\r           %d %d |        %6d %6d %7d |          %6d %6d %7d  %5d   '
                      % (va, vb, pc['ta'], pc['tb'], pc['count'], cb['ea'], cb['eb'], cb['count'], cb['bad']),
                      end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cb1 = dict(cb)
        lg.tx_pwm(h, PIN_PWM, 0, 0)
        lg.gpio_write(h, PIN_PWM, 0)
        time.sleep(0.1)
        for c in cbs:
            c.cancel()
        lg.gpiochip_close(h)
    dt = time.perf_counter() - t0
    n = max(1, pc['n'])
    print('\n')
    print('polling: %d reads (%.0f per s)' % (pc['n'], pc['n'] / dt))
    print('  A switched %d times, high %.0f %% of the time' % (pc['ta'], 100 * pc['ha'] / n))
    print('  B switched %d times, high %.0f %% of the time' % (pc['tb'], 100 * pc['hb'] / n))
    print('  time in AB states 00 / 01 / 10 / 11: %s %%' % ' / '.join('%.0f' % (100 * x / n) for x in states))
    print('  decoded count %+d, invalid jumps %d' % (pc['count'], pc['bad']))
    cd = {k: cb1[k] - cb0[k] for k in ('ea', 'eb', 'count', 'bad', 'repeat', 'other')}
    print('callbacks: A %d edges, B %d edges, count %+d, missed %d, repeated levels %d, watchdog %d'
          % (cd['ea'], cd['eb'], cd['count'], cd['bad'], cd['repeat'], cd['other']))
    print()

    # ---- verdict
    ta, tb = pc['ta'], pc['tb']
    tot = ta + tb
    if tot < 8:
        print('VERDICT: neither pin switched. Shaft not turned, encoder not powered (blue 5 V, green GND), '
              'or both signal paths broken.')
    elif min(ta, tb) < 0.1 * max(ta, tb):
        dead, pin, hi = ('B', pb, pc['hb']) if tb < ta else ('A', pa, pc['ha'])
        lvl = 'HIGH' if hi > n / 2 else 'LOW'
        print('VERDICT: channel %s (GPIO%d, pin %s) is stuck %s at the Pi. The read circuit, not the decoding.'
              % (dead, pin, PHYS.get(pin, '?'), lvl))
        print('  With one channel stuck, the count just flips between 0 and +-1, as in your raw.csv.')
        print('  Next: with the motor off, meter that wire from the encoder to GND while turning the shaft,')
        print('  first at the encoder side of the divider (should swing 0 / 5 V), then at the Pi pin (0 / ~2.5 V).')
        print('  If the Pi pin swings on the meter but this probe still sees it stuck, the GPIO is damaged:')
        print('  move that wire to a spare pin and rerun with --pins, e.g. --pins %s.'
              % ('%d 6' % pa if dead == 'B' else '5 %d' % pb))
    elif states[0] + states[3] > 0.97 * n or states[1] + states[2] > 0.97 * n:
        print('VERDICT: A and B switch together (always equal or always opposite): they are the same signal.')
        print('  Both Pi pins are probably on one encoder wire or one divider. Check the yellow and white paths.')
    elif abs(pc['count']) < 0.2 * tot and a.duty == 0:
        print('VERDICT: both channels switch but the count goes nowhere. Turn in one direction only and retry;')
        print('  if it still goes nowhere, the signals are noisy (loose ground or a marginal divider).')
    elif abs(cd['count'] - pc['count']) > max(8, 0.1 * abs(pc['count'])):
        print('VERDICT: the pins read fine but the callback path disagrees (polling %+d, callbacks %+d).'
              % (pc['count'], cd['count']))
        print('  That is a software problem. Send me this output.')
    else:
        print('VERDICT: encoder and read circuit look healthy; polling and callbacks agree. '
              '%+d counts = %.1f motor turns.' % (pc['count'], pc['count'] / 48))
    return 0


if __name__ == '__main__':
    sys.exit(main())
