"""Run belt_speed_test.py through its Raspberry Pi code path with a stand-in
lgpio: software PWM calls, DIR, fault input, quadrature decoding from callback
levels, the measured-angle prompt, and a clean stop. No Pi needed."""
import os, sys, io, types, threading, time, csv, glob, faulthandler
faulthandler.dump_traceback_later(30, exit=True)   # fail instead of hanging
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..'))
results = []
def check(name, ok):
    results.append(ok); print(('PASS ' if ok else 'FAIL ') + name)

lg = types.ModuleType('lgpio')
lg.BOTH_EDGES, lg.SET_PULL_UP = 2, 32
st = dict(levels={17: 0, 27: 0, 22: 1}, cbs={}, pwm=[], writes=[], claims={})
lg.gpiochip_open = lambda n: 100 + n if n == 0 else (_ for _ in ()).throw(OSError())
lg.gpiochip_close = lambda h: None
lg.gpio_get_chip_info = lambda h: (0, 54, 'gpiochip0', 'pinctrl-bcm2711')
lg.gpio_claim_output = lambda h, p, v=0: st['claims'].__setitem__(p, 'out')
lg.gpio_claim_input = lambda h, p, f=0: st['claims'].__setitem__(p, 'in')
lg.gpio_claim_alert = lambda h, p, e, f=0: st['claims'].__setitem__(p, 'alert')
lg.gpio_read = lambda h, p: st['levels'].get(p, 0)
def gw(h, p, v): st['writes'].append((p, v)); st['levels'][p] = v
lg.gpio_write = gw
lg.tx_pwm = lambda h, p, f, d, *a: st['pwm'].append((p, f, d))
class CB:
    def __init__(self, p, f): self.p, self.f = p, f; st['cbs'][p] = self
    def cancel(self): st['cbs'].pop(self.p, None)
lg.callback = lambda h, p, e, f: CB(p, f)
sys.modules['lgpio'] = lg
import belt_speed_test as B

# encoder feeder: while PWM is running, step the quadrature one way (A leads B)
spin = {'on': False, 'n': 0}
SEQ = [(27, 1), (17, 1), (27, 0), (17, 0)]
def feeder():
    k = 0
    while spin.get('run', True):
        if spin['on'] and 17 in st['cbs'] and 27 in st['cbs']:
            p, l = SEQ[k % 4]; k += 1
            st['levels'][p] = l
            st['cbs'][p].f(0, p, l, 0)
            spin['n'] += 1
            time.sleep(0.0002)
        else:
            time.sleep(0.001)
th = threading.Thread(target=feeder, daemon=True); th.start()
orig = lg.tx_pwm
def tx(h, p, f, d, *a):
    orig(h, p, f, d); spin['on'] = f > 0
lg.tx_pwm = tx
orig_w = lg.gpio_write
def gw2(h, p, v):
    orig_w(h, p, v)
    if p == 18: spin['on'] = bool(v)
lg.gpio_write = gw2

sys.stdin = io.StringIO('\n350\n\n\n')              # Enter, measured 350, Enter, skip
out = '/tmp/claude_belt_pi'
sys.argv = ['belt_speed_test.py', '--duties', '40,100', '--time', '0.5', '--out', out]
rc = B.main()
spin['run'] = False
check('run completed, exit 0', rc == 0)
check('pins: PWM + DIR outputs, encoder alerts, FLT input',
      st['claims'] == {18: 'out', 23: 'out', 17: 'alert', 27: 'alert', 22: 'in'})
duties = sorted({round(d, 1) for p, f, d in st['pwm'] if p == 18 and f > 0})
check('software PWM at 1000 Hz on GPIO18, ramping up to 40 %% (max seen %.1f %%)' % max(d for d in duties if d <= 40.0),
      all(f in (0, 1000) for p, f, d in st['pwm']) and abs(max(d for d in duties if d <= 40.0) - 40.0) < 0.01)
check('100 % duty holds the pin high (no PWM)', (18, 1) in st['writes'])
check('ends with PWM stopped and the pin low', st['pwm'][-1][1] == 0 and [w for w in st['writes'] if w[0] == 18][-1] == (18, 0))
run = sorted(glob.glob(out + '/*'))[-1]
rows = list(csv.DictReader(open(run + '/summary.csv')))
check('summary: 2 runs, encoder angle recorded, no missed edges', len(rows) == 2 and abs(float(rows[0]['encoder_output_deg'])) > 0
      and rows[0]['missed_edges'] == '0')
r0 = rows[0]
enc = abs(float(r0['encoder_output_deg']))
check('measured 350° logged; slip = encoder - measured = %s° (%s %%)' % (r0['slip_deg'], r0['slip_pct']),
      r0['measured_output_deg'] == '350.0' and abs(float(r0['slip_deg']) - (enc - 350)) < 0.02)
check('second run skipped the measurement', rows[1]['measured_output_deg'] == '')
check('raw.csv written with samples for both runs',
      {r['run'] for r in csv.DictReader(open(run + '/raw.csv'))} == {'1', '2'})

# decoding from callback levels: a skipped state counts as a missed edge, not motion
rig = B.PiRig(1000)
rig.count = rig.bad = 0; rig._a = rig._b = 0; rig._state = 0
rig._edge(0, 17, 1, 0); rig._edge(0, 27, 1, 0)          # 00 -> 10 -> 11: one way, 2 counts
c1 = rig.count
rig._edge(0, 17, 0, 0); rig._edge(0, 27, 0, 0)          # 11 -> 01 -> 00: same way, 2 more
rig._edge(0, 17, 1, 0); rig._a = 0
rig._state = 0; rig._a, rig._b = 1, 1
rig._edge(0, 17, 1, 0)                                  # 00 -> 11 in one event: invalid
check('decoding: 4 steps = %d counts, a 00->11 jump = missed edge (%d)' % (abs(c1) * 2, rig.bad),
      abs(c1) == 2 and rig.bad == 1)
rig._edge(0, 17, 2, 0)
check('watchdog callbacks (level 2) are ignored', rig.bad == 1)
rig.close()
print('%d/%d passed' % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
