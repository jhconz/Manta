"""Stand-ins for lgpio and /sys/class/pwm, so the Raspberry Pi code path
can run anywhere. Import this before slip_rig."""
import os, sys, tempfile, types

# ---- stub lgpio
lg = types.ModuleType('lgpio')
lg.BOTH_EDGES, lg.SET_PULL_UP = 2, 32
state = {'levels': {}, 'cbs': {}, 'chips': {0: 'pinctrl-rp1', 4: None}, 'claims': {}}
def gpiochip_open(n):
    if n not in state['chips'] or state['chips'][n] is None:
        raise OSError('no chip %d' % n)
    return 100 + n
lg.gpiochip_open = gpiochip_open
lg.gpiochip_close = lambda h: None
lg.gpio_get_chip_info = lambda h: (0, 54, 'gpiochip%d' % (h - 100), state['chips'][h - 100])
lg.gpio_claim_output = lambda h, p, v=0: (state['claims'].__setitem__(p, 'out'), state['levels'].__setitem__(p, v))
lg.gpio_claim_alert = lambda h, p, e, f=0: state['claims'].__setitem__(p, 'alert')
lg.gpio_claim_input = lambda h, p, f=0: (state['claims'].__setitem__(p, 'in'), state['levels'].setdefault(p, 1))
lg.gpio_read = lambda h, p: state['levels'].get(p, 0)
lg.gpio_write = lambda h, p, v: state['levels'].__setitem__(p, v)
class CB:
    def __init__(self, p, f): self.p, self.f = p, f; state['cbs'][p] = self
    def cancel(self): state['cbs'].pop(self.p, None)
lg.callback = lambda h, p, e, f: CB(p, f)
sys.modules['lgpio'] = lg


def fake_sysfs(npwm, chipname):
    root = tempfile.mkdtemp()
    c = os.path.join(root, chipname); os.makedirs(c)
    open(os.path.join(c, 'npwm'), 'w').write(str(npwm))
    open(os.path.join(c, 'export'), 'w').write('')
    ch = 2 if npwm == 4 else 0
    d = os.path.join(c, 'pwm%d' % ch); os.makedirs(d)          # as the kernel creates on export
    for f in ('enable', 'period', 'duty_cycle'):
        open(os.path.join(d, f), 'w').write('0')
    return root, d

def rd(d, f): return int(open(os.path.join(d, f)).read())

