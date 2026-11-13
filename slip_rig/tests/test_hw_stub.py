"""Exercise HardwareRig against a fake /sys/class/pwm tree and a stub lgpio
(no Pi needed): chip / channel detection on Pi 4 and Pi 5 layouts, PWM
writes, DIR, fault input, and x4 quadrature decoding (including missed edges)."""
import os, sys, tempfile, types
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

results = []
def check(name, ok):
    results.append(ok); print(('PASS ' if ok else 'FAIL ') + name)

from hw_stubs import lg, state, CB, fake_sysfs, rd

import slip_rig as S

# Pi 5 layout: RP1 PWM chip with 4 channels (may be pwmchip0 or pwmchip2 depending on kernel)
for chipname in ('pwmchip2', 'pwmchip0'):
    root, d = fake_sysfs(4, chipname)
    rig = S.HardwareRig(sysfs=root)
    check('Pi 5 (%s): picks channel 2 for GPIO18 -> %s' % (chipname, rig.pwm_path), rig.pwm_path == d)
    check('PWM 20 kHz period, enabled, duty 0', rd(d, 'period') == 50000 and rd(d, 'enable') == 1 and rd(d, 'duty_cycle') == 0)
    rig.set_duty(0.6)
    check('duty 60 % -> 30000 ns', rd(d, 'duty_cycle') == 30000)
    rig.set_duty(1.7)
    check('duty clamped to 100 %', rd(d, 'duty_cycle') == 50000)
    rig.close()
    check('close: duty 0, PWM disabled, callbacks cancelled',
          rd(d, 'duty_cycle') == 0 and rd(d, 'enable') == 0 and not state['cbs'])

# older Pi 5 kernels: header on gpiochip4
state['chips'] = {0: 'some-other-chip', 4: 'pinctrl-rp1'}
root2, d2 = fake_sysfs(4, 'pwmchip2')
rig2 = S.HardwareRig(sysfs=root2)
check('older Pi 5 kernel: header found on gpiochip4', rig2.gpiochip == 4)
rig2.close()

# Pi 4 layout: 2-channel chip, header on gpiochip0 labelled pinctrl-bcm2711
state['chips'] = {0: 'pinctrl-bcm2711'}
root, d = fake_sysfs(2, 'pwmchip0')
rig = S.HardwareRig(sysfs=root)
check('Pi 4: pwmchip0 channel 0', rig.pwm_path == d and rig.gpiochip == 0)

# pins
check('pins claimed: DIR out, encoder alerts, FLT input with pull-up',
      state['claims'].get(23) == 'out' and state['claims'].get(17) == 'alert'
      and state['claims'].get(27) == 'alert' and state['claims'].get(22) == 'in')
rig.set_dir(True); a = state['levels'][23]
rig.set_dir(False); b = state['levels'][23]
check('DIR: fwd -> 0, rev -> 1', (a, b) == (0, 1))
state['levels'][22] = 0
check('fault input low -> fault()', rig.fault())
state['levels'][22] = 1

# quadrature: forward sequence 00 -> 01 -> 11 -> 10 -> 00 (A is the high bit)
def edge(pin, level):
    state['levels'][pin] = level
    state['cbs'][pin].f(0, pin, level, 0)
state['levels'][17] = 0; state['levels'][27] = 0
rig._state = 0; rig.count = 0
seq = [(27, 1), (17, 1), (27, 0), (17, 0)]          # one full cycle one way
for _ in range(25):
    for p, l in seq:
        edge(p, l)
c_fwd = rig.count
for _ in range(10):                                  # reverse: 00 -> 10 -> 11 -> 01 -> 00
    for p, l in [(17, 1), (27, 1), (17, 0), (27, 0)]:
        edge(p, l)
check('x4 decoding: 25 cycles one way = %d counts (100), then 10 back -> %d (60)' % (c_fwd, rig.count),
      abs(c_fwd) == 100 and abs(rig.count) == 60 and rig.bad == 0)
# a missed edge: jump 00 -> 11 directly
state['levels'][17] = 0; state['levels'][27] = 0; rig._state = 0
state['levels'][17] = 1; state['levels'][27] = 1
state['cbs'][17].f(0, 17, 1, 0)
check('a missed edge (00 -> 11) is counted as bad, not as motion', rig.bad == 1 and abs(rig.count) == 60)
rig.close()

# --invert-enc flips the sign
state['chips'] = {0: 'pinctrl-bcm2711'}
root3, d3 = fake_sysfs(2, 'pwmchip0')
rig3 = S.HardwareRig(sysfs=root3, invert_enc=True)
state['levels'][17] = 0; state['levels'][27] = 0; rig3._state = 0; rig3.count = 0
for p, l in seq:
    edge(p, l)
check('--invert-enc: the same forward cycle counts %+d (was %+d per cycle)' % (rig3.count, c_fwd // 25),
      rig3.count == -(c_fwd // 25))
rig3.close()

print('%d/%d passed' % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
