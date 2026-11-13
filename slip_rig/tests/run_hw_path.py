"""Run slip_rig.main() through the Raspberry Pi code path with stub lgpio and a
fake sysfs PWM tree (real timing, no encoder motion): checks the run completes,
writes its files and leaves the PWM off."""
import os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here); sys.path.insert(0, os.path.join(here, '..'))
from hw_stubs import state, fake_sysfs
state['chips'] = {0: 'pinctrl-rp1'}
root, d = fake_sysfs(4, 'pwmchip2')
sys.argv = ['slip_rig.py', '--duties', '30,60', '--hold', '0.3', '--rest', '0.1', '--yes',
            '--sysfs', root, '--out', '/tmp/claude_hwruns']
import slip_rig
rc = slip_rig.main()
duty, en = open(os.path.join(d, 'duty_cycle')).read(), open(os.path.join(d, 'enable')).read()
print('exit %s, final duty_cycle %s, enable %s' % (rc, duty, en))
sys.exit(0 if rc == 0 and duty == '0' and en == '0' else 1)
