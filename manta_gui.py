# -*- coding: utf-8 -*-
"""
manta_gui.py - touchscreen controller for the manta Pico, protocol 2.0.

Hardware: Raspberry Pi 4B, 5" DSI 800x480 touchscreen,
          UART to the Pico on /dev/serial0 (GPIO14 TX, GPIO15 RX), 115200 8N1.
Files:    pico_protocol.py, pico_link.py, host_storage.py, numpad.py (optional).
Depends:  pyserial  (pip install pyserial)

Tabs: PRIMARY (active sequence, start, return to zero, ballast descend /
neutral / purge), SEQUENCE (define, run, save to a preset), MANUAL (per-motor
forward / reverse / stop, set zero, ballast jog and set neutral), FILES
(preset library and logs), SETTINGS (PID gains, slip limits, descend depth).
STOP and POWER DOWN are always in the top bar.
"""
import threading
import tkinter as tk
from tkinter import ttk

import pico_protocol as P
from pico_link import PicoLink
from host_storage import PresetLibrary, LogStore, HostSettings

try:
    from numpad import NumPad
except ImportError:                                    # fallback keypad
    class NumPad:
        _open = None

        @classmethod
        def force_close(cls):
            if cls._open is not None and cls._open.winfo_exists():
                cls._open.destroy()
            cls._open = None

        @classmethod
        def edit(cls, root, title, initial, cast, formatter, vmin, vmax, on_commit):
            cls.force_close()
            dlg = tk.Toplevel(root)
            cls._open = dlg
            dlg.title(title)
            dlg.configure(bg='#34495e')
            dlg.transient(root)
            dlg.geometry('300x340+250+70')
            var = tk.StringVar(value=formatter.format(initial))
            tk.Label(dlg, text='%s  (%s to %s)' % (title, vmin, vmax), bg='#34495e', fg='white',
                     font=('DejaVu Sans', 11, 'bold')).pack(pady=4)
            tk.Entry(dlg, textvariable=var, font=('DejaVu Sans', 18), justify='right',
                     width=12).pack(pady=4)
            pad = tk.Frame(dlg, bg='#34495e')
            pad.pack()
            keys = ['7', '8', '9', '4', '5', '6', '1', '2', '3', '-', '0', '.']
            for k, key in enumerate(keys):
                tk.Button(pad, text=key, width=4, height=1, font=('DejaVu Sans', 14),
                          command=lambda c=key: var.set(var.get() + c)).grid(row=k // 3, column=k % 3)
            row = tk.Frame(dlg, bg='#34495e')
            row.pack(pady=6)

            def commit():
                try:
                    v = cast(float(var.get())) if cast is int else cast(var.get())
                except ValueError:
                    return
                on_commit(max(vmin, min(vmax, v)))
                cls.force_close()
            tk.Button(row, text='⌫', width=4, font=('DejaVu Sans', 13),
                      command=lambda: var.set(var.get()[:-1])).pack(side='left', padx=3)
            tk.Button(row, text='CLEAR', width=5, font=('DejaVu Sans', 13),
                      command=lambda: var.set('')).pack(side='left', padx=3)
            tk.Button(row, text='OK', width=5, font=('DejaVu Sans', 13, 'bold'),
                      command=commit).pack(side='left', padx=3)


BG, BAR, PANEL, TEXT, DIM = '#2c3e50', '#34495e', '#22313f', 'white', '#bdc3c7'
GREEN, ORANGE, RED, BLUE, GREY = '#2ecc71', '#f39c12', '#e74c3c', '#2980b9', '#7f8c8d'
YELLOW = '#f1c40f'
MOTORS = ('M1', 'M2', 'M3', 'M4')
CPR = 48 * 34.014                     # encoder counts per output revolution


def mask_names(mask):
    return ', '.join(MOTORS[i] for i in range(4) if mask & (1 << i)) or 'none'


def motor_name(i):
    return 'Ballast' if i == P.MANUAL_BALLAST else MOTORS[i]


def pct(raw):
    return raw / 65535 * 100


class MantaGUI:
    W, H = 800, 480
    BAT = {'fine': ('BATTERY: FINE', GREEN), 'low': ('BATTERY: LOW', ORANGE),
           'critical': ('BATTERY: CRITICAL', RED)}

    # Ballast and return-to-zero tasks run on the Pico's motor thread and end
    # with an EVENT. If none arrives (link lost) the lockout clears after this.
    TASK_TIMEOUT_S = {'purge': 70, 'goto': 70, 'rtz': 25}
    TASK_LABELS = {'purge': 'Purging the ballast', 'goto': 'Moving the ballast',
                   'rtz': 'Returning motors to zero'}

    def __init__(self, root, link=None, library=None, logs=None, settings=None):
        self.root = root
        self.link = link
        self.lib = library or PresetLibrary()
        self.logs = logs or LogStore()
        self.settings = settings or HostSettings()

        self.seq_running = False
        self.awaiting_log = False
        self.shutting_down = False
        self.positions = [0, 0, 0, 0]
        self.zero_known = 0
        self.manual_running = {}          # motor index -> MANUAL_FORWARD / MANUAL_REVERSE
        self.pico_sync = None             # (count, crc) last reported by the Pico
        self.wiper = None                 # ballast wiper raw, last reported
        self.neutral = None               # stored neutral raw; None = not set on the Pico
        self.task = None                  # 'purge' / 'goto' / 'rtz' while one runs
        self.duty_limit = 80.0            # drive PWM limit %, as last read from the Pico
        self._task_serial = 0
        self._dialogs = {}

        root.title('Manta Controller')
        root.geometry('%dx%d' % (self.W, self.H))
        root.configure(bg=BG)
        # root.attributes('-fullscreen', True)

        self._styles()
        self._top_bar()
        self._status_bar()
        self.nb = ttk.Notebook(root)
        self.nb.pack(fill='both', expand=True, padx=6, pady=(4, 2))
        self.tab_main = PrimaryTab(self.nb, self)
        self.tab_seq = SequenceTab(self.nb, self)
        self.tab_man = ManualTab(self.nb, self)
        self.tab_files = FilesTab(self.nb, self)
        self.tab_set = SettingsTab(self.nb, self)
        for tab, text in ((self.tab_main, 'PRIMARY'), (self.tab_seq, 'SEQUENCE'),
                          (self.tab_man, 'MANUAL'), (self.tab_files, 'FILES'),
                          (self.tab_set, 'SETTINGS')):
            self.nb.add(tab.frame, text=text)

        if self.link:
            self.link.on_message = lambda m: self.root.after(0, self._handle, m)
            self.root.after(200, self._startup)
            self.root.after(1000, self._tick)
        else:
            self.set_link(False, 'NO SERIAL')

    # ================================================================ layout
    def _styles(self):
        s = ttk.Style()
        s.theme_use('default')
        s.configure('TFrame', background=BG)
        s.configure('TLabel', background=BG, foreground=TEXT, font=('DejaVu Sans', 11))
        s.configure('Head.TLabel', font=('DejaVu Sans', 11, 'bold'), foreground=DIM)
        s.configure('Big.TLabel', font=('DejaVu Sans', 13, 'bold'))
        s.configure('TButton', font=('DejaVu Sans', 11, 'bold'), padding=6)
        s.configure('Go.TButton', background=GREEN, foreground='white')
        s.configure('Stop.TButton', background=RED, foreground='white',
                    font=('DejaVu Sans', 13, 'bold'), padding=6)
        s.configure('Power.TButton', background=GREY, foreground='white')
        s.configure('Active.TButton', background=BLUE, foreground='white')
        s.configure('Dialog.TButton', font=('DejaVu Sans', 14, 'bold'), padding=10)
        s.configure('Abort.TButton', font=('DejaVu Sans', 14, 'bold'), padding=10,
                    background=RED, foreground='white')
        s.configure('TNotebook', background=BG, borderwidth=0)
        s.configure('Panel.TFrame', background=PANEL)
        s.configure('Panel.TLabel', background=PANEL)
        s.configure('PanelHead.TLabel', background=PANEL, font=('DejaVu Sans', 11, 'bold'),
                    foreground=DIM)
        s.configure('Big.TButton', font=('DejaVu Sans', 13, 'bold'), padding=(8, 12))
        s.configure('BigNarrow.TButton', font=('DejaVu Sans', 12, 'bold'), padding=(2, 12))
        s.configure('BigGo.TButton', font=('DejaVu Sans', 15, 'bold'), padding=(8, 16),
                    background=GREEN, foreground='white')
        s.configure('TNotebook.Tab', font=('DejaVu Sans', 12, 'bold'), padding=(16, 6),
                    background=BAR, foreground=DIM)
        s.map('TNotebook.Tab', background=[('selected', BLUE)], foreground=[('selected', 'white')])
        s.configure('Treeview', font=('DejaVu Sans', 11), rowheight=28,
                    background=PANEL, fieldbackground=PANEL, foreground=TEXT)
        s.configure('Treeview.Heading', font=('DejaVu Sans', 10, 'bold'))
        s.map('Treeview', background=[('selected', BLUE)])

    def _top_bar(self):
        bar = tk.Frame(self.root, bg=BAR, height=48)
        bar.pack(fill='x', side='top')
        bar.pack_propagate(False)
        tk.Label(bar, text='MANTA', bg=BAR, fg='white',
                 font=('DejaVu Sans', 16, 'bold')).pack(side='left', padx=(14, 8))
        self.link_label = tk.Label(bar, text='LINK …', bg=GREY, fg='white',
                                   font=('DejaVu Sans', 10, 'bold'), padx=8, pady=4)
        self.link_label.pack(side='left', padx=4)
        self.bat_label = tk.Label(bar, text=self.BAT['fine'][0], bg=self.BAT['fine'][1], fg='white',
                                  font=('DejaVu Sans', 10, 'bold'), padx=8, pady=4)
        self.bat_label.pack(side='left', padx=4)
        ttk.Button(bar, text='POWER DOWN', style='Power.TButton',
                   command=self.on_powerdown).pack(side='right', padx=(4, 10), pady=6)
        ttk.Button(bar, text='■  STOP', style='Stop.TButton', width=9,
                   command=self.on_stop).pack(side='right', padx=4, pady=4)

    def _status_bar(self):
        self.status_var = tk.StringVar(value='Ready.')
        bar = tk.Frame(self.root, bg='#1a252f', height=24)
        bar.pack(fill='x', side='bottom')
        bar.pack_propagate(False)
        tk.Label(bar, textvariable=self.status_var, bg='#1a252f', fg=DIM,
                 font=('DejaVu Sans', 10), anchor='w').pack(fill='x', padx=10)

    # ================================================================ helpers
    def status(self, msg):
        self.status_var.set(msg)

    def set_link(self, ok, text=None):
        self.link_label.configure(text=text or ('LINK OK' if ok else 'NO LINK'),
                                  bg=GREEN if ok else RED)

    def set_battery(self, level):
        text, color = self.BAT[level]
        self.bat_label.configure(text=text, bg=color)

    def busy(self):
        """True while the Pico can't take motion commands from this GUI."""
        return self.seq_running or self.awaiting_log or self.shutting_down or self.task is not None

    def current_tab(self):
        return self.nb.nametowidget(self.nb.select()) if self.nb.select() else None

    def show_tab(self, tab):
        self.nb.select(tab.frame)

    def duty_limit_changed(self, pct):
        """The Pico's drive PWM limit is now pct %: clamp the editors to it."""
        self.duty_limit = pct
        self.tab_seq.apply_duty_limit()
        self.tab_man.apply_duty_limit()
        self.tab_set.show_duty_limit()

    def ballast_changed(self):
        self.tab_man.refresh_ballast()
        self.tab_main.refresh_ballast()

    def request(self, frame, done=P.wait_done, ok=None, fail=None, timeout=1.0,
                ack_grace=None, quiet=False):
        """Queue a command; ok(result) / fail(result) run on the GUI thread."""
        if not self.link:
            self.status('No link to the Pico.')
            return

        def finished(res):
            self.root.after(0, self._finished, res, ok, fail, quiet)
        self.link.submit(frame, done, timeout, callback=finished, ack_grace=ack_grace)

    def _finished(self, res, ok, fail, quiet):
        if res.ok:
            if ok:
                ok(res)
            return
        if res.error == 'timeout':
            self.set_link(False)
        if fail:
            fail(res)
        elif not quiet:
            self.status('%s: %s' % (res.name, res.error_text()))

    def dialog(self, key, title, message, color=BAR, buttons=None, modal=True, auto_close_ms=None):
        """Centered popup. key: one dialog per key (an older one is replaced)."""
        self.close_dialog(key)
        dlg = tk.Toplevel(self.root)
        dlg.title(title)
        dlg.configure(bg=color)
        dlg.transient(self.root)
        dlg.protocol('WM_DELETE_WINDOW', lambda: None)
        w, h = 520, 230
        dlg.geometry('%dx%d+%d+%d' % (w, h, (self.W - w) // 2, (self.H - h) // 2))
        dlg.resizable(False, False)
        tk.Label(dlg, text=title, bg=color, fg='white',
                 font=('DejaVu Sans', 16, 'bold')).pack(pady=(14, 4))
        msg = tk.Label(dlg, text=message, bg=color, fg='white', font=('DejaVu Sans', 12),
                       wraplength=480, justify='center')
        msg.pack(pady=6, padx=20)
        dlg.msg = msg
        row = tk.Frame(dlg, bg=color)
        row.pack(side='bottom', fill='x', padx=14, pady=12)
        for label, style, cb in (buttons if buttons is not None
                                 else [('OK', 'Dialog.TButton', lambda: self.close_dialog(key))]):
            ttk.Button(row, text=label, style=style, command=cb).pack(
                side='left', expand=True, fill='both', padx=4)
        if modal:
            dlg.update_idletasks()
            try:
                dlg.grab_set()
            except tk.TclError:
                pass
        if auto_close_ms:
            self.root.after(auto_close_ms, lambda: self.close_dialog(key, only=dlg))
        self._dialogs[key] = dlg
        return dlg

    def close_dialog(self, key, only=None):
        dlg = self._dialogs.get(key)
        if dlg is not None and (only is None or only is dlg):
            if dlg.winfo_exists():
                dlg.destroy()
            self._dialogs.pop(key, None)

    def update_dialog(self, key, message):
        dlg = self._dialogs.get(key)
        if dlg is not None and dlg.winfo_exists():
            dlg.msg.configure(text=message)

    def confirm(self, title, message, on_yes, color=BAR):
        def yes():
            self.close_dialog('confirm')
            on_yes()
        self.dialog('confirm', title, message, color,
                    [('CANCEL', 'Dialog.TButton', lambda: self.close_dialog('confirm')),
                     ('YES', 'Dialog.TButton', yes)])

    # ================================================================ startup / timers
    def _startup(self):
        def pong(res):
            f = res.first(P.ST_PONG)
            ver = (f.i1, f.i2)
            self.set_link(True, 'LINK OK  v%d.%d' % ver)
            if ver[0] != P.PROTOCOL_VERSION[0]:
                self.dialog('ver', 'Protocol mismatch',
                            'The Pico speaks protocol %d.%d; this controller expects %d.%d.\n'
                            'Update one side before running anything.' % (ver + P.PROTOCOL_VERSION), RED)
            self.check_sync(quiet=True)
            self.refresh_positions()
            self.query_neutral()
            self.tab_set.read_all(quiet=True)
        self.request(P.ping(), P.wait_pong, ok=pong,
                     fail=lambda r: (self.set_link(False), self.status('The Pico is not answering.')))

    def _tick(self):
        """Every second: manual keepalive, position polling, link check."""
        try:
            if self.link and not self.shutting_down:
                self.tab_man.keepalive()
                if (self.current_tab() in (self.tab_man.frame, self.tab_main.frame)
                        and not self.busy() and self.link.queue_length() < 2):
                    self.refresh_positions(quiet=True)
                self._ping_count = getattr(self, '_ping_count', 0) + 1
                if self._ping_count % 5 == 0 and not self.busy() and self.link.queue_length() == 0:
                    self.request(P.ping(), P.wait_pong, quiet=True,
                                 ok=lambda r: self.set_link(True, 'LINK OK  v%d.%d' % (
                                     r.first(P.ST_PONG).i1, r.first(P.ST_PONG).i2)))
        finally:
            self.root.after(1000, self._tick)

    def refresh_positions(self, quiet=False):
        """Positions, wiper and zero mask. The F6 / F8 frames also pass
        through _handle, which stores them; this adds the zero mask."""
        def ok(res):
            self.zero_known = res.first(P.ST_DONE).i1
            self.tab_man.refresh()
            self.tab_main.refresh_zero()
        self.request(P.get_positions(), ok=ok, quiet=quiet)

    def query_neutral(self):
        def ok(res):
            d = res.first(P.ST_DONE)
            self.neutral = (d.i1 << 8) | d.i2
            self.ballast_changed()

        def fail(res):
            if isinstance(res.error, P.Frame) and res.error.i1 == 0x0A:   # none stored yet
                self.neutral = None
                self.ballast_changed()
        self.request(P.ballast_neutral(2), ok=ok, fail=fail, quiet=True)

    # ================================================================ ballast / RTZ tasks
    def start_task(self, kind, frame, detail=''):
        """Start a Pico motor-thread task (purge / goto / rtz). The ACK alone
        means it started; its EVENT ends it. Controls lock meanwhile."""
        if self.busy():
            self.status('Busy: wait for the current task to finish.')
            return
        if self.manual_running:
            self.status('Stop the manual motors first.')
            return
        label = self.TASK_LABELS[kind]

        def started(res):
            self.task = kind
            self._task_serial += 1
            serial = self._task_serial
            self.status('%s%s…' % (label, detail))
            self.dialog('lockout', label, '%s%s.\n\nControls are locked until it finishes.'
                        % (label, detail), BLUE, [('ABORT', 'Abort.TButton', self.on_stop)])
            self.root.after(self.TASK_TIMEOUT_S[kind] * 1000, lambda: self._task_timeout(serial))
        self.request(frame, done=None, ack_grace=0.15, ok=started,
                     fail=lambda r: self.status('%s: %s' % (label, r.error_text())))

    def _task_done(self):
        self.task = None
        self.close_dialog('lockout')
        self.refresh_positions(quiet=True)

    def _task_timeout(self, serial):
        if self.task and serial == self._task_serial:
            self.status('%s: no reply from the Pico. Controls unlocked.' % self.TASK_LABELS[self.task])
            self.task = None
            self.close_dialog('lockout')

    def check_sync(self, quiet=False):
        def ok(res):
            f = res.first(P.ST_PRESET_SYNC)
            self.pico_sync = (f.i1, P.u16(f))
            self.tab_files.show_sync()
        self.request(P.preset_sync(), P.wait_sync, ok=ok, quiet=quiet)

    # ================================================================ top-bar actions
    def on_stop(self):
        NumPad.force_close()
        if self.link:
            self.link.send_now(P.stop())
        self.manual_running.clear()
        self.tab_man.refresh()
        if self.task:
            self.update_dialog('lockout', 'STOP sent. Waiting for the Pico to confirm…')
        self.status('STOP sent.')

    def on_powerdown(self):
        if not self.link or self.shutting_down:
            return
        self.confirm('Power down',
                     'Return the motors to zero, purge the ballast and put the Pico to sleep?',
                     self._powerdown, GREY)

    def _powerdown(self):
        self.shutting_down = True
        self.task = None
        self.close_dialog('lockout')
        self.manual_running.clear()
        self.dialog('power', 'Shutting down',
                    'Returning motors to zero…', GREY, buttons=[])

        def ok(res):
            self.shutting_down = False
            rtz = [m for m in res.frames(P.ST_EVENT) if m.i1 == P.EV_RTZ]
            note = ''
            if rtz and rtz[0].i3:
                note = '\n%s did not reach zero (%s).' % (mask_names(rtz[0].i3),
                                                         P.RESULTS.get(rtz[0].i2, '?'))
            self.set_link(False, 'PICO ASLEEP')
            self.dialog('power', 'Pico asleep',
                        'Switch off the main power now.\nThe drivers and boost stay powered '
                        'until the switch is off.' + note, GREY)

        def fail(res):
            self.shutting_down = False
            self.dialog('power', 'Power down failed', res.error_text(), RED)
        self.request(P.powerdown(), P.wait_done, ok=ok, fail=fail, timeout=100)

    # ================================================================ messages from the Pico
    def _handle(self, m):
        if isinstance(m, P.LogBlock):
            self._got_log(m)
            return
        st = m.status
        if st == P.ST_BAT_WARN:
            self.set_battery('low')
            self.status('Battery LOW.')
            NumPad.force_close()
            self.dialog('bat', 'Low Battery', 'Low battery: shut down after this run.', ORANGE)
        elif st == P.ST_BAT_CRIT:
            self.set_battery('critical')
            self.status('Battery CRITICAL: emergency shutdown.')
            NumPad.force_close()
            self.close_dialog('lockout')
            self.seq_running = self.awaiting_log = False
            self.task = None
            self.manual_running.clear()
            self.dialog('bat', 'BATTERY CRITICAL',
                        'Emergency shutdown: the Pico is purging the ballast, saving motor '
                        'positions and going to sleep.\nRecharge before next use.', RED)
        elif st == P.ST_EVENT:
            self._event(m)
        elif st == P.ST_BALLAST:
            self.wiper = P.u16(m)
            if not m.i1 & 1:
                self.neutral = None
            self.ballast_changed()
        elif st == P.ST_POSITION:
            i, c, _ = P.position(m)
            self.positions[i] = c
            self.tab_man.refresh()
            if self.shutting_down:
                self.update_dialog('power', 'Positions reported. Purging the ballast…')
        elif st == P.ST_BOOT:
            self.zero_known = m.i2
            self.tab_man.refresh()
            if m.i2 != 0x0F:
                self.dialog('boot', 'Zero unknown',
                            'Last shutdown: %s.\nZero is unknown for %s: those motors were not '
                            'moved.\nUse the MANUAL tab to line them up, then SET ZERO.'
                            % (P.SHUTDOWN_TYPES.get(m.i1, '?'), mask_names(0x0F & ~m.i2)), ORANGE)
            self.tab_main.refresh_zero()
        elif st == P.ST_PRESET_SYNC:
            self.pico_sync = (m.i1, P.u16(m))
            self.tab_files.show_sync()
        elif st == P.ST_ERROR:
            if m.i1 == P.ERR_DRIVER_FAULT:
                self.manual_running.clear()
                self.tab_man.refresh()
                bits = m.i2
                which = ', '.join(n for b, n in ((1, 'M1'), (2, 'M2'), (4, 'M3 or M4'),
                                                 (0x80, 'fault pins unreadable')) if bits & b)
                self.dialog('fault', 'DRIVER FAULT',
                            'All motors stopped.\nFault: %s.' % which, RED)
            elif m.i1 == P.ERR_SLIP:
                self.status('Slip protection cut %s.' % mask_names(m.i2))

    def _event(self, m):
        tag = m.i1
        if tag == P.EV_SEQUENCE:
            self.seq_running = False
            self.awaiting_log = True
            self.update_dialog('lockout', 'Sequence finished. Receiving the log…')
            self.status('Sequence finished. Receiving the log…')
            self.root.after(15000, self._log_timeout)
        elif tag == P.EV_RTZ:
            if m.i3:
                self.status('Return to zero: %s not at zero (%s).'
                            % (mask_names(m.i3), P.RESULTS.get(m.i2, '?')))
            else:
                self.status('All motors returned to zero.')
            if self.shutting_down:
                self.update_dialog('power', 'Motors at zero. Reporting positions…')
            elif self.task == 'rtz':
                self._task_done()
        elif tag in (P.EV_PURGE, P.EV_GOTO):
            what = 'Purge' if tag == P.EV_PURGE else 'Ballast move'
            self.status('%s finished: %s.' % (what, P.RESULTS.get(m.i2, '?')))
            self.wiper = m.i3 << 8                     # high byte only; refreshed below
            self.ballast_changed()
            if self.shutting_down:
                self.update_dialog('power', 'Ballast purged (%s). Going to sleep…'
                                   % P.RESULTS.get(m.i2, '?'))
            elif self.task in ('purge', 'goto'):
                self._task_done()
        elif tag == P.EV_MANUAL:
            self.manual_running.pop(m.i3, None)
            self.tab_man.refresh()
            self.status('%s stopped: %s.' % (motor_name(m.i3), P.MANUAL_END.get(m.i2, '?')))

    def _got_log(self, block):
        self.awaiting_log = False
        self.close_dialog('lockout')
        if not block.crc_ok:
            self.status('A sequence log arrived damaged (CRC mismatch). Use FILES > FETCH LAST.')
            return
        try:
            name = self.logs.save(block.raw)
            hdr, _ = P.decode_log(block.raw)
            self.status('Log saved: %s (%d samples, %s).'
                        % (name, hdr['n_samples'], P.flags_text(hdr['flags'])))
        except (OSError, ValueError) as e:
            self.status('Could not save the log: %s' % e)
        self.tab_files.refresh_logs()

    def _log_timeout(self):
        if self.awaiting_log:
            self.awaiting_log = False
            self.close_dialog('lockout')
            self.status('No log arrived. Use FILES > FETCH LAST.')

    # ================================================================ running sequences
    def run_sequence(self, preset_id, cycles, label):
        """Every run goes through a preset id (0xFE = the temporary slot)."""
        def started(res):
            self.seq_running = True
            self.manual_running.clear()
            self.tab_man.refresh()
            self.status('Running %s, %d cycle(s)…' % (label, cycles))
            self.dialog('lockout', 'Sequence running',
                        '%s, %d cycle(s).\n\nControls are locked until it finishes.' % (label, cycles),
                        BLUE, [('ABORT', 'Abort.TButton', self.on_stop)])
        self.request(P.preset_run(preset_id, cycles), done=None, ack_grace=0.15, ok=started,
                     fail=lambda r: self.status('Could not start: %s' % r.error_text()))


# ===================================================================== tabs
class _Tab:
    def __init__(self, nb, app):
        self.app = app
        self.frame = ttk.Frame(nb)

    def num_entry(self, parent, var, title, cast, fmt, vmin, vmax, width=8):
        e = tk.Entry(parent, textvariable=var, font=('DejaVu Sans', 13), width=width,
                     justify='right', readonlybackground='white', state='readonly')
        e.bind('<Button-1>', lambda ev: self._numpad(var, title, cast, fmt, vmin, vmax))
        return e

    def _numpad(self, var, title, cast, fmt, vmin, vmax):
        if self.app.busy():
            return
        if callable(vmax):                  # e.g. the drive PWM limit, which can change
            vmax = vmax()
        try:
            cur = cast(float(var.get()))
        except ValueError:
            cur = vmin
        NumPad.edit(self.app.root, title=title, initial=cur, cast=cast, formatter=fmt,
                    vmin=vmin, vmax=vmax, on_commit=lambda v: var.set(fmt.format(v)))


class PrimaryTab(_Tab):
    """Day-to-day operation: pick the active sequence and start it, return
    the motors to zero, and move the ballast (descend / neutral / purge)."""
    GAUGE_W, GAUGE_H = 350, 44
    RIGHT_W = 384

    def __init__(self, nb, app):
        super().__init__(nb, app)
        f = self.frame
        right = ttk.Frame(f, style='Panel.TFrame', width=self.RIGHT_W)   # fixed width
        right.pack(side='right', fill='y', padx=(5, 10), pady=8)
        right.pack_propagate(False)
        left = ttk.Frame(f, style='Panel.TFrame')
        left.pack(side='left', fill='both', expand=True, padx=(10, 5), pady=8)

        # ---- active sequence
        ttk.Label(left, text='ACTIVE SEQUENCE  (tap a preset)', style='PanelHead.TLabel').pack(
            anchor='w', padx=10, pady=(8, 2))
        self.tree = ttk.Treeview(left, columns=('slot', 'name', 'period', 'cycles'),
                                 show='headings', height=4, selectmode='browse')
        for col, text, w in (('slot', 'Slot', 44), ('name', 'Name', 130), ('period', 'Period', 66),
                             ('cycles', 'Cycles', 58)):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=w, anchor='w' if col == 'name' else 'center')
        self.tree.pack(fill='both', expand=True, padx=10, pady=2)
        self.tree.bind('<<TreeviewSelect>>', self.on_select)
        row = ttk.Frame(left, style='Panel.TFrame')
        row.pack(fill='x', padx=10, pady=(6, 2))
        self.active_label = ttk.Label(row, text='Active: none', style='Panel.TLabel',
                                      font=('DejaVu Sans', 13, 'bold'))
        self.active_label.pack(side='left')
        self.cycles = tk.StringVar(value=str(app.settings['active_cycles']))
        self.num_entry(row, self.cycles, 'Cycles', int, '{:d}', 1, 1000, 5).pack(side='right')
        ttk.Label(row, text='Cycles', style='Panel.TLabel').pack(side='right', padx=6)
        self.cycles.trace_add('write', lambda *a: self._save_cycles())
        ttk.Button(left, text='▶  START SEQUENCE', style='BigGo.TButton',
                   command=self.on_start).pack(fill='x', padx=10, pady=(6, 10))

        # ---- motors
        ttk.Label(right, text='DRIVE MOTORS', style='PanelHead.TLabel').pack(anchor='w', padx=10, pady=(8, 2))
        zrow = ttk.Frame(right, style='Panel.TFrame')
        zrow.pack(fill='x', padx=10)
        self.zero_badges = []
        for name in MOTORS:
            lab = tk.Label(zrow, text=name + ' ?', width=7, font=('DejaVu Sans', 10, 'bold'),
                           fg='white', bg=GREY, pady=3)
            lab.pack(side='left', padx=2, expand=True, fill='x')
            self.zero_badges.append(lab)
        ttk.Button(right, text='⟲  RETURN TO ZERO', style='Big.TButton',
                   command=self.on_rtz).pack(fill='x', padx=10, pady=(6, 8))

        # ---- ballast
        ttk.Label(right, text='BALLAST', style='PanelHead.TLabel').pack(anchor='w', padx=10)
        self.gauge = tk.Canvas(right, width=self.GAUGE_W, height=self.GAUGE_H, bg=PANEL,
                               highlightthickness=0)
        self.gauge.pack(padx=10, pady=(2, 0))
        self.ballast_label = ttk.Label(right, text='', style='Panel.TLabel', foreground=DIM,
                                       font=('DejaVu Sans', 10))
        self.ballast_label.pack(anchor='w', padx=10)
        brow = ttk.Frame(right, style='Panel.TFrame')
        brow.pack(fill='x', padx=10, pady=(4, 10))
        for text, cmd in (('▼ DESCEND', self.on_descend), ('NEUTRAL', self.on_neutral),
                          ('▲ PURGE', self.on_purge)):
            ttk.Button(brow, text=text, style='BigNarrow.TButton', command=cmd).pack(
                side='left', expand=True, fill='x', padx=2)
        self.refresh_presets()
        self.refresh_zero()
        self.refresh_ballast()

    # ---- active sequence
    def refresh_presets(self):
        self.tree.delete(*self.tree.get_children())
        for slot, e in sorted(self.app.lib.presets.items()):
            self.tree.insert('', 'end', iid=str(slot), values=(
                slot, e['name'], '%.2f s' % e['period'], e.get('cycles', 1)))
        active = self.app.settings['active_preset']
        e = self.app.lib.presets.get(active) if active is not None else None
        if e is None:
            self.active_label.configure(text='Active: none')
        else:
            self.active_label.configure(text='Active: %d · %s' % (active, e['name'][:14]))
            if self.tree.selection() != (str(active),):
                self.tree.selection_set(str(active))
                self.tree.see(str(active))

    def on_select(self, _ev=None):
        sel = self.tree.selection()
        if not sel:
            return
        slot = int(sel[0])
        e = self.app.lib.presets.get(slot)
        if e is None or slot == self.app.settings['active_preset']:
            return
        self.app.settings['active_preset'] = slot
        self.cycles.set(str(e.get('cycles', 1)))
        self.active_label.configure(text='Active: %d · %s' % (slot, e['name'][:14]))
        self.app.status('Active sequence: "%s".' % e['name'])

    def _save_cycles(self):
        try:
            self.app.settings['active_cycles'] = max(1, int(self.cycles.get()))
        except ValueError:
            pass

    def on_start(self):
        if self.app.busy():
            return
        slot = self.app.settings['active_preset']
        e = self.app.lib.presets.get(slot) if slot is not None else None
        if e is None:
            self.app.status('Pick the active sequence first.')
            return
        cycles = self.app.settings['active_cycles']
        over = [MOTORS[i] for i, (d, _, _) in enumerate(e['motors']) if d / 65535 * 100 > self.app.duty_limit + 0.05]

        def go():
            self.app.run_sequence(slot, cycles, '"%s"' % e['name'])
            if over:
                self.app.status('"%s": %s above the %.0f %% PWM limit run at the limit.'
                                % (e['name'], ', '.join(over), self.app.duty_limit))
        sync = self.app.pico_sync
        if sync is not None and (sync[1] != self.app.lib.crc() or sync[0] != len(self.app.lib.presets)):
            self.app.confirm('Presets out of sync',
                             'The Pico\'s presets differ from this controller\'s, so slot %d may '
                             'not hold "%s" there.\nRun anyway? (FILES > PUSH ALL fixes it.)'
                             % (slot, e['name']), go, ORANGE)
        else:
            go()

    # ---- motors
    def refresh_zero(self):
        for i, lab in enumerate(self.zero_badges):
            known = self.app.zero_known & (1 << i)
            lab.configure(text='%s %s' % (MOTORS[i], '✓' if known else '?'),
                          bg=GREEN if known else ORANGE)

    def on_rtz(self):
        if self.app.busy():
            return
        unknown = 0x0F & ~self.app.zero_known
        go = lambda: self.app.start_task('rtz', P.return_to_zero())
        if unknown:
            self.app.confirm('Return to zero',
                             'Zero is unknown for %s, so %s will not move.\n'
                             'Return the others to zero?' % (mask_names(unknown),
                                                             'it' if unknown in (1, 2, 4, 8) else 'they'),
                             go, ORANGE)
        else:
            go()

    # ---- ballast
    def descend_target(self):
        if self.app.neutral is None:
            return None
        return min(P.BALLAST_FULL_RAW,
                   int(self.app.neutral + self.app.settings['descend_pct'] / 100 * 65535))

    def refresh_ballast(self):
        g, W, H = self.gauge, self.GAUGE_W, self.GAUGE_H
        g.delete('all')
        x0, x1, y0, y1 = 4, W - 4, 6, 26
        xs = lambda raw: x0 + (x1 - x0) * raw / 65535
        g.create_rectangle(x0, y0, x1, y1, fill=BG, outline=GREY)
        g.create_rectangle(x0, y0, xs(P.BALLAST_EMPTY_RAW), y1, fill=GREY, outline='')
        g.create_rectangle(xs(P.BALLAST_FULL_RAW), y0, x1, y1, fill=GREY, outline='')
        w, n, d = self.app.wiper, self.app.neutral, self.descend_target()
        if w is not None:
            g.create_rectangle(x0, y0, xs(w), y1, fill=BLUE, outline='')
        for raw, color, tag in ((n, YELLOW, 'N'), (d, ORANGE, 'D')):
            if raw is not None:
                x = xs(raw)
                g.create_line(x, y0 - 4, x, y1 + 4, fill=color, width=3)
                g.create_text(x, y1 + 11, text=tag, fill=color, font=('DejaVu Sans', 9, 'bold'))
        g.create_text(x0, y1 + 11, text='empty', fill=DIM, anchor='w', font=('DejaVu Sans', 8))
        g.create_text(x1, y1 + 11, text='full', fill=DIM, anchor='e', font=('DejaVu Sans', 8))
        parts = ['now ' + ('—' if w is None else '%.1f %%' % pct(w)),
                 'neutral ' + ('not set' if n is None else '%.1f %%' % pct(n))]
        if d is not None:
            parts.append('descend %.1f %%' % pct(d))
        self.ballast_label.configure(text='   '.join(parts))

    def _need_neutral(self):
        if self.app.neutral is None:
            self.app.status('No neutral point stored: set it on MANUAL > SET NEUTRAL first.')
            return True
        return False

    def on_descend(self):
        if self._need_neutral():
            return
        t = self.descend_target()
        self.app.start_task('goto', P.ballast_goto(1, t), ' to %.1f %% (descend)' % pct(t))

    def on_neutral(self):
        if self._need_neutral():
            return
        self.app.start_task('goto', P.ballast_goto(0), ' to neutral')

    def on_purge(self):
        self.app.start_task('purge', P.ballast_purge())


class SettingsTab(_Tab):
    """Pico settings (read back from the Pico, saved there on APPLY) and
    controller settings (saved here at once)."""
    GAIN = (float, '{:.2f}', 0.0, 655.35)

    def __init__(self, nb, app):
        super().__init__(nb, app)
        f = self.frame
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        bal = self._panel(0, 0, 'BALLAST PID  (wiper position)')
        rtz = self._panel(0, 1, 'RETURN-TO-ZERO PID  (all four motors)')
        slip = self._panel(1, 0, 'SLIP LIMITS  (degrees, 0 = off)')
        ctl = self._panel(1, 1, 'CONTROLLER')
        lim = self._panel(2, 0, 'DRIVE PWM LIMIT  (M1–M4, all motion)', columnspan=2)

        self.bal = self._gain_row(bal, 'Ballast')
        self._buttons(bal, self.read_ballast, self.apply_ballast)
        self.rtz = self._gain_row(rtz, 'Return-to-zero')
        self._buttons(rtz, self.read_rtz, self.apply_rtz)

        g = ttk.Frame(slip, style='Panel.TFrame')
        g.pack(fill='x', padx=10, pady=4)
        self.slip = []
        for i, name in enumerate(MOTORS):
            ttk.Label(g, text=name, style='Panel.TLabel').grid(row=0, column=i, padx=4)
            v = tk.StringVar(value='—')
            self.num_entry(g, v, '%s slip limit (deg)' % name, float, '{:.1f}', 0.0, 6553.5, 6).grid(
                row=1, column=i, padx=4, pady=2)
            self.slip.append(v)
        self._buttons(slip, self.read_slip, self.apply_slip)

        g = ttk.Frame(lim, style='Panel.TFrame')
        g.pack(fill='x', padx=10, pady=(4, 8))
        ttk.Label(g, text='Max duty %', style='Panel.TLabel').pack(side='left')
        self.limit = tk.StringVar(value='—')
        self.num_entry(g, self.limit, 'Drive PWM limit (%)', float, '{:.0f}', 10.0, 100.0, 5).pack(
            side='left', padx=8)
        ttk.Button(g, text='READ', command=self.read_duty).pack(side='left', padx=2)
        ttk.Button(g, text='APPLY', command=self.apply_duty).pack(side='left', padx=2)
        ttk.Label(g, text='Saved on the Pico. 80 % = 5 A budget in water.',
                  style='Panel.TLabel', foreground=DIM, font=('DejaVu Sans', 9)).pack(side='left', padx=10)

        g = ttk.Frame(ctl, style='Panel.TFrame')
        g.pack(fill='x', padx=10, pady=4)
        ttk.Label(g, text='Descend depth %', style='Panel.TLabel').grid(row=0, column=0, sticky='w')
        self.descend = tk.StringVar(value='{:.1f}'.format(app.settings['descend_pct']))
        self.num_entry(g, self.descend, 'Descend depth (% of travel past neutral)', float, '{:.1f}',
                       0.5, 50.0, 6).grid(row=0, column=1, padx=8, pady=2)
        self.descend.trace_add('write', lambda *a: self._save_descend())
        ttk.Label(ctl, text='PRIMARY > DESCEND fills the ballast to neutral plus this share of '
                            'the wiper travel. Saved on this controller.',
                  style='Panel.TLabel', foreground=DIM, font=('DejaVu Sans', 9),
                  wraplength=340).pack(anchor='w', padx=10, pady=(2, 8))

    # ---- layout helpers
    def _panel(self, r, c, title, columnspan=1):
        p = ttk.Frame(self.frame, style='Panel.TFrame')
        p.grid(row=r, column=c, columnspan=columnspan, sticky='nsew', padx=6, pady=4)
        ttk.Label(p, text=title, style='PanelHead.TLabel').pack(anchor='w', padx=10, pady=(6, 0))
        return p

    def _gain_row(self, parent, what):
        g = ttk.Frame(parent, style='Panel.TFrame')
        g.pack(fill='x', padx=10, pady=4)
        out = []
        for i, name in enumerate(('Kp', 'Ki', 'Kd')):
            ttk.Label(g, text=name, style='Panel.TLabel').grid(row=0, column=i, padx=6)
            v = tk.StringVar(value='—')
            self.num_entry(g, v, '%s %s' % (what, name), *self.GAIN, width=7).grid(
                row=1, column=i, padx=6, pady=2)
            out.append(v)
        return out

    def _buttons(self, parent, read, apply):
        row = ttk.Frame(parent, style='Panel.TFrame')
        row.pack(fill='x', padx=10, pady=(2, 8))
        ttk.Button(row, text='READ', command=read).pack(side='left', expand=True, fill='x', padx=2)
        ttk.Button(row, text='APPLY', command=apply).pack(side='left', expand=True, fill='x', padx=2)

    @staticmethod
    def _floats(vars_):
        try:
            return [float(v.get()) for v in vars_]
        except ValueError:
            return None

    # ---- Pico settings
    def read_all(self, quiet=False):
        self.read_ballast(quiet)
        self.read_rtz(quiet)
        self.read_slip(quiet)
        self.read_duty(quiet)

    # ---- drive PWM limit
    def show_duty_limit(self):
        self.limit.set('{:.0f}'.format(self.app.duty_limit))

    def read_duty(self, quiet=False):
        self.app.request(P.duty_limit(0), quiet=quiet,
                         ok=lambda r: self.app.duty_limit_changed(P.duty_limit_pct(r)))

    def apply_duty(self):
        if self.app.busy():
            self.app.status('Busy: apply settings once the Pico is idle.')
            return
        try:
            pct = float(self.limit.get())
        except ValueError:
            self.app.status('Press READ first: the PWM limit is not a number.')
            return

        def ok(r):
            got = P.duty_limit_pct(r)
            self.app.duty_limit_changed(got)
            self.app.status('Drive PWM limit saved on the Pico: %.0f %%.' % got)
        self.app.request(P.duty_limit(1, pct), ok=ok,
                         fail=lambda r: self.app.status('PWM limit: %s' % r.error_text()))

    def _show_gains(self, vars_, g):
        for v, x in zip(vars_, g):
            v.set('—' if x is None else '{:.2f}'.format(x))

    def read_ballast(self, quiet=False):
        self.app.request(P.ballast_pid(0), P.wait_pid, quiet=quiet,
                         ok=lambda r: self._show_gains(self.bal, P.gains(r)))

    def read_rtz(self, quiet=False):
        self.app.request(P.rtz_pid(0), P.wait_pid, quiet=quiet,
                         ok=lambda r: self._show_gains(self.rtz, P.gains(r, P.RTZ_GAIN_TAG)))

    def read_slip(self, quiet=False):
        def ok(r):
            for v, deg in zip(self.slip, P.slip_limits(r)):
                v.set('{:.1f}'.format(deg or 0.0))
        self.app.request(P.slip_limit(0), P.wait_slip, quiet=quiet, ok=ok)

    def _apply_gains(self, vars_, build, tag, what):
        if self.app.busy():
            self.app.status('Busy: apply settings once the Pico is idle.')
            return
        g = self._floats(vars_)
        if g is None:
            self.app.status('Press READ first: a %s gain is not a number.' % what)
            return

        def ok(r):
            self._show_gains(vars_, P.gains(r, tag))
            self.app.status('%s gains saved on the Pico: Kp %.2f  Ki %.2f  Kd %.2f.' % ((what,) + tuple(g)))
        self.app.request(build(1, *g), P.wait_pid, ok=ok,
                         fail=lambda r: self.app.status('%s gains: %s' % (what, r.error_text())))

    def apply_ballast(self):
        self._apply_gains(self.bal, P.ballast_pid, 0, 'Ballast')

    def apply_rtz(self):
        self._apply_gains(self.rtz, P.rtz_pid, P.RTZ_GAIN_TAG, 'Return-to-zero')

    def apply_slip(self):
        if self.app.busy():
            self.app.status('Busy: apply settings once the Pico is idle.')
            return
        vals = self._floats(self.slip)
        if vals is None:
            self.app.status('Press READ first: a slip limit is not a number.')
            return

        def ok(r):
            got = P.slip_limits(r)
            for v, deg in zip(self.slip, got):
                v.set('{:.1f}'.format(deg or 0.0))
            self.app.status('Slip limits saved on the Pico: %s.' % ', '.join(
                '%s %s' % (n, 'off' if d is None else '%.1f°' % d) for n, d in zip(MOTORS, got)))
        for i, deg in enumerate(vals):
            self.app.request(P.slip_limit(1, i, deg), P.wait_slip, ok=ok if i == 3 else None,
                             fail=lambda r, i=i: self.app.status('%s slip limit: %s' % (MOTORS[i], r.error_text())))

    # ---- controller settings
    def _save_descend(self):
        try:
            self.app.settings['descend_pct'] = float(self.descend.get())
        except ValueError:
            return
        self.app.tab_main.refresh_ballast()


class SequenceTab(_Tab):
    """Define a 4-motor sequence; run it now (temporary slot) or save it to a preset."""
    FIELDS = (('duty', 'Duty %', float, '{:.1f}', 0.0, 100.0, 50.0),
              ('latent', 'Latent', float, '{:.2f}', 0.0, 1.0, 0.0),
              ('phase', 'Phase', float, '{:.2f}', -1.0, 1.0, 0.0))

    def __init__(self, nb, app):
        super().__init__(nb, app)
        f = self.frame
        grid = ttk.Frame(f)
        grid.pack(side='left', fill='both', expand=True, padx=(14, 6), pady=10)
        ttk.Label(grid, text='Motor', style='Head.TLabel').grid(row=0, column=0, padx=6, pady=4)
        self.vars = []
        for c, (_, label, *_r) in enumerate(self.FIELDS):
            ttk.Label(grid, text=label, style='Head.TLabel').grid(row=0, column=c + 1, padx=6)
        for i, name in enumerate(MOTORS):
            ttk.Label(grid, text=name, style='Big.TLabel').grid(row=i + 1, column=0, padx=6, pady=7)
            row = {}
            for c, (key, label, cast, fmt, lo, hi, default) in enumerate(self.FIELDS):
                v = tk.StringVar(value=fmt.format(default))
                if key == 'duty':                       # capped by the Pico's drive PWM limit
                    hi = lambda: self.app.duty_limit
                self.num_entry(grid, v, '%s %s' % (name, label), cast, fmt, lo, hi).grid(
                    row=i + 1, column=c + 1, padx=6, pady=7)
                row[key] = v
            self.vars.append(row)
        ttk.Label(grid, text='Phase 1.0 = opposite stroke.  Latent = part of each stroke left off.',
                  foreground=DIM, font=('DejaVu Sans', 9)).grid(row=6, column=0, columnspan=4, pady=(8, 0))
        self.limit_note = ttk.Label(grid, text='', foreground=DIM, font=('DejaVu Sans', 9))
        self.limit_note.grid(row=7, column=0, columnspan=4)
        self.apply_duty_limit()

        side = ttk.Frame(f)
        side.pack(side='right', fill='y', padx=(6, 14), pady=10)
        self.period = tk.StringVar(value='1.00')
        self.cycles = tk.StringVar(value='1')
        self.slot = tk.StringVar(value='0')
        self.name = tk.StringVar(value='Preset 0')
        rows = (('Period (s)', self.period, float, '{:.2f}', 0.05, 4.0),
                ('Cycles', self.cycles, int, '{:d}', 1, 100))
        for r, (label, var, cast, fmt, lo, hi) in enumerate(rows):
            ttk.Label(side, text=label).grid(row=r, column=0, sticky='w', pady=5)
            self.num_entry(side, var, label, cast, fmt, lo, hi, 7).grid(row=r, column=1, pady=5)
        ttk.Button(side, text='▶  RUN NOW', style='Go.TButton',
                   command=self.on_run).grid(row=2, column=0, columnspan=2, sticky='ew', pady=(8, 14))
        ttk.Label(side, text='Preset slot').grid(row=3, column=0, sticky='w', pady=5)
        self.num_entry(side, self.slot, 'Preset slot', int, '{:d}', 0, P.MAX_PRESET_ID, 7).grid(
            row=3, column=1, pady=5)
        ttk.Label(side, text='Name').grid(row=4, column=0, sticky='w', pady=5)
        tk.Entry(side, textvariable=self.name, font=('DejaVu Sans', 12), width=12).grid(
            row=4, column=1, pady=5)
        ttk.Button(side, text='SAVE TO PRESET', command=self.on_save).grid(
            row=5, column=0, columnspan=2, sticky='ew', pady=8)

    def apply_duty_limit(self, quiet=False):
        """Pull any duty above the drive PWM limit down to it."""
        lim = self.app.duty_limit
        self.limit_note.configure(text='Duty is capped at the %.0f %% drive PWM limit (SETTINGS).' % lim)
        clamped = []
        for name, v in zip(MOTORS, self.vars):
            try:
                if float(v['duty'].get()) > lim + 1e-9:
                    v['duty'].set('{:.1f}'.format(lim))
                    clamped.append(name)
            except ValueError:
                pass
        if clamped and not quiet:
            self.app.status('Duty for %s lowered to the %.0f %% PWM limit.' % (', '.join(clamped), lim))
        return clamped

    # ---- values
    def read(self):
        """-> (period, cycles, motors as (duty u16, latent, phase)) or None."""
        try:
            period = float(self.period.get())
            cycles = int(self.cycles.get())
            motors = [(int(round(float(v['duty'].get()) / 100 * 65535)),
                       float(v['latent'].get()), float(v['phase'].get())) for v in self.vars]
        except ValueError:
            self.app.status('A sequence value is not a number.')
            return None
        return period, cycles, motors

    def load(self, slot, entry):
        self.slot.set(str(slot))
        self.name.set(entry['name'])
        self.period.set('{:.2f}'.format(entry['period']))
        self.cycles.set(str(entry.get('cycles', 1)))
        for v, (d, l, p) in zip(self.vars, entry['motors']):
            v['duty'].set('{:.1f}'.format(d / 65535 * 100))
            v['latent'].set('{:.2f}'.format(l))
            v['phase'].set('{:.2f}'.format(p))
        self.apply_duty_limit()

    # ---- actions
    def on_run(self):
        if self.app.busy():
            return
        vals = self.read()
        if not vals:
            return
        period, cycles, motors = vals
        self.app.request(P.preset_define(P.TEMP_PRESET_ID, period, motors),
                         ok=lambda r: self.app.run_sequence(P.TEMP_PRESET_ID, cycles, 'Sequence'),
                         fail=lambda r: self.app.status('Could not load the sequence: %s' % r.error_text()))

    def on_save(self):
        if self.app.busy():
            return
        vals = self.read()
        if not vals:
            return
        try:
            slot = int(self.slot.get())
        except ValueError:
            return
        name = self.name.get().strip() or 'Preset %d' % slot
        existing = self.app.lib.presets.get(slot)
        # TODO(slave side): PRESET_DEFINE overwrites a slot silently. The Pico should
        # refuse to overwrite an existing slot unless the frame carries an overwrite
        # flag (e.g. a new ERR_EXISTS 0x0C), so it bounces back for permission. Until
        # then this check uses the controller's own library, which can be out of date
        # if presets were written from elsewhere.
        if existing is not None:
            self.app.confirm('Overwrite preset?',
                             'Slot %d already holds "%s".\nReplace it with "%s"?'
                             % (slot, existing['name'], name),
                             lambda: self._save(slot, name, *vals), ORANGE)
        else:
            self._save(slot, name, *vals)

    def _save(self, slot, name, period, cycles, motors):
        def ok(res):
            self.app.lib.put(slot, name, period, motors, cycles)
            count = res.first(P.ST_DONE).i2
            self.app.status('Saved "%s" to slot %d. The Pico holds %d preset(s).' % (name, slot, count))
            self.app.tab_files.refresh_presets()
            self.app.check_sync(quiet=True)
        self.app.request(P.preset_define(slot, period, motors), ok=ok,
                         fail=lambda r: self.app.status('Save failed: %s' % r.error_text()))


class ManualTab(_Tab):
    """Forward / reverse / stop per motor; motors run until STOP. Keepalive is
    automatic. The ballast row jogs the ballast and stores the neutral point."""

    def __init__(self, nb, app):
        super().__init__(nb, app)
        f = self.frame
        top = ttk.Frame(f)
        top.pack(fill='x', padx=14, pady=(8, 2))
        ttk.Label(top, text='Speed %').pack(side='left')
        self.speed = tk.StringVar(value='10')
        self.num_entry(top, self.speed, 'Manual speed %', int, '{:d}', 1,
                       lambda: int(self.app.duty_limit), 5).pack(side='left', padx=8)
        ttk.Label(top, text='(drive motors; 10 = run slow)', foreground=DIM).pack(side='left')
        ttk.Button(top, text='ZERO ALL', command=lambda: self.on_zero(0x0F)).pack(side='right', padx=4)
        ttk.Button(top, text='REFRESH', command=lambda: app.refresh_positions()).pack(side='right', padx=4)

        grid = ttk.Frame(f)
        grid.pack(fill='both', expand=True, padx=14, pady=2)
        for c, head in enumerate(('', 'Position', 'Zero', '', '', '', '')):
            ttk.Label(grid, text=head, style='Head.TLabel').grid(row=0, column=c, padx=4)
        self.rows = []
        for i, name in enumerate(MOTORS):
            r = i + 1
            ttk.Label(grid, text=name, style='Big.TLabel').grid(row=r, column=0, padx=(0, 8), pady=3)
            pos = ttk.Label(grid, text='—', width=15, font=('DejaVu Sans', 12))
            pos.grid(row=r, column=1, padx=4)
            badge = tk.Label(grid, text='?', width=9, font=('DejaVu Sans', 9, 'bold'), fg='white', bg=GREY)
            badge.grid(row=r, column=2, padx=6)
            rev = ttk.Button(grid, text='◀ REV', width=7,
                             command=lambda i=i: self.on_move(i, P.MANUAL_REVERSE))
            stop = ttk.Button(grid, text='■ STOP', width=7, command=lambda i=i: self.on_halt(i))
            fwd = ttk.Button(grid, text='FWD ▶', width=7,
                             command=lambda i=i: self.on_move(i, P.MANUAL_FORWARD))
            zero = ttk.Button(grid, text='SET ZERO', command=lambda i=i: self.on_zero(1 << i))
            for c, b in enumerate((rev, stop, fwd, zero)):
                b.grid(row=r, column=3 + c, padx=3, pady=3, sticky='ew')
            self.rows.append((pos, badge, rev, fwd))

        # ---- ballast: jog (Pico default 40 %, soft limits on the Pico) and set neutral
        ttk.Separator(grid).grid(row=5, column=0, columnspan=7, sticky='ew', pady=(6, 4))
        b = P.MANUAL_BALLAST
        ttk.Label(grid, text='BAL', style='Big.TLabel').grid(row=6, column=0, padx=(0, 8), pady=3)
        self.wiper_label = ttk.Label(grid, text='—', width=15, font=('DejaVu Sans', 12))
        self.wiper_label.grid(row=6, column=1, padx=4)
        self.neutral_badge = tk.Label(grid, text='NO NEUTRAL', width=9,
                                      font=('DejaVu Sans', 9, 'bold'), fg='white', bg=ORANGE)
        self.neutral_badge.grid(row=6, column=2, padx=6)
        self.b_empty = ttk.Button(grid, text='◀ EMPTY', width=7,
                                  command=lambda: self.on_move(b, P.MANUAL_REVERSE))
        self.b_fill = ttk.Button(grid, text='FILL ▶', width=7,
                                 command=lambda: self.on_move(b, P.MANUAL_FORWARD))
        for c, w in enumerate((self.b_empty,
                               ttk.Button(grid, text='■ STOP', width=7, command=lambda: self.on_halt(b)),
                               self.b_fill,
                               ttk.Button(grid, text='SET NEUTRAL', command=self.on_neutral))):
            w.grid(row=6, column=3 + c, padx=3, pady=3, sticky='ew')
        ttk.Label(f, text='Motors run until STOP; the controller re-sends the command every second, '
                          'and the Pico stops a motor within 3 s if the link drops. '
                          'The ballast stops by itself at its empty and full limits.',
                  foreground=DIM, font=('DejaVu Sans', 9), wraplength=770).pack(pady=(0, 4))

    def apply_duty_limit(self):
        try:
            if int(self.speed.get()) > self.app.duty_limit:
                self.speed.set(str(int(self.app.duty_limit)))
        except ValueError:
            pass

    def speed_raw(self, i=0):
        if i == P.MANUAL_BALLAST:
            return 0                                   # the Pico's ballast default (40 %)
        try:
            return int(max(1, min(100, int(self.speed.get()))) / 100 * 65535)
        except ValueError:
            return 0                                   # the Pico's default (10 %)

    def refresh(self):
        for i, (pos, badge, rev, fwd) in enumerate(self.rows):
            c = self.app.positions[i]
            pos.configure(text='%+8.1f°  (%d)' % (c * 360 / CPR, c))
            known = self.app.zero_known & (1 << i)
            badge.configure(text='KNOWN' if known else 'UNKNOWN', bg=GREEN if known else ORANGE)
            action = self.app.manual_running.get(i)
            rev.configure(style='Active.TButton' if action == P.MANUAL_REVERSE else 'TButton')
            fwd.configure(style='Active.TButton' if action == P.MANUAL_FORWARD else 'TButton')
        self.refresh_ballast()

    def refresh_ballast(self):
        w, n = self.app.wiper, self.app.neutral
        self.wiper_label.configure(text='—' if w is None else '%5.1f %%  (%d)' % (pct(w), w))
        self.neutral_badge.configure(text='NO NEUTRAL' if n is None else 'N %.1f %%' % pct(n),
                                     bg=ORANGE if n is None else GREEN)
        action = self.app.manual_running.get(P.MANUAL_BALLAST)
        self.b_empty.configure(style='Active.TButton' if action == P.MANUAL_REVERSE else 'TButton')
        self.b_fill.configure(style='Active.TButton' if action == P.MANUAL_FORWARD else 'TButton')

    def on_move(self, i, action):
        if self.app.busy():
            return
        def ok(res):
            self.app.manual_running[i] = action
            self.refresh()
            if i == P.MANUAL_BALLAST:
                self.app.status('Ballast %s. Press STOP to halt.'
                                % ('filling' if action == P.MANUAL_FORWARD else 'emptying'))
            else:
                self.app.status('%s %s at %s %%. Press STOP to halt.' % (
                    MOTORS[i], 'forward' if action == P.MANUAL_FORWARD else 'reverse', self.speed.get()))

        def fail(r):
            if i == P.MANUAL_BALLAST and isinstance(r.error, P.Frame) and r.error.i1 == 0x08:
                self.app.status('The ballast is already %s.'
                                % ('full' if action == P.MANUAL_FORWARD else 'empty'))
            else:
                self.app.status('%s: %s' % (motor_name(i), r.error_text()))
        self.app.request(P.manual(i, action, self.speed_raw(i)), ok=ok, fail=fail)

    def on_halt(self, i):
        self.app.manual_running.pop(i, None)
        self.refresh()
        # The F6 / F8 that follows the DONE is stored by the app's message handler.
        self.app.request(P.manual(i, P.MANUAL_STOP), P.wait_done_then_position)

    def keepalive(self):
        for i, action in list(self.app.manual_running.items()):
            self.app.request(P.manual(i, action, self.speed_raw(i)), quiet=True,
                             fail=lambda r, i=i: (self.app.manual_running.pop(i, None), self.refresh()))

    def on_zero(self, mask):
        if self.app.busy():
            return
        if any(mask & (1 << i) for i in self.app.manual_running if i < 4):
            self.app.status('Stop the motor before setting its zero.')
            return
        def ok(res):
            self.app.zero_known = res.first(P.ST_DONE).i1
            for i in range(4):
                if mask & (1 << i):
                    self.app.positions[i] = 0
            self.refresh()
            self.app.tab_main.refresh_zero()
            self.app.status('Zero set for %s.' % mask_names(mask))
        self.app.confirm('Set zero', 'Make the current position of %s its zero?' % mask_names(mask),
                         lambda: self.app.request(P.set_zero(mask), ok=ok,
                                                  fail=lambda r: self.app.status('Set zero: %s' % r.error_text())))

    def on_neutral(self):
        """Store the ballast's current position as neutral buoyancy (captured on the Pico)."""
        if self.app.busy():
            return
        if P.MANUAL_BALLAST in self.app.manual_running:
            self.app.status('Stop the ballast before setting neutral.')
            return
        here = '' if self.app.wiper is None else ' (%.1f %%)' % pct(self.app.wiper)

        def ok(res):
            d = res.first(P.ST_DONE)
            self.app.neutral = (d.i1 << 8) | d.i2
            self.app.ballast_changed()
            self.app.status('Neutral stored: %.1f %% (%d).' % (pct(self.app.neutral), self.app.neutral))
        self.app.confirm('Set neutral', 'Store the ballast\'s current position%s as neutral buoyancy?' % here,
                         lambda: self.app.request(P.ballast_neutral(0), ok=ok,
                                                  fail=lambda r: self.app.status('Set neutral: %s' % r.error_text())))


class FilesTab(_Tab):
    """Preset library (this controller) and sequence logs."""

    def __init__(self, nb, app):
        super().__init__(nb, app)
        f = self.frame
        left = ttk.Frame(f)
        left.pack(side='left', fill='both', expand=True, padx=(10, 5), pady=8)
        right = ttk.Frame(f)
        right.pack(side='right', fill='both', expand=True, padx=(5, 10), pady=8)

        ttk.Label(left, text='PRESETS', style='Head.TLabel').pack(anchor='w')
        self.ptree = ttk.Treeview(left, columns=('slot', 'name', 'period', 'cycles'),
                                  show='headings', height=6, selectmode='browse')
        for col, text, w in (('slot', 'Slot', 46), ('name', 'Name', 150), ('period', 'Period', 66),
                             ('cycles', 'Cycles', 60)):
            self.ptree.heading(col, text=text)
            self.ptree.column(col, width=w, anchor='w' if col == 'name' else 'center')
        self.ptree.pack(fill='both', expand=True, pady=4)
        self.sync_label = tk.Label(left, text='Pico: not checked', bg=GREY, fg='white',
                                   font=('DejaVu Sans', 10, 'bold'), pady=3)
        self.sync_label.pack(fill='x', pady=(2, 4))
        b1 = ttk.Frame(left)
        b1.pack(fill='x')
        for text, cmd in (('LOAD', self.on_load), ('RUN', self.on_run), ('DELETE', self.on_delete)):
            ttk.Button(b1, text=text, command=cmd).pack(side='left', expand=True, fill='x', padx=2)
        b2 = ttk.Frame(left)
        b2.pack(fill='x', pady=(4, 0))
        ttk.Button(b2, text='CHECK SYNC', command=lambda: app.check_sync()).pack(
            side='left', expand=True, fill='x', padx=2)
        ttk.Button(b2, text='PUSH ALL TO PICO', command=self.on_push).pack(
            side='left', expand=True, fill='x', padx=2)

        ttk.Label(right, text='SEQUENCE LOGS', style='Head.TLabel').pack(anchor='w')
        self.ltree = ttk.Treeview(right, columns=('when', 'preset', 'n', 'result'),
                                  show='headings', height=6, selectmode='browse')
        for col, text, w in (('when', 'Recorded', 128), ('preset', 'Preset', 60), ('n', 'Samples', 66),
                             ('result', 'Result', 96)):
            self.ltree.heading(col, text=text)
            self.ltree.column(col, width=w, anchor='center')
        self.ltree.pack(fill='both', expand=True, pady=4)
        tk.Label(right, text=app.logs.folder, bg=BG, fg=DIM, font=('DejaVu Sans', 9),
                 anchor='w').pack(fill='x', pady=(2, 4))
        b3 = ttk.Frame(right)
        b3.pack(fill='x')
        for text, cmd in (('FETCH LAST', self.on_fetch), ('EXPORT CSV', self.on_export),
                          ('DELETE', self.on_delete_log)):
            ttk.Button(b3, text=text, command=cmd).pack(side='left', expand=True, fill='x', padx=2)
        self.refresh_presets()
        self.refresh_logs()

    # ---- presets
    def refresh_presets(self):
        self.ptree.delete(*self.ptree.get_children())
        for slot, e in sorted(self.app.lib.presets.items()):
            self.ptree.insert('', 'end', iid=str(slot), values=(
                slot, e['name'], '%.2f s' % e['period'], e.get('cycles', 1)))
        self.show_sync()
        if hasattr(self.app, 'tab_main'):
            self.app.tab_main.refresh_presets()

    def show_sync(self):
        if self.app.pico_sync is None:
            self.sync_label.configure(text='Pico: not checked', bg=GREY)
            return
        count, crc = self.app.pico_sync
        if crc == self.app.lib.crc() and count == len(self.app.lib.presets):
            self.sync_label.configure(text='Pico: in sync (%d presets)' % count, bg=GREEN)
        else:
            self.sync_label.configure(text='Pico: OUT OF SYNC (Pico %d, here %d)'
                                      % (count, len(self.app.lib.presets)), bg=ORANGE)

    def _selected_preset(self):
        sel = self.ptree.selection()
        if not sel:
            self.app.status('Select a preset first.')
            return None, None
        slot = int(sel[0])
        return slot, self.app.lib.presets.get(slot)

    def on_load(self):
        slot, e = self._selected_preset()
        if e:
            self.app.tab_seq.load(slot, e)
            self.app.show_tab(self.app.tab_seq)
            self.app.status('Loaded "%s" into the editor.' % e['name'])

    def on_run(self):
        slot, e = self._selected_preset()
        if e and not self.app.busy():
            self.app.run_sequence(slot, e.get('cycles', 1), '"%s"' % e['name'])

    def on_delete(self):
        slot, e = self._selected_preset()
        if not e or self.app.busy():
            return
        def go():
            def ok(res):
                self.app.lib.delete(slot)
                self.refresh_presets()
                self.app.check_sync(quiet=True)
                self.app.status('Deleted "%s" (slot %d).' % (e['name'], slot))
            self.app.request(P.preset_delete(slot), ok=ok,
                             fail=lambda r: self.app.status('Delete failed: %s' % r.error_text()))
        self.app.confirm('Delete preset', 'Delete "%s" (slot %d) here and on the Pico?' % (e['name'], slot), go)

    def on_push(self):
        if self.app.busy() or not self.app.link:
            return
        n = len(self.app.lib.presets)
        self.app.confirm('Push all presets',
                         'Replace every preset on the Pico with the %d preset(s) stored here?' % n,
                         self._push, ORANGE)

    def _push(self):
        lib, link, app = self.app.lib, self.app.link, self.app

        def work():                                     # off the GUI thread
            r = link.call(P.preset_delete(P.DELETE_ALL))
            if not r.ok:
                return 'Clearing the Pico failed: %s' % r.error_text()
            for slot, e in sorted(lib.presets.items()):
                r = link.call(P.preset_define(slot, e['period'], e['motors']))
                if not r.ok:
                    return 'Slot %d failed: %s' % (slot, r.error_text())
            r = link.call(P.preset_sync(), P.wait_sync)
            if not r.ok:
                return 'Sync check failed: %s' % r.error_text()
            f = r.first(P.ST_PRESET_SYNC)
            app.pico_sync = (f.i1, P.u16(f))
            return None

        def run():
            err = work()
            app.root.after(0, lambda: (self.show_sync(),
                                       app.status(err or 'All %d preset(s) pushed to the Pico.'
                                                  % len(lib.presets))))
        app.status('Pushing presets…')
        threading.Thread(target=run, daemon=True).start()

    # ---- logs
    def refresh_logs(self):
        self.ltree.delete(*self.ltree.get_children())
        for name, hdr in self.app.logs.list():
            when = '%s-%s %s:%s:%s' % (name[8:10], name[10:12], name[13:15], name[15:17], name[17:19])
            if hdr is None:
                vals = (when, '?', '?', 'unreadable')
            else:
                pre = 'temp' if hdr['preset'] == P.TEMP_PRESET_ID else str(hdr['preset'])
                vals = (when, pre, hdr['n_samples'], P.flags_text(hdr['flags']))
            self.ltree.insert('', 'end', iid=name, values=vals)

    def _selected_log(self):
        sel = self.ltree.selection()
        if not sel:
            self.app.status('Select a log first.')
            return None
        return sel[0]

    def on_fetch(self):
        if self.app.busy():
            return
        self.app.status('Fetching the last log from the Pico…')
        # The log itself arrives as a LogBlock message and is saved by _got_log.
        self.app.request(P.log_get(), P.wait_log, timeout=12,
                         fail=lambda r: self.app.status('Fetch failed: %s' % r.error_text()))

    def on_export(self):
        name = self._selected_log()
        if name:
            try:
                self.app.status('Exported %s' % self.app.logs.export_csv(name))
            except (OSError, ValueError) as e:
                self.app.status('Export failed: %s' % e)

    def on_delete_log(self):
        name = self._selected_log()
        if name:
            self.app.confirm('Delete log', 'Delete %s (and its CSV)?' % name,
                             lambda: (self.app.logs.delete(name), self.refresh_logs()))


# ===================================================================== entry point
if __name__ == '__main__':
    try:
        link = PicoLink(port='/dev/serial0', baudrate=115200)
    except Exception as e:
        print('WARNING: could not open serial: %s' % e)
        link = None
    root = tk.Tk()
    gui = MantaGUI(root, link=link)
    try:
        root.mainloop()
    finally:
        if link:
            link.close()
