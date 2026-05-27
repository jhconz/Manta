# -*- coding: utf-8 -*-
"""
manta_controller_gui.py — Touchscreen control panel for the manta Pico

Hardware: Raspberry Pi 4B with 5" DSI 800x480 capacitive touchscreen
          UART connection to Pico via /dev/serial0 (GPIO14 TX, GPIO15 RX)
          
Dependencies:
  - pyserial (pip install pyserial)
"""
import tkinter as tk
from tkinter import ttk
import serial
import struct
import threading
import time
from numpad import NumPad


# =============================================================================
#  Protocol — must match the Pico's onboard_pi class
# =============================================================================

# Command codes
CMD_STOP        = 0x01
CMD_SET_MOTOR   = 0x02
CMD_SEQ_ADD     = 0x03
CMD_SEQ_RUN     = 0x04
CMD_GET_STATUS  = 0x05
CMD_POWERDOWN   = 0x06 

# Status codes
STATUS_IDLE      = 0x00
STATUS_ACK       = 0x01
STATUS_DONE      = 0x02
STATUS_ERROR     = 0x03
STATUS_BAT_WARN  = 0xF0
STATUS_BAT_CRIT  = 0xF1

RESPONSE_SIZE = 5


def xor_checksum(data):
    c = 0
    for b in data:
        c ^= b
    return c


# =============================================================================
#  Serial communication
# =============================================================================

class PicoLink:
    """Serial wrapper for talking to the Pico. Runs an RX thread that
    parses incoming response frames and dispatches them to a callback."""
    
    def __init__(self, port='/dev/serial0', baudrate=115200, on_message=None):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.on_message = on_message
        self._tx_lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
    
    def close(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
    
    def send_frame(self, frame_bytes):
        with self._tx_lock:
            self.ser.write(frame_bytes)
    
    def send_stop(self):
        frame = bytes([CMD_STOP, 0])
        self.send_frame(frame + bytes([xor_checksum(frame)]))
    
    def send_set_motor(self, m0_duty, m1_duty, period, m0_latent, m1_latent, phase):
        payload = struct.pack('>BHHffff',
                              CMD_SET_MOTOR,
                              int(m0_duty), int(m1_duty),
                              float(period),
                              float(m0_latent), float(m1_latent),
                              float(phase))
        self.send_frame(payload + bytes([xor_checksum(payload)]))
    
    def send_seq_run(self, cycles):
        payload = struct.pack('>BH', CMD_SEQ_RUN, int(cycles))
        self.send_frame(payload + bytes([xor_checksum(payload)]))
    
    def send_powerdown(self):
        frame = bytes([CMD_POWERDOWN, 0])
        self.send_frame(frame + bytes([xor_checksum(frame)]))
    
    def _rx_loop(self):
        while not self._stop:
            try:
                data = self.ser.read(RESPONSE_SIZE)
            except Exception:
                time.sleep(0.1)
                continue
            
            if not data:
                continue
            self._rx_buffer.extend(data)
            
            while len(self._rx_buffer) >= RESPONSE_SIZE:
                frame = bytes(self._rx_buffer[:RESPONSE_SIZE])
                if xor_checksum(frame[:4]) != frame[4]:
                    del self._rx_buffer[0]
                    continue
                del self._rx_buffer[:RESPONSE_SIZE]
                if self.on_message:
                    self.on_message(frame[0], frame[1], frame[2], frame[3])


# =============================================================================
#  GUI
# =============================================================================

class MantaControllerGUI:
    SCREEN_W = 800
    SCREEN_H = 480
    
    BAT_FINE     = 'fine'
    BAT_LOW      = 'low'
    BAT_CRITICAL = 'critical'
    
    BAT_COLORS = {
        BAT_FINE:     '#2ecc71',
        BAT_LOW:      '#f39c12',
        BAT_CRITICAL: '#e74c3c',
    }
    BAT_LABELS = {
        BAT_FINE:     'BATTERY: FINE',
        BAT_LOW:      'BATTERY: LOW',
        BAT_CRITICAL: 'BATTERY: CRITICAL',
    }
    
    PARAM_SPECS = [
        # (key, label, default, format string, type)
        ('m0_duty',   "M0 Duty",     30000,   '{:d}',   int),
        ('m1_duty',   "M1 Duty",     30000,   '{:d}',   int),
        ('period',    "Period (s)",  1.0,     '{:.2f}', float),
        ('m0_latent', "M0 Latent",   0.0,     '{:.2f}', float),
        ('m1_latent', "M1 Latent",   0.0,     '{:.2f}', float),
        ('phase',     "Phase",       0.0,     '{:.2f}', float),
        ('cycles',    "Cycles",      1,       '{:d}',   int),
    ]
    PARAM_LIMITS = {
        'm0_duty':   (0,     65535),
        'm1_duty':   (0,     65535),
        'period':    (0.05, 4.0),
        'm0_latent': (0.0, 1.0),
        'm1_latent': (0.0, 1.0),
        'phase':     (-1.0,  1.0),
        'cycles':    (1, 100),
    }
    
    def __init__(self, root, link=None):
        self.root = root
        self.link = link
        
        # Dialog state
        self._lockout_dialog = None
        self._battery_warn_dialog = None
        self._battery_crit_dialog = None
        self._sequence_running = False
        self._pending_cycles = None
        
        root.title("Manta Controller")
        root.geometry(f"{self.SCREEN_W}x{self.SCREEN_H}")
        root.configure(bg='#2c3e50')
        # root.attributes('-fullscreen', True)
        # root.bind('<Escape>', lambda e: root.attributes('-fullscreen', False))
        
        self._build_styles()
        self._build_top_bar()
        self._build_param_grid()
        self._build_button_bar()
        self._build_status_line()
        
        if self.link:
            self.link.on_message = self._on_pico_message
    
    # =====================================================================
    #  Layout construction
    # =====================================================================
    
    def _build_styles(self):
        style = ttk.Style()
        style.theme_use('default')
        style.configure('TLabel', background='#2c3e50', foreground='white',
                        font=('DejaVu Sans', 11))
        style.configure('Param.TLabel', font=('DejaVu Sans', 12, 'bold'))
        style.configure('Action.TButton', font=('DejaVu Sans', 12, 'bold'), padding=8)
        style.configure('Stop.TButton', font=('DejaVu Sans', 12, 'bold'), padding=8,
                        background='#e74c3c', foreground='white')
        style.configure('Power.TButton', font=('DejaVu Sans', 12, 'bold'), padding=8,
                        background='#7f8c8d', foreground='white')
        style.configure('Dialog.TButton', font=('DejaVu Sans', 14, 'bold'), padding=12)
        style.configure('Abort.TButton', font=('DejaVu Sans', 14, 'bold'), padding=12,
                        background='#e74c3c', foreground='white')
    
    def _build_top_bar(self):
        bar = tk.Frame(self.root, bg='#34495e', height=50)
        bar.pack(fill='x', side='top')
        bar.pack_propagate(False)
        tk.Label(bar, text="MANTA CONTROLLER",
                 bg='#34495e', fg='white',
                 font=('DejaVu Sans', 16, 'bold')).pack(side='left', padx=20)
        self.battery_label = tk.Label(bar, text=self.BAT_LABELS[self.BAT_FINE],
                                      bg=self.BAT_COLORS[self.BAT_FINE], fg='white',
                                      font=('DejaVu Sans', 12, 'bold'),
                                      padx=15, pady=6)
        self.battery_label.pack(side='right', padx=15, pady=8)
    
    def _build_param_grid(self):
        frame = tk.Frame(self.root, bg='#2c3e50')
        frame.pack(fill='both', expand=True, padx=20, pady=10)
        self.vars = {}
        self.entries = {}
        for i, (key, label_text, default, fmt, _type) in enumerate(self.PARAM_SPECS):
            col = i % 2
            row = i // 2
            cell = tk.Frame(frame, bg='#2c3e50')
            cell.grid(row=row, column=col, sticky='ew', padx=10, pady=6)
            frame.columnconfigure(col, weight=1)
            ttk.Label(cell, text=label_text, style='Param.TLabel',
                      width=12).pack(side='left')
            var = tk.StringVar(value=fmt.format(default))
            entry = tk.Entry(cell, textvariable=var, font=('DejaVu Sans', 14),
                             width=10, justify='right')
            entry.pack(side='left', padx=8, fill='x', expand=True)
            entry.bind('<Button-1>', lambda e, k=key: self._open_numpad(k))
            self.vars[key] = var
            self.entries[key] = entry
    
    def _build_button_bar(self):
        bar = tk.Frame(self.root, bg='#2c3e50', height=70)
        bar.pack(fill='x', side='bottom', padx=15, pady=10)
        bar.pack_propagate(False)
        ttk.Button(bar, text="UPDATE PARAMS", style='Action.TButton',
                   command=self.on_update_params).pack(side='left', expand=True,
                                                       fill='both', padx=4)
        ttk.Button(bar, text="START SEQUENCE", style='Action.TButton',
                   command=self.on_start_sequence).pack(side='left', expand=True,
                                                        fill='both', padx=4)
        ttk.Button(bar, text="STOP", style='Stop.TButton',
                   command=self.on_stop).pack(side='left', expand=True,
                                              fill='both', padx=4)
        ttk.Button(bar, text="POWER DOWN PICO", style='Power.TButton',
                   command=self.on_powerdown).pack(side='left', expand=True,
                                                   fill='both', padx=4)
    
    def _build_status_line(self):
        self.status_var = tk.StringVar(value="Ready.")
        bar = tk.Frame(self.root, bg='#1a252f', height=24)
        bar.pack(fill='x', side='bottom')
        bar.pack_propagate(False)
        tk.Label(bar, textvariable=self.status_var,
                 bg='#1a252f', fg='#bdc3c7',
                 font=('DejaVu Sans', 10), anchor='w').pack(fill='x', padx=10)
    
    # =====================================================================
    #  Dialog helpers
    # =====================================================================
    
    def _make_dialog(self, title, message, bg_color='#34495e',
                     buttons=None, modal=True, auto_close_ms=None):
        """Create a centered popup dialog. Returns the Toplevel widget."""
        dlg = tk.Toplevel(self.root)
        dlg.title(title)
        dlg.configure(bg=bg_color)
        dlg.transient(self.root)
        if modal:
            dlg.grab_set()
        dlg.protocol("WM_DELETE_WINDOW", lambda: None)
        
        w, h = 500, 200
        x = (self.SCREEN_W - w) // 2
        y = (self.SCREEN_H - h) // 2
        dlg.geometry(f"{w}x{h}+{x}+{y}")
        dlg.resizable(False, False)
        
        tk.Label(dlg, text=title, bg=bg_color, fg='white',
                 font=('DejaVu Sans', 16, 'bold')).pack(pady=(15, 5))
        tk.Label(dlg, text=message, bg=bg_color, fg='white',
                 font=('DejaVu Sans', 12), wraplength=460,
                 justify='center').pack(pady=10, padx=20)
        
        btn_frame = tk.Frame(dlg, bg=bg_color)
        btn_frame.pack(side='bottom', fill='x', padx=15, pady=15)
        
        if not buttons:
            buttons = [("OK", 'Dialog.TButton', dlg.destroy)]
        
        for label, style, callback in buttons:
            ttk.Button(btn_frame, text=label, style=style,
                       command=callback).pack(side='left', expand=True,
                                              fill='both', padx=4)
        
        if auto_close_ms is not None:
            self.root.after(auto_close_ms,
                            lambda: dlg.destroy() if dlg.winfo_exists() else None)
        
        return dlg
    
    # =====================================================================
    #  Specific dialogs
    # =====================================================================
    
    def _show_lockout(self, cycles):
        """Modal dialog shown while a motor sequence is running."""
        if self._lockout_dialog and self._lockout_dialog.winfo_exists():
            return
        
        def abort():
            if self.link:
                self.link.send_stop()
        
        self._lockout_dialog = self._make_dialog(
            title="Motor Sequence Running",
            message=f"Running {cycles} cycle(s).\n\n"
                    "Controls are locked until the sequence completes.\n"
                    "Press ABORT to stop early.",
            bg_color='#2980b9',
            buttons=[("ABORT", 'Abort.TButton', abort)],
            modal=True
        )
    
    def _dismiss_lockout(self):
        if self._lockout_dialog and self._lockout_dialog.winfo_exists():
            self._lockout_dialog.destroy()
        self._lockout_dialog = None
    
    def _show_battery_warning(self):
        """Modal warning, requires acknowledgement.
        If a lockout is showing, hide it and restore on dismiss."""
        NumPad.force_close()
        if self._battery_warn_dialog and self._battery_warn_dialog.winfo_exists():
            return
        
        lockout_was_open = (self._lockout_dialog is not None
                            and self._lockout_dialog.winfo_exists())
        if lockout_was_open:
            self._lockout_dialog.withdraw()
        
        def acknowledge():
            self._battery_warn_dialog.destroy()
            self._battery_warn_dialog = None
            # Restore lockout only if a sequence is still running and the
            # lockout dialog still exists (it may have been destroyed if
            # the sequence completed while warning was up)
            if (lockout_was_open and self._sequence_running
                    and self._lockout_dialog is not None
                    and self._lockout_dialog.winfo_exists()):
                self._lockout_dialog.deiconify()
                self._lockout_dialog.grab_set()
        
        self._battery_warn_dialog = self._make_dialog(
            title="Low Battery",
            message="Low battery — recommend shutdown after this cycle.",
            bg_color='#f39c12',
            buttons=[("ACKNOWLEDGE", 'Dialog.TButton', acknowledge)],
            modal=True
        )
    
    def _show_battery_critical(self):
        """Modal, requires acknowledgement. Pico is shutting down regardless."""
        NumPad.force_close()
        if self._battery_crit_dialog and self._battery_crit_dialog.winfo_exists():
            return
        
        # Critical supersedes everything else
        self._dismiss_lockout()
        if self._battery_warn_dialog and self._battery_warn_dialog.winfo_exists():
            self._battery_warn_dialog.destroy()
            self._battery_warn_dialog = None
        
        def acknowledge():
            self._battery_crit_dialog.destroy()
            self._battery_crit_dialog = None
        
        self._battery_crit_dialog = self._make_dialog(
            title="BATTERY CRITICAL",
            message="Battery level critical.\n"
                    "Automatic shutdown initiated.\n\n"
                    "The Pico is powering down — recharge before next use.",
            bg_color='#c0392b',
            buttons=[("ACKNOWLEDGE", 'Dialog.TButton', acknowledge)],
            modal=True
        )
    
    def _show_powerdown_confirmation(self):
        self._make_dialog(
            title="Powering Down",
            message="Powerdown command sent.\nThe Pico is going to sleep.",
            bg_color='#7f8c8d',
            buttons=[("OK", 'Dialog.TButton', lambda: None)],
            modal=False,
            auto_close_ms=3000
        )
    
    def _show_error(self, code):
        self._make_dialog(
            title="Pico Error",
            message=f"The Pico reported an error.\n\nCode: 0x{code:02X}",
            bg_color='#c0392b',
            modal=True,
            auto_close_ms=5000
        )
    
    # =====================================================================
    #  Keypad integration
    # =====================================================================
    
    def _open_numpad(self, key):
        if self._sequence_running:
            return
        
        if (self._battery_crit_dialog is not None and self._battery_crit_dialog.winfo_exists()):
            return 
        _, label, _, fmt, cast = next(s for s in self.PARAM_SPECS if s[0] == key)
        vmin, vmax = self.PARAM_LIMITS[key]
        var = self.vars[key]
        try:
            current = cast(var.get())
        except (ValueError, TypeError):
            current = 0 if cast is int else 0.0
        NumPad.edit(
            self.root,
            title=label,
            initial=current,
            cast=cast,
            formatter=fmt,
            vmin=vmin,
            vmax=vmax,
            on_commit=lambda v: var.set(fmt.format(v)),
        )
    
    # =====================================================================
    #  Public state setters
    # =====================================================================
    
    def set_battery_status(self, status):
        if status not in self.BAT_LABELS:
            return
        self.battery_label.configure(text=self.BAT_LABELS[status],
                                     bg=self.BAT_COLORS[status])
    
    def set_status_message(self, msg):
        self.status_var.set(msg)
    
    # =====================================================================
    #  Reading parameters
    # =====================================================================
    
    def get_motor_params(self):
        try:
            return {
                'm0_duty':   int(self.vars['m0_duty'].get()),
                'm1_duty':   int(self.vars['m1_duty'].get()),
                'period':    float(self.vars['period'].get()),
                'm0_latent': float(self.vars['m0_latent'].get()),
                'm1_latent': float(self.vars['m1_latent'].get()),
                'phase':     float(self.vars['phase'].get()),
            }
        except ValueError:
            self.set_status_message("ERROR: invalid parameter value")
            return None
    
    def get_cycles(self):
        try:
            return int(self.vars['cycles'].get())
        except ValueError:
            return None
    
    # =====================================================================
    #  Button handlers
    # =====================================================================
    
    def on_update_params(self):
        if self._sequence_running:
            return
        params = self.get_motor_params()
        if params is None or self.link is None:
            return
        self.link.send_set_motor(**params)
        self.set_status_message("Parameters sent.")
    
    def on_start_sequence(self):
        if self._sequence_running:
            return
        cycles = self.get_cycles()
        if cycles is None or cycles <= 0 or self.link is None:
            self.set_status_message("ERROR: invalid cycle count")
            return
        self.link.send_seq_run(cycles)
        self._pending_cycles = cycles
        self.set_status_message(f"Starting sequence ({cycles} cycles)...")
    
    def on_stop(self):
        if self.link is None:
            return
        self.link.send_stop()
        self.set_status_message("Stop sent.")
    
    def on_powerdown(self):
        if self.link is None:
            return
        self.link.send_powerdown()
        self._show_powerdown_confirmation()
        self.set_status_message("Powerdown sent.")        
    
    # =====================================================================
    #  Inbound message handling
    # =====================================================================
    
    def _on_pico_message(self, status, info1, info2, info3):
        # Marshal from RX thread to main GUI thread
        self.root.after(0, self._handle_pico_message, status, info1, info2, info3)
    
    def _handle_pico_message(self, status, info1, info2, info3):
        if status == STATUS_BAT_WARN:
            self.set_battery_status(self.BAT_LOW)
            self.set_status_message("Battery LOW.")
            self._show_battery_warning()
        
        elif status == STATUS_BAT_CRIT:
            self.set_battery_status(self.BAT_CRITICAL)
            self.set_status_message("Battery CRITICAL.")
            self._show_battery_critical()
        
        elif status == STATUS_ACK:
            # ACK for SEQ_RUN → open lockout
            if self._pending_cycles is not None:
                self._sequence_running = True
                self._show_lockout(self._pending_cycles)
                self._pending_cycles = None
            self.set_status_message("Command acknowledged.")
        
        elif status == STATUS_DONE:
            if info1 == 0x10:
                # Sequence complete
                self._sequence_running = False
                self._dismiss_lockout()
                self.set_status_message("Sequence complete.")
            else:
                if self._sequence_running:
                    self._sequence_running = False
                    self._dismiss_lockout()
                    self.set_status_message("Sequence aborted.")
                else:
                    self.set_status_message("Command complete.")
        
        elif status == STATUS_ERROR:
            self.set_status_message(f"Pico error (code 0x{info1:02X}).")
            self._show_error(info1)
            if self._sequence_running:
                self._sequence_running = False
                self._dismiss_lockout()
                
        


# =============================================================================
#  Entry point
# =============================================================================

if __name__ == "__main__":
    try:
        link = PicoLink(port='/dev/serial0', baudrate=115200)
    except Exception as e:
        print(f"WARNING: could not open serial: {e}")
        link = None
    
    root = tk.Tk()
    gui = MantaControllerGUI(root, link=link)
    
    try:
        root.mainloop()
    finally:
        if link:
            link.close()