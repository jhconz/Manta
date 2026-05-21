# -*- coding: utf-8 -*-
"""
numpad.py — Fullscreen touch number pad for the Manta controller GUI.

Drop-in replacement for Roger Woollett's Keypad. Designed for an 800x480
DSI capacitive touchscreen running Tkinter.

Usage:
    NumPad.edit(
        root,
        title="Period (s)",
        initial=1.0,
        cast=float,
        formatter='{:.2f}',
        vmin=0.05,
        vmax=10.0,
        on_commit=lambda v: entry_var.set(formatter.format(v)),
    )
"""
import tkinter as tk
from tkinter import ttk


class NumPad(tk.Toplevel):
    # ---- Visual constants (match MantaControllerGUI) ----
    BG       = '#2c3e50'
    FG       = 'white'
    ERR_FG   = '#e74c3c'
    BTN_BG   = '#34495e'
    BTN_FG   = 'white'
    OK_BG    = '#2ecc71'
    CANCEL_BG = '#7f8c8d'

    SCREEN_W = 800
    SCREEN_H = 480

    # ---- Class-level "modal" registry so only one pad is up at a time ----
    _active = None

    @classmethod
    def edit(cls, parent, title, initial, cast=int, formatter='{}',
             vmin=None, vmax=None, on_commit=None):
        """Open the numpad. If one is already open, raise it instead."""
        if cls._active is not None and cls._active.winfo_exists():
            cls._active.lift()
            return cls._active
        pad = cls(parent, title, initial, cast, formatter, vmin, vmax, on_commit)
        cls._active = pad
        return pad

    def __init__(self, parent, title, initial, cast, formatter,
                 vmin, vmax, on_commit):
        super().__init__(parent)
        self.title("Edit value")
        self.configure(bg=self.BG)
        self.geometry(f"{self.SCREEN_W}x{self.SCREEN_H}+0+0")
        self.attributes('-fullscreen', True)
        self.transient(parent)
        self.grab_set()                          # modal
        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind('<Escape>', lambda e: self._cancel())

        self.cast = cast
        self.formatter = formatter
        self.vmin = vmin
        self.vmax = vmax
        self.on_commit = on_commit
        self.allow_float = (cast is float) or ('.' in formatter)

        # Working buffer — start with the initial value as a string the user
        # can edit. Strip trailing zeros for ints, keep formatter output for floats.
        if cast is int:
            self.buf = str(int(initial))
        else:
            self.buf = formatter.format(initial)

        self._build_ui(title)
        self._refresh_display()

    # ------------------------------------------------------------------ UI
    def _build_ui(self, title):
        # Root grid: 5 rows (header / value / error / digits / bottom)
        self.columnconfigure(0, weight=1)
        for r in range(5):
            self.rowconfigure(r, weight=1)

        # --- Header: parameter label ---
        hdr = tk.Label(self, text=title, bg=self.BG, fg=self.FG,
                       font=('DejaVu Sans', 20, 'bold'))
        hdr.grid(row=0, column=0, sticky='nsew', pady=(8, 0))

        # --- Current value (live) ---
        self.value_var = tk.StringVar()
        val = tk.Label(self, textvariable=self.value_var, bg=self.BG, fg=self.FG,
                       font=('DejaVu Sans Mono', 32, 'bold'))
        val.grid(row=1, column=0, sticky='nsew')

        # --- Error line ---
        self.err_var = tk.StringVar(value='')
        err = tk.Label(self, textvariable=self.err_var, bg=self.BG, fg=self.ERR_FG,
                       font=('DejaVu Sans', 13, 'bold'))
        err.grid(row=2, column=0, sticky='nsew')

        # --- Button area: holds digit grid + bottom row ---
        btns = tk.Frame(self, bg=self.BG)
        btns.grid(row=3, column=0, rowspan=2, sticky='nsew', padx=20, pady=(0, 16))
        for c in range(12):                      # 12 cols → LCM of 3, 4, 2
            btns.columnconfigure(c, weight=1, uniform='pad')
        for r in range(5):
            btns.rowconfigure(r, weight=1, uniform='pad')

        # Digits 1-9 in a 3x3 grid, spanning 4 cols each (3 buttons × 4 = 12)
        for i, d in enumerate(['1','2','3','4','5','6','7','8','9']):
            row, col = divmod(i, 3)
            self._mkbtn(btns, d, lambda c=d: self._press(c),
                        bg=self.BTN_BG).grid(
                row=row, column=col*4, columnspan=4, sticky='nsew',
                padx=4, pady=4)

        # Row 3: '.' and '+/-' — only show if floats / negatives allowed
        dot_state = 'normal' if self.allow_float else 'disabled'
        neg_state = 'normal' if (self.vmin is None or self.vmin < 0) else 'disabled'

        self._mkbtn(btns, '.', lambda: self._press('.'),
                    bg=self.BTN_BG, state=dot_state).grid(
            row=3, column=0, columnspan=6, sticky='nsew', padx=4, pady=4)
        self._mkbtn(btns, '+/-', self._toggle_sign,
                    bg=self.BTN_BG, state=neg_state).grid(
            row=3, column=6, columnspan=6, sticky='nsew', padx=4, pady=4)

        # Row 4: backspace | 0 | cancel | enter (3 cols each)
        self._mkbtn(btns, '⌫', self._backspace,
                    bg=self.BTN_BG).grid(
            row=4, column=0, columnspan=3, sticky='nsew', padx=4, pady=4)
        self._mkbtn(btns, '0', lambda: self._press('0'),
                    bg=self.BTN_BG).grid(
            row=4, column=3, columnspan=3, sticky='nsew', padx=4, pady=4)
        self._mkbtn(btns, 'Cancel', self._cancel,
                    bg=self.CANCEL_BG).grid(
            row=4, column=6, columnspan=3, sticky='nsew', padx=4, pady=4)
        self._mkbtn(btns, 'Enter', self._commit,
                    bg=self.OK_BG).grid(
            row=4, column=9, columnspan=3, sticky='nsew', padx=4, pady=4)

    def _mkbtn(self, parent, text, cmd, bg, state='normal'):
        return tk.Button(parent, text=text, command=cmd,
                         bg=bg, fg=self.BTN_FG,
                         activebackground=bg, activeforeground=self.BTN_FG,
                         font=('DejaVu Sans', 22, 'bold'),
                         relief='flat', bd=0, state=state,
                         disabledforeground='#555')

    # ------------------------------------------------------------------ Keys
    def _press(self, ch):
        # Prevent two decimal points
        if ch == '.' and '.' in self.buf:
            return
        # If the current buffer is just '0' (or '-0'), replace rather than append
        if ch.isdigit():
            if self.buf == '0':
                self.buf = ch
            elif self.buf == '-0':
                self.buf = '-' + ch
            else:
                self.buf += ch
        else:
            self.buf += ch
        self._refresh_display()

    def _backspace(self):
        # If we'd be left with just '-', clear it entirely
        if len(self.buf) <= 1 or self.buf == '-0':
            self.buf = '0'
        else:
            self.buf = self.buf[:-1]
            if self.buf == '-':
                self.buf = '0'
        self._refresh_display()

    def _toggle_sign(self):
        if self.buf.startswith('-'):
            self.buf = self.buf[1:]
        else:
            self.buf = '-' + self.buf
        self._refresh_display()

    # ------------------------------------------------------------------ Validation
    def _parse(self):
        """Return (value, error_msg). value is None if invalid."""
        s = self.buf.strip()
        if s in ('', '-', '.', '-.'):
            return None, "Enter a value"
        try:
            v = self.cast(s)
        except (ValueError, TypeError):
            return None, "Not a valid number"
        if self.vmin is not None and v < self.vmin:
            return None, f"Min is {self.formatter.format(self.vmin)}"
        if self.vmax is not None and v > self.vmax:
            return None, f"Max is {self.formatter.format(self.vmax)}"
        return v, ''

    def _refresh_display(self):
        # Show whatever is in the buffer verbatim while editing —
        # don't reformat mid-typing or the cursor "jumps"
        self.value_var.set(self.buf if self.buf else '0')
        # Clear stale error as soon as the user starts editing again
        self.err_var.set('')

    def _commit(self):
        v, err = self._parse()
        if err:
            self.err_var.set(err)
            return
        if self.on_commit:
            self.on_commit(v)
        self._close()

    def _cancel(self):
        self._close()

    def _close(self):
        NumPad._active = None
        self.grab_release()
        self.destroy()
        
    @classmethod
    def force_close(cls):
        """Dismiss any active numpad immediately, without committing.
        Safe to call when no numpad is open."""
        if cls._active is not None and cls._active.winfo_exists():
            cls._active._cancel()
        