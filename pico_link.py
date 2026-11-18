# -*- coding: utf-8 -*-
"""
pico_link.py - serial link to the Pico.

- An RX thread parses everything the Pico sends (pico_protocol.Parser) and
  hands every message to on_message (the GUI marshals it to its own thread).
- A worker thread sends commands one at a time. Each request waits for the
  Pico's ACK and then for its reply (a matcher from pico_protocol), so replies
  are never confused between commands. Callbacks run on the worker thread.
- send_now() bypasses the queue for STOP.
"""
import queue
import threading
import time

import pico_protocol as P


class Result:
    def __init__(self, name):
        self.name = name
        self.ok = False
        self.error = None          # ERROR frame, or 'timeout' / 'no link'
        self.msgs = []             # messages after the ACK

    def frames(self, status):
        return [m for m in self.msgs if isinstance(m, P.Frame) and m.status == status]

    def first(self, status):
        f = self.frames(status)
        return f[0] if f else None

    @property
    def log(self):
        for m in self.msgs:
            if isinstance(m, P.LogBlock):
                return m
        return None

    def error_text(self):
        if isinstance(self.error, P.Frame):
            return P.describe_error(self.error)
        return str(self.error)


class _Request:
    def __init__(self, frame, done, timeout, ack_grace, name):
        self.frame = frame
        self.done = done            # matcher, or None = ACK alone completes it
        self.timeout = timeout
        self.ack_grace = ack_grace  # ACK-only: how long to wait for an immediate error
        self.result = Result(name)
        self.acked = False
        self.complete = threading.Event()

    def offer(self, msg):
        if self.complete.is_set():
            return
        if isinstance(msg, P.Frame) and msg.status == P.ST_ERROR \
                and msg.i1 not in P.UNSOLICITED_ERRORS:
            self.result.error = msg
            self.complete.set()
            return
        if not self.acked:
            if isinstance(msg, P.Frame) and msg.status == P.ST_ACK:
                self.acked = True
            return
        self.result.msgs.append(msg)
        if self.done is not None and self.done(self.result.msgs):
            self.result.ok = True
            self.complete.set()


class PicoLink:
    def __init__(self, port='/dev/serial0', baudrate=115200, on_message=None, serial_obj=None):
        if serial_obj is None:
            import serial                       # pyserial
            serial_obj = serial.Serial(port, baudrate, timeout=0.05)
        self.ser = serial_obj
        self.on_message = on_message
        self._parser = P.Parser()
        self._tx_lock = threading.Lock()
        self._pending = None
        self._pending_lock = threading.Lock()
        self._queue = queue.Queue()
        self._stop = False
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._rx.start()
        self._worker.start()

    # ------------------------------------------------------------- requests
    def submit(self, frame, done=P.wait_done, timeout=1.0, name='', callback=None,
               ack_grace=None):
        """Queue a command. done=None: an ACK (and no error within ack_grace
        seconds) is success - for commands that start a task. callback(Result)
        runs on the worker thread."""
        req = _Request(frame, done, timeout, ack_grace if done is None else None,
                       name or P.CMD_NAMES.get(frame[0], hex(frame[0])))
        self._queue.put((req, callback))
        return req

    def call(self, frame, done=P.wait_done, timeout=1.0, name='', ack_grace=None):
        """Blocking version of submit, for use OFF the GUI thread."""
        box = {}
        ev = threading.Event()
        def cb(res):
            box['r'] = res
            ev.set()
        self.submit(frame, done, timeout, name, cb, ack_grace)
        ev.wait(timeout + 5)
        return box.get('r') or Result(name)

    def queue_length(self):
        return self._queue.qsize()

    def send_now(self, frame):
        """Write immediately, skipping the queue (STOP)."""
        with self._tx_lock:
            self.ser.write(frame)

    def close(self):
        self._stop = True
        self._queue.put((None, None))
        try:
            self.ser.close()
        except Exception:
            pass

    # ------------------------------------------------------------- threads
    def _work_loop(self):
        while not self._stop:
            req, cb = self._queue.get()
            if req is None:
                return
            with self._pending_lock:
                self._pending = req
            try:
                with self._tx_lock:
                    self.ser.write(req.frame)
            except Exception as e:
                req.result.error = 'no link (%s)' % e
                req.complete.set()
            deadline = time.monotonic() + req.timeout
            while not req.complete.is_set():
                if req.done is None and req.acked:
                    # Task-starting command: an error would follow the ACK at once.
                    if not req.complete.wait(req.ack_grace or 0.15):
                        req.result.ok = True
                        req.complete.set()
                    break
                if not req.complete.wait(0.01) and time.monotonic() > deadline:
                    req.result.error = 'timeout'
                    req.complete.set()
            with self._pending_lock:
                self._pending = None
            if cb:
                try:
                    cb(req.result)
                except Exception:
                    pass

    def _rx_loop(self):
        while not self._stop:
            try:
                data = self.ser.read(64)
            except Exception:
                time.sleep(0.1)
                continue
            if not data:
                continue
            for msg in self._parser.feed(data):
                with self._pending_lock:
                    if self._pending is not None:
                        self._pending.offer(msg)
                if self.on_message:
                    try:
                        self.on_message(msg)
                    except Exception:
                        pass
