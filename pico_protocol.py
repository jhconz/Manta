# -*- coding: utf-8 -*-
"""
pico_protocol.py - UART protocol 2.0 for the Quad G2 Pico (firmware robot_io.py).

Pure Python: frame builders, a stream parser (including the raw log block),
and decoders. No serial port and no GUI here, so it can be tested on its own.
The byte-level reference is PROTOCOL_CHANGES.md.
"""
import binascii
import struct
from collections import namedtuple

PROTOCOL_VERSION = (2, 2)
N_MOTORS = 4
TEMP_PRESET_ID = 0xFE          # one-off sequences (RAM only on the Pico)
DELETE_ALL = 0xFF
MAX_PRESET_ID = 253
MAX_PRESETS = 32

# ---------------------------------------------------------------- commands
CMD_STOP            = 0x01
CMD_SET_MOTOR       = 0x02     # legacy 2-channel
CMD_SEQ_ADD         = 0x03     # placeholder
CMD_SEQ_RUN         = 0x04     # legacy
CMD_GET_STATUS      = 0x05
CMD_POWERDOWN       = 0x06
CMD_PING            = 0x07
CMD_PRESET_DEFINE   = 0x08
CMD_PRESET_RUN      = 0x09
CMD_PRESET_DELETE   = 0x0A
CMD_PRESET_SYNC     = 0x0B
CMD_BALLAST_NEUTRAL = 0x0C
CMD_BALLAST_PURGE   = 0x0D
CMD_BALLAST_GOTO    = 0x0E
CMD_BALLAST_PID     = 0x0F
CMD_LOG_GET         = 0x10
CMD_MANUAL          = 0x11
CMD_SET_ZERO        = 0x12
CMD_GET_POSITIONS   = 0x13
CMD_RETURN_TO_ZERO  = 0x14
CMD_SLIP_LIMIT      = 0x15
CMD_RTZ_PID         = 0x16
CMD_DUTY_LIMIT      = 0x17

CMD_NAMES = {
    0x01: 'STOP', 0x02: 'SET_MOTOR', 0x03: 'SEQ_ADD', 0x04: 'SEQ_RUN',
    0x05: 'GET_STATUS', 0x06: 'POWERDOWN', 0x07: 'PING',
    0x08: 'PRESET_DEFINE', 0x09: 'PRESET_RUN', 0x0A: 'PRESET_DELETE',
    0x0B: 'PRESET_SYNC', 0x0C: 'BALLAST_NEUTRAL', 0x0D: 'BALLAST_PURGE',
    0x0E: 'BALLAST_GOTO', 0x0F: 'BALLAST_PID', 0x10: 'LOG_GET',
    0x11: 'MANUAL', 0x12: 'SET_ZERO', 0x13: 'GET_POSITIONS',
    0x14: 'RETURN_TO_ZERO', 0x15: 'SLIP_LIMIT', 0x16: 'RTZ_PID', 0x17: 'DUTY_LIMIT',
}

# ---------------------------------------------------------------- status bytes
ST_ACK         = 0x01
ST_DONE        = 0x02
ST_ERROR       = 0x03
ST_PONG        = 0x04
ST_EVENT       = 0xE0          # a task or manual move ended (unsolicited)
ST_BAT_WARN    = 0xF0
ST_BAT_CRIT    = 0xF1
ST_PRESET_SYNC = 0xF2
ST_PID_GAIN    = 0xF3
ST_LOG_BEGIN   = 0xF4
ST_LOG_END     = 0xF5
ST_POSITION    = 0xF6
ST_BOOT        = 0xF7
ST_BALLAST     = 0xF8          # info1 b0 = neutral stored, info2/3 = wiper raw
ST_SLIP        = 0xF9          # info1 = motor, info2/3 = limit in 0.1 deg (0 = off)

# EVENT tags (info1)
EV_SEQUENCE = 0x10             # info2/3 unused
EV_PURGE    = 0x11             # info2 = result, info3 = wiper high byte
EV_GOTO     = 0x12             # info2 = result, info3 = wiper high byte
EV_RTZ      = 0x13             # info2 = result, info3 = motors NOT at zero
EV_MANUAL   = 0x14             # info2 = reason, info3 = motor index

# ERROR codes (info1)
ERRORS = {
    0x01: 'bad checksum',
    0x02: 'frame could not be unpacked',
    0x04: 'busy',
    0x05: 'no sequence set',
    0x06: 'unknown preset',
    0x07: 'preset table full',
    0x08: 'bad value',
    0x09: 'flash save failed',
    0x0A: 'no neutral stored',
    0x0B: 'no log on the Pico',
    0x10: 'exception in a Pico handler',
    0x20: 'driver fault',
    0x21: 'slip protection cut a motor',
    0xFF: 'unknown command',
}
ERR_BUSY = 0x04
ERR_DRIVER_FAULT = 0x20
ERR_SLIP = 0x21
UNSOLICITED_ERRORS = (ERR_DRIVER_FAULT, ERR_SLIP)

RESULTS = {0: 'OK', 1: 'stalled', 2: 'timeout', 3: 'wrong direction', 4: 'stopped',
           5: 'past soft limit', 6: 'zero unknown', 7: 'outside slip limit'}
MANUAL_END = {0: 'stopped', 1: 'keepalive timeout', 2: 'slip limit', 3: 'pre-empted',
              4: 'soft limit'}
SHUTDOWN_TYPES = {0: 'unclean', 1: 'normal', 2: 'emergency'}

MANUAL_STOP, MANUAL_FORWARD, MANUAL_REVERSE = 0, 1, 2
MANUAL_BALLAST = 4             # manual index of the ballast: FORWARD fills, REVERSE empties

# Ballast wiper (raw 0-65535): purge stops at the empty limit; goto and
# manual moves stay between the two soft limits. Fuller = higher reading.
BALLAST_EMPTY_RAW = 1000
BALLAST_FULL_RAW = 64535
RTZ_GAIN_TAG = 0x10            # RTZ_PID replies: F3 info1 = 0x10 + gain index

# ---------------------------------------------------------------- messages
Frame = namedtuple('Frame', 'status i1 i2 i3')        # one 5-byte frame
LogBlock = namedtuple('LogBlock', 'raw crc_ok')       # a complete F4 ... F5 transfer


def checksum(data):
    c = 0
    for b in data:
        c ^= b
    return c


def _frame(fmt, *vals):
    body = struct.pack(fmt, *vals)
    return body + bytes([checksum(body)])


def _simple(cmd):
    return _frame('>BB', cmd, 0)


# ---------------------------------------------------------------- builders
def stop():                 return _simple(CMD_STOP)
def get_status():           return _simple(CMD_GET_STATUS)
def powerdown():            return _simple(CMD_POWERDOWN)
def ping():                 return _simple(CMD_PING)
def preset_sync():          return _simple(CMD_PRESET_SYNC)
def ballast_purge():        return _simple(CMD_BALLAST_PURGE)
def log_get():              return _simple(CMD_LOG_GET)
def get_positions():        return _simple(CMD_GET_POSITIONS)
def return_to_zero():       return _simple(CMD_RETURN_TO_ZERO)


def preset_define(preset_id, period, motors):
    """motors: 4 x (duty 0-65535, latent 0-1, phase -1..1) for M1..M4."""
    flat = []
    for duty, latent, phase in motors:
        flat += [int(duty), float(latent), float(phase)]
    return _frame('>BBf' + 'Hff' * N_MOTORS, CMD_PRESET_DEFINE, preset_id, float(period), *flat)


def preset_run(preset_id, cycles):
    return _frame('>BBH', CMD_PRESET_RUN, preset_id, int(cycles))


def preset_delete(preset_id):
    return _frame('>BB', CMD_PRESET_DELETE, preset_id)


def ballast_neutral(mode, value=0):          # mode 0 capture, 1 set, 2 query
    return _frame('>BBH', CMD_BALLAST_NEUTRAL, mode, int(value))


def ballast_goto(mode, value=0):             # mode 0 neutral, 1 value
    return _frame('>BBH', CMD_BALLAST_GOTO, mode, int(value))


def ballast_pid(mode, kp=0.0, ki=0.0, kd=0.0):   # mode 0 query, 1 set
    return _frame('>BBfff', CMD_BALLAST_PID, mode, kp, ki, kd)


def rtz_pid(mode, kp=0.0, ki=0.0, kd=0.0):       # mode 0 query, 1 set
    return _frame('>BBfff', CMD_RTZ_PID, mode, kp, ki, kd)


def duty_limit(mode, percent=0.0):                 # mode 0 query, 1 set; 10-100 %
    return _frame('>BBH', CMD_DUTY_LIMIT, mode, int(round(max(0.0, min(100.0, percent)) / 100 * 65535)))


def duty_limit_pct(res):
    """Drive PWM limit (%) from a DUTY_LIMIT reply."""
    d = res.first(ST_DONE)
    return ((d.i1 << 8) | d.i2) / 65535 * 100


def slip_limit(mode, motor=0, degrees=0.0):        # mode 0 query, 1 set; 0 deg = off
    return _frame('>BBBH', CMD_SLIP_LIMIT, mode, motor,
                  int(round(max(0.0, min(6553.5, degrees)) * 10)))


def manual(motor, action, speed_raw=0):
    """motor 0-3, or MANUAL_BALLAST. action: MANUAL_STOP / MANUAL_FORWARD / MANUAL_REVERSE.
    speed_raw: 0 = the Pico's default (10 %, ballast 40 %), else duty x 65535."""
    return _frame('>BBBH', CMD_MANUAL, motor, action, int(speed_raw))


def set_zero(mask):
    return _frame('>BB', CMD_SET_ZERO, mask & 0x0F)


# ---------------------------------------------------------------- parser
class Parser:
    """Feed it bytes as they arrive; it returns complete messages.

    Normally the stream is 5-byte frames. A valid LOG_BEGIN (F4) frame
    switches to raw mode for exactly the stated byte count, after which a
    LOG_END (F5) frame must follow; the result is one LogBlock."""

    def __init__(self):
        self._buf = bytearray()
        self._raw_left = 0
        self._raw = None

    def feed(self, data):
        self._buf.extend(data)
        out = []
        while True:
            if self._raw is not None:
                if self._raw_left:
                    take = min(self._raw_left, len(self._buf))
                    self._raw.extend(self._buf[:take])
                    del self._buf[:take]
                    self._raw_left -= take
                    if self._raw_left:
                        return out
                if len(self._buf) < 5:
                    return out
                end = bytes(self._buf[:5])
                raw, self._raw = bytes(self._raw), None
                ok = (end[0] == ST_LOG_END and checksum(end[:4]) == end[4]
                      and ((end[1] << 8) | end[2]) == binascii.crc_hqx(raw, 0xFFFF))
                if end[0] == ST_LOG_END and checksum(end[:4]) == end[4]:
                    del self._buf[:5]          # consume a well-formed end frame
                out.append(LogBlock(raw, ok))
                continue

            if len(self._buf) < 5:
                return out
            f = bytes(self._buf[:5])
            if checksum(f[:4]) != f[4]:
                del self._buf[0]              # lost sync: slide one byte
                continue
            del self._buf[:5]
            if f[0] == ST_LOG_BEGIN:
                self._raw_left = (f[1] << 16) | (f[2] << 8) | f[3]
                self._raw = bytearray()
                continue
            out.append(Frame(f[0], f[1], f[2], f[3]))


# ---------------------------------------------------------------- decoders
def position(frame):
    """F6 frame -> (motor index, count, clipped)."""
    v = (frame.i2 << 8) | frame.i3
    if v >= 0x8000:
        v -= 0x10000
    return frame.i1 & 0x7F, v, bool(frame.i1 & 0x80)


def gains(res, tag=0):
    """kp, ki, kd from the three F3 frames of a PID reply (tag RTZ_GAIN_TAG for RTZ)."""
    g = [None] * 3
    for f in res.frames(ST_PID_GAIN):
        if tag <= f.i1 < tag + 3:
            g[f.i1 - tag] = u16(f) / 100
    return g


def slip_limits(res):
    """-> [deg or None (off)] x 4 from a SLIP_LIMIT reply."""
    out = [None] * N_MOTORS
    for f in res.frames(ST_SLIP):
        if f.i1 < N_MOTORS:
            out[f.i1] = u16(f) / 10 or None
    return out


def u16(frame):
    """info2/info3 as an unsigned 16-bit value (neutral, sync CRC...)."""
    return (frame.i2 << 8) | frame.i3


def presets_crc(presets):
    """CRC the Pico reports in PRESET_SYNC.
    presets: {id: (period, [(duty, latent, phase)] x 4)}"""
    data = b''.join(
        struct.pack('>Bf' + 'Hff' * N_MOTORS, pid, period,
                    *[x for m in motors for x in (int(m[0]), m[1], m[2])])
        for pid, (period, motors) in sorted(presets.items()))
    return binascii.crc_hqx(data, 0xFFFF)


LOG_HDR = '>4sBBBBHHHff'
LOG_HDR_SIZE = struct.calcsize(LOG_HDR)
LOG_SAMPLE = '>Iiiii'
LOG_SAMPLE_SIZE = struct.calcsize(LOG_SAMPLE)
LOG_FLAGS = {0x01: 'ended early', 0x02: 'buffer full', 0x04: 'slip cut'}


def decode_log(raw):
    """-> (header dict, [(t_ms, c1, c2, c3, c4), ...]). Raises ValueError."""
    if len(raw) < LOG_HDR_SIZE:
        raise ValueError('log too short')
    (magic, ver, n_motors, pid, flags, cycles, interval, n,
     cpr, period) = struct.unpack_from(LOG_HDR, raw)
    if magic != b'SQLG':
        raise ValueError('not a sequence log')
    if len(raw) < LOG_HDR_SIZE + n * LOG_SAMPLE_SIZE:
        raise ValueError('log truncated')
    samples = [struct.unpack_from(LOG_SAMPLE, raw, LOG_HDR_SIZE + k * LOG_SAMPLE_SIZE)
               for k in range(n)]
    hdr = dict(version=ver, n_motors=n_motors, preset=pid, flags=flags, cycles=cycles,
               interval_ms=interval, n_samples=n, counts_per_rev=cpr, period=period)
    return hdr, samples


def flags_text(flags):
    return ', '.join(t for b, t in LOG_FLAGS.items() if flags & b) or 'complete'


def describe_error(frame):
    text = ERRORS.get(frame.i1, 'error 0x%02X' % frame.i1)
    if frame.i1 == 0x10:
        text += ' (%s)' % CMD_NAMES.get(frame.i2, '0x%02X' % frame.i2)
    return text


# ---------------------------------------------------------------- reply matchers
# Each takes the list of messages that arrived after a command's ACK and
# says whether the reply is complete.
def wait_done(msgs):        return any(isinstance(m, Frame) and m.status == ST_DONE for m in msgs)
def wait_pong(msgs):        return any(isinstance(m, Frame) and m.status == ST_PONG for m in msgs)
def wait_sync(msgs):        return any(isinstance(m, Frame) and m.status == ST_PRESET_SYNC for m in msgs)
def wait_log(msgs):         return any(isinstance(m, LogBlock) for m in msgs)
def wait_pid(msgs):         return sum(isinstance(m, Frame) and m.status == ST_PID_GAIN for m in msgs) >= 3
def wait_slip(msgs):        return wait_done(msgs)          # 4 x F9 come before the DONE


def wait_done_then_position(msgs):
    """MANUAL stop: DONE, then that motor's F6 (the ballast's F8)."""
    seen_done = False
    for m in msgs:
        if isinstance(m, Frame) and m.status == ST_DONE:
            seen_done = True
        elif seen_done and isinstance(m, Frame) and m.status in (ST_POSITION, ST_BALLAST):
            return True
    return False
