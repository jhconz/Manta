# -*- coding: utf-8 -*-
"""
robot_io.py - Onboard Pico controller for the Quad G2 signal board (MicroPython).

Architecture (carried over from onboard_pi.py):
  - Main thread:   UART polling + frame parsing + command dispatch + battery
                   and fault monitoring. Runs RobotIO.run() forever.
  - Motor thread:  Spawned on demand by CMD_SEQ_RUN (on core 1). Builds and
                   executes the motion schedule, then exits.

The original UART protocol (0x01-0x07) is unchanged: same command codes,
frame sizes, response frame and XOR checksum. New, additive commands:
  0x08 PRESET_DEFINE  cmd, id, period f32, 4 x [duty u16, latent f32, phase f32], chk (47 B)
                      -> DONE(info1=id, info2=preset count) or ERROR(0x07 full / 0x08 bad value)
  0x09 PRESET_RUN     cmd, id, cycles u16, chk (5 B)
                      -> ACK, then EVENT(0x10) when the sequence ends; ERROR(0x06) if id unknown
  0x0A PRESET_DELETE  cmd, id (0xFF = all), chk (3 B) -> DONE(info1=id, info2=count)
  0x0B-0x17           sync, ballast, log, manual, zero, positions, return to zero,
                      slip limits, return-to-zero gains, drive PWM limit: see
                      PROTOCOL_CHANGES.md, the byte-level reference (protocol 2.2).
GET_STATUS now returns DONE(info1=running, info2=fault bits, info3=preset count).
Per motor: duty 0-65535, latent 0-1 (fraction of each half-stroke left off),
phase -1..1 (shift in half-periods; 1.0 = opposite stroke to a phase-0 motor).
Each on / off / reverse edge is a smooth step (Abaqus SMOOTH STEP curve,
SMOOTH_STEP_S long, centred on the edge) rather than a jump.
Everything runs as a preset. Id 0xFE is a temporary slot (RAM only, never saved
or synced): PRESET_DEFINE 0xFE sets it (allowed mid-run), PRESET_RUN 0xFE runs it,
and the legacy SET_MOTOR / SEQ_RUN pair fills and runs the same slot. Stored ids: 0-253.

Pin map (final board):
  Left edge   M1  PWM GP0  DIR GP1  FLT GP2   ENC A/B GP3/GP4
              M2  PWM GP5  DIR GP6  FLT GP7   ENC A/B GP8/GP9
              FLT_SH GP10 (M3 + M4, open-drain wire-OR)   spare GP11
              UART0 TX GP12 / RX GP13
              Ballast PWM GP14  DIR GP15  (no fault line)
  Right edge  M3  PWM GP17 DIR GP16             ENC A/B GP18/GP19
              M4  PWM GP26 DIR GP22             ENC A/B GP20/GP21
              Battery sense GP27 (ADC1)   Ballast pot GP28 (ADC2)
"""
from machine import Pin, PWM, UART, ADC
import machine
import rp2
import _thread
import struct
import time
import json
import os

PWM_FREQ = 20_000            # G2 accepts up to 100 kHz; 20 kHz is above hearing
BALLAST_MAX_DUTY = 0.65      # 12 V JGY370 on the 16 V rail: ~10.4 V average
# Drive-motor PWM limit (M1-M4), settable over UART (DUTY_LIMIT) and saved.
# 80 % keeps four loaded motors inside the 5 A battery / boost budget in water
# (switching simulation, 2026-11-10). Applies to sequences, manual moves and
# return to zero; the ballast keeps its own BALLAST_MAX_DUTY.
DRIVE_MAX_DUTY_DEFAULT = 0.80
DRIVE_MAX_DUTY_MIN = 0.10

COUNTS_PER_MOTOR_REV = 48              # 25D encoder, both edges of both channels
GEAR_RATIO = 34.014                    # Pololu #5724 - confirm against your listing
COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO   # ~1632.7


# ======================================================================
#  Quadrature decoder (x4) on PIO, after the Raspberry Pi pico-examples
#  version. It jumps into a 16-entry table at address 0 (mov pc, isr), so
#  it is padded to exactly 32 instructions, which forces it to load at
#  offset 0 of an empty PIO block. All four encoders share one copy (PIO0).
# ======================================================================
@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _quadrature():
    jmp("update")       # 00 -> 00
    jmp("decrement")    # 00 -> 01
    jmp("increment")    # 00 -> 10
    jmp("update")       # 00 -> 11
    jmp("increment")    # 01 -> 00
    jmp("update")       # 01 -> 01
    jmp("update")       # 01 -> 10
    jmp("decrement")    # 01 -> 11
    jmp("decrement")    # 10 -> 00
    jmp("update")       # 10 -> 01
    jmp("update")       # 10 -> 10
    jmp("increment")    # 10 -> 11
    jmp("update")       # 11 -> 00
    jmp("increment")    # 11 -> 01
    jmp("decrement")    # 11 -> 10
    jmp("update")       # 11 -> 11
    label("decrement")
    jmp(y_dec, "update")
    wrap_target()
    label("update")
    mov(isr, y)
    push(noblock)
    out(isr, 2)         # previous AB
    in_(pins, 2)        # current AB -> ISR = prev<<2 | curr
    mov(osr, isr)
    mov(pc, isr)        # jump into the table
    label("increment")
    mov(y, invert(y))
    jmp(y_dec, "inc_cont")
    label("inc_cont")
    mov(y, invert(y))
    wrap()
    nop()               # padding to 32 instructions (never executed)
    nop()
    nop()
    nop()
    nop()
    nop()


class Encoder:
    def __init__(self, sm_id, pin_a, invert=False):
        Pin(pin_a, Pin.IN)              # divider already holds the line; no pulls
        Pin(pin_a + 1, Pin.IN)          # B must be the next GPIO
        self._sm = rp2.StateMachine(sm_id, _quadrature, in_base=Pin(pin_a))
        self._sign = -1 if invert else 1
        self.offset = 0                 # added to every reading (restored position)
        self._sm.active(1)

    def count(self):
        # The FIFO holds stale values: drain it plus one fresh reading.
        sm = self._sm
        v = 0
        for _ in range(sm.rx_fifo() + 1):
            v = sm.get()
        if v & 0x8000_0000:
            v -= 0x1_0000_0000
        return self._sign * v + self.offset

    def set_count(self, value):
        """Make the current position read as value."""
        self.offset += value - self.count()


class DriverFault(Exception):
    """A G2 driver reported a fault (FLT low), or the fault pins could not be
    read. Raised by RobotIO.check_faults(); every handler stops all motors."""
    def __init__(self, bits):
        super().__init__(bits)
        self.bits = bits


class PID:
    """Position PID, working in normalised units (0..1 of the wiper's travel).
    - The integral is stored already multiplied by ki, so changing ki mid-run
      causes no jump; it is clamped to +/- i_limit and frozen while the output
      is saturated in the same direction (anti-windup).
    - The derivative acts on the measurement (no kick when the setpoint changes)
      and is low-pass filtered, because the pot reading is noisy."""
    def __init__(self, kp, ki, kd, out_limit=1.0, i_limit=0.5, d_alpha=0.3):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.d_alpha = d_alpha
        self.reset(0.0)

    def reset(self, measurement):
        self._i = 0.0
        self._d = 0.0
        self._prev = measurement

    def reset_integral(self):
        self._i = 0.0

    def update(self, setpoint, measurement, dt):
        e = setpoint - measurement
        p = self.kp * e
        rate = -(measurement - self._prev) / dt if dt > 0 else 0.0
        self._prev = measurement
        self._d += self.d_alpha * (rate - self._d)
        d = self.kd * self._d

        i_new = max(-self.i_limit, min(self.i_limit, self._i + self.ki * e * dt))
        u = p + i_new + d
        if abs(u) > self.out_limit and (u > 0) == (e > 0):
            i_new = self._i                    # saturated: don't wind up further
        self._i = i_new
        u = p + self._i + d
        return max(-self.out_limit, min(self.out_limit, u))


class Motor:
    """One G2 channel: PWM magnitude + DIR, plus its encoder if it has one.
    PWM low = brake."""
    VEL_WINDOW_US = 10_000     # velocity is computed over at least this interval

    def __init__(self, pwm_pin, dir_pin, max_duty=1.0, invert=False,
                 enc_sm=None, enc_pin_a=None, enc_invert=False):
        self.dir = Pin(dir_pin, Pin.OUT, value=0)
        self._dir_level = 0        # last level written to DIR
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(PWM_FREQ)
        self.pwm.duty_u16(0)
        self.max_duty = max_duty
        self.invert = invert

        # Encoder (optional): PIO state machine + A pin (B is A + 1)
        self.enc = Encoder(enc_sm, enc_pin_a, enc_invert) if enc_pin_a is not None else None
        self.count = 0             # encoder counts since power-up
        self.velocity = 0.0        # counts per second
        self._vel_count = 0
        self._vel_us = time.ticks_us()

        # Driver slip protection: the most the output shaft may turn (degrees)
        # either side of its position at the start of a sequence before the
        # motor is cut, so variable actuator tension can't make the timing belt
        # skip. None = disabled. Set per motor once measured.
        self.slip_limit_deg = None
        self._slip_ref = 0
        self.slip_tripped = False  # once tripped, set() is ignored until re-armed

    def read_encoder(self):
        """Update count and velocity from the encoder. Returns the count."""
        if self.enc is None:
            return 0
        self.count = self.enc.count()
        now = time.ticks_us()
        dt = time.ticks_diff(now, self._vel_us)
        if dt >= self.VEL_WINDOW_US:
            self.velocity = (self.count - self._vel_count) * 1_000_000 / dt
            self._vel_count = self.count
            self._vel_us = now
        return self.count

    def output_rpm(self):
        return self.velocity / COUNTS_PER_OUTPUT_REV * 60

    def output_revs(self):
        return self.count / COUNTS_PER_OUTPUT_REV

    def arm_slip(self):
        """Take the current position as the reference and clear any trip."""
        self._slip_ref = self.count
        self.slip_tripped = False

    SLIP_LOOKAHEAD_S = 0.020   # also cut if the shaft WILL pass the limit within this time

    def slip_exceeded(self):
        """True if the shaft has passed slip_limit_deg from the reference, or
        will within SLIP_LOOKAHEAD_S at its current speed (so a fast motor is
        cut before the limit rather than just after it)."""
        if self.slip_limit_deg is None or self.enc is None:
            return False
        limit = self.slip_limit_deg / 360 * COUNTS_PER_OUTPUT_REV
        now = self.count - self._slip_ref
        ahead = now + self.velocity * self.SLIP_LOOKAHEAD_S
        return abs(now) > limit or abs(ahead) > limit

    def set(self, speed):
        """speed in -1.0 .. 1.0; magnitude clamped to max_duty.
        Ignored (motor braked) while slip protection has this motor cut."""
        if self.slip_tripped:
            self.pwm.duty_u16(0)
            return
        speed = max(-1.0, min(1.0, speed))
        if self.invert:
            speed = -speed
        level = 1 if speed < 0 else 0
        if level != self._dir_level:
            # Never reverse under drive: duty to 0 first, then DIR, then the
            # new duty (a few microseconds at zero).
            self.pwm.duty_u16(0)
            self.dir.value(level)
            self._dir_level = level
        self.pwm.duty_u16(int(min(abs(speed), self.max_duty) * 65535))

    def brake(self):
        self.pwm.duty_u16(0)


class RobotIO:
    # ---------- Command codes ----------
    CMD_STOP        = 0x01
    CMD_SET_MOTOR   = 0x02
    CMD_SEQ_ADD     = 0x03
    CMD_SEQ_RUN     = 0x04
    CMD_GET_STATUS  = 0x05
    CMD_POWERDOWN   = 0x06
    CMD_PING        = 0x07
    CMD_PRESET_DEFINE = 0x08   # store a 4-motor schedule under a preset id
    CMD_PRESET_RUN    = 0x09   # run a stored preset for N cycles
    CMD_PRESET_DELETE = 0x0A   # delete one preset (id 0xFF = delete all)
    CMD_PRESET_SYNC   = 0x0B   # ask for the preset count + CRC (STATUS_PRESET_SYNC)
    CMD_BALLAST_NEUTRAL = 0x0C # capture / set / query the neutral-buoyancy wiper value
    CMD_BALLAST_PURGE   = 0x0D # drive the ballast to empty (wiper zero)
    CMD_BALLAST_GOTO    = 0x0E # PID to neutral or to a given wiper value
    CMD_BALLAST_PID     = 0x0F # set / query the ballast PID gains
    CMD_LOG_GET         = 0x10 # resend the last sequence log
    CMD_MANUAL          = 0x11 # manual override: forward / reverse / stop one drive motor
    CMD_SET_ZERO        = 0x12 # make the current position zero for the motors in a mask
    CMD_GET_POSITIONS   = 0x13 # report all four positions, the ballast wiper, and which zeros are known
    CMD_RETURN_TO_ZERO  = 0x14 # start a return to zero on the motor thread (EVENT 0x13 when done)
    CMD_SLIP_LIMIT      = 0x15 # set / query the slip limit of each drive motor
    CMD_RTZ_PID         = 0x16 # set / query the return-to-zero PID gains (shared by all four)
    CMD_DUTY_LIMIT      = 0x17 # set / query the drive-motor PWM limit
    # ---------- Status codes ----------
    STATUS_IDLE      = 0x00
    STATUS_ACK       = 0x01
    STATUS_DONE      = 0x02
    STATUS_ERROR     = 0x03
    STATUS_PONG      = 0x04
    STATUS_BAT_WARN  = 0xF0
    STATUS_BAT_CRIT  = 0xF1
    STATUS_PRESET_SYNC = 0xF2  # info1 = count, info2/info3 = CRC-16 high/low byte
    STATUS_PID_GAIN    = 0xF3  # info1 = gain index (0 kp, 1 ki, 2 kd), info2/info3 = gain x100 (u16)
    STATUS_LOG_BEGIN   = 0xF4  # info1..info3 = byte count (u24); exactly that many raw bytes follow
    STATUS_LOG_END     = 0xF5  # info1/info2 = CRC-16 of the raw bytes, info3 = 0
    STATUS_POSITION    = 0xF6  # info1 = motor index (b7 set = value clipped), info2/info3 = count (i16)
    STATUS_BOOT        = 0xF7  # info1 = last shutdown type, info2 = mask of motors whose position was restored
    STATUS_BALLAST     = 0xF8  # info1 = b0 neutral stored, info2/info3 = wiper raw (u16)
    STATUS_SLIP        = 0xF9  # info1 = motor index, info2/info3 = slip limit in 0.1 deg (u16, 0 = off)
    # ---------- EVENT: a motor-thread task or manual move ended (unsolicited) ----------
    # Kept separate from DONE so a command reply can never look like an event.
    STATUS_EVENT     = 0xE0    # info1 = event tag below
    DONE_SEQUENCE    = 0x10    # a sequence finished
    DONE_PURGE       = 0x11    # a ballast purge finished: info2 = result, info3 = wiper high byte
    DONE_GOTO        = 0x12    # a ballast goto finished:  info2 = result, info3 = wiper high byte
    DONE_RTZ         = 0x13    # return-to-zero finished: info2 = result, info3 = motors NOT at zero (b0 M1 .. b3 M4)
    DONE_MANUAL      = 0x14    # manual motion ended by itself: info2 = reason, info3 = motor index
    # ---------- Error info codes (info1 of STATUS_ERROR) ----------
    ERR_CHECKSUM     = 0x01
    ERR_UNPACK       = 0x02
    ERR_BUSY         = 0x04
    ERR_NO_PARAMS    = 0x05
    ERR_NO_PRESET    = 0x06    # info2 = requested preset id
    ERR_PRESET_FULL  = 0x07
    ERR_BAD_VALUE    = 0x08    # e.g. period <= 0
    ERR_FLASH        = 0x09    # changed in RAM but could not be saved to flash
    ERR_NO_NEUTRAL   = 0x0A    # neutral query before any neutral was stored
    ERR_NO_LOG       = 0x0B    # LOG_GET before any sequence has been logged
    ERR_HANDLER      = 0x10
    ERR_DRIVER_FAULT = 0x20    # new: info2 = fault bits (b0 M1, b1 M2, b2 M3-or-M4, b7 pins unreadable)
    ERR_SLIP_LIMIT   = 0x21    # slip protection cut a motor: info2 = motor bits (b0 M1 .. b3 M4)
    FAULT_READ_ERROR = 0x80    # fault bit: the fault pins could not be read
    ERR_UNKNOWN_CMD  = 0xFF

    # ---------- Frame sizes ----------
    CMD_SIZES = {
        CMD_STOP:        3,
        CMD_SET_MOTOR:  22,
        CMD_SEQ_ADD:    22,
        CMD_SEQ_RUN:     4,
        CMD_GET_STATUS:  3,
        CMD_POWERDOWN:   3,
        CMD_PING:        3,
        CMD_PRESET_DEFINE: 47,
        CMD_PRESET_RUN:     5,
        CMD_PRESET_DELETE:  3,
        CMD_PRESET_SYNC:    3,
        CMD_BALLAST_NEUTRAL: 5,
        CMD_BALLAST_PURGE:   3,
        CMD_BALLAST_GOTO:    5,
        CMD_BALLAST_PID:    15,
        CMD_LOG_GET:         3,
        CMD_MANUAL:          6,
        CMD_SET_ZERO:        3,
        CMD_GET_POSITIONS:   3,
        CMD_RETURN_TO_ZERO:  3,
        CMD_SLIP_LIMIT:      6,
        CMD_RTZ_PID:        15,
        CMD_DUTY_LIMIT:      5,
    }
    RESPONSE_SIZE = 5

    # ---------- Frame layouts (big-endian, last byte = XOR checksum) ----------
    # PRESET_DEFINE: cmd, id, period(f32), then 4 x [duty(u16), latent(f32), phase(f32)]
    #                for M1..M4, chk                                     = 47 bytes
    # PRESET_RUN:    cmd, id, cycles(u16), chk                           =  5 bytes
    # PRESET_DELETE: cmd, id (0xFF = all), chk                           =  3 bytes
    FMT_PRESET_DEFINE = '>BBf' + 'Hff' * 4 + 'B'
    FMT_PRESET_RUN    = '>BBHB'
    FMT_PRESET_DELETE = '>BBB'
    MAX_PRESETS       = 32       # stored presets, ids 0..253
    TEMP_PRESET_ID    = 0xFE     # reserved: one-off sequence, RAM only, never saved or synced
    PRESET_FILE       = 'presets.json'
    SETTINGS_FILE     = 'settings.json'   # ballast neutral, and later other settings

    # ---------- Sequence log ----------
    # Recorded in RAM during the run (flash writes would stall both cores),
    # written to LOG_FILE and streamed to the host once the sequence ends.
    LOG_FILE            = 'seq_log.bin'
    LOG_MAX_SAMPLES     = 2000      # 2000 x 20 B = 40 KB, allocated once at start-up
    LOG_MIN_INTERVAL_MS = 10        # finest interval; coarser for long sequences so it all fits
    LOG_VERSION         = 1
    LOG_HDR_FMT         = '>4sBBBBHHHff'   # magic, version, n_motors, preset id, flags,
                                           # cycles, interval ms, n_samples,
                                           # counts per output rev, period s  (22 bytes)
    LOG_SAMPLE_FMT      = '>Iiiii'  # t ms since start (u32), M1..M4 encoder counts (i32)
    LOG_SAMPLE_SIZE     = 20
    LOG_FLAG_INCOMPLETE = 0x01      # ended early: STOP, fault or error
    LOG_FLAG_TRUNCATED  = 0x02      # buffer filled before the end (should not happen)
    LOG_FLAG_SLIP       = 0x04      # slip protection cut at least one motor
    LOG_CHUNK           = 512       # bytes per UART write when streaming the file

    # ---------- Ballast ----------
    # Sign = the drive direction that LOWERS the wiper reading (empties the
    # ballast). Flip it if a purge runs the wrong way. Magnitude is capped at
    # BALLAST_MAX_DUTY by the Motor object.
    BALLAST_PURGE_SPEED = -1.0
    PURGE_ZERO_RAW      = 1000      # wiper reading counted as empty (~1.5 % of travel, off the hard stop)
    PURGE_SAMPLE_MS     = 20
    PURGE_STALL_MS      = 1000      # the wiper must fall at least
    PURGE_STALL_DELTA   = 200       # this much (raw, ~0.3 % of travel) per window, or the motor stops
    PURGE_TIMEOUT_MS    = 60_000
    PURGE_WRONG_DIR_RAW = 1500      # rising this far above the start = direction is wrong
    # purge results (DONE_PURGE info2)
    PURGE_OK        = 0x00          # reached zero
    PURGE_STALLED   = 0x01          # stopped moving before zero
    PURGE_TIMEOUT   = 0x02
    PURGE_WRONG_DIR = 0x03          # wiper rose: flip BALLAST_PURGE_SPEED
    PURGE_STOPPED   = 0x04          # STOP command or driver fault
    BALLAST_LIMIT   = 0x05          # goto only: wiper left the soft limits
    # neutral command modes
    NEUTRAL_CAPTURE = 0             # store the current wiper reading
    NEUTRAL_SET     = 1             # store the value in the frame
    NEUTRAL_QUERY   = 2             # report the stored value, change nothing
    FMT_BALLAST_NEUTRAL = '>BBHB'   # cmd, mode, value (u16), chk

    # ---------- Ballast position control (PID, tune on the real ballast) ----------
    BALLAST_PID_DEFAULT = (15.0, 1.0, 0.2)   # kp, ki, kd - placeholders until tuned

    # ---------- Return to zero (drive motors, PID on encoder counts) ----------
    # Error is in output-shaft revolutions: kp = 4 gives full output 1/4 rev away.
    RTZ_PID_DEFAULT  = (4.0, 0.5, 0.05)      # kp, ki, kd - placeholders until tuned
    RTZ_TOL_COUNTS   = 5             # done when within this (~1 deg at the output) ...
    RTZ_SETTLE_MS    = 200           # ... for every motor, continuously for this long
    RTZ_MIN_OUT      = 0.08          # smallest drive that overcomes friction
    RTZ_SAMPLE_MS    = 5
    RTZ_STALL_MS     = 500           # a driven motor must move at least
    RTZ_STALL_COUNTS = 10            # this many counts per window
    RTZ_WRONG_DIR_COUNTS = 200       # error growing by this much = direction is wrong
    RTZ_TIMEOUT_MS   = 15_000
    # results beyond the shared purge/goto codes
    RTZ_NO_POSITIONS = 0x06          # no valid saved positions (unclean shutdown): did not move
    RTZ_OUT_OF_RANGE = 0x07          # a saved position was outside that motor's slip limit
    # shutdown types (stored in settings.json, reported at boot)
    SHUTDOWN_UNCLEAN   = 0          # power cut / crash: nothing was saved
    SHUTDOWN_NORMAL    = 1          # POWERDOWN command: returned to zero first
    SHUTDOWN_EMERGENCY = 2          # battery critical: positions saved, no return to zero

    # ---------- Manual override (comms thread only) ----------
    MANUAL_DEFAULT_DUTY = 0.10      # "run slow" unless the command gives a speed
    MANUAL_TIMEOUT_MS   = 3000      # keepalive: stop if not re-sent for this long. None = off
    MANUAL_STOP         = 0
    MANUAL_FORWARD      = 1
    MANUAL_REVERSE      = 2
    FMT_MANUAL          = '>BBBHB'  # cmd, motor 0-3, action, speed (u16, 0 = default), chk
    # why manual motion ended (DONE_MANUAL info2)
    MANUAL_END_STOP      = 0        # STOP action for that motor
    MANUAL_END_TIMEOUT   = 1        # keepalive expired
    MANUAL_END_SLIP      = 2        # slip protection
    MANUAL_END_PREEMPTED = 3        # a sequence / purge / goto / return-to-zero started
    MANUAL_END_LIMIT     = 4        # ballast only: reached a soft limit (empty or full)
    # Manual index 4 is the ballast: FORWARD fills (wiper rises), REVERSE empties.
    # It has no encoder, so it stops at the soft limits instead of a slip limit.
    MANUAL_BALLAST       = 4
    BALLAST_MANUAL_DUTY  = 0.40     # default for the worm drive (10 % would not turn it)
    # ---------- Slip limits / return-to-zero gains ----------
    FMT_SLIP_LIMIT      = '>BBBHB'  # cmd, mode (0 query, 1 set), motor 0-3, limit 0.1 deg (u16, 0 = off), chk
    SLIP_MAX_DEG        = 6553.5
    RTZ_GAIN_TAG        = 0x10      # RTZ_PID replies: STATUS_PID_GAIN info1 = 0x10 + gain index
    FMT_DUTY_LIMIT      = '>BBHB'   # cmd, mode (0 query, 1 set), limit (u16, duty x 65535), chk
    BALLAST_FULL_RAW    = 64535     # soft limit at the full end (mirror of PURGE_ZERO_RAW)
    GOTO_TOL_RAW        = 300       # done when within this (~0.5 % of travel) ...
    GOTO_SETTLE_MS      = 300       # ... continuously for this long
    GOTO_MIN_OUT        = 0.15      # smallest drive that overcomes stiction (fraction of full)
    GOTO_SAMPLE_MS      = 20
    GOTO_TIMEOUT_MS     = 60_000
    GOTO_LIMIT_MARGIN   = 500       # stop if the wiper passes a soft limit by this much
    # goto modes and the PID command
    GOTO_NEUTRAL        = 0
    GOTO_VALUE          = 1
    FMT_BALLAST_GOTO    = '>BBHB'   # cmd, mode, value (u16), chk
    PID_QUERY           = 0
    PID_SET             = 1
    FMT_BALLAST_PID     = '>BBfffB' # cmd, mode, kp, ki, kd (f32), chk
    PROTOCOL_VERSION  = (2, 2)  # (major, minor) returned in PONG info1/info2
    N_MOTORS          = 4

    # ---------- Battery / ADC constants ----------
    DIVIDER_R1    = 46.4        # measured values of the 47k / 10k divider
    DIVIDER_R2    = 9.8
    DIVIDER_RATIO = (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2
    VREF          = 3.3
    ADC_MAX       = 65535

    V_WARNING     = 11.0
    V_SHUTDOWN    = 10.2
    V_OFFSET      = 0.20        # measured from bench supply
    SHUTDOWN_DEBOUNCE = 10
    HYSTERESIS    = 0.3

    # ---------- Timing ----------
    BATTERY_CHECK_MS = 200
    FAULT_CHECK_MS   = 20
    ENCODER_IDLE_MS  = 10       # encoder reads from the main loop while no sequence runs
    SLEEP_SLICE_US   = 2_000    # motor thread re-checks STOP and reads encoders this often
    # ---------- Sequence waveform ----------
    # Every on/off/reverse edge of a sequence is a smooth step (the Abaqus
    # *AMPLITUDE, DEFINITION=SMOOTH STEP curve) instead of a jump, so duty
    # ramps through zero before DIR flips. The ramp is centred on the original
    # edge, so each stroke's duty-time area (and so its travel) is unchanged.
    # It is shortened where two edges are closer than this. 0 = square wave.
    SMOOTH_STEP_S    = 0.080    # ramp time of one edge, seconds
    SEQ_UPDATE_US    = 2_000    # PWM re-computed this often during a sequence

    def __init__(self, uart_baud=115200):
        # ----- Drive motors (PWM + DIR + encoder on PIO0, x4 quadrature) -----
        # Set enc_invert=True on any motor whose count runs backwards to its drive.
        self.m1 = Motor(pwm_pin=0,  dir_pin=1,  enc_sm=0, enc_pin_a=3)    # slice 0A
        self.m2 = Motor(pwm_pin=5,  dir_pin=6,  enc_sm=1, enc_pin_a=8)    # slice 2B
        self.m3 = Motor(pwm_pin=17, dir_pin=16, enc_sm=2, enc_pin_a=18)   # slice 0B (same freq as M1)
        self.m4 = Motor(pwm_pin=26, dir_pin=22, enc_sm=3, enc_pin_a=20)   # slice 5A
        self.motors = (self.m1, self.m2, self.m3, self.m4)

        # ----- Ballast (12 V worm motor, hard duty cap) -----
        self.ballast = Motor(pwm_pin=14, dir_pin=15, max_duty=BALLAST_MAX_DUTY)  # slice 7A

        # ----- Legacy 2-channel mapping (CMD_SET_MOTOR) -----
        # CMD_SET_MOTOR carries two channels (m0, m1). Each is applied to the
        # motors listed here; e.g. ((self.m1, self.m3), (self.m2, self.m4)).
        self.seq_channels = ((self.m1,), (self.m2,))

        # ----- Preset event schedules, defined over UART -----
        # id (0-254) -> {'period': s, 'motors': [(duty_u16, latent, phase)] x 4}
        self.presets = {}

        # ----- Fault inputs (G2 FLT: open-drain, active low) -----
        self.flt1  = Pin(2,  Pin.IN, Pin.PULL_UP)
        self.flt2  = Pin(7,  Pin.IN, Pin.PULL_UP)
        self.flt34 = Pin(10, Pin.IN, Pin.PULL_UP)   # M3 or M4
        self._fault_reported = False

        # ----- UART0 -----
        self.uart = UART(0,
                         baudrate=uart_baud,
                         bits=8,
                         parity=None,
                         stop=1,
                         tx=Pin(12),
                         rx=Pin(13),
                         timeout=10,
                         rxbuf=128)
        self.rx_buffer = bytearray()
        self._uart_lock = _thread.allocate_lock()

        # ----- Analog -----
        self.batt_adc = ADC(27)    # ADC1, 47k/10k divider
        self.pot_adc  = ADC(28)    # ADC2, B103 wiper across 3V3/AGND
        self.shutdown_counter = 0
        self.warning_flag = False

        # ----- Spare -----
        self.spare = Pin(11, Pin.IN, Pin.PULL_DOWN)

        # ----- Motion state -----
        self.running = False
        self._temp_preset = None       # schedule held under TEMP_PRESET_ID

        # ----- Settings stored in flash -----
        self.ballast_neutral = None    # raw wiper value (0-65535) for neutral buoyancy
        self.ballast_pid = PID(*self.BALLAST_PID_DEFAULT)
        self.rtz_pids = [PID(*self.RTZ_PID_DEFAULT) for _ in range(self.N_MOTORS)]
        self.drive_max_duty = DRIVE_MAX_DUTY_DEFAULT
        self._saved_counts = None      # encoder counts written at the last clean power-down
        self._positions_valid = [False] * self.N_MOTORS   # per motor; True only between a clean power-down and the next boot
        self._last_shutdown = self.SHUTDOWN_UNCLEAN
        self._zero_known = [False] * self.N_MOTORS        # per motor: restored at boot or set by SET_ZERO
        self._manual = {}              # motor index (4 = ballast) -> ticks_ms of its last manual command
        self._ballast_action = None    # MANUAL_FORWARD / MANUAL_REVERSE while the ballast is jogged
        self._wiper = 0                # last ballast wiper reading (raw), for reports while busy

        # ----- Sequence log (allocated once, so the heap never fragments) -----
        self._log_buf = bytearray(self.LOG_MAX_SAMPLES * self.LOG_SAMPLE_SIZE)
        self._log_n = 0
        self._log_hdr = None
        self._log_pending = False      # set by the motor thread; main loop saves + sends
        self._crc_table = self._make_crc_table()
        self.load_settings()
        self._apply_duty_limit()
        self._motor_thread_active = False

    # =====================================================================
    #  Faults
    # =====================================================================

    def faults(self):
        """Bitmask of active faults: b0 M1, b1 M2, b2 M3-or-M4.
        If the pins cannot be read, returns FAULT_READ_ERROR so the
        board fails safe (treated as a fault)."""
        try:
            return ((not self.flt1.value()) << 0 |
                    (not self.flt2.value()) << 1 |
                    (not self.flt34.value()) << 2)
        except Exception:
            return self.FAULT_READ_ERROR

    def check_faults(self):
        """Raise DriverFault if any fault is active. Call it from any loop:
        the motion loops call it every slice, so a fault ends a sequence,
        purge or goto within one step, and the main loop every FAULT_CHECK_MS."""
        f = self.faults()
        if f:
            raise DriverFault(f)
        self._fault_reported = False          # faults cleared: report the next one

    def _handle_fault(self, exc):
        """Stop every motor and report the fault once per occurrence."""
        self.stop_all_motors()
        if not self._fault_reported:
            self._fault_reported = True
            self.send_response(self.STATUS_ERROR, self.ERR_DRIVER_FAULT, exc.bits)

    # =====================================================================
    #  Battery monitor
    # =====================================================================

    def read_battery_voltage(self, samples=16):
        reading = 0
        for _ in range(samples):
            reading += self.batt_adc.read_u16()
        v_adc = (reading / (samples * self.ADC_MAX)) * self.VREF
        return v_adc * self.DIVIDER_RATIO - self.V_OFFSET

    def check_battery_voltage(self):
        v = self.read_battery_voltage()

        if v <= self.V_SHUTDOWN:
            self.shutdown_counter += 1
            if self.shutdown_counter >= self.SHUTDOWN_DEBOUNCE:
                self.send_response(self.STATUS_BAT_CRIT)
                self.software_shutdown(emergency=True)
        elif v <= self.V_WARNING:
            self.shutdown_counter = 0
            if not self.warning_flag:
                self.send_response(self.STATUS_BAT_WARN)
                self.warning_flag = True
        else:
            self.shutdown_counter = 0
            if v > self.V_WARNING + self.HYSTERESIS:
                self.warning_flag = False

    def pot_fraction(self, samples=8):
        """Ballast position, 0.0 .. 1.0 of the pot's travel."""
        return sum(self.pot_adc.read_u16() for _ in range(samples)) / samples / self.ADC_MAX

    def pot_raw(self, samples=8):
        """Ballast wiper, raw 0-65535 (averaged). Also kept in self._wiper."""
        self._wiper = sum(self.pot_adc.read_u16() for _ in range(samples)) // samples
        return self._wiper

    def report_ballast(self, read=True):
        """STATUS_BALLAST: wiper raw and whether a neutral is stored. read=False
        uses the last reading, for when the motor thread owns the ADC."""
        v = self.pot_raw() if read else self._wiper
        self.send_response(self.STATUS_BALLAST, 1 if self.ballast_neutral is not None else 0,
                           v >> 8, v & 0xFF)

    # =====================================================================
    #  Ballast
    # =====================================================================

    def set_ballast_neutral(self, value):
        """Store the neutral-buoyancy wiper value (raw 0-65535) in flash."""
        self.ballast_neutral = max(0, min(self.ADC_MAX, int(value)))
        return self.save_settings()

    def purge_ballast(self, check_faults=True):
        """Blocking. Drive the ballast toward wiper zero and stop at zero, on a
        stall (the wiper stops falling: end stop, drift or noise), on timeout,
        if the wiper rises (wrong direction), or on STOP. A driver fault raises
        DriverFault (motor braked first) unless check_faults is False, which
        the power-off protocol uses so a drive-motor fault cannot block
        surfacing. Returns (result, final wiper raw)."""
        pos = start = self.pot_raw()
        if pos <= self.PURGE_ZERO_RAW:
            return self.PURGE_OK, pos

        self.running = True
        t0 = t_window = time.ticks_ms()
        window_ref = start
        result = self.PURGE_TIMEOUT
        self.ballast.set(self.BALLAST_PURGE_SPEED)
        try:
            while True:
                if not self.running:
                    result = self.PURGE_STOPPED
                    break
                if check_faults:
                    self.check_faults()
                pos = self.pot_raw()
                now = time.ticks_ms()
                if pos <= self.PURGE_ZERO_RAW:
                    result = self.PURGE_OK
                    break
                if pos > start + self.PURGE_WRONG_DIR_RAW:
                    result = self.PURGE_WRONG_DIR
                    break
                if time.ticks_diff(now, t_window) >= self.PURGE_STALL_MS:
                    if window_ref - pos < self.PURGE_STALL_DELTA:
                        result = self.PURGE_STALLED
                        break
                    window_ref, t_window = pos, now
                if time.ticks_diff(now, t0) >= self.PURGE_TIMEOUT_MS:
                    result = self.PURGE_TIMEOUT
                    break
                self.read_encoders()
                time.sleep_ms(self.PURGE_SAMPLE_MS)
        finally:
            self.ballast.brake()
            self.running = False
        return result, pos

    def _purge_thread_entry(self):
        result, pos = self.PURGE_STOPPED, 0
        try:
            result, pos = self.purge_ballast()
        except DriverFault as e:
            self._handle_fault(e)
            pos = self.pot_raw()
        except Exception:
            self.stop_all_motors()
            self.send_response(self.STATUS_ERROR, self.ERR_HANDLER, self.CMD_BALLAST_PURGE)
        finally:
            self._motor_thread_active = False
        self.send_response(self.STATUS_EVENT, self.DONE_PURGE, result, pos >> 8)

    def start_purge(self):
        """Run purge_ballast on the motor thread. Returns an error code or None."""
        if self._motor_thread_active:
            return self.ERR_BUSY
        self.manual_stop_all()
        self._motor_thread_active = True
        _thread.start_new_thread(self._purge_thread_entry, ())
        return None

    def set_ballast_gains(self, kp, ki, kd):
        """Set and save PID gains. Returns an error code or None."""
        if not all(0 <= g < 655.36 for g in (kp, ki, kd)):    # also rejects NaN
            return self.ERR_BAD_VALUE
        pid = self.ballast_pid
        pid.kp, pid.ki, pid.kd = kp, ki, kd
        return None if self.save_settings() else self.ERR_FLASH

    def set_rtz_gains(self, kp, ki, kd):
        """Set and save the return-to-zero gains (one set, used by all four
        drive motors). Returns an error code or None."""
        if not all(0 <= g < 655.36 for g in (kp, ki, kd)):
            return self.ERR_BAD_VALUE
        for pid in self.rtz_pids:
            pid.kp, pid.ki, pid.kd = kp, ki, kd
        return None if self.save_settings() else self.ERR_FLASH

    def set_slip_limit(self, idx, tenths):
        """Set and save one drive motor's slip limit, in 0.1 deg (0 = off).
        Returns an error code or None."""
        if not 0 <= idx < self.N_MOTORS:
            return self.ERR_BAD_VALUE
        self.motors[idx].slip_limit_deg = tenths / 10 if tenths else None
        return None if self.save_settings() else self.ERR_FLASH

    def _apply_duty_limit(self):
        for m in self.motors:
            m.max_duty = self.drive_max_duty

    def set_duty_limit(self, raw):
        """Set and save the drive-motor PWM limit (raw u16 = duty x 65535).
        Takes effect at once, also on motors already moving. Returns an error
        code or None."""
        limit = raw / self.ADC_MAX
        if not DRIVE_MAX_DUTY_MIN <= limit <= 1.0:
            return self.ERR_BAD_VALUE
        self.drive_max_duty = limit
        self._apply_duty_limit()
        return None if self.save_settings() else self.ERR_FLASH

    def report_slip_limits(self):
        for i, m in enumerate(self.motors):
            v = 0 if m.slip_limit_deg is None else min(0xFFFF, int(round(m.slip_limit_deg * 10)))
            self.send_response(self.STATUS_SLIP, i, v >> 8, v & 0xFF)

    def ballast_goto(self, target):
        """Blocking PID move of the ballast wiper to target (raw 0-65535).
        The worm drive holds position unpowered, so the motor is braked
        whenever the wiper is within GOTO_TOL_RAW; the move is done once it
        stays there for GOTO_SETTLE_MS. Returns (result, final wiper raw)."""
        target = max(self.PURGE_ZERO_RAW, min(self.BALLAST_FULL_RAW, int(target)))
        # +1 output raises the wiper; BALLAST_PURGE_SPEED's sign lowers it.
        raise_sign = -1 if self.BALLAST_PURGE_SPEED > 0 else 1
        pid = self.ballast_pid

        pos = self.pot_raw()
        pid.reset(pos / self.ADC_MAX)
        err0 = abs(target - pos)
        self.running = True
        t0 = t_prev = t_window = time.ticks_ms()
        settle_start = None
        window_ref = pos
        result = self.PURGE_TIMEOUT
        try:
            while True:
                if not self.running:
                    result = self.PURGE_STOPPED
                    break
                self.check_faults()
                pos = self.pot_raw()
                now = time.ticks_ms()
                dt = max(1, time.ticks_diff(now, t_prev)) / 1000
                t_prev = now
                err = target - pos

                if abs(err) <= self.GOTO_TOL_RAW:
                    self.ballast.brake()
                    pid.reset_integral()
                    window_ref, t_window = pos, now       # not driving: no stall check
                    if settle_start is None:
                        settle_start = now
                    elif time.ticks_diff(now, settle_start) >= self.GOTO_SETTLE_MS:
                        result = self.PURGE_OK
                        break
                else:
                    settle_start = None
                    u = pid.update(target / self.ADC_MAX, pos / self.ADC_MAX, dt)
                    if abs(u) < self.GOTO_MIN_OUT:
                        u = self.GOTO_MIN_OUT if err > 0 else -self.GOTO_MIN_OUT
                    self.ballast.set(u * raise_sign)

                    if time.ticks_diff(now, t_window) >= self.PURGE_STALL_MS:
                        if abs(pos - window_ref) < self.PURGE_STALL_DELTA:
                            result = self.PURGE_STALLED
                            break
                        window_ref, t_window = pos, now

                if abs(err) > err0 + self.PURGE_WRONG_DIR_RAW:
                    result = self.PURGE_WRONG_DIR
                    break
                if (pos < self.PURGE_ZERO_RAW - self.GOTO_LIMIT_MARGIN or
                        pos > self.BALLAST_FULL_RAW + self.GOTO_LIMIT_MARGIN):
                    result = self.BALLAST_LIMIT
                    break
                if time.ticks_diff(now, t0) >= self.GOTO_TIMEOUT_MS:
                    result = self.PURGE_TIMEOUT
                    break
                self.read_encoders()
                time.sleep_ms(self.GOTO_SAMPLE_MS)
        finally:
            self.ballast.brake()
            self.running = False
        return result, pos

    def _goto_thread_entry(self, target):
        result, pos = self.PURGE_STOPPED, 0
        try:
            result, pos = self.ballast_goto(target)
        except DriverFault as e:
            self._handle_fault(e)
            pos = self.pot_raw()
        except Exception:
            self.stop_all_motors()
            self.send_response(self.STATUS_ERROR, self.ERR_HANDLER, self.CMD_BALLAST_GOTO)
        finally:
            self._motor_thread_active = False
        self.send_response(self.STATUS_EVENT, self.DONE_GOTO, result, pos >> 8)

    def start_ballast_goto(self, target):
        """Run ballast_goto on the motor thread. Returns an error code or None."""
        if self._motor_thread_active:
            return self.ERR_BUSY
        self.manual_stop_all()
        self._motor_thread_active = True
        _thread.start_new_thread(self._goto_thread_entry, (target,))
        return None

    # =====================================================================
    #  Encoder positions across power cycles, and return to zero
    # =====================================================================

    def save_positions(self, shutdown_type):
        """Power-down: store the current encoder counts and the shutdown type.
        Each motor's count is only marked valid if this boot knew its zero."""
        self.read_encoders()
        self._saved_counts = [m.count for m in self.motors]
        self._positions_valid = list(self._zero_known)
        self._last_shutdown = shutdown_type
        return self.save_settings()

    def zero_known_mask(self):
        return sum(1 << i for i, k in enumerate(self._zero_known) if k)

    def report_positions(self, mask=0x0F, read=True):
        """Send one STATUS_POSITION frame per drive motor in mask (count as
        i16, clipped, with b7 of info1 set if it didn't fit). read=False uses
        the last counts, for when the motor thread owns the encoders."""
        if read:
            self.read_encoders()
        for i, m in enumerate(self.motors):
            if not mask & (1 << i):
                continue
            c = m.count
            clipped = c > 32767 or c < -32768
            v = max(-32768, min(32767, c)) & 0xFFFF
            self.send_response(self.STATUS_POSITION, i | (0x80 if clipped else 0), v >> 8, v & 0xFF)

    def restore_positions(self):
        """Boot: load the counts saved at the last clean power-down, then mark
        them used at once, so a later power cut without a clean shutdown
        can't make the next boot trust stale positions. Returns the mask of
        motors whose position was restored."""
        last = self._last_shutdown
        restored = 0
        if self._saved_counts is not None:
            for i, (m, c) in enumerate(zip(self.motors, self._saved_counts)):
                if self._positions_valid[i] and m.enc is not None:
                    m.enc.set_count(c)
                    self._zero_known[i] = True
                    restored |= 1 << i
            self.read_encoders()
        # Mark everything used: if this boot ends in a power cut, the next one
        # sees an unclean shutdown and no valid positions.
        self._positions_valid = [False] * self.N_MOTORS
        self._last_shutdown = self.SHUTDOWN_UNCLEAN
        self.save_settings()
        self.send_response(self.STATUS_BOOT, last, restored)
        return restored

    def set_zero_here(self, mask=0x0F):
        """Define the current position of each motor in mask as its zero."""
        for i, m in enumerate(self.motors):
            if mask & (1 << i) and m.enc is not None:
                m.enc.set_count(0)
                self._zero_known[i] = True
        self.read_encoders()

    def return_to_zero(self):
        """Blocking. Drive every drive motor to encoder count 0 with its own
        PID, all at once. A motor is left alone, and reported, if its zero is
        unknown (unclean shutdown, not re-zeroed) or its position is outside
        its slip limit; it is braked and reported if it stalls or runs the
        wrong way. Returns (result, bits of motors NOT at zero)."""
        self.read_encoders()
        result = self.PURGE_OK
        failed = 0
        active = []
        for i, m in enumerate(self.motors):
            m.slip_tripped = False
            if m.enc is None:
                continue
            if not self._zero_known[i]:
                failed |= 1 << i
                if result == self.PURGE_OK:
                    result = self.RTZ_NO_POSITIONS
                continue
            if m.slip_limit_deg is not None and \
                    abs(m.count) > m.slip_limit_deg / 360 * COUNTS_PER_OUTPUT_REV:
                failed |= 1 << i
                result = self.RTZ_OUT_OF_RANGE
                continue
            self.rtz_pids[i].reset(m.count / COUNTS_PER_OUTPUT_REV)
            active.append(i)
        if not active:
            return result, failed

        err0 = {i: abs(self.motors[i].count) for i in active}
        win_ref = {i: self.motors[i].count for i in active}
        settled = {i: False for i in active}
        self.running = True
        t0 = t_prev = t_win = time.ticks_ms()
        settle_start = None
        try:
            while active:
                if not self.running:
                    result = self.PURGE_STOPPED
                    for i in active:
                        failed |= 1 << i
                    break
                self.check_faults()
                self.read_encoders()
                now = time.ticks_ms()
                dt = max(1, time.ticks_diff(now, t_prev)) / 1000
                t_prev = now
                window_due = time.ticks_diff(now, t_win) >= self.RTZ_STALL_MS

                for i in list(active):
                    m = self.motors[i]
                    err = -m.count
                    if abs(err) <= self.RTZ_TOL_COUNTS:
                        m.brake()
                        self.rtz_pids[i].reset_integral()
                        settled[i] = True
                        win_ref[i] = m.count
                        continue
                    settled[i] = False
                    u = self.rtz_pids[i].update(0.0, m.count / COUNTS_PER_OUTPUT_REV, dt)
                    if abs(u) < self.RTZ_MIN_OUT:
                        u = self.RTZ_MIN_OUT if err > 0 else -self.RTZ_MIN_OUT
                    m.set(u)
                    bad = None
                    if abs(err) > err0[i] + self.RTZ_WRONG_DIR_COUNTS:
                        bad = self.PURGE_WRONG_DIR
                    elif window_due and abs(m.count - win_ref[i]) < self.RTZ_STALL_COUNTS:
                        bad = self.PURGE_STALLED
                    if bad is not None:
                        m.brake()
                        active.remove(i)
                        failed |= 1 << i
                        if result == self.PURGE_OK:
                            result = bad
                    elif window_due:
                        win_ref[i] = m.count
                if window_due:
                    t_win = now

                if active and all(settled[i] for i in active):
                    if settle_start is None:
                        settle_start = now
                    elif time.ticks_diff(now, settle_start) >= self.RTZ_SETTLE_MS:
                        break
                else:
                    settle_start = None
                if time.ticks_diff(now, t0) >= self.RTZ_TIMEOUT_MS:
                    for i in active:
                        if not settled[i]:
                            failed |= 1 << i
                    if result == self.PURGE_OK:
                        result = self.PURGE_TIMEOUT
                    break
                time.sleep_ms(self.RTZ_SAMPLE_MS)
        finally:
            for m in self.motors:
                m.brake()
            self.running = False
        return result, failed

    def _rtz_thread_entry(self):
        result, failed = self.PURGE_STOPPED, (1 << self.N_MOTORS) - 1
        try:
            result, failed = self.return_to_zero()
        except DriverFault as e:
            self._handle_fault(e)
        except Exception:
            self.stop_all_motors()
            self.send_response(self.STATUS_ERROR, self.ERR_HANDLER, self.CMD_RETURN_TO_ZERO)
        finally:
            self._motor_thread_active = False
        self.send_response(self.STATUS_EVENT, self.DONE_RTZ, result, failed)

    def start_return_to_zero(self):
        """Run return_to_zero on the motor thread. Returns an error code or None."""
        if self._motor_thread_active:
            return self.ERR_BUSY
        self.manual_stop_all()
        self._motor_thread_active = True
        _thread.start_new_thread(self._rtz_thread_entry, ())
        return None

    # =====================================================================
    #  Power-off protocol
    # =====================================================================

    def software_shutdown(self, emergency=False, final_done=False):
        """Normal (POWERDOWN command):
            stop -> return to zero (DONE 0x13) -> report positions (F6 x4)
            -> purge ballast (DONE 0x11) -> save positions -> DONE -> sleep
        Emergency (battery critical) - spend as little energy as possible on
        the drive motors:
            stop -> purge ballast (DONE 0x11) -> save positions -> sleep
        Positions go to settings.json either way, so the next boot can
        recover them and return to zero."""
        self.stop_all_motors()
        t0 = time.ticks_ms()
        while self._motor_thread_active and time.ticks_diff(time.ticks_ms(), t0) < 500:
            time.sleep_ms(5)

        if not emergency:
            try:
                result, failed = self.return_to_zero()
            except DriverFault as e:
                self._handle_fault(e)
                result, failed = self.PURGE_STOPPED, (1 << self.N_MOTORS) - 1
            except Exception:
                self.stop_all_motors()
                result, failed = self.PURGE_STOPPED, (1 << self.N_MOTORS) - 1
            self.send_response(self.STATUS_EVENT, self.DONE_RTZ, result, failed)
            self.report_positions()

        try:
            result, pos = self.purge_ballast(check_faults=False)
        except Exception:
            self.stop_all_motors()
            result, pos = self.PURGE_STOPPED, 0
        self.send_response(self.STATUS_EVENT, self.DONE_PURGE, result, pos >> 8)

        # Everything has stopped: record where the drive motors are.
        self.stop_all_motors()
        self.save_positions(self.SHUTDOWN_EMERGENCY if emergency else self.SHUTDOWN_NORMAL)
        if final_done:
            self.send_response(self.STATUS_DONE)

        try:
            self.uart.flush()
        except AttributeError:
            pass
        time.sleep_ms(50)
        self.stop_all_motors()
        machine.deepsleep()

    # =====================================================================
    #  Motor control - low level
    # =====================================================================

    def stop_all_motors(self):
        self.running = False
        self._manual.clear()
        self._ballast_action = None
        for m in self.motors:
            m.brake()
        self.ballast.brake()

    # =====================================================================
    #  Manual override - comms thread only, never on the motor thread
    # =====================================================================

    def manual_command(self, idx, action, speed_raw):
        """Forward / reverse / stop one drive motor. Default speed is
        MANUAL_DEFAULT_DUTY; speed_raw (1-65535) overrides it, still capped by
        the motor's max_duty. idx MANUAL_BALLAST (4) jogs the ballast:
        FORWARD fills, REVERSE empties, default BALLAST_MANUAL_DUTY, and it is
        refused (ERR_BAD_VALUE) if already at the soft limit it would push
        past. Returns an error code or None."""
        if not (0 <= idx <= self.MANUAL_BALLAST) or action not in (
                self.MANUAL_STOP, self.MANUAL_FORWARD, self.MANUAL_REVERSE):
            return self.ERR_BAD_VALUE
        ballast = idx == self.MANUAL_BALLAST
        m = self.ballast if ballast else self.motors[idx]
        if action == self.MANUAL_STOP:
            self._manual.pop(idx, None)
            m.brake()
            m.slip_tripped = False
            if ballast:
                self._ballast_action = None
            return None
        if self._motor_thread_active:
            return self.ERR_BUSY
        if ballast:
            if self._ballast_at_limit(action, self.pot_raw()):
                return self.ERR_BAD_VALUE
            duty = self.BALLAST_MANUAL_DUTY if speed_raw == 0 else speed_raw / self.ADC_MAX
            raise_sign = -1 if self.BALLAST_PURGE_SPEED > 0 else 1
            self._manual[idx] = time.ticks_ms()
            self._ballast_action = action
            m.set(duty * raise_sign * (1 if action == self.MANUAL_FORWARD else -1))
            return None
        duty = self.MANUAL_DEFAULT_DUTY if speed_raw == 0 else speed_raw / self.ADC_MAX
        if idx not in self._manual:
            self.read_encoders()
            m.arm_slip()                    # slip limit measured from where this push started
        self._manual[idx] = time.ticks_ms()  # also the keepalive
        m.set(duty if action == self.MANUAL_FORWARD else -duty)
        return None

    def _manual_end(self, idx, reason):
        """Stop a manual motor that ended by itself and tell the host why,
        followed by its position."""
        if self._manual.pop(idx, None) is None:
            return
        if idx == self.MANUAL_BALLAST:
            self.ballast.brake()
            self._ballast_action = None
            self.send_response(self.STATUS_EVENT, self.DONE_MANUAL, reason, idx)
            self.report_ballast(read=False)
            return
        m = self.motors[idx]
        m.brake()
        m.slip_tripped = False
        self.send_response(self.STATUS_EVENT, self.DONE_MANUAL, reason, idx)
        self.report_positions(1 << idx, read=False)

    def _ballast_at_limit(self, action, pos):
        return ((action == self.MANUAL_FORWARD and pos >= self.BALLAST_FULL_RAW) or
                (action == self.MANUAL_REVERSE and pos <= self.PURGE_ZERO_RAW))

    def manual_stop_all(self):
        """Pre-empt all manual motion (an automatic task is about to start)."""
        for idx in list(self._manual):
            self._manual_end(idx, self.MANUAL_END_PREEMPTED)

    def _manual_service(self):
        """Main loop: slip protection and keepalive for manual motors."""
        if not self._manual or self._motor_thread_active:
            return
        self.read_encoders()
        now = time.ticks_ms()
        for idx in list(self._manual):
            if idx == self.MANUAL_BALLAST:
                stop = (self.MANUAL_END_LIMIT
                        if self._ballast_at_limit(self._ballast_action, self.pot_raw()) else None)
            else:
                stop = self.MANUAL_END_SLIP if self.motors[idx].slip_exceeded() else None
            if stop is None and (self.MANUAL_TIMEOUT_MS is not None and
                                 time.ticks_diff(now, self._manual[idx]) >= self.MANUAL_TIMEOUT_MS):
                stop = self.MANUAL_END_TIMEOUT
            if stop is not None:
                self._manual_end(idx, stop)

    def set_motor_state(self, m0_duty, m1_duty, period, m0_latent, m1_latent, phase):
        """Legacy 2-channel CMD_SET_MOTOR -> the temporary preset (0xFE).
        Channel 0 motors get (m0_duty, m0_latent, phase 0); channel 1 motors get
        (m1_duty, m1_latent, phase). Motors in neither channel stay off.
        Atomic swap - safe to call while the motor thread is running."""
        motors = [(0, 0.0, 0.0)] * self.N_MOTORS
        for i, m in enumerate(self.motors):
            if m in self.seq_channels[0]:
                motors[i] = (m0_duty, m0_latent, 0.0)
            elif m in self.seq_channels[1]:
                motors[i] = (m1_duty, m1_latent, phase)
        self._temp_preset = {'period': period, 'motors': motors}

    def read_encoders(self):
        """Update count and velocity on every drive motor."""
        for m in self.motors:
            m.read_encoder()

    # =====================================================================
    #  Presets
    # =====================================================================

    def define_preset(self, preset_id, period, motors):
        """motors: 4 x (duty_u16, latent 0..1, phase -1..1) for M1..M4.
        Returns an error code, or None on success."""
        if not (0 <= preset_id <= self.TEMP_PRESET_ID):
            return self.ERR_BAD_VALUE
        if not (period > 0) or len(motors) != self.N_MOTORS:   # also rejects NaN
            return self.ERR_BAD_VALUE
        sched = {'period': period, 'motors': [tuple(m) for m in motors]}
        if preset_id == self.TEMP_PRESET_ID:
            self._temp_preset = sched          # RAM only
            return None
        if preset_id not in self.presets and len(self.presets) >= self.MAX_PRESETS:
            return self.ERR_PRESET_FULL
        self.presets[preset_id] = sched
        return None

    def get_preset(self, preset_id):
        """Schedule for an id (stored or the temporary slot), or None."""
        if preset_id == self.TEMP_PRESET_ID:
            return self._temp_preset
        return self.presets.get(preset_id)

    def run_preset(self, preset_id, cycles):
        """The single entry point for running anything. Returns an error
        code, or None once the motor thread has started."""
        sched = self.get_preset(preset_id)
        if sched is None:
            return self.ERR_NO_PRESET
        if cycles <= 0:
            return self.ERR_BAD_VALUE
        if self._motor_thread_active or self._log_pending:
            return self.ERR_BUSY
        self.manual_stop_all()
        self._motor_thread_active = True
        _thread.start_new_thread(self._motor_thread_entry, (sched, cycles, preset_id))
        return None

    def delete_preset(self, preset_id):
        if preset_id == self.TEMP_PRESET_ID:
            self._temp_preset = None
        elif preset_id == 0xFF:
            self.presets.clear()
        else:
            self.presets.pop(preset_id, None)

    # ----- Preset persistence and sync -----

    def save_presets(self):
        """Write presets to flash. Write-then-rename, so a reset mid-save
        leaves the previous file intact. Returns True on success."""
        data = {str(k): {'period': v['period'], 'motors': [list(m) for m in v['motors']]}
                for k, v in self.presets.items()}
        return self._write_json(self.PRESET_FILE, data)

    @staticmethod
    def _write_json(path, data):
        """Write-then-rename, so a reset mid-save leaves the old file intact."""
        tmp = path + '.tmp'
        try:
            with open(tmp, 'w') as f:
                json.dump(data, f)
            try:
                os.remove(path)
            except OSError:
                pass
            os.rename(tmp, path)
            return True
        except Exception:
            return False

    def save_settings(self):
        pid, rtz = self.ballast_pid, self.rtz_pids[0]
        return self._write_json(self.SETTINGS_FILE, {
            'ballast_neutral': self.ballast_neutral,
            'ballast_pid': [pid.kp, pid.ki, pid.kd],
            'rtz_pid': [rtz.kp, rtz.ki, rtz.kd],
            'slip_limit_deg': [m.slip_limit_deg for m in self.motors],
            'drive_max_duty': self.drive_max_duty,
            'encoder_counts': self._saved_counts,
            'positions_valid': self._positions_valid,
            'last_shutdown': self._last_shutdown,
        })

    def load_settings(self):
        try:
            with open(self.SETTINGS_FILE) as f:
                data = json.load(f)
            v = data.get('ballast_neutral')
            self.ballast_neutral = None if v is None else max(0, min(self.ADC_MAX, int(v)))
            g = data.get('ballast_pid')
            if g is not None and len(g) == 3 and all(x >= 0 for x in g):
                self.ballast_pid.kp, self.ballast_pid.ki, self.ballast_pid.kd = (float(x) for x in g)
            g = data.get('rtz_pid')
            if g is not None and len(g) == 3 and all(x >= 0 for x in g):
                for p in self.rtz_pids:
                    p.kp, p.ki, p.kd = (float(x) for x in g)
            v = data.get('drive_max_duty')
            if v is not None and DRIVE_MAX_DUTY_MIN <= v <= 1.0:
                self.drive_max_duty = float(v)
            s = data.get('slip_limit_deg')
            if isinstance(s, list) and len(s) == self.N_MOTORS:
                for m, v in zip(self.motors, s):
                    m.slip_limit_deg = None if v is None or v <= 0 else float(v)
            c = data.get('encoder_counts')
            if c is not None and len(c) == self.N_MOTORS:
                self._saved_counts = [int(x) for x in c]
                pv = data.get('positions_valid')
                if isinstance(pv, list) and len(pv) == self.N_MOTORS:
                    self._positions_valid = [bool(x) for x in pv]
                else:
                    self._positions_valid = [bool(pv)] * self.N_MOTORS
            self._last_shutdown = int(data.get('last_shutdown', self.SHUTDOWN_UNCLEAN))
        except Exception:
            self.ballast_neutral = None

    def load_presets(self):
        """Read presets from flash, skipping any entry that fails validation.
        Returns the number loaded."""
        self.presets = {}
        try:
            with open(self.PRESET_FILE) as f:
                data = json.load(f)
        except Exception:
            return 0                    # no file yet, or unreadable
        for k, v in data.items():
            try:
                if int(k) >= self.TEMP_PRESET_ID:
                    continue
                self.define_preset(int(k), float(v['period']),
                                   [(int(d), float(l), float(p)) for d, l, p in v['motors']])
            except Exception:
                pass
        return len(self.presets)

    def presets_crc(self):
        """CRC-16/CCITT-FALSE over the preset table, so the host can compare
        its copy without reading every preset back. Byte stream, ids ascending:
          id (u8), period (f32), then M1..M4: duty (u16), latent (f32), phase (f32)
        all big-endian. An empty table gives 0xFFFF."""
        crc = 0xFFFF
        for pid in sorted(self.presets):
            p = self.presets[pid]
            vals = [pid, p['period']]
            for m in p['motors']:
                vals.extend(m)
            crc = self._crc16(struct.pack('>Bf' + 'Hff' * self.N_MOTORS, *vals), crc)
        return crc

    @staticmethod
    def _make_crc_table():
        table = []
        for i in range(256):
            c = i << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
            table.append(c)
        return table

    def _crc16(self, data, crc=0xFFFF):
        """CRC-16/CCITT-FALSE (poly 0x1021), continuing from crc.
        Matches Python's binascii.crc_hqx(data, 0xFFFF)."""
        t = self._crc_table
        for b in data:
            crc = ((crc << 8) & 0xFFFF) ^ t[(crc >> 8) ^ b]
        return crc

    def send_preset_sync(self):
        crc = self.presets_crc()
        self.send_response(self.STATUS_PRESET_SYNC, len(self.presets), crc >> 8, crc & 0xFF)

    def sync_presets(self):
        """Startup sync: reload presets from flash, then tell the host what is
        stored (count + CRC). The host compares with its own table and resends
        any presets that differ (or PRESET_DELETE 0xFF and resends all)."""
        self.load_presets()
        self.send_preset_sync()

    # =====================================================================
    #  Motor control - sequence building and execution
    # =====================================================================

    @staticmethod
    def smooth_step(xi):
        """Abaqus SMOOTH STEP amplitude between two data points:
        a = a0 + (a1 - a0) * xi^3 * (10 - 15 xi + 6 xi^2),  xi = (t - t0) / (t1 - t0).
        Zero first and second derivatives at both ends. xi is clamped to 0..1."""
        if xi <= 0.0:
            return 0.0
        if xi >= 1.0:
            return 1.0
        return xi * xi * xi * (10.0 + xi * (-15.0 + 6.0 * xi))

    def _build_motion_schedule(self, sched, n_cycles):
        """sched = {'period': s, 'motors': [(duty_u16, latent, phase)] x 4}.
        Each motor follows the same fwd/rev pattern (quarter, n half-periods,
        quarter), scaled by its duty, shortened by its latent fraction, and
        shifted by phase x half-period.

        Returns (tracks, total_duration). tracks[i] is motor i's amplitude
        table: [(t, speed), ...] in time order. Between two points the speed
        follows smooth_step() (constant if both values are equal; a jump if
        both times are equal). Empty for a motor with duty 0."""
        T = sched['period']
        half = T / 2.0
        quarter = T / 4.0

        segments = []
        t = 0.0
        segments.append((t, quarter, 'fwd'))
        t += quarter
        for i in range(2 * n_cycles - 1):
            direction = 'rev' if i % 2 == 0 else 'fwd'
            segments.append((t, half, direction))
            t += half
        segments.append((t, quarter, 'fwd'))
        t += quarter
        total_duration = t

        tracks = []
        for duty, latent, phase in sched['motors']:
            speed = min(duty / self.ADC_MAX, self.drive_max_duty)   # u16 duty -> 0..1, capped
            if speed <= 0:
                tracks.append([])
                continue
            latent = max(0.0, min(1.0, latent))
            phase = max(-1.0, min(1.0, phase))
            edges = self._square_edges(segments, speed, latent, phase * half)
            tracks.append(self._smooth_track(edges, self.SMOOTH_STEP_S))

        # Phase offsets and ramps that start before t = 0 shift everything later.
        times = [pts[0][0] for pts in tracks if pts]
        min_t = min(times) if times else 0.0
        if min_t < 0:
            tracks = [[(t - min_t, v) for t, v in pts] for pts in tracks]
            total_duration -= min_t
        for pts in tracks:
            if pts:
                total_duration = max(total_duration, pts[-1][0])
        return tracks, total_duration

    @staticmethod
    def _square_edges(segments, speed, latent, offset):
        """The square wave as edges [(t, before, after)]: + speed for 'fwd',
        - speed for 'rev' (sets DIR), 0 while off. An off edge and an on
        edge at the same instant merge into one fwd <-> rev edge."""
        edges = []
        for start, duration, direction in segments:
            on_time = duration * (1.0 - latent)
            if on_time <= 0:
                continue
            sp = speed if direction == 'fwd' else -speed
            t_on, t_off = start + offset, start + offset + on_time
            if edges and edges[-1][2] == 0.0 and abs(edges[-1][0] - t_on) < 1e-9:
                edges[-1] = (t_on, edges[-1][1], sp)       # merge off+on
            else:
                edges.append((t_on, 0.0, sp))
            edges.append((t_off, sp, 0.0))
        return edges

    @staticmethod
    def _smooth_track(edges, ramp):
        """Edges -> amplitude table. Each edge becomes a smooth step from
        (t - r/2, before) to (t + r/2, after); r is `ramp`, cut down to the
        gap to the neighbouring edges so steps never overlap. Because the
        curve is point-symmetric about its midpoint, the area under each
        stroke matches the square wave's."""
        pts = []
        n = len(edges)
        for k, (t, a, b) in enumerate(edges):
            r = ramp
            if k > 0:
                r = min(r, t - edges[k - 1][0])
            if k < n - 1:
                r = min(r, edges[k + 1][0] - t)
            r = max(0.0, r)
            pts.append((t - r / 2, a))
            pts.append((t + r / 2, b))
        return pts

    def _sleep_until(self, target_us):
        """Wait in short slices: read encoders, check faults, honour STOP."""
        while self.running:
            self.read_encoders()
            self._check_slip()
            self._log_sample()
            self.check_faults()             # raises DriverFault
            wait = time.ticks_diff(target_us, time.ticks_us())
            if wait <= 0:
                return True
            time.sleep_us(min(wait, self.SLEEP_SLICE_US))
        return False

    # ----- Driver slip protection -----

    def _check_slip(self):
        """Cut any motor that has passed its slip limit, keep the others
        running, and report the newly cut motors once."""
        newly = 0
        for i, m in enumerate(self.motors):
            if not m.slip_tripped and m.slip_exceeded():
                m.slip_tripped = True
                m.brake()
                newly |= 1 << i
        if newly:
            if self._log_hdr is not None:
                self._log_hdr[1] |= self.LOG_FLAG_SLIP
            self.send_response(self.STATUS_ERROR, self.ERR_SLIP_LIMIT, newly)

    # ----- Sequence log -----

    def _log_start(self, preset_id, cycles, period, total_s):
        """Pick the interval so the whole sequence fits in the buffer."""
        interval_ms = max(self.LOG_MIN_INTERVAL_MS,
                          int(total_s * 1000 / (self.LOG_MAX_SAMPLES - 2)) + 1)
        self._log_interval_us = interval_ms * 1000
        self._log_n = 0
        self._log_hdr = [preset_id, 0, cycles, interval_ms, period]   # flags at [1]
        self._log_t0 = self._log_next = time.ticks_us()
        self._log_sample(force=True)
        for m in self.motors:
            m.arm_slip()

    def _log_sample(self, force=False):
        """Record t + all four counts if the interval is due. Uses the counts
        from the read_encoders() call that precedes it in the loop."""
        now = time.ticks_us()
        if not force and time.ticks_diff(now, self._log_next) < 0:
            return
        t_ms = time.ticks_diff(now, self._log_t0) // 1000
        slot = self._log_n
        # A forced sample in the same millisecond as the previous one replaces
        # it, so timestamps stay strictly increasing.
        if slot and struct.unpack_from('>I', self._log_buf,
                                       (slot - 1) * self.LOG_SAMPLE_SIZE)[0] == t_ms:
            slot -= 1
        elif slot >= self.LOG_MAX_SAMPLES:
            self._log_hdr[1] |= self.LOG_FLAG_TRUNCATED
            return
        struct.pack_into(self.LOG_SAMPLE_FMT, self._log_buf,
                         slot * self.LOG_SAMPLE_SIZE, t_ms,
                         *[m.count for m in self.motors])
        self._log_n = slot + 1
        self._log_next = time.ticks_add(self._log_next, self._log_interval_us)
        if time.ticks_diff(self._log_next, now) < 0:      # fell behind: don't burst
            self._log_next = time.ticks_add(now, self._log_interval_us)

    def save_log(self):
        """Write header + samples to LOG_FILE (main thread only)."""
        pid, flags, cycles, interval_ms, period = self._log_hdr
        hdr = struct.pack(self.LOG_HDR_FMT, b'SQLG', self.LOG_VERSION, self.N_MOTORS,
                          pid, flags, cycles, interval_ms, self._log_n,
                          COUNTS_PER_OUTPUT_REV, period)
        tmp = self.LOG_FILE + '.tmp'
        try:
            with open(tmp, 'wb') as f:
                f.write(hdr)
                f.write(memoryview(self._log_buf)[:self._log_n * self.LOG_SAMPLE_SIZE])
            try:
                os.remove(self.LOG_FILE)
            except OSError:
                pass
            os.rename(tmp, self.LOG_FILE)
            return True
        except Exception:
            return False

    def send_log(self):
        """Stream LOG_FILE: F4 <len u24>, raw bytes, F5 <crc hi> <crc lo>.
        Holds the UART lock throughout so no other frame lands mid-transfer.
        Returns False if there is no log file."""
        try:
            size = os.stat(self.LOG_FILE)[6]
            f = open(self.LOG_FILE, 'rb')
        except OSError:
            return False
        crc = 0xFFFF
        chunk = bytearray(self.LOG_CHUNK)
        with self._uart_lock:
            b1, b2, b3 = (size >> 16) & 0xFF, (size >> 8) & 0xFF, size & 0xFF
            self.uart.write(bytes([self.STATUS_LOG_BEGIN, b1, b2, b3,
                                   self.STATUS_LOG_BEGIN ^ b1 ^ b2 ^ b3]))
            try:
                while True:
                    n = f.readinto(chunk)
                    if not n:
                        break
                    view = memoryview(chunk)[:n]
                    crc = self._crc16(view, crc)
                    self.uart.write(view)
            finally:
                f.close()
            hi, lo = crc >> 8, crc & 0xFF
            self.uart.write(bytes([self.STATUS_LOG_END, hi, lo, 0,
                                   self.STATUS_LOG_END ^ hi ^ lo]))
        return True

    def _finish_log(self):
        """Main loop: save the finished log to flash, then send it."""
        self._log_pending = False
        if not self.save_log():
            self.send_response(self.STATUS_ERROR, self.ERR_FLASH, self.CMD_LOG_GET)
            return
        self.send_log()

    def _track_value(self, pts, k, t):
        """Speed of one amplitude table at time t (s). k = index to start
        searching from (times only increase); returns (speed, new k)."""
        n = len(pts)
        if n == 0:
            return 0.0, 0
        while k < n - 1 and t >= pts[k + 1][0]:
            k += 1
        if k >= n - 1 or t < pts[0][0]:
            return (pts[-1][1] if k >= n - 1 else pts[0][1]), k
        t0, a0 = pts[k]
        t1, a1 = pts[k + 1]
        if a0 == a1 or t1 <= t0:
            return a0, k
        return a0 + (a1 - a0) * self.smooth_step((t - t0) / (t1 - t0)), k

    def _run_motion(self, tracks, total_duration):
        """Play the amplitude tables, re-computing every motor's duty each
        SEQ_UPDATE_US. Returns True if it ran to the end, False if stopped."""
        start_us = time.ticks_us()
        end_us = int(total_duration * 1_000_000)
        cursor = [0] * self.N_MOTORS
        last = [None] * self.N_MOTORS
        step = self.SEQ_UPDATE_US
        while True:
            t_us = time.ticks_diff(time.ticks_us(), start_us)
            t = t_us / 1_000_000
            for i, pts in enumerate(tracks):
                v, cursor[i] = self._track_value(pts, cursor[i], t)
                if v != last[i]:                    # holds don't rewrite the PWM
                    self.motors[i].set(v)
                    last[i] = v
            if t_us >= end_us:
                break
            nxt = min(end_us, (t_us // step + 1) * step)
            if not self._sleep_until(time.ticks_add(start_us, nxt)):
                self.stop_all_motors()
                return False
        self.stop_all_motors()
        return True

    def _motor_thread_entry(self, sched, n_cycles, preset_id=0xFE):
        completed = False
        logging = False
        try:
            tracks, total_duration = self._build_motion_schedule(sched, n_cycles)
            self.running = True
            self._log_start(preset_id, n_cycles, sched['period'], total_duration)
            logging = True
            completed = self._run_motion(tracks, total_duration)
        except DriverFault as e:
            self._handle_fault(e)
        except Exception:
            # Never leave motors running because the thread died.
            self.stop_all_motors()
            self.send_response(self.STATUS_ERROR, self.ERR_HANDLER, self.CMD_SEQ_RUN)
        finally:
            if logging:
                try:
                    self.read_encoders()            # final position, after the stop
                    self._log_sample(force=True)
                except Exception:
                    pass
                if not completed:
                    self._log_hdr[1] |= self.LOG_FLAG_INCOMPLETE
            self._motor_thread_active = False
        self.send_response(self.STATUS_EVENT, self.DONE_SEQUENCE)
        if logging:
            self._log_pending = True    # after DONE, so the log always follows it

    # =====================================================================
    #  UART communication
    # =====================================================================

    def send_response(self, status, info1=0, info2=0, info3=0):
        chk = status ^ info1 ^ info2 ^ info3
        frame = bytes([status, info1, info2, info3, chk])
        with self._uart_lock:
            self.uart.write(frame)

    def _try_parse_frames(self):
        while len(self.rx_buffer) > 0:
            cmd = self.rx_buffer[0]
            size = self.CMD_SIZES.get(cmd)
            if size is None:
                self.rx_buffer = self.rx_buffer[1:]
                continue
            if len(self.rx_buffer) < size:
                return
            frame = bytes(self.rx_buffer[:size])
            self.rx_buffer = self.rx_buffer[size:]
            self._dispatch_command(frame)

    @staticmethod
    def _xor_checksum(data):
        c = 0
        for b in data:
            c ^= b
        return c

    def _dispatch_command(self, frame):
        cmd = frame[0]
        chk = frame[-1]
        if self._xor_checksum(frame[:-1]) != chk:
            self.send_response(self.STATUS_ERROR, self.ERR_CHECKSUM)
            return

        self.send_response(self.STATUS_ACK)

        try:
            if cmd == self.CMD_STOP:
                self.stop_all_motors()
                self.send_response(self.STATUS_DONE)

            elif cmd == self.CMD_SET_MOTOR:
                try:
                    _, m0_duty, m1_duty, period, m0_lat, m1_lat, phase, _ = \
                        struct.unpack('>BHHffffB', frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                self.set_motor_state(m0_duty, m1_duty, period, m0_lat, m1_lat, phase)
                self.send_response(self.STATUS_DONE)

            elif cmd == self.CMD_SEQ_ADD:
                self.send_response(self.STATUS_DONE)

            elif cmd == self.CMD_SEQ_RUN:
                try:
                    _, cycles, _ = struct.unpack('>BHB', frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                err = self.run_preset(self.TEMP_PRESET_ID, cycles)
                if err in (self.ERR_NO_PRESET, self.ERR_BAD_VALUE):
                    err = self.ERR_NO_PARAMS        # legacy code, as in protocol 1.0
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err)

            elif cmd == self.CMD_PRESET_DEFINE:
                try:
                    vals = struct.unpack(self.FMT_PRESET_DEFINE, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                preset_id, period = vals[1], vals[2]
                temp = preset_id == self.TEMP_PRESET_ID
                if self._motor_thread_active and not temp:
                    # Flash writes are unsafe while core 1 is running a sequence.
                    self.send_response(self.STATUS_ERROR, self.ERR_BUSY, preset_id)
                    return
                motors = [vals[3 + 3 * i: 6 + 3 * i] for i in range(self.N_MOTORS)]
                err = self.define_preset(preset_id, period, motors)
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err, preset_id)
                    return
                if not temp and not self.save_presets():
                    self.send_response(self.STATUS_ERROR, self.ERR_FLASH, preset_id)
                    return
                self.send_response(self.STATUS_DONE, preset_id, len(self.presets))

            elif cmd == self.CMD_PRESET_RUN:
                try:
                    _, preset_id, cycles, _ = struct.unpack(self.FMT_PRESET_RUN, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                err = self.run_preset(preset_id, cycles)
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err, preset_id)

            elif cmd == self.CMD_PRESET_DELETE:
                _, preset_id, _ = struct.unpack(self.FMT_PRESET_DELETE, frame)
                temp = preset_id == self.TEMP_PRESET_ID
                if self._motor_thread_active and not temp:
                    self.send_response(self.STATUS_ERROR, self.ERR_BUSY, preset_id)
                    return
                self.delete_preset(preset_id)
                if not temp and not self.save_presets():
                    self.send_response(self.STATUS_ERROR, self.ERR_FLASH, preset_id)
                    return
                self.send_response(self.STATUS_DONE, preset_id, len(self.presets))

            elif cmd == self.CMD_PRESET_SYNC:
                self.send_preset_sync()

            elif cmd == self.CMD_GET_STATUS:
                info1 = 1 if self.running else 0
                self.send_response(self.STATUS_DONE, info1, self.faults(), len(self.presets))

            elif cmd == self.CMD_POWERDOWN:
                # Purge first; replies: DONE_PURGE result, then DONE, then sleep.
                self.software_shutdown(final_done=True)

            elif cmd == self.CMD_BALLAST_NEUTRAL:
                try:
                    _, mode, value, _ = struct.unpack(self.FMT_BALLAST_NEUTRAL, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                if mode == self.NEUTRAL_QUERY:
                    if self.ballast_neutral is None:
                        self.send_response(self.STATUS_ERROR, self.ERR_NO_NEUTRAL)
                        return
                elif mode in (self.NEUTRAL_CAPTURE, self.NEUTRAL_SET):
                    if self._motor_thread_active:
                        self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                        return
                    if not self.set_ballast_neutral(
                            self.pot_raw(32) if mode == self.NEUTRAL_CAPTURE else value):
                        self.send_response(self.STATUS_ERROR, self.ERR_FLASH)
                        return
                else:
                    self.send_response(self.STATUS_ERROR, self.ERR_BAD_VALUE)
                    return
                v = self.ballast_neutral
                self.send_response(self.STATUS_DONE, v >> 8, v & 0xFF, mode)

            elif cmd == self.CMD_BALLAST_PURGE:
                err = self.start_purge()
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err)

            elif cmd == self.CMD_BALLAST_GOTO:
                try:
                    _, mode, value, _ = struct.unpack(self.FMT_BALLAST_GOTO, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                if mode == self.GOTO_NEUTRAL:
                    if self.ballast_neutral is None:
                        self.send_response(self.STATUS_ERROR, self.ERR_NO_NEUTRAL)
                        return
                    target = self.ballast_neutral
                elif mode == self.GOTO_VALUE:
                    target = value
                else:
                    self.send_response(self.STATUS_ERROR, self.ERR_BAD_VALUE)
                    return
                err = self.start_ballast_goto(target)
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err)

            elif cmd == self.CMD_BALLAST_PID:
                try:
                    _, mode, kp, ki, kd, _ = struct.unpack(self.FMT_BALLAST_PID, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                if mode == self.PID_SET:
                    if self._motor_thread_active:
                        self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                        return
                    err = self.set_ballast_gains(kp, ki, kd)
                    if err is not None:
                        self.send_response(self.STATUS_ERROR, err)
                        return
                elif mode != self.PID_QUERY:
                    self.send_response(self.STATUS_ERROR, self.ERR_BAD_VALUE)
                    return
                # Reply with the gains now in force, one frame each (x100, u16).
                pid = self.ballast_pid
                for idx, g in enumerate((pid.kp, pid.ki, pid.kd)):
                    v = int(round(g * 100))
                    self.send_response(self.STATUS_PID_GAIN, idx, v >> 8, v & 0xFF)

            elif cmd == self.CMD_MANUAL:
                try:
                    _, idx, action, speed_raw, _ = struct.unpack(self.FMT_MANUAL, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                err = self.manual_command(idx, action, speed_raw)
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err, idx)
                    return
                self.send_response(self.STATUS_DONE, idx, action)
                if action == self.MANUAL_STOP:
                    if idx == self.MANUAL_BALLAST:
                        self.report_ballast(read=not self._motor_thread_active)
                    else:
                        self.report_positions(1 << idx, read=not self._motor_thread_active)

            elif cmd == self.CMD_SET_ZERO:
                mask = frame[1] & 0x0F
                if self._motor_thread_active or any(mask & (1 << i) for i in self._manual):
                    self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                    return
                self.set_zero_here(mask)
                self.send_response(self.STATUS_DONE, self.zero_known_mask())

            elif cmd == self.CMD_GET_POSITIONS:
                idle = not self._motor_thread_active
                self.report_positions(read=idle)
                self.report_ballast(read=idle)
                self.send_response(self.STATUS_DONE, self.zero_known_mask())

            elif cmd == self.CMD_RETURN_TO_ZERO:
                # ACK now; EVENT 0x13 (result, motors not at zero) when it ends.
                err = self.start_return_to_zero()
                if err is not None:
                    self.send_response(self.STATUS_ERROR, err)

            elif cmd == self.CMD_SLIP_LIMIT:
                try:
                    _, mode, idx, tenths, _ = struct.unpack(self.FMT_SLIP_LIMIT, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                if mode == self.PID_SET:
                    if self._motor_thread_active:
                        self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                        return
                    err = self.set_slip_limit(idx, tenths)
                    if err is not None:
                        self.send_response(self.STATUS_ERROR, err, idx)
                        return
                elif mode != self.PID_QUERY:
                    self.send_response(self.STATUS_ERROR, self.ERR_BAD_VALUE)
                    return
                self.report_slip_limits()        # all four, then DONE
                self.send_response(self.STATUS_DONE, mode)

            elif cmd == self.CMD_DUTY_LIMIT:
                try:
                    _, mode, raw, _ = struct.unpack(self.FMT_DUTY_LIMIT, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                if mode == self.PID_SET:
                    if self._motor_thread_active:
                        self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                        return
                    err = self.set_duty_limit(raw)
                    if err is not None:
                        self.send_response(self.STATUS_ERROR, err)
                        return
                elif mode != self.PID_QUERY:
                    self.send_response(self.STATUS_ERROR, self.ERR_BAD_VALUE)
                    return
                v = int(round(self.drive_max_duty * self.ADC_MAX))
                self.send_response(self.STATUS_DONE, v >> 8, v & 0xFF, mode)

            elif cmd == self.CMD_RTZ_PID:
                try:
                    _, mode, kp, ki, kd, _ = struct.unpack(self.FMT_BALLAST_PID, frame)
                except Exception:
                    self.send_response(self.STATUS_ERROR, self.ERR_UNPACK)
                    return
                if mode == self.PID_SET:
                    if self._motor_thread_active:
                        self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                        return
                    err = self.set_rtz_gains(kp, ki, kd)
                    if err is not None:
                        self.send_response(self.STATUS_ERROR, err)
                        return
                elif mode != self.PID_QUERY:
                    self.send_response(self.STATUS_ERROR, self.ERR_BAD_VALUE)
                    return
                pid = self.rtz_pids[0]
                for idx, g in enumerate((pid.kp, pid.ki, pid.kd)):
                    v = int(round(g * 100))
                    self.send_response(self.STATUS_PID_GAIN, self.RTZ_GAIN_TAG + idx, v >> 8, v & 0xFF)

            elif cmd == self.CMD_LOG_GET:
                if self._motor_thread_active:
                    self.send_response(self.STATUS_ERROR, self.ERR_BUSY)
                elif not self.send_log():
                    self.send_response(self.STATUS_ERROR, self.ERR_NO_LOG)

            elif cmd == self.CMD_PING:
                self.send_response(self.STATUS_PONG, *self.PROTOCOL_VERSION)

            else:
                self.send_response(self.STATUS_ERROR, self.ERR_UNKNOWN_CMD)

        except Exception:
            # Any unhandled exception inside a handler; info2 says which one.
            self.send_response(self.STATUS_ERROR, self.ERR_HANDLER, cmd)

    # =====================================================================
    #  Main loop entry point
    # =====================================================================

    def run(self):
        """Run forever on the main thread: battery and fault checks plus UART
        polling. Motion runs on its own thread spawned by CMD_SEQ_RUN."""
        self.sync_presets()             # reload from flash and announce to the host
        self.restore_positions()        # reports F7 (last shutdown type, restored motors)
        self.start_return_to_zero()     # motors with a known zero return to it (DONE 0x13)
        last_battery_check = last_fault_check = last_encoder_read = time.ticks_ms()

        while True:
            now = time.ticks_ms()

            # While a sequence runs, the motor thread reads the encoders; reading
            # from both cores at once would split the PIO FIFO between them.
            if (not self._motor_thread_active and
                    time.ticks_diff(now, last_encoder_read) >= self.ENCODER_IDLE_MS):
                self.read_encoders()
                last_encoder_read = now

            self._manual_service()      # keepalive + slip protection for manual motors

            if time.ticks_diff(now, last_fault_check) >= self.FAULT_CHECK_MS:
                try:
                    self.check_faults()
                except DriverFault as e:
                    self._handle_fault(e)
                last_fault_check = now

            if self._log_pending and not self._motor_thread_active:
                self._finish_log()

            if time.ticks_diff(now, last_battery_check) >= self.BATTERY_CHECK_MS:
                self.check_battery_voltage()
                last_battery_check = now

            if self.uart.any():
                data = self.uart.read()
                if data:
                    self.rx_buffer.extend(data)
                    self._try_parse_frames()

            time.sleep_ms(5)


# ----- Entry point -----
if __name__ == "__main__":
    io = RobotIO()
    io.run()
