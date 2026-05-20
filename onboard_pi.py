# -*- coding: utf-8 -*-
"""
manta_pi_2_motor.py — Onboard Pico controller for 2-motor manta system

Architecture:
  - Main thread:        UART IRQ + frame parsing + command dispatch + battery
                        monitoring. Runs onboard_pi.run() forever.
  - Motor thread:       Spawned on demand when CMD_SEQ_RUN arrives. Builds
                        and executes motion schedule, then exits.
"""
from machine import Pin, PWM, UART, ADC
import machine
import _thread
import struct
import time


class onboard_pi:
    # ---------- Command codes ----------
    CMD_STOP        = 0x01
    CMD_SET_MOTOR   = 0x02
    CMD_SEQ_ADD     = 0x03
    CMD_SEQ_RUN     = 0x04
    CMD_GET_STATUS  = 0x05
    
    # ---------- Status codes ----------
    STATUS_IDLE      = 0x00
    STATUS_ACK       = 0x01
    STATUS_DONE      = 0x02
    STATUS_ERROR     = 0x03
    STATUS_BAT_WARN  = 0xF0
    STATUS_BAT_CRIT  = 0xF1
    
    # ---------- Frame sizes ----------
    CMD_SIZES = {
        CMD_STOP:        3,
        CMD_SET_MOTOR:  22,
        CMD_SEQ_ADD:    22,
        CMD_SEQ_RUN:     4,
        CMD_GET_STATUS:  3,
    }
    RESPONSE_SIZE = 5
    
    # ---------- Battery / ADC constants ----------
    DIVIDER_R1   = 47.0
    DIVIDER_R2   = 9.3
    DIVIDER_RATIO = (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2
    VREF         = 3.3
    ADC_MAX      = 65535
    
    V_WARNING    = 11.0
    V_SHUTDOWN   = 10.2
    SHUTDOWN_DEBOUNCE = 10
    HYSTERESIS   = 0.3
    
    # ---------- Timing ----------
    BATTERY_CHECK_MS = 200
    
    def __init__(self,
                 motor_pins=((0, 1), (2, 3)),
                 uart_pins=(16, 17),
                 adc_pin=28):
        # ----- Motor PWM -----
        self.pwm0a = PWM(Pin(motor_pins[0][0]))
        self.pwm0b = PWM(Pin(motor_pins[0][1]))
        self.pwm1a = PWM(Pin(motor_pins[1][0]))
        self.pwm1b = PWM(Pin(motor_pins[1][1]))
        self.pwm0a.freq(20000)
        self.pwm1a.freq(40000)
        self.running = False
        self._motor_params = None
        self._motor_thread_active = False
        
        # ----- UART -----
        self.uart = UART(0,
                         baudrate=115200,
                         bits=8,
                         parity=None,
                         stop=1,
                         tx=Pin(uart_pins[0]),
                         rx=Pin(uart_pins[1]),
                         timeout=10,
                         rxbuf=128)
        
        self.rx_buffer = bytearray()
        self._uart_lock = _thread.allocate_lock()
        
        try:
            self.uart.irq(trigger=UART.IRQ_RXIDLE, handler=self._on_uart_rx)
            self._use_irq = True
        except Exception:
            self._use_irq = False
        
        # ----- Battery monitor -----
        self.battery_voltage = ADC(Pin(adc_pin))
        self.shutdown_counter = 0
        self.warning_flag = False
    
    # =====================================================================
    #  Battery monitor
    # =====================================================================
    
    def read_battery_voltage(self, samples=16):
        reading = 0
        for _ in range(samples):
            reading += self.battery_voltage.read_u16()
        v_adc = (reading / (samples * self.ADC_MAX)) * self.VREF
        return v_adc * self.DIVIDER_RATIO
    
    def check_battery_voltage(self):
        v = self.read_battery_voltage()
        
        if v <= self.V_SHUTDOWN:
            self.shutdown_counter += 1
            if self.shutdown_counter >= self.SHUTDOWN_DEBOUNCE:
                self.send_response(self.STATUS_BAT_CRIT)
                self.software_shutdown()
        elif v <= self.V_WARNING:
            self.shutdown_counter = 0
            if not self.warning_flag:
                self.send_response(self.STATUS_BAT_WARN)
                self.warning_flag = True
        else:
            self.shutdown_counter = 0
            if v > self.V_WARNING + self.HYSTERESIS:
                self.warning_flag = False
    
    def software_shutdown(self):
        try:
            self.uart.flush()
        except AttributeError:
            pass
        time.sleep_ms(50)
        self.stop_all_motors()
        machine.deepsleep()
    
    # =====================================================================
    #  Motor control — low level
    # =====================================================================
    
    def stop_all_motors(self):
        for pwm in (self.pwm0a, self.pwm0b, self.pwm1a, self.pwm1b):
            pwm.duty_u16(0)
        self.running = False
        
    def set_motor_state(self, m0_duty, m1_duty, period, m0_latent, m1_latent, phase):
        """Atomic dict swap — safe to call while motor thread is running."""
        self._motor_params = {
            'm0_duty':   m0_duty,
            'm1_duty':   m1_duty,
            'period':    period,
            'm0_latent': m0_latent,
            'm1_latent': m1_latent,
            'phase':     phase,
        }
    
    # =====================================================================
    #  Motor control — sequence building and execution
    # =====================================================================
    
    def _build_motion_schedule(self, params, n_cycles):
        T   = params['period']
        phi = params['phase']
        m0L = params['m0_latent']
        m1L = params['m1_latent']
        m0d = params['m0_duty']
        m1d = params['m1_duty']
        
        half = T / 2.0
        quarter = T / 4.0
        
        m0L = max(0.0, min(1.0, m0L))
        m1L = max(0.0, min(1.0, m1L))
        phi = max(-1.0, min(1.0, phi))
        
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
        m1_offset = phi * half
        
        events = []
        events.extend(self._segments_to_events(segments, self.pwm0a, self.pwm0b, m0d, m0L, 0.0))
        events.extend(self._segments_to_events(segments, self.pwm1a, self.pwm1b, m1d, m1L, m1_offset))
        
        if events:
            min_t = min(t for t, _, _ in events)
            if min_t < 0:
                events = [(t - min_t, ch, d) for t, ch, d in events]
                total_duration -= min_t
        
        events.append((total_duration, self.pwm0a, 0))
        events.append((total_duration, self.pwm0b, 0))
        events.append((total_duration, self.pwm1a, 0))
        events.append((total_duration, self.pwm1b, 0))
        
        events.sort(key=lambda e: (e[0], e[2]))
        
        return events, total_duration
    
    def _segments_to_events(self, segments, fwd_ch, rev_ch, duty, latent, offset):
        events = []
        for start, duration, direction in segments:
            on_time = duration * (1.0 - latent)
            if on_time <= 0:
                continue
            ch = fwd_ch if direction == 'fwd' else rev_ch
            t_on  = start + offset
            t_off = start + offset + on_time
            events.append((t_on,  ch, duty))
            events.append((t_off, ch, 0))
        return events
    
    def _run_motion(self, events, total_duration):
        start_us = time.ticks_us()
        
        for t_sec, channel, duty in events:
            if not self.running:        # was self._sequence_running
                break
            target_us = time.ticks_add(start_us, int(t_sec * 1_000_000))
            wait = time.ticks_diff(target_us, time.ticks_us())
            if wait > 0:
                time.sleep_us(wait)
            channel.duty_u16(duty)
        
        self.stop_all_motors()
    
    def _motor_thread_entry(self, n_cycles):
        try:
            events, total_duration = self._build_motion_schedule(self._motor_params, n_cycles)
            self.running = True         # was: self._sequence_running = True; self.running = True
            self._run_motion(events, total_duration)
            self.send_response(self.STATUS_DONE, 0x10)
        finally:
            self._motor_thread_active = False
    
    # =====================================================================
    #  UART communication
    # =====================================================================
    
    def send_response(self, status, info1=0, info2=0, info3=0):
        chk = status ^ info1 ^ info2 ^ info3
        frame = bytes([status, info1, info2, info3, chk])
        with self._uart_lock:
            self.uart.write(frame)
    
    def _on_uart_rx(self, uart):
        data = uart.read()
        if data:
            self.rx_buffer.extend(data)
            self._try_parse_frames()
    
    def _try_parse_frames(self):
        while len(self.rx_buffer) > 0:
            cmd = self.rx_buffer[0]
            size = self.CMD_SIZES.get(cmd)
            if size is None:
                del self.rx_buffer[0]
                continue
            if len(self.rx_buffer) < size:
                return
            frame = bytes(self.rx_buffer[:size])
            del self.rx_buffer[:size]
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
            self.send_response(self.STATUS_ERROR, 0x01)
            return
        
        self.send_response(self.STATUS_ACK)
        
        if cmd == self.CMD_STOP:
            self.stop_all_motors()
            self.send_response(self.STATUS_DONE)
        
        elif cmd == self.CMD_SET_MOTOR:
            try:
                _, m0_duty, m1_duty, period, m0_lat, m1_lat, phase, _ = \
                    struct.unpack('>BHHffffB', frame)
            except Exception:
                self.send_response(self.STATUS_ERROR, 0x02)
                return
            self.set_motor_state(m0_duty, m1_duty, period, m0_lat, m1_lat, phase)
            self.send_response(self.STATUS_DONE)
        
        elif cmd == self.CMD_SEQ_ADD:
            self.send_response(self.STATUS_DONE)
        
        elif cmd == self.CMD_SEQ_RUN:
            try:
                _, cycles, _ = struct.unpack('>BHB', frame)
            except Exception:
                self.send_response(self.STATUS_ERROR, 0x02)
                return
            
            if self._motor_thread_active:
                self.send_response(self.STATUS_ERROR, 0x04)  # busy
                return
            if self._motor_params is None or cycles <= 0:
                self.send_response(self.STATUS_ERROR, 0x05)  # invalid request
                return
            
            self._motor_thread_active = True
            _thread.start_new_thread(self._motor_thread_entry, (cycles,))
        
        elif cmd == self.CMD_GET_STATUS:
            info1 = 1 if self.running else 0
            self.send_response(self.STATUS_DONE, info1)
        
        else:
            self.send_response(self.STATUS_ERROR, 0xFF)
    
    # =====================================================================
    #  Main loop entry point
    # =====================================================================
    
    def run(self):
        """Run forever on the main thread. Handles battery checks and
        (if no IRQ available) UART polling. Motor work runs on its own thread
        spawned by CMD_SEQ_RUN."""
        last_battery_check = time.ticks_ms()
        
        while True:
            now = time.ticks_ms()
            
            if time.ticks_diff(now, last_battery_check) >= self.BATTERY_CHECK_MS:
                self.check_battery_voltage()
                last_battery_check = now
            
            if not self._use_irq and self.uart.any():
                self.rx_buffer.extend(self.uart.read())
                self._try_parse_frames()
            
            time.sleep_ms(10)


# ----- Entry point -----
if __name__ == "__main__":
    pi = onboard_pi()
    pi.run()