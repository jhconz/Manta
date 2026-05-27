# -*- coding: utf-8 -*-
"""
uart_test_pico.py - Minimal UART check for the Pico side.

Run this ON THE PICO (save as main.py, or run from Thonny).

What it does:
  - Opens UART0 on GP16 (TX) / GP17 (RX) - matches onboard_pi.py.
  - Blinks the onboard LED once at startup so you can SEE it's running.
  - Every 1 second, sends "PICO_HELLO\n" out the TX pin.
  - Reads the battery ADC on GP28 and reports voltage on request.
  - Whenever bytes arrive on RX, acts on them:
        "VBAT?"  -> replies "VBAT:<volts>\n"  (battery voltage)
        anything else -> echoes it back prefixed with "ECHO:"

How to read the result:
  - LED blinks once at boot   -> the script is running at all.
  - Pi sees "PICO_HELLO"      -> Pico TX -> Pi RX wire is good.
  - LED flashes when Pi sends -> Pi TX -> Pico RX wire is good.
  - Pi sees "ECHO:<its text>" -> both directions work, full round trip.
  - Pi sees "VBAT:11.83"      -> ADC + divider math + UART all working.
"""
from machine import Pin, UART, ADC
import time

# ---- Pins: must match how you physically wire it ----
UART_ID = 0
TX_PIN  = 16
RX_PIN  = 17
ADC_PIN = 28          # battery divider output - matches onboard_pi.py
BAUD    = 115200

# ---- Battery / ADC constants (copied from onboard_pi.py) ----
DIVIDER_R1    = 46.4
DIVIDER_R2    = 9.8
DIVIDER_RATIO = (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2
VREF          = 3.3
ADC_MAX       = 65535

led = Pin("LED", Pin.OUT)   # onboard LED (use Pin(25) if "LED" fails on a plain Pico)

uart = UART(UART_ID, baudrate=BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN),
            bits=8, parity=None, stop=1, timeout=10)

battery_adc = ADC(Pin(ADC_PIN))


def read_battery_voltage(samples=16):
    """Average several ADC reads, scale through the divider.
    Returns (battery_volts, raw_adc_pin_volts) so you can sanity-check
    the divider against a multimeter reading on the ADC pin itself."""
    reading = 0
    for _ in range(samples):
        reading += battery_adc.read_u16()
    v_adc = (reading / (samples * ADC_MAX)) * VREF
    return v_adc * DIVIDER_RATIO, v_adc


# Startup blink: proves the script is executing.
for _ in range(3):
    led.on(); time.sleep_ms(120)
    led.off(); time.sleep_ms(120)

print("uart_test_pico running. TX=GP%d RX=GP%d ADC=GP%d @ %d baud"
      % (TX_PIN, RX_PIN, ADC_PIN, BAUD))

last_send = time.ticks_ms()
rx_buffer = bytearray()

while True:
    # Send a heartbeat once per second.
    now = time.ticks_ms()
    if time.ticks_diff(now, last_send) >= 1000:
        uart.write(b"PICO_HELLO\n")
        last_send = now

    # Handle anything received.
    if uart.any():
        data = uart.read()
        if data:
            led.on()
            rx_buffer.extend(data)

            # Process complete newline-terminated commands.
            while b"\n" in rx_buffer:
                line, _, rest = rx_buffer.partition(b"\n")
                rx_buffer = bytearray(rest)
                cmd = line.strip()

                if cmd == b"VBAT?":
                    vbat, vpin = read_battery_voltage()
                    # vbat = battery estimate; vpin = raw voltage at the ADC pin
                    uart.write(b"VBAT:%.2f VPIN:%.3f\n" % (vbat, vpin))
                    print("VBAT? -> %.2f V (pin %.3f V)" % (vbat, vpin))
                elif cmd:
                    uart.write(b"ECHO:" + cmd + b"\n")
                    print("got:", cmd)

            time.sleep_ms(50)
            led.off()

    time.sleep_ms(10)
