#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_test_pi.py - Minimal UART check for the Raspberry Pi side.

Run this ON THE PI:  python3 uart_test_pi.py

What it does:
  - Opens /dev/serial0 at 115200.
  - Every 2 seconds, sends "PI_PING\n" to the Pico.
  - Every 5 seconds, sends "VBAT?\n" to ask for battery voltage.
  - Continuously prints any bytes it receives.
  - You can also type a command + Enter to send it manually
    (e.g. type  VBAT?  and press Enter for an on-demand reading).

How to read the result:
  - You see "PICO_HELLO"            -> Pico TX -> Pi RX wire works.
  - You see "ECHO:PI_PING"          -> Pi TX -> Pico RX works too.
  - You see "VBAT:11.83 VPIN:2.064" -> ADC + divider + UART all good.
        VBAT = battery voltage estimate
        VPIN = raw volts at the ADC pin; compare this to your multimeter
  - You see NOTHING                 -> see the checklist printed at startup.
  - You see garbled characters      -> baud mismatch, or no common ground.

Two test modes:
  1. LOOPBACK (no Pico): jumper Pi GPIO14 (pin 8) to GPIO15 (pin 10).
     You should see "PI_PING" come back. (VBAT? will just echo back too,
     since there's no Pico to answer it.) Proves the Pi UART is configured
     correctly. Do this FIRST.
  2. REAL (with Pico): wire Pi TX -> Pico RX, Pi RX -> Pico TX, common GND,
     and run uart_test_pico.py on the Pico.
"""
import serial
import time
import sys
import threading

PORT = "/dev/serial0"
BAUD = 115200

CHECKLIST = """
No data received. Tomorrow, check these in order:

  1. Serial console disabled?  sudo raspi-config
       Interface Options -> Serial Port
       "login shell over serial?"  -> NO
       "serial hardware enabled?"  -> YES   then reboot
  2. Which device is serial0?  ls -l /dev/serial0
       On a Pi 4 you may want 'dtoverlay=disable-bt' in
       /boot/firmware/config.txt so serial0 is the good PL011 UART.
  3. Wiring: Pi TX (GPIO14, pin 8)  -> Pico RX
            Pi RX (GPIO15, pin 10) -> Pico TX     (TX and RX must CROSS)
            Pi GND <-> Pico GND    (common ground is mandatory)
  4. Pico pins: uart_test_pico.py uses GP16=TX, GP17=RX, GP28=ADC.
  5. Is the Pico script actually running? It blinks the LED 3x at boot.
"""


def reader_thread(ser, state):
    """Continuously print whatever arrives from the serial port."""
    while not state["stop"]:
        try:
            data = ser.read(64)
        except Exception:
            break
        if data:
            state["got_anything"] = True
            # repr() so non-printable bytes are visible, not silently eaten
            print("  <- received: %r" % data)
        time.sleep(0.02)


def input_thread(ser, state):
    """Let the user type a command + Enter to send it manually."""
    while not state["stop"]:
        try:
            line = sys.stdin.readline()
        except Exception:
            break
        if not line:
            break
        line = line.strip()
        if line:
            ser.write(line.encode() + b"\n")
            print("  -> sent (manual): %s" % line)


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print("Could not open %s: %s" % (PORT, e))
        print("Is the port name right? Try: ls -l /dev/serial*")
        sys.exit(1)

    print("uart_test_pi running on %s @ %d baud." % (PORT, BAUD))
    print("Auto-sending PI_PING every 2s and VBAT? every 5s.")
    print("You can also type a command (e.g. VBAT?) + Enter. Ctrl-C to quit.\n")

    state = {"stop": False, "got_anything": False}

    rx = threading.Thread(target=reader_thread, args=(ser, state), daemon=True)
    rx.start()
    tx_in = threading.Thread(target=input_thread, args=(ser, state), daemon=True)
    tx_in.start()

    last_ping = 0
    last_vbat = 0
    start = time.time()

    try:
        while True:
            now = time.time()

            if now - last_ping >= 2.0:
                ser.write(b"PI_PING\n")
                print("  -> sent PI_PING")
                last_ping = now

            if now - last_vbat >= 5.0:
                ser.write(b"VBAT?\n")
                print("  -> sent VBAT?  (asking Pico for battery voltage)")
                last_vbat = now

            # After 8 seconds of silence, print the troubleshooting checklist.
            if not state["got_anything"] and now - start > 8:
                print(CHECKLIST)
                start = now  # don't spam; repeat every 8s

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        state["stop"] = True
        ser.close()


if __name__ == "__main__":
    main()
