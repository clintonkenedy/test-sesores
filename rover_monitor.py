#!/usr/bin/env python3
"""Feed RTCM corrections to a rover receiver and report its RTK fix quality.

This is the rover half of a LAN test. It:
  1. connects to the base machine over the network and pulls corrections,
  2. writes them into the rover receiver's serial port,
  3. reads the receiver's NMEA back, and
  4. prints fix mode, satellites, position, precision and correction age.

The number that matters is the fix mode. Watch it climb:
  NO FIX -> SINGLE (metres) -> RTK FLOAT (decimetres) -> RTK FIXED (centimetres)

If the fix never leaves SINGLE, corrections are not reaching the receiver -
check that the rover's ESP32 bridge passes bytes IN (host -> receiver), not
only out. Age of corrections rising instead of resetting each second means the
same thing.

Injection and reading run on separate threads because pyserial can be read and
written concurrently, and select() does not work on serial ports on Windows.

Example:
    python rover_monitor.py --list
    python rover_monitor.py --base-host 192.168.1.50 --port COM5
"""

import argparse
import socket
import sys
import threading
import time

import serial
from serial.tools import list_ports

# ===========================================================================
#  CONFIGURATION - edit these, or override any of them on the command line.
# ===========================================================================

# The base machine running rtcm_to_lora.py in server mode.
BASE_HOST = "192.168.1.50"
BASE_PORT = 8887

# Serial port of the ROVER receiver. Windows: "COM5". macOS/Linux: "/dev/cu...".
SERIAL_PORT = "COM5" if sys.platform.startswith("win") else "/dev/cu.usbserial-0002"
SERIAL_BAUD = 115200

RECONNECT_DELAY = 3.0

# ===========================================================================

FIX_QUALITY = {
    0: "NO FIX",
    1: "SINGLE",       # autonomous, a few metres
    2: "DGPS",         # ~1 m
    4: "RTK FIXED",    # ~1-2 cm  <- the goal
    5: "RTK FLOAT",    # ~10-50 cm
    6: "DEAD RECKON",
}


def show_ports():
    """List visible serial ports so the rover's adapter can be identified."""
    ports = sorted(list_ports.comports())
    if not ports:
        print("No serial ports found. Check the cable and the USB-serial driver.")
        return
    print("Available serial ports:")
    for port in ports:
        print("  {:<22} {}".format(port.device, port.description))


def nmea_fields(line):
    """Split an NMEA line into fields, or return None if it is not one."""
    text = line.decode("ascii", "replace").strip()
    if not text.startswith("$"):
        return None
    return text[1:].split("*", 1)[0].split(",")


def dm_to_degrees(value, hemisphere):
    """Convert NMEA ddmm.mmmm plus hemisphere into signed decimal degrees."""
    if not value:
        return None
    raw = float(value)
    degrees = int(raw // 100)
    decimal = degrees + (raw - degrees * 100) / 60.0
    return -decimal if hemisphere in ("S", "W") else decimal


class RoverState:
    """Latest fix figures, updated from GGA and GST sentences."""

    def __init__(self):
        self.quality = 0
        self.sats = 0
        self.lat = None
        self.lon = None
        self.alt = None
        self.age = None            # seconds since last correction used
        self.h_sigma = None        # horizontal std deviation, metres

    def update_gga(self, f):
        if len(f) < 15:
            return
        self.quality = int(f[6]) if f[6] else 0
        self.sats = int(f[7]) if f[7] else 0
        self.lat = dm_to_degrees(f[2], f[3])
        self.lon = dm_to_degrees(f[4], f[5])
        self.alt = float(f[9]) if f[9] else None
        self.age = float(f[13]) if f[13] else None

    def update_gst(self, f):
        # Horizontal precision from the lat/lon std deviations (fields 6, 7).
        if len(f) < 9:
            return
        try:
            lat_sigma = float(f[6])
            lon_sigma = float(f[7])
        except ValueError:
            return
        self.h_sigma = (lat_sigma ** 2 + lon_sigma ** 2) ** 0.5

    def line(self):
        name = FIX_QUALITY.get(self.quality, "?{}".format(self.quality))
        parts = ["{:<10} sats={:<2}".format(name, self.sats)]
        if self.lat is not None and self.lon is not None:
            parts.append("{:.7f}, {:.7f}".format(self.lat, self.lon))
        if self.alt is not None:
            parts.append("{:>7.2f} m".format(self.alt))
        if self.h_sigma is not None:
            parts.append("+/-{:.3f} m".format(self.h_sigma))
        if self.age is not None:
            parts.append("age {:.0f}s".format(self.age))
        return "  ".join(parts)


def inject_corrections(stop, rover, host, port):
    """Pull corrections from the base and write them into the rover receiver."""
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=5) as link:
                print("  corrections: connected to {}:{}".format(host, port))
                link.settimeout(1.0)
                while not stop.is_set():
                    try:
                        chunk = link.recv(1024)
                    except socket.timeout:
                        continue
                    if not chunk:
                        print("  corrections: base closed the connection")
                        break
                    rover.write(chunk)
        except OSError as error:
            print("  corrections: cannot reach {}:{} - {}".format(host, port, error))
        if not stop.is_set():
            time.sleep(RECONNECT_DELAY)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base-host", default=BASE_HOST,
                        help="base machine address (default: %(default)s)")
    parser.add_argument("--base-port", type=int, default=BASE_PORT,
                        help="base correction port (default: %(default)s)")
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="rover receiver serial port (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--list", action="store_true",
                        help="list available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    print("Rover receiver on {} at {} baud".format(args.port, args.baud))
    print("Corrections from {}:{}".format(args.base_host, args.base_port))
    print("Watch the fix climb: SINGLE -> RTK FLOAT -> RTK FIXED\n")

    state = RoverState()
    stop = threading.Event()

    try:
        rover = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
        return

    pump = threading.Thread(
        target=inject_corrections,
        args=(stop, rover, args.base_host, args.base_port),
        daemon=True)
    pump.start()

    try:
        while True:
            line = rover.readline()
            if not line:
                continue
            fields = nmea_fields(line)
            if not fields:
                continue
            kind = fields[0]
            if kind.endswith("GGA"):
                state.update_gga(fields)
                print(state.line())
            elif kind.endswith("GST"):
                state.update_gst(fields)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop.set()
        rover.close()


if __name__ == "__main__":
    main()
