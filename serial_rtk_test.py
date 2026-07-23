#!/usr/bin/env python3
"""Drive an RTK rover from the Mac: transmit corrections, receive its fix.

Talks to the LC29H over a serial port (through the transparent usb_bridge.ino
on the ESP32). It does two jobs at once, so both directions can be checked:

  TX (transmision)  feeds RTCM3 corrections into the receiver, from either the
                    live base TCP server or a recorded .rtcm file.
  RX (recepcion)    reads the receiver's NMEA back and prints the fix, the
                    correction age and the precision.

This takes the WiFi rover firmware out of the picture: if the rover fixes here,
the LC29H and its wiring are good.

Examples:
    python serial_rtk_test.py --list
    python serial_rtk_test.py --port /dev/cu.usbserial-0002            # RX only
    python serial_rtk_test.py --port /dev/cu.usbserial-0002 --base-host 192.168.1.162
    python serial_rtk_test.py --port /dev/cu.usbserial-0002 --rtcm-file base.rtcm
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

# Serial port of the ESP32 running usb_bridge.ino.
SERIAL_PORT = "COM5" if sys.platform.startswith("win") else "/dev/cu.usbserial-0002"
SERIAL_BAUD = 115200

# Live corrections from the base machine running rtcm_to_lora.py --mode server.
BASE_HOST = "192.168.1.162"
BASE_PORT = 8887

RECONNECT_DELAY = 3.0
STATS_EVERY = 5.0

# ===========================================================================

FIX_QUALITY = {
    0: "NO FIX", 1: "SINGLE", 2: "DGPS",
    4: "RTK FIXED", 5: "RTK FLOAT", 6: "DEAD RECKON",
}


def show_ports():
    ports = sorted(list_ports.comports())
    if not ports:
        print("No serial ports found. Check the cable and the USB-serial driver.")
        return
    print("Available serial ports:")
    for port in ports:
        print("  {:<22} {}".format(port.device, port.description))


class Counters:
    """Shared byte tallies for both directions, guarded by a lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.tx = 0          # corrections written to the receiver
        self.rx = 0          # bytes read from the receiver
        self.nmea = 0        # complete NMEA sentences seen

    def add_tx(self, n):
        with self.lock:
            self.tx += n

    def add_rx(self, n):
        with self.lock:
            self.rx += n

    def add_nmea(self):
        with self.lock:
            self.nmea += 1


def dm_to_degrees(value, hemisphere):
    if not value:
        return None
    raw = float(value)
    degrees = int(raw // 100)
    decimal = degrees + (raw - degrees * 100) / 60.0
    return -decimal if hemisphere in ("S", "W") else decimal


def report_fix(fields, precision):
    """Print a fix summary from a GGA field list."""
    if len(fields) < 15:
        return precision
    quality = int(fields[6]) if fields[6] else 0
    sats = fields[7] or "?"
    lat = dm_to_degrees(fields[2], fields[3])
    lon = dm_to_degrees(fields[4], fields[5])
    age = fields[13] or "?"

    line = "  RX  {:<10} sats={:<3}".format(
        FIX_QUALITY.get(quality, "?{}".format(quality)), sats)
    if lat is not None and lon is not None:
        line += "  {:.7f}, {:.7f}".format(lat, lon)
    if precision is not None:
        line += "  +/-{:.3f} m".format(precision)
    line += "  age={}s".format(age)
    print(line)
    return precision


def update_precision(fields):
    """Horizontal precision from a GST field list (lat/lon std, fields 6 and 7)."""
    if len(fields) < 9:
        return None
    try:
        return (float(fields[6]) ** 2 + float(fields[7]) ** 2) ** 0.5
    except ValueError:
        return None


def inject_from_tcp(stop, rover, counters, host, port):
    """TX: pull corrections from the base over TCP and write them to the rover."""
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=5) as link:
                print("  TX  corrections connected to {}:{}".format(host, port))
                link.settimeout(1.0)
                while not stop.is_set():
                    try:
                        chunk = link.recv(1024)
                    except socket.timeout:
                        continue
                    if not chunk:
                        print("  TX  base closed the connection")
                        break
                    rover.write(chunk)
                    counters.add_tx(len(chunk))
        except OSError as error:
            print("  TX  cannot reach {}:{} - {}".format(host, port, error))
        if not stop.is_set():
            time.sleep(RECONNECT_DELAY)


def nmea_sentence(body):
    """Wrap an NMEA body with $, its XOR checksum and CRLF."""
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return "${}*{:02X}\r\n".format(body, checksum).encode("ascii")


def inject_probe(stop, rover, counters):
    """TX: query the receiver so ANY reply proves the injection wire works.

    No corrections involved: $PQTMVERNO asks the LC29H for its firmware
    version and $PAIR021 for its boot status. If either answer shows up in
    the RX stream, bytes are reaching the receiver's RXD pin.
    """
    queries = [nmea_sentence("PQTMVERNO"), nmea_sentence("PAIR021")]
    print("  TX  probe mode: querying the receiver every 3 s")
    print("  TX  a $PQTMVERNO or $PAIR001 reply below = injection wire WORKS")
    while not stop.is_set():
        for query in queries:
            rover.write(query)
            counters.add_tx(len(query))
        time.sleep(3.0)


def inject_from_file(stop, rover, counters, path):
    """TX: replay a recorded RTCM file on a loop, throttled to ~1 KB/s."""
    try:
        with open(path, "rb") as handle:
            data = handle.read()
    except OSError as error:
        print("  TX  cannot read {} - {}".format(path, error))
        return
    if not data:
        print("  TX  {} is empty".format(path))
        return
    print("  TX  replaying {} ({} bytes) on a loop".format(path, len(data)))
    while not stop.is_set():
        for i in range(0, len(data), 256):
            if stop.is_set():
                return
            block = data[i:i + 256]
            rover.write(block)
            counters.add_tx(len(block))
            time.sleep(0.25)   # keep near the real ~1 KB/s base rate


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="serial port of the ESP32 bridge (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--base-host", default=None,
                        help="pull corrections from this base (TX). Omit for RX only.")
    parser.add_argument("--base-port", type=int, default=BASE_PORT,
                        help="base correction port (default: %(default)s)")
    parser.add_argument("--rtcm-file", default=None,
                        help="replay corrections from this file (TX) instead of TCP")
    parser.add_argument("--probe", action="store_true",
                        help="TX test without a base: query the receiver and "
                             "watch for its reply")
    parser.add_argument("--list", action="store_true",
                        help="list available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    try:
        rover = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
        return

    counters = Counters()
    stop = threading.Event()
    injector = None

    if args.probe:
        injector = threading.Thread(target=inject_probe,
                                    args=(stop, rover, counters),
                                    daemon=True)
    elif args.rtcm_file:
        injector = threading.Thread(target=inject_from_file,
                                    args=(stop, rover, counters, args.rtcm_file),
                                    daemon=True)
    elif args.base_host:
        injector = threading.Thread(target=inject_from_tcp,
                                    args=(stop, rover, counters,
                                          args.base_host, args.base_port),
                                    daemon=True)

    print("Serial {} at {} baud".format(args.port, args.baud))
    if injector is None:
        print("RX only - no corrections being sent. "
              "Add --base-host or --rtcm-file to transmit.\n")
    else:
        print("Transmitting corrections and reading the fix back.\n")
        injector.start()

    precision = None
    line = bytearray()
    last_stats = time.monotonic()

    try:
        while True:
            data = rover.read(256)
            if data:
                counters.add_rx(len(data))
                for byte in data:
                    if byte == 0x0A:
                        text = line.decode("ascii", "replace").strip()
                        line.clear()
                        if not text.startswith("$"):
                            continue
                        counters.add_nmea()
                        fields = text[1:].split("*", 1)[0].split(",")
                        if fields[0].endswith("GGA"):
                            precision = report_fix(fields, precision)
                        elif fields[0].endswith("GST"):
                            precision = update_precision(fields)
                        elif fields[0].startswith(("PQTM", "PAIR")):
                            # A reply to the probe: bytes ARE reaching the
                            # receiver's RXD, so the injection wire works.
                            print("  RX  REPLY << {}".format(text))
                    elif byte != 0x0D and len(line) < 200:
                        line.append(byte)

            now = time.monotonic()
            if now - last_stats >= STATS_EVERY:
                last_stats = now
                with counters.lock:
                    print("  ..  TX={} B   RX={} B   NMEA lines={}".format(
                        counters.tx, counters.rx, counters.nmea))
                if counters.rx == 0:
                    print("  ..  WARNING: nothing received - check LC29H TX->GPIO16, "
                          "GND, baud")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop.set()
        rover.close()


if __name__ == "__main__":
    main()
