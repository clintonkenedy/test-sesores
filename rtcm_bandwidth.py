#!/usr/bin/env python3
"""Measure per-message-type bandwidth on a raw GNSS serial stream.

Frames RTCM3 messages and NMEA sentences without decoding their payloads, then
reports how many bytes per second each message type costs. Use it to decide
which messages are worth keeping on a bandwidth-constrained link.

An RTCM3 frame only needs five bytes to identify:

    0xD3 | 6 reserved bits + 10 length bits | payload | CRC-24Q
                                              `- first 12 bits = message number

Example:
    python rtcm_bandwidth.py --list
    python rtcm_bandwidth.py --seconds 60
"""

import argparse
import sys
import time
from collections import defaultdict

import serial
from serial.tools import list_ports

# ===========================================================================
#  CONFIGURATION - edit these values, or override any of them on the command
#  line. The command line always wins.
# ===========================================================================

# Serial port of the base. Windows: "COM3". macOS/Linux: "/dev/cu.usbserial-0001".
SERIAL_PORT = "COM3" if sys.platform.startswith("win") else "/dev/cu.usbserial-0001"
SERIAL_BAUD = 115200

# How long to sample before printing the table.
SAMPLE_SECONDS = 60.0

# ===========================================================================

RTCM_PREAMBLE = 0xD3
NMEA_START = ord("$")
RTCM_HEADER_LEN = 3
RTCM_CRC_LEN = 3


def rtcm_message_number(payload):
    """Return DF002, the 12-bit message number opening every RTCM3 payload."""
    return (payload[0] << 4) | (payload[1] >> 4)


def show_ports():
    """List visible serial ports so the base's adapter can be identified."""
    ports = sorted(list_ports.comports())
    if not ports:
        print("No serial ports found. Check the cable and the USB-serial driver.")
        return
    print("Available serial ports:")
    for port in ports:
        print("  {:<22} {}".format(port.device, port.description))


def read_frames(port, baudrate, duration):
    """Yield (label, frame_size_bytes) for each message seen within duration."""
    deadline = time.monotonic() + duration
    with serial.Serial(port, baudrate, timeout=1) as stream:
        while time.monotonic() < deadline:
            lead = stream.read(1)
            if not lead:
                continue

            if lead[0] == RTCM_PREAMBLE:
                header = stream.read(2)
                if len(header) < 2:
                    continue
                length = ((header[0] & 0x03) << 8) | header[1]
                body = stream.read(length + RTCM_CRC_LEN)
                # A truncated or degenerate frame means we lost sync; drop it
                # and let the next 0xD3 resynchronise us.
                if length < 2 or len(body) < length + RTCM_CRC_LEN:
                    continue
                label = "RTCM {}".format(rtcm_message_number(body))
                yield label, RTCM_HEADER_LEN + length + RTCM_CRC_LEN

            elif lead[0] == NMEA_START:
                sentence = stream.readline()
                if not sentence:
                    continue
                talker = sentence.split(b",", 1)[0].decode("ascii", "replace")
                yield "NMEA {}".format(talker.strip()), 1 + len(sentence)


def report(totals, counts, elapsed):
    """Print a bandwidth table sorted by cost, heaviest message first."""
    grand_total = sum(totals.values())
    if not grand_total:
        print("No framed messages received. Check the port, baud rate and wiring.")
        return

    print("\nSampled {:.1f} s\n".format(elapsed))
    print("{:<18} {:>6} {:>8} {:>10} {:>10} {:>8}".format(
        "MESSAGE", "COUNT", "MSG/S", "BYTES/S", "BITS/S", "SHARE"))
    print("-" * 64)

    for label in sorted(totals, key=totals.get, reverse=True):
        bytes_per_second = totals[label] / elapsed
        print("{:<18} {:>6} {:>8.2f} {:>10.1f} {:>10.0f} {:>7.1f}%".format(
            label,
            counts[label],
            counts[label] / elapsed,
            bytes_per_second,
            bytes_per_second * 8,
            100 * totals[label] / grand_total))

    total_bps = grand_total / elapsed
    print("-" * 64)
    print("{:<18} {:>6} {:>8} {:>10.1f} {:>10.0f} {:>7.1f}%".format(
        "TOTAL", sum(counts.values()), "", total_bps, total_bps * 8, 100.0))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="serial device (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--seconds", type=float, default=SAMPLE_SECONDS,
                        help="sampling window (default: %(default)s)")
    parser.add_argument("--list", action="store_true",
                        help="list available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    counts = defaultdict(int)
    totals = defaultdict(int)

    print("Sampling {} at {} baud for {:.0f} s...".format(
        args.port, args.baud, args.seconds))

    started = time.monotonic()
    try:
        for label, size in read_frames(args.port, args.baud, args.seconds):
            counts[label] += 1
            totals[label] += size
    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
        return
    except KeyboardInterrupt:
        print("\nInterrupted, reporting what was captured.")

    report(totals, counts, time.monotonic() - started)


if __name__ == "__main__":
    main()
