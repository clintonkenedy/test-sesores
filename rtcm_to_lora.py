#!/usr/bin/env python3
"""Forward a filtered RTCM3 correction stream from a serial GNSS base to a LoRa DTU.

Reads the raw stream coming from the base, frames each RTCM3 message, drops the
ones a rover cannot use, and writes every surviving message to the radio.

Each message is written on its own. The radio caps a single transmission at 240
bytes and holds only 1000 bytes of receive buffer, so pushing raw byte chunks
risks silent truncation or overflow. One framed message per write stays inside
both limits by construction.

NMEA sentences are always dropped: they are base telemetry, not corrections.

In TCP mode the DTU must be configured as a TCP *Server*; this script is the
client that connects to it, and reconnects on its own if the link drops.

Example:
    python rtcm_to_lora.py --list
    python rtcm_to_lora.py --dry-run
    python rtcm_to_lora.py --host 192.168.1.100 --link-port 8887
"""

import argparse
import socket
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

# How to reach the LoRa DTU. "tcp" requires the DTU in TCP Server mode.
LINK_MODE = "tcp"                       # "tcp" or "udp"
DTU_HOST = "192.168.1.100"
DTU_PORT = 8887

# RTCM message numbers never forwarded.
# 1114 is QZSS: an Asia-Pacific constellation, always empty at this base.
DROP_MESSAGES = {1114}

# Minimum seconds between forwards of a given message.
# 1005 carries the base position, and the base does not move.
THROTTLE_SECONDS = {1005: 10.0}

# Radio limits, from the E90-DTU(900SL30) datasheet.
RADIO_MAX_PACKET = 240

# Link behaviour.
CONNECT_TIMEOUT = 5.0
RECONNECT_DELAY = 3.0
STATS_EVERY = 10.0

# ===========================================================================

RTCM_PREAMBLE = 0xD3
NMEA_START = ord("$")
RTCM_CRC_LEN = 3


def show_ports():
    """List visible serial ports so the base's adapter can be identified."""
    ports = sorted(list_ports.comports())
    if not ports:
        print("No serial ports found. Check the cable and the USB-serial driver.")
        return
    print("Available serial ports:")
    for port in ports:
        print("  {:<22} {}".format(port.device, port.description))


def read_messages(stream):
    """Yield (message_number, raw_frame). message_number is None for NMEA."""
    while True:
        lead = stream.read(1)
        if not lead:
            continue

        if lead[0] == RTCM_PREAMBLE:
            header = stream.read(2)
            if len(header) < 2:
                continue
            length = ((header[0] & 0x03) << 8) | header[1]
            body = stream.read(length + RTCM_CRC_LEN)
            # Truncated or degenerate frame means we lost sync; drop it and let
            # the next preamble resynchronise us.
            if length < 2 or len(body) < length + RTCM_CRC_LEN:
                continue
            number = (body[0] << 4) | (body[1] >> 4)
            yield number, lead + header + body

        elif lead[0] == NMEA_START:
            sentence = stream.readline()
            if sentence:
                yield None, lead + sentence


class RadioLink:
    """Delivers messages to the DTU, reconnecting on its own when TCP drops."""

    def __init__(self, mode, host, port, enabled=True):
        self.mode = mode
        self.address = (host, port)
        self.enabled = enabled
        self.socket = None
        self.next_retry = 0.0

    def _open(self):
        if self.mode == "udp":
            return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(CONNECT_TIMEOUT)
        sock.connect(self.address)
        # Nagle's algorithm would hold small correction messages back to
        # coalesce them. Freshness matters more than efficiency here.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("  connected to {}:{}".format(*self.address))
        return sock

    def _ensure_open(self):
        if self.socket is not None:
            return True
        now = time.monotonic()
        if now < self.next_retry:
            return False
        self.next_retry = now + RECONNECT_DELAY
        try:
            self.socket = self._open()
            return True
        except OSError as error:
            print("  cannot reach {}:{} - {}".format(
                self.address[0], self.address[1], error))
            return False

    def send(self, payload):
        """Return True if the payload was handed to the radio."""
        if not self.enabled:
            return True
        if not self._ensure_open():
            return False
        try:
            if self.mode == "udp":
                self.socket.sendto(payload, self.address)
            else:
                self.socket.sendall(payload)
            return True
        except OSError as error:
            print("  link lost - {}".format(error))
            self.close()
            return False

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None


def print_stats(sent, dropped, unsent, elapsed):
    """Summarise what went out over the radio versus what was filtered away."""
    sent_bytes = sum(sent.values())
    dropped_bytes = sum(dropped.values())
    total = sent_bytes + dropped_bytes + unsent
    if not total:
        print("No messages received. Check the serial port, baud rate and wiring.")
        return

    print("\n--- {:.0f} s ---".format(elapsed))
    for label in sorted(sent, key=sent.get, reverse=True):
        print("  SENT     {:<16} {:>7.1f} B/s".format(label, sent[label] / elapsed))
    for label in sorted(dropped, key=dropped.get, reverse=True):
        print("  dropped  {:<16} {:>7.1f} B/s".format(label, dropped[label] / elapsed))
    if unsent:
        print("  LOST     link down       {:>7.1f} B/s".format(unsent / elapsed))
    print("  -> over the air: {:.1f} B/s of {:.1f} B/s  ({:.0f}% saved)".format(
        sent_bytes / elapsed, total / elapsed, 100 * dropped_bytes / total))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="serial device of the base (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--mode", choices=("tcp", "udp"), default=LINK_MODE,
                        help="how to reach the DTU (default: %(default)s)")
    parser.add_argument("--host", default=DTU_HOST,
                        help="IP address of the LoRa DTU (default: %(default)s)")
    parser.add_argument("--link-port", type=int, default=DTU_PORT,
                        help="port the DTU listens on (default: %(default)s)")
    parser.add_argument("--drop", type=int, nargs="*", default=sorted(DROP_MESSAGES),
                        metavar="MSG",
                        help="RTCM message numbers to discard (default: %(default)s)")
    parser.add_argument("--dry-run", action="store_true",
                        help="filter and report without transmitting")
    parser.add_argument("--stats-every", type=float, default=STATS_EVERY,
                        help="seconds between stat reports (default: %(default)s)")
    parser.add_argument("--list", action="store_true",
                        help="list available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    drop = set(args.drop)
    link = RadioLink(args.mode, args.host, args.link_port, enabled=not args.dry_run)

    sent = defaultdict(int)
    dropped = defaultdict(int)
    unsent = 0
    last_forwarded = {}

    if args.dry_run:
        print("DRY RUN - nothing will be transmitted.")
    else:
        print("Forwarding over {} to {}:{}".format(
            args.mode.upper(), args.host, args.link_port))
    print("Reading {} at {} baud. Dropping NMEA and RTCM {}.".format(
        args.port, args.baud, sorted(drop) or "nothing"))

    started = time.monotonic()
    next_report = started + args.stats_every

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as stream:
            for number, frame in read_messages(stream):
                now = time.monotonic()

                if number is None:
                    dropped["NMEA"] += len(frame)
                elif number in drop:
                    dropped["RTCM {}".format(number)] += len(frame)
                elif (number in THROTTLE_SECONDS
                      and now - last_forwarded.get(number, 0)
                      < THROTTLE_SECONDS[number]):
                    dropped["RTCM {} (rate)".format(number)] += len(frame)
                else:
                    if len(frame) > RADIO_MAX_PACKET:
                        print("  WARNING: RTCM {} is {} bytes, over the {} byte "
                              "radio limit".format(number, len(frame),
                                                   RADIO_MAX_PACKET))
                    if link.send(frame):
                        last_forwarded[number] = now
                        sent["RTCM {}".format(number)] += len(frame)
                    else:
                        # Dropping a stale correction beats queueing it: by the
                        # time the link returns, the rover needs the next epoch.
                        unsent += len(frame)

                if now >= next_report:
                    print_stats(sent, dropped, unsent, now - started)
                    next_report = now + args.stats_every

    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
    except KeyboardInterrupt:
        print_stats(sent, dropped, unsent, time.monotonic() - started)
        print("\nStopped.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
