#!/usr/bin/env python3
"""Forward a filtered RTCM3 correction stream from a serial GNSS base.

Reads the raw stream coming from the base, frames each RTCM3 message, drops the
ones a rover cannot use, and delivers every surviving message over the chosen
link.

Three link modes:
  tcp     - client; connects to a LoRa DTU running as TCP Server
  udp     - client; sends datagrams to a LoRa DTU
  server  - TCP server; rovers connect in over the LAN (WiFi test)

Each message is delivered on its own. A LoRa DTU caps a single transmission at
240 bytes and holds only 1000 bytes of receive buffer, so pushing raw byte
chunks risks silent truncation or overflow. One framed message per write stays
inside both limits by construction; over the LAN it keeps message boundaries
clean for the rover.

NMEA sentences are always dropped: they are base telemetry, not corrections.

Example:
    python rtcm_to_lora.py --list
    python rtcm_to_lora.py --dry-run
    python rtcm_to_lora.py --mode server          # WiFi LAN test
    python rtcm_to_lora.py --host 192.168.1.100   # to a DTU (TCP client)
"""

import argparse
import select
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

# How corrections leave this machine.
#   "tcp"    -> connect out to a DTU in TCP Server mode
#   "udp"    -> send datagrams to a DTU
#   "server" -> listen so rovers connect in over the LAN (the WiFi test)
LINK_MODE = "server"

# Client modes (tcp/udp): where the DTU is.
DTU_HOST = "192.168.1.100"
# Both: the TCP/UDP port. In server mode this is the port rovers connect to.
LINK_PORT = 8887
# Server mode: which interface to listen on. 0.0.0.0 = all, so WiFi clients reach it.
LISTEN_HOST = "0.0.0.0"

# RTCM message numbers never forwarded.
# 1114 is QZSS: an Asia-Pacific constellation, always empty at this base.
DROP_MESSAGES = {1114}

# Minimum seconds between forwards of a given message.
# 1005 carries the base position, and the base does not move.
THROTTLE_SECONDS = {1005: 10.0}

# Radio limit, from the E90-DTU(900SL30) datasheet.
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


def _disable_nagle(sock):
    """Small correction messages must go out now, not wait to be coalesced."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


class ClientLink:
    """Delivers messages to one DTU, reconnecting on its own when TCP drops."""

    def __init__(self, mode, host, port):
        self.mode = mode
        self.address = (host, port)
        self.socket = None
        self.next_retry = 0.0

    def _open(self):
        if self.mode == "udp":
            return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(CONNECT_TIMEOUT)
        sock.connect(self.address)
        _disable_nagle(sock)
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

    def status(self):
        return "connected" if self.socket else "waiting"

    def poll(self):
        """A DTU client has no rover telemetry to read back here."""

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None


class ServerLink:
    """Listens for rover connections and broadcasts each message to all of them."""

    def __init__(self, host, port):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(8)
        self.listener.setblocking(False)
        self.clients = {}       # conn -> address
        self.inbox = {}         # conn -> partial line buffer of rover telemetry
        print("  listening on {}:{} - waiting for rovers".format(host, port))

    def _accept_new(self):
        while True:
            ready, _, _ = select.select([self.listener], [], [], 0)
            if not ready:
                return
            conn, address = self.listener.accept()
            _disable_nagle(conn)
            conn.setblocking(False)
            self.clients[conn] = address
            self.inbox[conn] = b""
            print("  rover connected from {}:{}  ({} total)".format(
                address[0], address[1], len(self.clients)))

    def _drop(self, conn, why):
        address = self.clients.pop(conn, ("?", 0))
        self.inbox.pop(conn, None)
        conn.close()
        print("  rover {}:{} {}  ({} left)".format(
            address[0], address[1], why, len(self.clients)))

    def poll(self):
        """Read anything rovers send back and print it (the fix summary)."""
        self._accept_new()
        socks = list(self.clients)
        if not socks:
            return
        readable, _, _ = select.select(socks, [], [], 0)
        for conn in readable:
            try:
                data = conn.recv(512)
            except OSError:
                data = b""
            if not data:
                self._drop(conn, "gone")
                continue
            buffer = self.inbox[conn] + data
            *lines, self.inbox[conn] = buffer.split(b"\n")
            host = self.clients[conn][0]
            for line in lines:
                text = line.decode("ascii", "replace").strip()
                if text:
                    print("  <- {}  {}".format(host, text))

    def send(self, payload):
        self._accept_new()
        for conn in list(self.clients):
            try:
                conn.sendall(payload)
            except OSError:
                self._drop(conn, "gone")
        # Corrections with no rover attached are not "lost" - nobody is there
        # yet - so a healthy broadcast is always a success.
        return True

    def status(self):
        return "{} rover(s)".format(len(self.clients))

    def close(self):
        for conn in self.clients:
            conn.close()
        self.clients.clear()
        self.listener.close()


def make_link(mode, host, listen_host, port, dry_run):
    """Build the link for the chosen mode, or None for a dry run."""
    if dry_run:
        return None
    if mode == "server":
        return ServerLink(listen_host, port)
    return ClientLink(mode, host, port)


def print_stats(sent, dropped, unsent, elapsed, link):
    """Summarise what went out versus what was filtered away."""
    sent_bytes = sum(sent.values())
    dropped_bytes = sum(dropped.values())
    total = sent_bytes + dropped_bytes + unsent
    if not total:
        print("No messages received. Check the serial port, baud rate and wiring.")
        return

    where = "  [{}]".format(link.status()) if link is not None else ""
    print("\n--- {:.0f} s ---{}".format(elapsed, where))
    for label in sorted(sent, key=sent.get, reverse=True):
        print("  SENT     {:<16} {:>7.1f} B/s".format(label, sent[label] / elapsed))
    for label in sorted(dropped, key=dropped.get, reverse=True):
        print("  dropped  {:<16} {:>7.1f} B/s".format(label, dropped[label] / elapsed))
    if unsent:
        print("  LOST     link down       {:>7.1f} B/s".format(unsent / elapsed))
    print("  -> forwarded: {:.1f} B/s of {:.1f} B/s  ({:.0f}% saved)".format(
        sent_bytes / elapsed, total / elapsed, 100 * dropped_bytes / total))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="serial device of the base (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--mode", choices=("tcp", "udp", "server"), default=LINK_MODE,
                        help="tcp/udp connect to a DTU, server listens (default: "
                             "%(default)s)")
    parser.add_argument("--host", default=DTU_HOST,
                        help="DTU address for tcp/udp (default: %(default)s)")
    parser.add_argument("--link-port", type=int, default=LINK_PORT,
                        help="tcp/udp/listen port (default: %(default)s)")
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
    link = make_link(args.mode, args.host, LISTEN_HOST, args.link_port, args.dry_run)

    sent = defaultdict(int)
    dropped = defaultdict(int)
    unsent = 0
    last_forwarded = {}

    if args.dry_run:
        print("DRY RUN - nothing will be transmitted.")
    elif args.mode == "server":
        print("Serving corrections on port {} (rovers connect in)".format(
            args.link_port))
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
                if link is not None:
                    link.poll()   # surface anything a rover sent back

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
                    if link is None or link.send(frame):
                        last_forwarded[number] = now
                        sent["RTCM {}".format(number)] += len(frame)
                    else:
                        # Dropping a stale correction beats queueing it: by the
                        # time the link returns, the rover needs the next epoch.
                        unsent += len(frame)

                if now >= next_report:
                    print_stats(sent, dropped, unsent, now - started, link)
                    next_report = now + args.stats_every

    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
    except KeyboardInterrupt:
        print_stats(sent, dropped, unsent, time.monotonic() - started, link)
        print("\nStopped.")
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
