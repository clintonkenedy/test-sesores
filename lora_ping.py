#!/usr/bin/env python3
"""Link test for the E90-DTU LoRa radio path - run BEFORE sending corrections.

Measures whether the radio pipe moves bytes, how fast, and how many it drops.
Three modes:

  through  (bench, DEFAULT)  Both DTUs' Ethernet plugged into the same LAN.
           One machine sends numbered packets into DTU A and receives them
           back from DTU C after the RF hop. One clock -> exact one-way
           latency, loss and throughput. This is the corrections direction.

  echo     (field, far end)  Run on the computer plugged into the far DTU:
           returns every line it receives.

  ping     (field, near end) Sends packets into the near DTU and waits for
           the echo back: round-trip time over two RF hops.

Both DTUs must already share frequency/channel, air rate and packet size, be
in TCP Server mode, and have ANTENNAS CONNECTED (transmitting without an
antenna destroys the amplifier). On the bench: minimum TX power.

Examples:
    python lora_ping.py --tx 192.168.1.100:8887 --rx 192.168.1.101:8887
    python lora_ping.py --tx ... --rx ... --size 200 --interval 0.25
    python lora_ping.py --mode echo --dtu 192.168.1.101:8887
    python lora_ping.py --mode ping --dtu 192.168.1.100:8887
"""

import argparse
import socket
import statistics
import sys
import threading
import time

# ===========================================================================
#  CONFIGURATION - edit, or override on the command line.
# ===========================================================================

TX_DTU = "192.168.1.100:8887"    # DTU that transmits (A)
RX_DTU = "192.168.1.101:8887"    # DTU that receives after the RF hop (C)

PACKET_SIZE = 100                # bytes per test packet (RTCM frames: 30-210)
INTERVAL = 1.0                   # seconds between packets
COUNT = 0                        # 0 = until Ctrl+C
REPORT_EVERY = 10.0

# ===========================================================================


def parse_hostport(text):
    host, _, port = text.rpartition(":")
    return host, int(port)


def connect(label, hostport):
    host, port = parse_hostport(hostport)
    sock = socket.create_connection((host, port), timeout=5)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("  {}: connected to {}:{}".format(label, host, port))
    return sock


def build_packet(seq, size):
    """Numbered, timestamped, newline-framed, padded to the requested size."""
    head = "LP {} {} ".format(seq, time.monotonic_ns())
    pad = max(0, size - len(head) - 1)
    return (head + "x" * pad + "\n").encode("ascii")


def read_lines(sock, on_line, stop):
    """Reassemble newline-framed packets from a TCP stream."""
    buffer = bytearray()
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            print("  link closed by the DTU")
            break
        buffer.extend(chunk)
        while True:
            newline = buffer.find(b"\n")
            if newline < 0:
                break
            line = bytes(buffer[:newline])
            del buffer[:newline + 1]
            on_line(line)


class Tally:
    def __init__(self):
        self.lock = threading.Lock()
        self.sent = 0
        self.got = 0
        self.dup = 0
        self.reordered = 0
        self.latencies = []      # ms
        self.seen = set()
        self.highest = -1

    def record(self, seq, sent_ns):
        latency_ms = (time.monotonic_ns() - sent_ns) / 1e6
        with self.lock:
            if seq in self.seen:
                self.dup += 1
                return
            self.seen.add(seq)
            self.got += 1
            if seq < self.highest:
                self.reordered += 1
            self.highest = max(self.highest, seq)
            self.latencies.append(latency_ms)

    def report(self, label):
        with self.lock:
            if not self.sent:
                return
            loss = 100.0 * (self.sent - self.got) / self.sent
            line = "  {}: {}/{} received  loss {:.1f}%".format(
                label, self.got, self.sent, loss)
            if self.latencies:
                lat = sorted(self.latencies)
                line += ("  latency ms min/avg/max = "
                         "{:.0f}/{:.0f}/{:.0f}  (p95 {:.0f})".format(
                             lat[0], statistics.fmean(lat), lat[-1],
                             lat[int(len(lat) * 0.95) - 1 if len(lat) > 1 else 0]))
            if self.dup or self.reordered:
                line += "  dup {}  reordered {}".format(self.dup, self.reordered)
            print(line)


def on_test_line(line, tally):
    parts = line.split(b" ", 3)
    if len(parts) >= 3 and parts[0] == b"LP":
        try:
            tally.record(int(parts[1]), int(parts[2]))
        except ValueError:
            pass


def run_through(args):
    """One-way over the RF hop: send into TX DTU, receive from RX DTU."""
    print("THROUGH test: {} ~~RF~~ {}".format(args.tx, args.rx))
    print("Packet {} B every {} s. Ctrl+C to stop.\n".format(
        args.size, args.interval))
    tx = connect("tx", args.tx)
    rx = connect("rx", args.rx)
    tally = Tally()
    stop = threading.Event()
    reader = threading.Thread(
        target=read_lines, args=(rx, lambda l: on_test_line(l, tally), stop),
        daemon=True)
    reader.start()

    airtime_ms = args.size * 8 / 38400 * 1000
    print("  (estimated airtime per packet at 38.4 kbps: ~{:.0f} ms)\n".format(
        airtime_ms))

    next_report = time.monotonic() + args.report_every
    seq = 0
    try:
        while args.count == 0 or seq < args.count:
            tx.sendall(build_packet(seq, args.size))
            with tally.lock:
                tally.sent += 1
            seq += 1
            time.sleep(args.interval)
            if time.monotonic() >= next_report:
                tally.report("through")
                next_report = time.monotonic() + args.report_every
        time.sleep(2.0)          # let the last packets land
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        tally.report("FINAL")
        tx.close()
        rx.close()


def run_echo(args):
    """Far side: bounce every received line straight back."""
    print("ECHO responder on {} - Ctrl+C to stop.".format(args.dtu))
    while True:
        try:
            sock = connect("echo", args.dtu)
        except OSError as error:
            print("  cannot reach DTU: {} - retrying".format(error))
            time.sleep(3)
            continue
        stop = threading.Event()
        try:
            read_lines(sock, lambda l: sock.sendall(l + b"\n"), stop)
        except KeyboardInterrupt:
            sock.close()
            return
        sock.close()
        time.sleep(3)


def run_ping(args):
    """Near side: round trip over two RF hops, needs echo running remotely."""
    print("PING via {} (round trip, echo must run at the far end)\n".format(
        args.dtu))
    sock = connect("ping", args.dtu)
    tally = Tally()
    stop = threading.Event()
    reader = threading.Thread(
        target=read_lines, args=(sock, lambda l: on_test_line(l, tally), stop),
        daemon=True)
    reader.start()

    next_report = time.monotonic() + args.report_every
    seq = 0
    try:
        while args.count == 0 or seq < args.count:
            sock.sendall(build_packet(seq, args.size))
            with tally.lock:
                tally.sent += 1
            seq += 1
            time.sleep(args.interval)
            if time.monotonic() >= next_report:
                tally.report("ping")
                next_report = time.monotonic() + args.report_every
        time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        tally.report("FINAL")
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("through", "echo", "ping"),
                        default="through")
    parser.add_argument("--tx", default=TX_DTU,
                        help="through: transmitting DTU (default: %(default)s)")
    parser.add_argument("--rx", default=RX_DTU,
                        help="through: receiving DTU (default: %(default)s)")
    parser.add_argument("--dtu", default=TX_DTU,
                        help="echo/ping: the local DTU (default: %(default)s)")
    parser.add_argument("--size", type=int, default=PACKET_SIZE,
                        help="packet size in bytes (default: %(default)s)")
    parser.add_argument("--interval", type=float, default=INTERVAL,
                        help="seconds between packets (default: %(default)s)")
    parser.add_argument("--count", type=int, default=COUNT,
                        help="packets to send, 0 = endless (default: %(default)s)")
    parser.add_argument("--report-every", type=float, default=REPORT_EVERY)
    args = parser.parse_args()

    try:
        if args.mode == "through":
            run_through(args)
        elif args.mode == "echo":
            run_echo(args)
        else:
            run_ping(args)
    except OSError as error:
        print("Cannot reach the DTU: {}".format(error))
        print("Check: DTU powered, TCP Server mode, IP/port, same subnet.")


if __name__ == "__main__":
    main()
