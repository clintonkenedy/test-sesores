#!/usr/bin/env python3
"""LoRa gateway: corrections out through DTU A, truck telemetry in through DTU C.

The split-role architecture (no repeater):

    LC29H base --COM--> [optimize] --TCP--> DTU A ~~RF~~> trucks
    trucks ~~RF~~> DTU C --TCP--> telemetry log + console

Corrections ride the epoch-aware optimizer (same levels as rtcm_optimizer).
Telemetry lines received from DTU C are timestamped into the same log the
dashboard reads (rtk_live_server.py), so the live map keeps working unchanged.

Because radio broadcast has no "client connected" event, the 1005 (base
position) is repeated every 5 s here instead of 10: a truck that switches on
mid-stream waits at most 5 s before it can start using corrections.

A truck simulator role is included for the bench: point it at the spare DTU
(B) and it plays a truck - counts the corrections that arrive over the air
and answers with a fake GGA telemetry line each second.

Examples:
    python lora_gateway.py --port COM5 --level 3
    python lora_gateway.py --port COM5 --level 3 --epoch-div 2
    python lora_gateway.py --mode sim --dtu 192.168.4.104:8887
"""

import argparse
import socket
import threading
import time
from collections import defaultdict
from datetime import datetime

import serial

import rtcm_optimizer
from rtcm_optimizer import EpochFilter
from rtcm_to_lora import ClientLink, open_serial, read_messages, show_ports

# ===========================================================================
#  CONFIGURATION - edit here, or override on the command line.
# ===========================================================================

SERIAL_PORT = "COM5"
SERIAL_BAUD = 115200

DTU_A = "192.168.4.103:8887"     # corrections OUT (base -> trucks)
DTU_C = "192.168.4.102:8887"     # telemetry IN   (trucks -> base)

LEVEL = 3
EPOCH_DIVIDER = 1

# Broadcast radio: no per-client replay possible, so repeat the base position
# often enough that a truck switching on never waits long.
BROADCAST_1005_SECONDS = 5.0

TELEMETRY_LOG = "rover_telemetry.log"
STATS_EVERY = 10.0

# ===========================================================================


def parse_hostport(text):
    host, _, port = text.rpartition(":")
    return host, int(port)


def log_telemetry_line(log, counters, text):
    """Timestamp one line from the trucks into the dashboard's log."""
    counters["telemetry_lines"] += 1
    source, _, rest = text.partition(" ")
    if rest and 1 <= len(source) <= 8 and not source.startswith(("$", "{")):
        payload = rest
    else:
        source, payload = "lora", text
    log.write("{}\t{}\t{}\n".format(
        datetime.now().isoformat(timespec="seconds"), source, payload))
    log.flush()
    if not payload.startswith("$"):
        print("  [C] <- {}  {}".format(source, payload))


class DuplexLink(ClientLink):
    """One TCP connection doing both jobs: corrections out, telemetry in.

    Field finding (2026-07-24 walk test): the E90's small TCP stack degraded
    after ~28 min of holding TWO client connections - it silently dropped the
    corrections socket the rover's fix died with it. A single connection
    halves the DTU's load, and the RF data it echoes back IS the telemetry,
    so nothing is lost by reading instead of draining.
    """

    def __init__(self, host, port, on_line):
        super().__init__("tcp", host, port)
        self.on_line = on_line
        self.buffer = bytearray()

    def poll(self):
        if self.socket is None:
            return
        try:
            self.socket.setblocking(False)
            while True:
                data = self.socket.recv(4096)
                if not data:
                    print("  [A] DTU closed the link - reconnecting")
                    self.close()
                    return
                self.buffer.extend(data)
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            self.close()
            return
        finally:
            if self.socket is not None:
                self.socket.setblocking(True)

        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                break
            text = bytes(self.buffer[:newline]).decode("ascii", "replace").strip()
            del self.buffer[:newline + 1]
            if text:
                self.on_line(text)


# ---------------------------------------------------------------------------
#  Telemetry side: DTU C -> log + console
# ---------------------------------------------------------------------------

def telemetry_listener(stop, dtu, log_path, counters):
    """Keep a connection to DTU C and log every line the trucks send."""
    host, port = parse_hostport(dtu)
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=5) as link:
                print("  [C] telemetry: connected to {}:{}".format(host, port))
                link.settimeout(0.5)
                buffer = bytearray()
                with open(log_path, "a", encoding="utf-8") as log:
                    while not stop.is_set():
                        try:
                            chunk = link.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            print("  [C] telemetry: DTU closed the link")
                            break
                        buffer.extend(chunk)
                        while True:
                            newline = buffer.find(b"\n")
                            if newline < 0:
                                break
                            text = bytes(buffer[:newline]).decode(
                                "ascii", "replace").strip()
                            del buffer[:newline + 1]
                            if not text:
                                continue
                            log_telemetry_line(log, counters, text)
        except OSError as error:
            print("  [C] telemetry: cannot reach {}:{} - {}".format(
                host, port, error))
        if not stop.is_set():
            time.sleep(3)


# ---------------------------------------------------------------------------
#  Corrections side: COM -> optimizer -> DTU A
# ---------------------------------------------------------------------------

def run_gateway(args):
    rtcm_optimizer.THROTTLE_SECONDS[1005] = args.rate_1005

    host, port = parse_hostport(args.dtu_a)
    counters = defaultdict(int)
    stop = threading.Event()
    single = (args.dtu_a == args.dtu_c)

    if single:
        # One DTU, ONE connection: telemetry rides back on the same socket.
        telemetry_log = open(args.log, "a", encoding="utf-8")
        link = DuplexLink(host, port,
                          lambda text: log_telemetry_line(
                              telemetry_log, counters, text))
    else:
        link = ClientLink("tcp", host, port)
        threading.Thread(target=telemetry_listener,
                         args=(stop, args.dtu_c, args.log, counters),
                         daemon=True).start()

    epoch_filter = EpochFilter(args.level, args.epoch_div)

    print("LoRa gateway")
    print("  corrections: {} -> optimizer L{} div{} -> DTU A {}".format(
        args.port, args.level, args.epoch_div, args.dtu_a))
    if single:
        print("  telemetry:   same connection (single-DTU duplex) -> {}".format(
            args.log))
    else:
        print("  telemetry:   DTU C {} -> {}".format(args.dtu_c, args.log))
    print("  1005 repeated every {:.0f} s (broadcast join time)\n".format(
        args.rate_1005))

    sent = defaultdict(int)
    dropped = defaultdict(int)
    started = time.monotonic()
    next_report = started + args.stats_every

    try:
        with open_serial(args.port, args.baud) as stream:
            for number, frame in read_messages(stream):
                now = time.monotonic()
                # Single-antenna mode: the DTU echoes everything it hears to
                # this socket too; drain it or the buffer fills over hours.
                link.poll()

                if number == -2:
                    if now >= next_report:
                        print("  ... no data from {} - check the base ESP32"
                              .format(args.port))
                        next_report = now + args.stats_every
                    continue
                if number is None or number == -1:
                    continue

                to_send, drops = epoch_filter.feed(number, frame, now)
                for label in drops:
                    dropped[label] += 1
                for label, out_frame in to_send:
                    if link.send(out_frame):
                        sent[label] += len(out_frame)
                    else:
                        counters["lost_bytes"] += len(out_frame)

                if now >= next_report:
                    next_report = now + args.stats_every
                    elapsed = now - started
                    total = sum(sent.values())
                    print("--- {:.0f} s ---  [A {}] out {:.1f} B/s | "
                          "[C] telemetry lines {}{}".format(
                              elapsed, link.status(), total / elapsed,
                              counters["telemetry_lines"],
                              "  | LOST {}B (A down)".format(
                                  counters["lost_bytes"])
                              if counters["lost_bytes"] else ""))
    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop.set()
        link.close()


# ---------------------------------------------------------------------------
#  Truck simulator: plays a truck through the spare DTU (bench test)
# ---------------------------------------------------------------------------

def run_sim(args):
    """Consume corrections off the air and answer with fake telemetry."""
    host, port = parse_hostport(args.dtu)
    print("TRUCK SIM via DTU {}:{} - counts corrections, sends a fake GGA/s"
          .format(host, port))
    while True:
        try:
            with socket.create_connection((host, port), timeout=5) as link:
                print("  sim: connected")
                link.settimeout(0.5)
                rx_bytes = 0
                frames = 0
                seq = 0
                last_send = 0.0
                last_report = time.monotonic()
                while True:
                    try:
                        chunk = link.recv(4096)
                        rx_bytes += len(chunk)
                        frames += chunk.count(b"\xd3")   # rough frame count
                    except socket.timeout:
                        pass
                    now = time.monotonic()
                    if now - last_send >= 1.0:
                        last_send = now
                        seq += 1
                        # Fake position near the Ananea bench, quality 4.
                        line = ("$GNGGA,120000.00,1438.8938,S,06937.2436,W,"
                                "4,32,0.8,4544.0,M,29.1,M,1.0,3335*00\r\n"
                                if seq % 5 else
                                "SIMTRUCK seq={} rx={}B frames~{}\r\n".format(
                                    seq, rx_bytes, frames))
                        link.sendall(line.encode("ascii"))
                    if now - last_report >= 10:
                        last_report = now
                        print("  sim: corrections received {} B (~{} frames)"
                              .format(rx_bytes, frames))
        except OSError as error:
            print("  sim: {} - retrying".format(error))
            time.sleep(3)
        except KeyboardInterrupt:
            print("\nStopped.")
            return


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("gateway", "sim"), default="gateway")
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="gateway: base serial port (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--dtu-a", default=DTU_A,
                        help="corrections DTU (default: %(default)s)")
    parser.add_argument("--dtu-c", default=DTU_C,
                        help="telemetry DTU (default: %(default)s)")
    parser.add_argument("--dtu", default=None,
                        help="sim: the spare DTU playing the truck")
    parser.add_argument("--level", type=int, choices=(1, 2, 3), default=LEVEL)
    parser.add_argument("--epoch-div", type=int, default=EPOCH_DIVIDER)
    parser.add_argument("--rate-1005", type=float, default=BROADCAST_1005_SECONDS)
    parser.add_argument("--log", default=TELEMETRY_LOG)
    parser.add_argument("--stats-every", type=float, default=STATS_EVERY)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return
    if args.mode == "sim":
        if not args.dtu:
            parser.error("--mode sim requires --dtu IP:PORT (the spare DTU)")
        run_sim(args)
    else:
        run_gateway(args)


if __name__ == "__main__":
    main()
