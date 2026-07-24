#!/usr/bin/env python3
"""UDP LoRa gateway: corrections out, telemetry in - zero connection state.

Replaces the TCP transport of lora_gateway.py after the field failure of
2026-07-24: the E90's embedded TCP stack silently dropped the corrections
socket after ~28 min (its own manual admits servers can reject reconnections
after incomplete 4-way closes). UDP has no handshake, no client table and no
zombies - there is nothing on the DTU that can rot, and everything survives
any reboot on either side with zero reconnection logic.

Datagram loss on the wired LAN hop is ~nil, and RTK already tolerates lost
epochs by design (the radio hop was never guaranteed either).

DTU CONFIGURATION (web -> Socket A):
    Work mode:   UDP Client
    Local port:  8886                 (where corrections arrive)
    Dest IP:     192.168.4.100        (this PC)
    Dest port:   8888                 (where telemetry lands)
  UDP Server mode also works (it replies to the last sender, and we send
  every couple of seconds) - UDP Client is simply more deterministic.
  Leave "timeout restart" at its 300 s default: with data flowing it never
  triggers, and if the DTU ever wedges it reboots itself. Free self-healing.

Health rule (UDP has no "connected" signal): telemetry age IS the link
status. The stats line warns when nothing has been heard for a while.

Example:
    python lora_gateway_udp.py --port COM5 --level 3 --epoch-div 2 --dtu 192.168.4.102:8886
"""

import argparse
import socket
import time
from collections import defaultdict

import serial

import rtcm_optimizer
from rtcm_optimizer import EpochFilter
from rtcm_to_lora import open_serial, read_messages, show_ports
from lora_gateway import log_telemetry_line, parse_hostport

# ===========================================================================
#  CONFIGURATION - edit here, or override on the command line.
# ===========================================================================

SERIAL_PORT = "COM5"
SERIAL_BAUD = 115200

DTU = "192.168.4.102:8886"       # the E90's IP and its UDP local port
LOCAL_PORT = 8888                # we listen here (= DTU's "dest port")

LEVEL = 3
EPOCH_DIVIDER = 2
BROADCAST_1005_SECONDS = 5.0

TELEMETRY_LOG = "rover_telemetry.log"
STATS_EVERY = 10.0
STALE_AFTER = 30.0               # warn if no telemetry for this long

# ===========================================================================


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="base serial port (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--dtu", default=DTU,
                        help="DTU ip:udp-port (default: %(default)s)")
    parser.add_argument("--local-port", type=int, default=LOCAL_PORT,
                        help="UDP port we listen on; must equal the DTU's "
                             "'dest port' (default: %(default)s)")
    parser.add_argument("--level", type=int, choices=(1, 2, 3), default=LEVEL)
    parser.add_argument("--epoch-div", type=int, default=EPOCH_DIVIDER)
    parser.add_argument("--rate-1005", type=float,
                        default=BROADCAST_1005_SECONDS)
    parser.add_argument("--log", default=TELEMETRY_LOG)
    parser.add_argument("--stats-every", type=float, default=STATS_EVERY)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    rtcm_optimizer.THROTTLE_SECONDS[1005] = args.rate_1005
    destination = parse_hostport(args.dtu)

    # One connectionless socket does everything: sendto() corrections,
    # recvfrom() telemetry. Nothing to connect, nothing to reconnect.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.local_port))
    sock.setblocking(False)

    epoch_filter = EpochFilter(args.level, args.epoch_div)
    telemetry_log = open(args.log, "a", encoding="utf-8")
    counters = defaultdict(int)
    line_buffer = bytearray()

    sent = defaultdict(int)
    last_telemetry = None

    print("UDP LoRa gateway (connectionless)")
    print("  corrections: {} -> optimizer L{} div{} -> {}:{} (UDP)".format(
        args.port, args.level, args.epoch_div, *destination))
    print("  telemetry:   UDP :{} -> {}".format(args.local_port, args.log))
    print("  1005 repeated every {:.0f} s\n".format(args.rate_1005))

    def pump_telemetry(now):
        """Drain every waiting datagram; newline-frame into lines."""
        nonlocal last_telemetry
        while True:
            try:
                data, _ = sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError:
                return
            last_telemetry = now
            line_buffer.extend(data)
            while True:
                newline = line_buffer.find(b"\n")
                if newline < 0:
                    break
                text = bytes(line_buffer[:newline]).decode(
                    "ascii", "replace").strip()
                del line_buffer[:newline + 1]
                if text:
                    log_telemetry_line(telemetry_log, counters, text)

    started = time.monotonic()
    next_report = started + args.stats_every

    try:
        with open_serial(args.port, args.baud) as stream:
            for number, frame in read_messages(stream):
                now = time.monotonic()
                pump_telemetry(now)

                if number == -2:
                    if now >= next_report:
                        print("  ... no data from {} - check the base ESP32"
                              .format(args.port))
                        next_report = now + args.stats_every
                    continue
                if number is None or number == -1:
                    continue

                to_send, _ = epoch_filter.feed(number, frame, now)
                for label, out_frame in to_send:
                    try:
                        sock.sendto(out_frame, destination)
                        sent[label] += len(out_frame)
                    except OSError as error:
                        # e.g. network cable pulled; datagrams are stateless,
                        # the next one simply goes out when the wire returns.
                        print("  sendto failed: {}".format(error))

                if now >= next_report:
                    next_report = now + args.stats_every
                    elapsed = now - started
                    total = sum(sent.values())
                    age = ("{:.0f}s ago".format(now - last_telemetry)
                           if last_telemetry else "never")
                    line = ("--- {:.0f} s ---  out {:.1f} B/s | telemetry "
                            "lines {} (last {})".format(
                                elapsed, total / elapsed,
                                counters["telemetry_lines"], age))
                    if last_telemetry and now - last_telemetry > STALE_AFTER:
                        line += "  | WARN: telemetry stale - DTU/truck down?"
                    print(line)

    except serial.SerialException as error:
        print("Could not open {}: {}".format(args.port, error))
        print("Run with --list to see which ports exist.")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()
        telemetry_log.close()


if __name__ == "__main__":
    main()
