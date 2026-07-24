#!/usr/bin/env python3
"""Aggressive, epoch-aware RTCM optimizer for the LoRa link (level 2).

Where rtcm_to_lora.py only strips NMEA, this one understands MSM epochs and
applies the deep cuts safely:

  level 1   NMEA out + 1005 every 10 s              (~650 B/s from 758)
  level 2   + only GPS(1074x)+BeiDou(1124x) kept,
              epoch terminator preserved            (~440 B/s)
  level 3   + terminator dropped: DF393 of the new
              last message is cleared and its
              CRC-24Q recomputed                    (~410 B/s)
  --epoch-div N   forward 1 epoch out of N (whole
              epochs, never loose messages)         (/N on observations)

The DF393 lesson is built in: MSM messages of an epoch chain together and the
last one carries DF393=0 ("epoch complete"). Middle messages may be removed;
the terminator may only be removed by promoting another message to terminator,
which means rewriting one bit and the frame's CRC - done here explicitly.

Validate every level with the same A/B methodology: 12 min static, compare
CEP95/%FIXED/age against the raw baseline before trusting it in the field.

Examples:
    python rtcm_optimizer.py --mode server --port COM5 --level 2
    python rtcm_optimizer.py --mode server --port COM5 --level 3 --epoch-div 2
    python rtcm_optimizer.py --dry-run --level 3
"""

import argparse
import time
from collections import defaultdict

import serial

from rtcm_to_lora import (LISTEN_HOST, crc24q, make_link, open_serial,
                          print_stats, read_messages, show_ports)

# ===========================================================================
#  CONFIGURATION - edit here, or override on the command line.
# ===========================================================================

SERIAL_PORT = "COM5"
SERIAL_BAUD = 115200
LINK_MODE = "server"
DTU_HOST = "192.168.1.100"
LINK_PORT = 8887

LEVEL = 2                 # 1, 2 or 3 (see module docstring)
EPOCH_DIVIDER = 1         # 1 = every epoch, 2 = every other epoch, ...

# MSM hundreds kept at level >= 2: 107x = GPS, 112x = BeiDou. Chosen from the
# measured bytes-per-satellite at this site (GLONASS is single-frequency,
# Galileo cost 28 B/s per visible satellite - the worst value).
KEEP_MSM_HUNDREDS = {107, 112}

# Non-MSM throttling: the base is static, its position repeats.
THROTTLE_SECONDS = {1005: 10.0}

STATS_EVERY = 10.0
# If the terminator never arrives (lost upstream), flush rather than stall.
MAX_EPOCH_BUFFER = 16

# ===========================================================================

# DF393 ("multiple message bit") lives 54 bits into every MSM payload:
# DF002(12) + DF003(12) + epoch time(30). Payload starts at frame byte 3,
# so bit 54 sits in frame byte 9, mask 0x02.
DF393_BYTE = 9
DF393_MASK = 0x02


def is_msm(number):
    """MSM observation messages: 1071-1077 GPS ... 1121-1127 BeiDou etc."""
    return 1071 <= number <= 1127 and (number % 10) in range(1, 8)


def df393(frame):
    return (frame[DF393_BYTE] & DF393_MASK) != 0


def clear_df393(frame):
    """Return a copy promoted to epoch terminator, with a valid CRC again."""
    patched = bytearray(frame)
    patched[DF393_BYTE] &= ~DF393_MASK & 0xFF
    crc = crc24q(patched[:-3])
    patched[-3:] = crc.to_bytes(3, "big")
    return bytes(patched)


class EpochFilter:
    """Buffers one MSM epoch at a time and emits the optimized version."""

    def __init__(self, level, epoch_divider):
        self.level = level
        self.epoch_divider = max(1, epoch_divider)
        self.buffer = []          # (number, frame) of the epoch in progress
        self.epochs = 0
        self.last_sent = {}       # throttle bookkeeping for non-MSM

    def feed(self, number, frame, now):
        """Return (to_send, drops): frames to forward and stat labels dropped."""
        if not is_msm(number):
            throttle = THROTTLE_SECONDS.get(number)
            if (throttle is not None
                    and now - self.last_sent.get(number, 0) < throttle):
                return [], ["RTCM {} (rate)".format(number)]
            self.last_sent[number] = now
            return [("RTCM {}".format(number), frame)], []

        self.buffer.append((number, frame))
        if df393(frame) and len(self.buffer) < MAX_EPOCH_BUFFER:
            return [], []          # epoch still open, keep buffering

        # Epoch complete (or safety flush): decide what survives.
        epoch, self.buffer = self.buffer, []
        self.epochs += 1

        if (self.epochs - 1) % self.epoch_divider:
            return [], ["epoch (div)"] * len(epoch)

        if self.level < 2:
            return [("RTCM {}".format(n), f) for n, f in epoch], []

        drops = []
        kept = []
        terminator = epoch[-1]
        for i, (n, f) in enumerate(epoch):
            if n // 10 in KEEP_MSM_HUNDREDS:
                kept.append((n, f))
            elif i < len(epoch) - 1:      # the terminator is decided below
                drops.append("RTCM {}".format(n))

        last_n, last_f = terminator
        if last_n // 10 in KEEP_MSM_HUNDREDS:
            pass                          # terminator already among the kept
        elif self.level >= 3 and kept:
            # Drop the foreign terminator; promote the last kept message.
            drops.append("RTCM {} (term)".format(last_n))
            n, f = kept[-1]
            kept[-1] = (n, clear_df393(f))
        else:
            kept.append(terminator)       # level 2: cheap, structurally safe

        return [("RTCM {}".format(n), f) for n, f in kept], drops


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="serial device of the base (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--mode", choices=("tcp", "udp", "server"),
                        default=LINK_MODE)
    parser.add_argument("--host", default=DTU_HOST)
    parser.add_argument("--link-port", type=int, default=LINK_PORT)
    parser.add_argument("--level", type=int, choices=(1, 2, 3), default=LEVEL)
    parser.add_argument("--epoch-div", type=int, default=EPOCH_DIVIDER,
                        metavar="N", help="forward 1 epoch out of N")
    parser.add_argument("--stats-every", type=float, default=STATS_EVERY)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    link = make_link(args.mode, args.host, LISTEN_HOST, args.link_port,
                     args.dry_run)
    epoch_filter = EpochFilter(args.level, args.epoch_div)

    sent = defaultdict(int)
    dropped = defaultdict(int)
    unsent = 0

    print("Optimizer level {} · epoch divider {} · keeping MSM {}".format(
        args.level, args.epoch_div,
        sorted(h * 10 + 4 for h in KEEP_MSM_HUNDREDS)))
    if args.dry_run:
        print("DRY RUN - nothing will be transmitted.")

    started = time.monotonic()
    next_report = started + args.stats_every

    try:
        with open_serial(args.port, args.baud) as stream:
            for number, frame in read_messages(stream):
                now = time.monotonic()
                if link is not None:
                    link.poll()

                if number == -2:              # idle: no serial data this second
                    if now >= next_report:
                        print("  ... no data from {} for a while - check the "
                              "base ESP32 / USB".format(args.port))
                        next_report = now + args.stats_every
                    continue
                if number is None:
                    dropped["NMEA"] += len(frame)
                    continue
                if number == -1:
                    dropped["RTCM bad-crc"] += len(frame)
                    continue

                to_send, drops = epoch_filter.feed(number, frame, now)
                for label in drops:
                    dropped[label] += 1   # counted in messages, cheap enough
                for label, out_frame in to_send:
                    if link is None or link.send(out_frame):
                        sent[label] += len(out_frame)
                    else:
                        unsent += len(out_frame)
                # Keep the latest throttled frames for instant replay to any
                # rover that connects mid-cycle (cuts startup by up to 10 s).
                if number in THROTTLE_SECONDS and hasattr(link, "welcome"):
                    link.welcome[number] = frame

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
