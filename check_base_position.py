#!/usr/bin/env python3
"""Show where the base CLAIMS to be, from its live RTCM 1005 message.

Connects to the correction server, waits for a 1005, converts its ECEF
coordinates to latitude/longitude, and prints them. Compare that against
where the base antenna physically is: if they disagree by more than a few
hundred metres, the rover will reject every correction and never leave
SINGLE - the exact "age=?" symptom.

A stale 1005 happens when the base did its survey-in at one site and was
then moved to another: it keeps broadcasting the old position until it
re-surveys. Fix: power-cycle the base with its antenna seeing sky and let
the 120 s survey-in finish.

Example:
    python check_base_position.py --host 192.168.1.162
"""

import argparse
import math
import socket
import time

from pyrtcm import RTCMReader

BASE_HOST = "192.168.1.162"
BASE_PORT = 8887

# WGS84
A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)
B = A * (1 - F)

# GPS time bookkeeping, to tell a live stream from a replayed recording.
GPS_EPOCH_UNIX = 315964800      # 1980-01-06 00:00:00 UTC
GPS_UTC_LEAP = 18               # GPS is ahead of UTC by 18 s
WEEK_SECONDS = 604800


def expected_tow():
    """What a live receiver's time-of-week should be right now, in seconds."""
    return (time.time() + GPS_UTC_LEAP - GPS_EPOCH_UNIX) % WEEK_SECONDS


def tow_offset(stream_tow):
    """Smallest signed gap between the stream's tow and now, in seconds."""
    diff = (stream_tow - expected_tow()) % WEEK_SECONDS
    if diff > WEEK_SECONDS / 2:
        diff -= WEEK_SECONDS
    return diff


def ecef_to_lla(x, y, z):
    """Bowring's method: ECEF metres -> geodetic lat/lon (deg) and height (m)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    theta = math.atan2(z * A, p * B)
    ep2 = (A * A - B * B) / (B * B)
    lat = math.atan2(z + ep2 * B * math.sin(theta) ** 3,
                     p - E2 * A * math.cos(theta) ** 3)
    n = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=BASE_HOST,
                        help="correction server (default: %(default)s)")
    parser.add_argument("--port", type=int, default=BASE_PORT,
                        help="correction port (default: %(default)s)")
    args = parser.parse_args()

    print("Connecting to {}:{} and waiting for an RTCM 1005...".format(
        args.host, args.port))
    print("(1005 is throttled to every 10 s, so this can take that long)\n")

    position = None      # from 1005
    offset = None        # stream time vs real time, from 1074/1094 tow

    with socket.create_connection((args.host, args.port), timeout=15) as link:
        stream = link.makefile("rb")
        reader = RTCMReader(stream)
        for _, parsed in reader:
            if parsed is None:
                continue

            if parsed.identity == "1005" and position is None:
                position = (parsed.DF025, parsed.DF026, parsed.DF027)
            elif parsed.identity == "1074" and offset is None:
                offset = tow_offset(parsed.DF004 / 1000.0)
            elif parsed.identity == "1094" and offset is None:
                offset = tow_offset(parsed.DF248 / 1000.0)

            if position is not None and offset is not None:
                break

    x, y, z = position
    lat, lon, alt = ecef_to_lla(x, y, z)
    print("The base claims to be at:")
    print("  ECEF     X={:.3f}  Y={:.3f}  Z={:.3f}".format(x, y, z))
    print("  lat/lon  {:.6f}, {:.6f}".format(lat, lon))
    print("  height   {:.0f} m (ellipsoidal)".format(alt))
    print("\nOpen it:  https://maps.google.com/?q={:.6f},{:.6f}".format(lat, lon))

    print("\nStream clock check (observation tow vs real time):")
    if abs(offset) < 10:
        print("  LIVE - timestamps are current (off by {:+.1f} s).".format(offset))
        print("  So this IS the receiver speaking now. If the position above")
        print("  is wrong, it is pinned in the module (fixed/flash), and a")
        print("  power-cycle alone will NOT change it.")
    else:
        hours = offset / 3600.0
        print("  STALE - timestamps are {:+.1f} h from now.".format(hours))
        print("  This stream is a RECORDING or simulator, not the live base.")
        print("  Check what is really feeding the server's COM port.")


if __name__ == "__main__":
    main()
