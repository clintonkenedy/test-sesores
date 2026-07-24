#!/usr/bin/env python3
"""Send PQTM configuration commands to the LC29H base and print its replies.

Built to fix a base whose position is pinned in flash: query the survey-in
configuration, switch it back to survey-in mode, save, and restart. Checksums
are computed automatically.

Run it on the machine where the base is plugged in (the correction server must
be STOPPED first - the port is exclusive). The ESP32 in between must forward
both directions; if --read gets no reply, flash usb_bridge.ino onto it first.

Typical repair sequence:
    python send_base_command.py --port COM5 --read
    python send_base_command.py --port COM5 --survey 120 30
    python send_base_command.py --port COM5 --save
    python send_base_command.py --port COM5 --restart
    (then wait ~3 min and confirm with check_base_position.py)
"""

import argparse
import sys
import time

import serial

SERIAL_PORT = "COM5" if sys.platform.startswith("win") else "/dev/cu.usbserial-0001"
SERIAL_BAUD = 115200
LISTEN_SECONDS = 4.0


def nmea_sentence(body):
    """Wrap an NMEA body with $, its XOR checksum and CRLF."""
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return "${}*{:02X}\r\n".format(body, checksum).encode("ascii")


def send_and_listen(port, baud, bodies):
    """Send each sentence, then print every $P... reply for a few seconds."""
    with serial.Serial(port, baud, timeout=0.5) as link:
        for body in bodies:
            sentence = nmea_sentence(body)
            print("-> {}".format(sentence.decode("ascii").strip()))
            link.write(sentence)
            time.sleep(0.2)

        print("   listening {} s for replies...".format(LISTEN_SECONDS))
        deadline = time.monotonic() + LISTEN_SECONDS
        line = bytearray()
        heard = False
        while time.monotonic() < deadline:
            data = link.read(256)
            for byte in data:
                if byte == 0x0A:
                    text = line.decode("ascii", "replace").strip()
                    line.clear()
                    # Only surface command replies; GGA/GSV chatter would bury them.
                    if text.startswith(("$PQTM", "$PAIR")):
                        print("<- {}".format(text))
                        heard = True
                elif byte != 0x0D and len(line) < 200:
                    line.append(byte)
        if not heard:
            print("   (no $PQTM/$PAIR reply - if this keeps happening, the")
            print("    ESP32 bridge is not forwarding PC -> LC29H; flash")
            print("    usb_bridge.ino onto it and retry)")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT,
                        help="serial port of the base (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--read", action="store_true",
                        help="query the current survey-in configuration")
    parser.add_argument("--survey", nargs=2, metavar=("SECONDS", "ACC_M"),
                        help="switch to survey-in mode, e.g. --survey 120 30")
    parser.add_argument("--fixed", nargs=3, metavar=("ECEF_X", "ECEF_Y", "ECEF_Z"),
                        help="pin the base to these ECEF coordinates (mode 2); "
                             "take them from check_base_position.py")
    parser.add_argument("--save", action="store_true",
                        help="persist the configuration to flash (PQTMSAVEPAR)")
    parser.add_argument("--restart", action="store_true",
                        help="restart the receiver so the new mode takes effect")
    parser.add_argument("--raw", default=None, metavar="BODY",
                        help="send an arbitrary body, e.g. --raw PQTMVERNO")
    args = parser.parse_args()

    bodies = []
    if args.read:
        bodies.append("PQTMCFGSVIN,R")
    if args.survey:
        duration, accuracy = args.survey
        # Mode 1 = survey-in. ECEF X/Y/Z are only used by fixed mode (2).
        bodies.append("PQTMCFGSVIN,W,1,{},{},0,0,0".format(duration, accuracy))
    if args.fixed:
        x, y, z = args.fixed
        # Mode 2 = fixed: the base broadcasts exactly these coordinates forever,
        # so power cycles no longer shift the position (and no survey blackout).
        bodies.append("PQTMCFGSVIN,W,2,0,0,{},{},{}".format(x, y, z))
    if args.save:
        bodies.append("PQTMSAVEPAR")
    if args.restart:
        bodies.append("PQTMSRR")
    if args.raw:
        bodies.append(args.raw)
    if not bodies:
        bodies.append("PQTMCFGSVIN,R")   # default: just look

    send_and_listen(args.port, args.baud, bodies)


if __name__ == "__main__":
    main()
