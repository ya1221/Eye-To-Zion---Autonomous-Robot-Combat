#!/usr/bin/env python3
"""
WitMotion D6 protocol sniffer — decodes YDLidar-format (0xAA 0x55) packets.

Usage:
    pip install pyserial
    python3 d6_protocol_test.py [/dev/ttyUSB0]

Stop any ROS node using the port first. If the D6 really speaks the
YDLidar protocol, you'll see rotation frequency + smooth angle/distance
readings. If you see nothing but raw resync messages, the ID is wrong.
"""
import sys
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 230400
HDR = b"\xaa\x55"


def angle(raw: int) -> float:
    """YDLidar angle field: (raw >> 1) / 64 degrees."""
    return (raw >> 1) / 64.0


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Listening on {PORT} @ {BAUD} baud ... Ctrl+C to stop\n")
    buf = bytearray()
    shown = 0
    try:
        while shown < 40:
            chunk = ser.read(512)
            if not chunk:
                print("(no data — check wiring/port)")
                continue
            buf += chunk

            while True:
                i = buf.find(HDR)
                if i < 0:
                    buf = buf[-1:]          # keep last byte (partial header)
                    break
                if len(buf) < i + 10:
                    break                    # wait for full sub-header
                ct, lsn = buf[i + 2], buf[i + 3]
                need = i + 10 + 3 * lsn      # 3 bytes/sample (intensity mode)
                if len(buf) < need:
                    break                    # wait for full packet

                fsa = int.from_bytes(buf[i + 4:i + 6], "little")
                lsa = int.from_bytes(buf[i + 6:i + 8], "little")

                if ct & 0x01:
                    freq = (ct >> 1) / 10.0
                    print(f"[start of scan]  rotation = {freq:.1f} Hz")
                else:
                    a0, a1 = angle(fsa), angle(lsa)
                    span = (a1 - a0) % 360
                    dists, intens = [], []
                    for k in range(lsn):
                        o = i + 10 + 3 * k
                        intens.append(buf[o])
                        dists.append(int.from_bytes(buf[o + 1:o + 3], "little"))
                    print(f"[scan] {lsn:2d} pts  {a0:6.1f} -> {a1:6.1f} deg "
                          f"(span {span:5.1f})  dist mm: "
                          f"min {min(dists):5d}  max {max(dists):5d}  "
                          f"first3 {dists[:3]}  intens~{sum(intens)//len(intens)}")
                    # NOTE: if distances look ~4x too large for your room,
                    # the triangle-variant scaling applies: dist_mm = raw >> 2

                buf = buf[need:]
                shown += 1
                if shown >= 40:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nDone.")


if __name__ == "__main__":
    main()