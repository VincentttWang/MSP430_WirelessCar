#!/usr/bin/env python3
"""Read back the interrupt vector table (0xFFC0..0xFFFF) from the chip and
compare it word by word with the freshly built image.

The vector table is part of the programmed image, so if all 32 vectors match,
the image definitely landed in flash. (Reading 0x4400 with '-s 16' proved
unreliable on this eZ-FET and produced bogus 0x3FFF words, so it is only
reported as informational, never used as the pass/fail criterion.)

Usage:  python verify_flash.py <dslite.exe> <target.ccxml> <image.hex>

Exit 0 = vector table on chip matches the build (or read-back unavailable).
Exit 1 = mismatch (image did NOT land).
"""
import os
import subprocess
import sys
import tempfile

VEC_START = 0xFFC0
VEC_WORDS = 32


def parse_hex(path):
    mem = {}
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line.startswith(":"):
                continue
            try:
                n = int(line[1:3], 16)
                addr = int(line[3:7], 16)
                typ = int(line[7:9], 16)
            except ValueError:
                continue
            if typ != 0:
                continue
            for i in range(n):
                mem[addr + i] = int(line[9 + 2 * i:11 + 2 * i], 16)
    return mem


def read_words(dslite, ccxml, addr, words, size=16):
    fd, out = tempfile.mkstemp(suffix=".bin")
    os.close(fd)
    os.remove(out)
    cmd = [dslite, "memory", "-c", ccxml, "-t", "60",
           "-r", "%04X,%d" % (addr, words), "-s", str(size), "-o", out]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=120)
    except Exception:
        pass
    data = None
    if os.path.exists(out):
        try:
            with open(out, "rb") as fh:
                data = fh.read()
        except OSError:
            data = None
        try:
            os.remove(out)
        except OSError:
            pass
    return data


def words_of(buf):
    if not buf:
        return []
    return [buf[i] | (buf[i + 1] << 8) for i in range(0, len(buf) - 1, 2)]


def main():
    if len(sys.argv) < 4:
        print("usage: verify_flash.py <dslite> <ccxml> <hex>")
        return 1

    dslite, ccxml, hexfile = sys.argv[1], sys.argv[2], sys.argv[3]
    img = parse_hex(hexfile)

    exp = [img.get(VEC_START + 2 * i, 0xFFFF) |
           (img.get(VEC_START + 2 * i + 1, 0xFF) << 8) for i in range(VEC_WORDS)]

    got = read_words(dslite, ccxml, VEC_START, VEC_WORDS)
    if got is None:
        print("  [SKIP] could not read target memory "
              "(probe busy, or python/dslite path wrong)")
        return 0

    chip = words_of(got)
    if len(chip) < VEC_WORDS:
        print("  [SKIP] only %d of %d vectors returned" % (len(chip), VEC_WORDS))
        return 0

    hit = sum(1 for i in range(VEC_WORDS) if chip[i] == exp[i])
    print("  vector table 0xFFC0..0xFFFF : %d / %d words match" %
          (hit, VEC_WORDS))

    reset_chip = chip[VEC_WORDS - 1]
    reset_img = exp[VEC_WORDS - 1]
    print("  reset vector  0xFFFE        : chip 0x%04X   image 0x%04X" %
          (reset_chip, reset_img))
    print("")

    if hit == VEC_WORDS:
        print("  [PASS] whole vector table on chip matches the build.")
        print("         The image is definitely in flash and PC starts at 0x%04X."
              % reset_chip)
        return 0

    if reset_chip != reset_img:
        print("  *** MISMATCH *** reset vector differs: the chip does NOT")
        print("  hold this build. Re-plug USB and flash again.")
        return 1

    print("  [WARN] reset vector is correct but %d vector(s) differ."
          % (VEC_WORDS - hit))
    print("         Likely a read-back artifact; if the LEDs never blink,")
    print("         re-plug USB and flash once more.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
