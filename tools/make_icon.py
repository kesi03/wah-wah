#!/usr/bin/env python3
"""Draw the VIBE FM afro icon as a 960x540 PNG using only the Python stdlib.

Run from the repository root:  python tools/make_icon.py
Writes firmware/wah-wah/icon.png
"""

import math
import struct
import zlib
from pathlib import Path

W, H = 960, 540
SS = 2  # supersampling for smooth edges
SW, SH = W * SS, H * SS

BG = (20, 13, 10)
STRIPE = (36, 24, 18)
ORANGE = (255, 140, 26)
GOLD = (242, 193, 78)
CREAM = (243, 229, 200)
AVO = (143, 164, 75)
BROWN = (52, 30, 20)


def blank():
    px = bytearray(SW * SH * 3)
    for y in range(SH):
        t = y / SH
        c = tuple(int(BG[i] + (8 + 6 * t) * (i == 0)) for i in range(3))
        for x in range(SW):
            o = (y * SW + x) * 3
            px[o:o + 3] = bytes(c)
    return px


def circle(px, cx, cy, r, color, inner=None):
    cx, cy, r = cx * SS, cy * SS, r * SS
    x0, x1 = max(0, int(cx - r - 1)), min(SW, int(cx + r + 2))
    y0, y1 = max(0, int(cy - r - 1)), min(SH, int(cy + r + 2))
    r2 = r * r
    for y in range(y0, y1):
        dy = y - cy
        for x in range(x0, x1):
            dx = x - cx
            d = dx * dx + dy * dy
            if d <= r2:
                col = color if inner is None or d >= inner * inner * SS * SS else inner
                o = (y * SW + x) * 3
                px[o:o + 3] = bytes(col)


def ring(px, cx, cy, r, thick, color):
    cx, cy, r = cx * SS, cy * SS, r * SS
    thick *= SS
    x0, x1 = max(0, int(cx - r - thick)), min(SW, int(cx + r + thick))
    y0, y1 = max(0, int(cy - r - thick)), min(SH, int(cy + r + thick))
    lo, hi = (r - thick) ** 2, (r + thick) ** 2
    for y in range(y0, y1):
        dy = y - cy
        for x in range(x0, x1):
            dx = x - cx
            d = dx * dx + dy * dy
            if lo <= d <= hi:
                o = (y * SW + x) * 3
                px[o:o + 3] = bytes(color)


def downsample(px):
    out = bytearray(W * H * 3)
    n = SS * SS
    for y in range(H):
        for x in range(W):
            r = g = b = 0
            for yy in range(SS):
                row = (y * SS + yy) * SW
                for xx in range(SS):
                    o = (row + x * SS + xx) * 3
                    r += px[o]; g += px[o + 1]; b += px[o + 2]
            o = (y * W + x) * 3
            out[o] = r // n; out[o + 1] = g // n; out[o + 2] = b // n
    return out


def png(rows):
    raw = b''.join(b'\x00' + bytes(row) for row in rows)
    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data +
                struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))
    return (b'\x89PNG\r\n\x1a\n' +
            chunk(b'IHDR', struct.pack('>IIBBBBB', W, H, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def main():
    px = blank()

    # warm 70s stripes behind the motif
    for i, (y, h, col) in enumerate(((0, 26, STRIPE), (H - 30, 30, STRIPE),
                                     (78, 6, BROWN), (H - 92, 6, BROWN))):
        for yy in range(y * SS, (y + h) * SS):
            for xx in range(SW):
                o = (yy * SW + xx) * 3
                px[o:o + 3] = bytes(col)

    cx, cy, r = 380, 268, 148
    ring(px, cx, cy, r + 42, 5, (90, 55, 32))

    # the afro itself: overlapping curls, orange and mustard
    for i in range(22):
        a = 2 * math.pi * i / 22
        x = cx + math.cos(a) * r * 0.74
        y = cy + math.sin(a) * r * 0.66 - r * 0.16
        circle(px, x, y, r * 0.45, ORANGE if i % 2 else GOLD)
    circle(px, cx, cy - r * 0.18, r * 0.82, ORANGE)

    # face, tiny eyes and a smile
    circle(px, cx, cy + r * 0.30, r * 0.50, CREAM)
    circle(px, cx - r * 0.17, cy + r * 0.26, r * 0.045, BG)
    circle(px, cx + r * 0.17, cy + r * 0.26, r * 0.045, BG)
    for i in range(28):
        t = i / 27
        a = math.pi * (0.18 + 0.64 * t)
        x = cx + math.cos(a) * r * 0.24
        y = cy + r * 0.42 + math.sin(a) * r * 0.10
        circle(px, x, y, r * 0.018, BROWN)

    # a couple of funky accent dots
    circle(px, 790, 190, 26, GOLD)
    circle(px, 850, 268, 14, AVO)
    circle(px, 790, 346, 20, ORANGE)

    small = downsample(px)
    rows = [small[y * W * 3:(y + 1) * W * 3] for y in range(H)]
    out = Path(__file__).resolve().parent.parent / 'firmware' / 'wah-wah' / 'icon.png'
    out.write_bytes(png(rows))
    print(f'wrote {out} ({out.stat().st_size} bytes)')


if __name__ == '__main__':
    main()
