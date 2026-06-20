#!/usr/bin/env python3
# Generate src/sim.ico (no external deps) - a small CNC-sim glyph: dark tile, tan stock slab,
# red funnel/tool above it. Two sizes (32x32, 16x16) packed into one .ico.
import struct, os

def render(n):
    px = bytearray(n * n * 4)  # BGRA, top-down here; flipped when written
    def put(x, y, r, g, b, a=255):
        if 0 <= x < n and 0 <= y < n:
            i = (y * n + x) * 4
            px[i] = b; px[i+1] = g; px[i+2] = r; px[i+3] = a
    s = n / 32.0
    for y in range(n):
        for x in range(n):
            put(x, y, 38, 42, 50)                       # dark slate background
    # tan stock slab across the lower third
    y0, y1 = int(20*s), int(27*s)
    x0, x1 = int(4*s), int(28*s)
    for y in range(y0, y1):
        for x in range(x0, x1):
            put(x, y, 210, 174, 107)
    # red funnel: wide near the top, narrowing to a point just above the stock
    cx = n / 2.0
    ty, by = int(5*s), int(19*s)
    for y in range(ty, by):
        half = (by - y) / float(by - ty) * (9*s)
        for x in range(int(cx - half), int(cx + half) + 1):
            put(x, y, 220, 45, 45)
    return px

def ico_image(n):
    px = render(n)
    # BITMAPINFOHEADER: height doubled (XOR colour + AND mask), bottom-up rows
    hdr = struct.pack('<IiiHHIIiiII', 40, n, n*2, 1, 32, 0, 0, 0, 0, 0, 0)
    color = bytearray()
    for y in range(n-1, -1, -1):                        # bottom-up
        color += px[y*n*4:(y+1)*n*4]
    mask_row = (((n + 31)//32) * 4)                     # 1bpp rows padded to 32-bit
    mask = bytes(mask_row * n)                          # all 0 (alpha handles transparency)
    return hdr + bytes(color) + mask

sizes = [32, 16]
imgs = [ico_image(n) for n in sizes]
out = bytearray(struct.pack('<HHH', 0, 1, len(sizes)))  # ICONDIR
offset = 6 + 16 * len(sizes)
for n, img in zip(sizes, imgs):
    out += struct.pack('<BBBBHHII', n & 0xFF, n & 0xFF, 0, 0, 1, 32, len(img), offset)
    offset += len(img)
for img in imgs:
    out += img

dst = os.path.join(os.path.dirname(__file__), '..', 'src', 'sim.ico')
with open(dst, 'wb') as f:
    f.write(out)
print('wrote', os.path.normpath(dst), len(out), 'bytes')
