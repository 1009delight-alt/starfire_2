"""把 render.exe 输出的 PPM 页面拼成一张放大 3 倍的 PNG（只用标准库）。
用法：python -I ppm2png.py <目录>  → <目录>/pages.png"""
import pathlib
import struct
import sys
import zlib

SCALE = 3
GAP = 6
COLS = 3


def read_ppm(path):
    data = path.read_bytes()
    parts = data.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    px = parts[3]
    return w, h, [[px[(y * w + x) * 3:(y * w + x) * 3 + 3] for x in range(w)] for y in range(h)]


def write_png(path, w, h, rows):
    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    path.write_bytes(png)


def main():
    d = pathlib.Path(sys.argv[1])
    pages = [read_ppm(p) for p in sorted(d.glob("*.ppm"))]
    if not pages:
        print("no ppm files")
        return 1
    pw, ph = pages[0][0] * SCALE, pages[0][1] * SCALE
    nrows = (len(pages) + COLS - 1) // COLS
    W = COLS * pw + (COLS + 1) * GAP
    H = nrows * ph + (nrows + 1) * GAP
    canvas = [bytearray(b"\x60\x60\x60" * W) for _ in range(H)]
    for i, (w, h, img) in enumerate(pages):
        ox = GAP + (i % COLS) * (pw + GAP)
        oy = GAP + (i // COLS) * (ph + GAP)
        for y in range(h):
            line = b"".join(img[y][x] * SCALE for x in range(w))
            for k in range(SCALE):
                canvas[oy + y * SCALE + k][ox * 3:(ox + pw) * 3] = line
    write_png(d / "pages.png", W, H, canvas)
    print(f"wrote {d / 'pages.png'} ({len(pages)} pages)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
