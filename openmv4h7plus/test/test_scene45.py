"""用右前 45° 合成素材离线跑比赛脚本的识别函数（不需要 OpenMV 硬件）。

素材在 openmv4h7plus/test/右前45度场景/（不入库，只在本机；320×240，与 FRAME_SIZE = "QVGA" 一致），真值在 标签.csv。
需要 Pillow、numpy、scipy（本机 py -3.13 有）：

    py -3.13 -I openmv4h7plus/test/test_scene45.py            # 汇总
    py -3.13 -I openmv4h7plus/test/test_scene45.py -v         # 另外打印逐帧/逐图细节

模拟的 img 只实现识别函数用到的接口：find_blobs（8 连通、merge=False、按阈值序号给 code）、
get_pixel（按 RGB565 量化）、width/height；rgb_to_lab 用 sRGB D65 公式。与 OpenMV 固件不会逐像素
一致，阈值边缘的结论要上板复核。
"""
import csv
import importlib.util
import os
import sys

import numpy as np
from PIL import Image, ImageSequence
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "右前45度场景")
VERBOSE = "-v" in sys.argv
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

spec = importlib.util.spec_from_file_location("mvmain", os.path.join(HERE, "..", "MAIN", "main.py"))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

COLOR_REPEAT = 3    # 动图每帧 120 ms，相机约 25 fps，每帧素材大约被拍到 3 次
SIGN_FRAMES = 10    # 静态标牌连续喂的帧数


def lab_of(rgb):
    """rgb: (..., 3) uint8 → (..., 3) float L, A, B（OpenMV 的范围：L 0~100，A/B -128~127）。"""
    c = rgb.astype(np.float64) / 255.0
    c = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    x = (c[..., 0] * 0.4124 + c[..., 1] * 0.3576 + c[..., 2] * 0.1805) / 0.95047
    y = c[..., 0] * 0.2126 + c[..., 1] * 0.7152 + c[..., 2] * 0.0722
    z = (c[..., 0] * 0.0193 + c[..., 1] * 0.1192 + c[..., 2] * 0.9505) / 1.08883

    def f(t):
        return np.where(t > 0.008856, np.cbrt(t), 7.787 * t + 16 / 116)
    fx, fy, fz = f(x), f(y), f(z)
    lab = np.stack([116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz)], axis=-1)
    return np.round(lab).clip([0, -128, -128], [100, 127, 127]).astype(np.int16)


def rgb565(rgb):
    """按 RGB565 量化后再展开到 8 位，模拟相机帧缓冲。"""
    r = (rgb[..., 0] >> 3).astype(np.uint16)
    g = (rgb[..., 1] >> 2).astype(np.uint16)
    b = (rgb[..., 2] >> 3).astype(np.uint16)
    return np.stack([(r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)], axis=-1).astype(np.uint8)


def to_lab(p):
    return tuple(int(v) for v in lab_of(np.array(p, dtype=np.uint8)))


class Blob:
    def __init__(self, x, y, w, h, pixels, cx, cy, code):
        self._r = (x, y, w, h)
        self._p, self._cx, self._cy, self._code = pixels, cx, cy, code

    def x(self): return self._r[0]
    def y(self): return self._r[1]
    def w(self): return self._r[2]
    def h(self): return self._r[3]
    def rect(self): return self._r
    def pixels(self): return self._p
    def cx(self): return self._cx
    def cy(self): return self._cy
    def code(self): return self._code
    def density(self): return self._p / float(self._r[2] * self._r[3])


class Img:
    def __init__(self, rgb):
        self.rgb = rgb565(rgb)
        self.lab = lab_of(self.rgb)

    def width(self): return self.rgb.shape[1]
    def height(self): return self.rgb.shape[0]

    def get_pixel(self, x, y):
        return tuple(int(v) for v in self.rgb[y, x])

    def find_blobs(self, ths, roi=None, pixels_threshold=10, area_threshold=10, merge=False):
        x0, y0, w0, h0 = roi or (0, 0, self.width(), self.height())
        x1, y1 = min(self.width(), x0 + w0), min(self.height(), y0 + h0)
        lab = self.lab[y0:y1, x0:x1]
        out = []
        for i, th in enumerate(ths):
            mask = ((lab[..., 0] >= th[0]) & (lab[..., 0] <= th[1]) & (lab[..., 1] >= th[2]) &
                    (lab[..., 1] <= th[3]) & (lab[..., 2] >= th[4]) & (lab[..., 2] <= th[5]))
            lbl, n = ndimage.label(mask, structure=np.ones((3, 3)))
            for k, sl in enumerate(ndimage.find_objects(lbl), 1):
                ys, xs = np.nonzero(lbl[sl] == k)
                px = len(xs)
                h, w = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
                if px < pixels_threshold or w * h < area_threshold:
                    continue
                out.append(Blob(x0 + sl[1].start, y0 + sl[0].start, w, h, px,
                                int(x0 + sl[1].start + xs.mean()), int(y0 + sl[0].start + ys.mean()), 1 << i))
        return out


def load_frames(path):
    im = Image.open(path)
    return [np.array(fr.convert("RGB")) for fr in ImageSequence.Iterator(im)]


def run_color(path):
    counter = m.Counter()
    trace = []
    for rgb in load_frames(path):
        c, how = m.detect_color(Img(rgb))
        trace.append("-" if c == m.EMPTY else ("?" if c == m.MULTI else m.LABELS[c]))
        for _ in range(COLOR_REPEAT):
            counter.update(c)
    return "".join(m.LABELS[i] for i in counter.order), "".join(trace)


def run_sign(path):
    img = Img(load_frames(path)[0])
    d, dx, sign = m.detect_turn(img, to_lab)
    found, cx, area, pb = m.detect_park(img, to_lab)
    tf, pf = m.TurnFilter(), m.ParkFilter()
    park = (0, 0, 0)
    for _ in range(SIGN_FRAMES):
        tf.update(d)
        park = pf.update(found, cx, area)
    info = "turn=%s dx=%+.2f sign=%s | park=%d cx=%+d area=%d rect=%s" % (
        d or "-", dx, sign.rect() if sign else None, found, cx, area, pb.rect() if pb else None)
    return tf.result, park[0] == 1, info


def main():
    rows = list(csv.DictReader(open(os.path.join(SCENE, "标签.csv"), encoding="utf-8-sig")))
    fails = []
    for r in rows:
        path = os.path.join(SCENE, r["文件"])
        name = r["文件"]
        if r["对象"] == "四色":
            got, trace = run_color(path)
            ok = got == r["真值"]
            if VERBOSE or not ok:
                print("%s  %-22s 真值 %s  得到 %-4s  逐帧 %s" % ("ok " if ok else "BAD", name, r["真值"], got, trace))
        else:
            want_turn = {"左转": "L", "右转": "R"}.get(r["对象"])
            want_park = r["对象"] == "P牌"
            turn, park, info = run_sign(path)
            ok = turn == want_turn and park == want_park
            if VERBOSE or not ok:
                print("%s  %-22s 期望 turn=%s park=%d  得到 turn=%s park=%d  %s" % (
                    "ok " if ok else "BAD", name, want_turn or "-", want_park, turn or "-", park, info))
        if not ok:
            fails.append(name)
    groups = {}
    for r in rows:
        g = groups.setdefault(r["对象"], [0, 0])
        g[0] += 1
        g[1] += r["文件"] not in fails
    print("\n" + "  ".join("%s %d/%d" % (k, v[1], v[0]) for k, v in groups.items()))
    print("总计 %d/%d 通过" % (len(rows) - len(fails), len(rows)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
