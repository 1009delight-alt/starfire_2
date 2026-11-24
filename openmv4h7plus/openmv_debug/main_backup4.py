"""星火杯四色顺序识别：摄像头朝向赛道侧面，物块依次穿过画面中的白框。

串口 UART3（P4=TX，P5=RX，115200），每条命令以换行结尾：
    R  清空结果      S  开始计数      Q  查询结果
    P  打印全部参数（可直接复制回下面保存）
    SET 名字 数值    运行中临时改参数，如 SET CONFIRM 2 / SET ROI 70 35 18 64 / SET TH_Y 30 100 -30 20 25 127
"""

# ======================= 参数 =======================
# --- 摄像头 ---
FRAME_SIZE = "QVGA"     # QQVGA / QVGA / VGA
EXPOSURE_US = None      # 画面过亮发白时填一个比 EXP= 读数小的值，如 8000；None = 自动后锁定
GAIN_DB = None          # 过亮时可填 0；None = 自动后锁定

# --- 运行方式 ---
SHOW_DEBUG = True       # IDE 画面上显示实时识别结果和 LAB 读数
USE_LCD = False         # 比赛必须用车载屏显示：接好屏后改 True
WAIT_START = False      # True：收到串口 S 才开始计数（接 STM32 时用）
AUTO_RESET_MS = 3000    # 调试用：记满 4 块后多少毫秒自动清空；比赛前必须改 0！

# --- 颜色阈值 (L下, L上, A下, A上, B下, B上)，对准中心小框看 LAB= 读数来调 ---
TH_R = (12, 95, 25, 127, 8, 127)
TH_G = (12, 95, -128, -33, -5, 127)
TH_B = (8, 85, -35, 127, -128, -18)
TH_Y = (35, 100, -32, 24, 28, 127)
# 找不到色块时，按中心 9 个点的色相投票作为补充（色相范围 起, 止；红色跨 0°）
HUE_R = (335, 22)
HUE_G = (78, 170)
HUE_B = (190, 265)
HUE_Y = (36, 72)
VOTES = 8               # 9 个点中至少几个同色才算数；填 10 关闭投票
MIN_SAT = 28            # 饱和度下限（%），低于它的点当作灰色忽略

# --- 白框和色块大小（按 160×120 填写，换分辨率自动换算） ---
ROI = (72, 35, 16, 64)  # 白框 (x, y, 宽, 高)
MIN_PIXELS = 80         # 色块最少像素数
MIN_W = 6               # 色块最小宽、高
MIN_H = 10
MIN_DENSITY = 0.45      # 色块填充率下限，提高可滤噪点

# --- 计数 ---
CONFIRM = 3             # 连续几帧认出同色才记一块
CLEAR = 3               # 两块之间至少几帧空框
REQUIRE_GAP = True      # True：任意两块之间都要空框；物块紧挨着时改 False

TUNABLE = ("TH_R", "TH_G", "TH_B", "TH_Y", "HUE_R", "HUE_G", "HUE_B", "HUE_Y", "VOTES",
           "MIN_SAT", "ROI", "MIN_PIXELS", "MIN_W", "MIN_H", "MIN_DENSITY", "CONFIRM",
           "CLEAR", "REQUIRE_GAP", "SHOW_DEBUG")

# ======================= 识别 =======================
LABELS = "RGBY"
SWATCHES = ((255, 0, 0), (0, 220, 0), (0, 80, 255), (255, 220, 0))
EMPTY, MULTI = -1, -2
K = {"QQVGA": 1, "QVGA": 2, "VGA": 4}[FRAME_SIZE]


class Cfg:
    """按当前分辨率换算后的实际值，参数变化后调用 update()。"""

    def update(self):
        self.th = [TH_R, TH_G, TH_B, TH_Y]
        self.hue = [HUE_R, HUE_G, HUE_B, HUE_Y]
        x, y, w, h = [v * K for v in ROI]
        self.roi = (x, y, w, h)
        sw, sh = max(1, w // 4), max(1, h // 4)
        self.sample = (x + (w - sw) // 2, y + (h - sh) // 2, sw, sh)   # 中心取色小框


cfg = Cfg()
cfg.update()


def blob_color(img):
    """白框内找色块：返回颜色编号，没有返回 EMPTY，多种颜色返回 MULTI。"""
    codes = 0
    for b in img.find_blobs(cfg.th, roi=cfg.roi, pixels_threshold=MIN_PIXELS * K * K,
                            area_threshold=MIN_PIXELS * K * K, merge=False):
        if b.w() >= MIN_W * K and b.h() >= MIN_H * K and b.density() >= MIN_DENSITY:
            codes |= b.code()          # 第 1~4 行阈值分别对应 1/2/4/8
    if not codes:
        return EMPTY
    return (1, 2, 4, 8).index(codes) if codes in (1, 2, 4, 8) else MULTI


def hue_of(r, g, b):
    hi, lo = max(r, g, b), min(r, g, b)
    d = hi - lo
    if hi < 35 or d * 100 < MIN_SAT * hi or d == 0:
        return None                     # 太暗或太灰
    if hi == r:
        return (60 * (g - b) // d) % 360
    if hi == g:
        return 120 + 60 * (b - r) // d
    return 240 + 60 * (r - g) // d


def vote_color(img):
    """中心小框 9 个点按色相投票。"""
    x, y, w, h = cfg.sample
    votes = [0] * 4
    for row in (1, 3, 5):
        for col in (1, 3, 5):
            hue = hue_of(*img.get_pixel(x + col * w // 6, y + row * h // 6))
            if hue is None:
                continue
            for i, (a, b) in enumerate(cfg.hue):
                if (a <= hue <= b) if a <= b else (hue >= a or hue <= b):
                    votes[i] += 1
                    break
    best = max(votes)
    return votes.index(best) if best >= VOTES else EMPTY


def detect(img):
    c = blob_color(img)
    if c != EMPTY:
        return c, "BLOB"
    c = vote_color(img)
    return c, ("VOTE" if c != EMPTY else "NONE")


# ======================= 计数 =======================
class Counter:
    def __init__(self):
        self.order = []
        self.last = EMPTY       # 刚记入、还没离开白框的颜色
        self.seen = EMPTY       # 当前连续看到的结果
        self.n = 0              # 已连续多少帧
        self.armed = True       # 是否允许记下一块

    def full(self):
        return len(self.order) == 4

    def update(self, c):
        """喂入一帧结果，记入新的一块时返回 True。"""
        if self.full():
            return False
        self.n = self.n + 1 if c == self.seen else 1
        self.seen = c
        if c == EMPTY and self.n >= CLEAR:
            self.last, self.armed = EMPTY, True
        ok = self.armed if REQUIRE_GAP else c != self.last
        if c >= 0 and ok and self.n >= CONFIRM:
            self.order.append(c)
            self.last, self.armed = c, False
            return True
        return False

    def text(self):
        return "ORDER=" + "-".join(LABELS[i] for i in self.order)


# ======================= 串口命令 =======================
def params_text():
    g = globals()
    return "\n".join("%s = %r" % (n, g[n]) for n in TUNABLE)


def set_param(name, vals):
    g = globals()
    if name not in TUNABLE:
        return "ERR unknown " + name
    old = g[name]
    try:
        if isinstance(old, bool):
            new = int(vals[0]) != 0
        elif isinstance(old, tuple):
            if len(vals) != len(old):
                return "ERR need %d numbers" % len(old)
            new = tuple(int(v) for v in vals)
        else:
            new = type(old)(vals[0])
    except (ValueError, IndexError):
        return "ERR bad value"
    g[name] = new
    cfg.update()
    return "OK %s = %r" % (name, new)


class App:
    def __init__(self):
        self.buf = b""
        self.reset()

    def reset(self):
        self.counter = Counter()
        self.running = not WAIT_START
        self.reported = False
        self.dirty = True       # 需要刷新车载屏

    def feed(self, data):
        """处理串口字节，返回回复列表。"""
        self.buf = (self.buf + data).replace(b"\r", b"\n")
        lines = self.buf.split(b"\n")
        self.buf = lines.pop()[-80:]            # 最后一段还没收到换行，留着
        return [self.command(l.decode().split()) for l in lines if l.strip()]

    def command(self, parts):
        cmd = parts[0].upper()
        if cmd == "R":
            self.reset()
            return "OK RESET"
        if cmd == "S":
            self.running = True
            return "OK START"
        if cmd == "Q":
            return self.counter.text()
        if cmd == "P":
            return params_text()
        if cmd == "SET" and len(parts) > 1:
            return set_param(parts[1].upper(), parts[2:])
        return "ERR unknown command"


# ======================= 显示 =======================
WHITE = (255, 255, 255)


def draw_order(img, order):
    """左上角 128×30 区域显示顺序，车载屏只刷新这一块。"""
    img.draw_rectangle((0, 0, 128, 30), color=(0, 0, 0), fill=True)
    for i in range(4):
        label = "-"
        if i < len(order):
            img.draw_rectangle((32 * i, 21, 28, 7), color=SWATCHES[order[i]], fill=True)
            label = LABELS[order[i]]
        img.draw_string(32 * i + 6, 2, label, color=WHITE, scale=2)


def draw_debug(img, c, how, app, fps, cam):
    s = img.get_statistics(roi=cfg.sample)      # 先取样再画线，免得把白线算进去
    name = LABELS[c] if c >= 0 else ("MULTI" if c == MULTI else "-")
    img.draw_rectangle(cfg.sample, color=WHITE)
    img.draw_string(0, img.height() - 24, "%s FPS=%.0f %s" %
                    ("RUN" if app.running else "WAIT", fps, cam), color=WHITE)
    img.draw_string(0, img.height() - 12, "NOW=%s %s LAB=%d,%d,%d" %
                    (name, how, s.l_median(), s.a_median(), s.b_median()), color=WHITE)


# ======================= 主程序 =======================
def setup_camera(sensor):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, FRAME_SIZE))
    sensor.skip_frames(time=2000)               # 等自动曝光/白平衡稳定后锁定
    sensor.set_auto_whitebal(False)
    if GAIN_DB is None:
        sensor.set_auto_gain(False)
    else:
        sensor.set_auto_gain(False, gain_db=GAIN_DB)
    if EXPOSURE_US:
        sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    else:
        sensor.set_auto_exposure(False)
    sensor.skip_frames(time=300)
    return "EXP=%d GAIN=%.0f" % (sensor.get_exposure_us(), sensor.get_gain_db())


def main():
    import sensor
    import time
    from pyb import UART

    cam = setup_camera(sensor)
    print(cam)
    if not USE_LCD:
        print("WARNING: USE_LCD=False，比赛时必须用车载屏显示")
    if AUTO_RESET_MS:
        print("WARNING: AUTO_RESET_MS=%d，比赛前改 0" % AUTO_RESET_MS)
    lcd = None
    if USE_LCD:
        import display
        lcd = display.SPIDisplay()
    uart = UART(3, 115200, timeout_char=0)
    clock = time.clock()
    app = App()
    full_at = None

    def reply(text):
        print(text)
        uart.write(text.replace("\n", "\r\n") + "\r\n")

    while True:
        clock.tick()
        img = sensor.snapshot()
        if uart.any():
            for text in app.feed(uart.read() or b""):
                reply(text)

        counter = app.counter
        counting = app.running and not counter.full()
        if counting or SHOW_DEBUG:
            c, how = detect(img)
            if counting and counter.update(c):
                app.dirty = True
            if SHOW_DEBUG:
                draw_debug(img, c, how, app, clock.fps(), cam)
        img.draw_rectangle(cfg.roi, color=WHITE)
        draw_order(img, counter.order)

        if app.dirty:
            print(counter.text())
            if lcd:
                lcd.write(img, roi=(0, 0, 128, 30), y=60)
            app.dirty = False
        if counter.full() and not app.reported:
            reply(counter.text())
            app.reported = True
            full_at = time.ticks_ms()
        if AUTO_RESET_MS and full_at is not None and \
                time.ticks_diff(time.ticks_ms(), full_at) > AUTO_RESET_MS:
            app.reset()
            full_at = None
            reply("AUTO RESET")


if __name__ == "__main__":
    main()
