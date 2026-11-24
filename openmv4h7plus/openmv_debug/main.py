"""星火杯赛题二视觉：颜色顺序识别 + 转向标志识别（OpenMV4 H7 Plus）。"""

# ======================= 开关 =======================
MODE = "DEBUG"      # "DEBUG" 调试：画面显示识别细节，颜色上电即计数，记满 3 秒自动清空
                    # "RACE"  比赛：车载屏显示结果，颜色收到 STM32 的 S 才计数，结果一直保持
COLOR_ON = False     # 颜色顺序识别
TURN_ON = True      # 转向标志识别

# 串口 UART3（P4=TX，P5=RX，115200）。STM32 发单个字母即可：
#   S 开始颜色计数   R 清空全部结果   Q 查询颜色   T 查询转向
# 主动发出：颜色记满时 "ORDER=R-G-B-Y"，转向确认时 "TURN=L" 或 "TURN=R"

# ======================= 摄像头 =======================
FRAME_SIZE = "QVGA"     # QQVGA / QVGA / VGA
EXPOSURE_US = None      # 画面过亮发白时填一个比 EXP= 读数小的值，如 8000；None = 自动后锁定
GAIN_DB = None          # 过亮时可填 0；None = 自动后锁定
LCD_IN_DEBUG = False    # 调试模式也用车载屏（比赛模式总是开）

# ======================= 颜色顺序参数 =======================
# 阈值 (L下, L上, A下, A上, B下, B上)，对准中心小框看 LAB= 读数来调
TH_R = (12, 95, 25, 127, 8, 127)
TH_G = (12, 95, -128, -33, -5, 127)
TH_B = (8, 85, -35, 127, -128, -18)
TH_Y = (35, 100, -32, 24, 28, 127)
# 找不到色块时，按中心 9 个点的色相投票作补充（起, 止；红色跨 0°）
HUE = ((335, 22), (78, 170), (190, 265), (36, 72))      # 红、绿、蓝、黄
VOTES = 8               # 9 个点中至少几个同色才算；填 10 关闭投票
MIN_SAT = 28            # 饱和度下限（%），低于它的点当作灰色
# 白框和色块大小（按 160×120 填写，换分辨率自动换算）
ROI = (72, 35, 16, 64)  # 白框 (x, y, 宽, 高)
MIN_PIXELS = 80
MIN_W, MIN_H = 6, 10
MIN_DENSITY = 0.45
CONFIRM = 3             # 连续几帧认出同色才记一块
CLEAR = 3               # 两块之间至少几帧空框
REQUIRE_GAP = True      # True：任意两块之间都要空框；物块紧挨着时改 False

# ======================= 转向标志参数 =======================
# 标志是深蓝色圆牌 + 白色箭头。比较箭头上半部和下半部的左右位置判断方向。
TH_SIGN = (0, 50, -10, 60, -128, -10)   # 深蓝底；B 上限别超过 -8，否则黑线会混进来
TH_ARROW = (55, 100, -25, 25, -25, 25)  # 白箭头
TURN_ROI = (0, 0, 160, 120)             # 搜索范围（按 160×120 填写）；默认全画面
TURN_MIN_PIXELS = 60    # 蓝牌最少像素，调大可忽略远处的牌
TURN_MIN_DX = 0.08      # 箭头偏移量门限，素材测试中正确样本都在 0.13 以上
TURN_CONFIRM = 5        # 连续几帧同一方向才确认

# ======================= 内部常量 =======================
DEBUG = MODE == "DEBUG"
AUTO_RESET_MS = 3000 if DEBUG else 0
USE_LCD = LCD_IN_DEBUG or not DEBUG
LABELS = "RGBY"
SWATCHES = ((255, 0, 0), (0, 220, 0), (0, 80, 255), (255, 220, 0))
WHITE = (255, 255, 255)
EMPTY, MULTI = -1, -2
K = {"QQVGA": 1, "QVGA": 2, "VGA": 4}[FRAME_SIZE]
COLOR_TH = [TH_R, TH_G, TH_B, TH_Y]
CROI = tuple(v * K for v in ROI)
_sw, _sh = max(1, CROI[2] // 4), max(1, CROI[3] // 4)
SAMPLE = (CROI[0] + (CROI[2] - _sw) // 2, CROI[1] + (CROI[3] - _sh) // 2, _sw, _sh)
TROI = tuple(v * K for v in TURN_ROI)


def in_th(lab, th):
    return th[0] <= lab[0] <= th[1] and th[2] <= lab[1] <= th[3] and th[4] <= lab[2] <= th[5]


# ======================= 颜色识别 =======================
def blob_color(img):
    """白框内找色块：返回颜色编号；没有返回 EMPTY；多种颜色返回 MULTI。"""
    codes = 0
    for b in img.find_blobs(COLOR_TH, roi=CROI, pixels_threshold=MIN_PIXELS * K * K,
                            area_threshold=MIN_PIXELS * K * K, merge=False):
        if b.w() >= MIN_W * K and b.h() >= MIN_H * K and b.density() >= MIN_DENSITY:
            codes |= b.code()          # 四行阈值分别对应 1/2/4/8
    if not codes:
        return EMPTY
    return (1, 2, 4, 8).index(codes) if codes in (1, 2, 4, 8) else MULTI


def hue_of(r, g, b):
    hi, lo = max(r, g, b), min(r, g, b)
    d = hi - lo
    if hi < 35 or d == 0 or d * 100 < MIN_SAT * hi:
        return None                     # 太暗或太灰
    if hi == r:
        return (60 * (g - b) // d) % 360
    if hi == g:
        return 120 + 60 * (b - r) // d
    return 240 + 60 * (r - g) // d


def vote_color(img):
    x, y, w, h = SAMPLE
    votes = [0] * 4
    for row in (1, 3, 5):
        for col in (1, 3, 5):
            hue = hue_of(*img.get_pixel(x + col * w // 6, y + row * h // 6))
            if hue is None:
                continue
            for i, (a, b) in enumerate(HUE):
                if (a <= hue <= b) if a <= b else (hue >= a or hue <= b):
                    votes[i] += 1
                    break
    best = max(votes)
    return votes.index(best) if best >= VOTES else EMPTY


def detect_color(img):
    c = blob_color(img)
    if c != EMPTY:
        return c, "BLOB"
    c = vote_color(img)
    return c, ("VOTE" if c != EMPTY else "NONE")


class Counter:
    def __init__(self):
        self.order = []
        self.last = EMPTY       # 刚记入、还没离开白框的颜色
        self.seen = EMPTY       # 当前连续看到的结果
        self.n = 0
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


# ======================= 转向识别 =======================
def find_sign(img, to_lab):
    """找最大的深蓝色圆牌，返回 blob 或 None。四角是蓝色的（方牌、蓝色物块）会被排除。"""
    best = None
    for b in img.find_blobs([TH_SIGN], roi=TROI, pixels_threshold=TURN_MIN_PIXELS * K * K,
                            area_threshold=TURN_MIN_PIXELS * K * K, merge=False):
        x, y, w, h = b.x(), b.y(), b.w(), b.h()
        if not 0.7 <= w / h <= 1.4:
            continue
        dx, dy = w // 10, h // 10
        corners = ((x + dx, y + dy), (x + w - 1 - dx, y + dy),
                   (x + dx, y + h - 1 - dy), (x + w - 1 - dx, y + h - 1 - dy))
        if sum(in_th(to_lab(img.get_pixel(px, py)), TH_SIGN) for px, py in corners) > 1:
            continue
        if best is None or b.pixels() > best.pixels():
            best = b
    return best


def arrow_x(img, sign, top):
    """牌内上半或下半部分白色像素的平均 x；只算靠近圆心的部分，避开牌外白边。"""
    x, y, w, h = sign.x(), sign.y(), sign.w(), sign.h()
    cx, cy = x + w / 2, y + h / 2
    half = (x, y, w, max(1, h // 2)) if top else (x, y + h // 2, w, max(1, h - h // 2))
    total = moment = 0
    for b in img.find_blobs([TH_ARROW], roi=half, pixels_threshold=2, area_threshold=2, merge=False):
        if (b.cx() - cx) ** 2 + (b.cy() - cy) ** 2 < (0.4 * w) ** 2:
            total += b.pixels()
            moment += b.cx() * b.pixels()
    return moment / total if total else None


def detect_turn(img, to_lab):
    """返回 ("L"/"R"/None, 偏移量, 牌子 blob)。"""
    sign = find_sign(img, to_lab)
    if sign is None:
        return None, 0, None
    top, bottom = arrow_x(img, sign, True), arrow_x(img, sign, False)
    if top is None or bottom is None:
        return None, 0, sign
    dx = (top - bottom) / sign.w()      # 左转：箭头头部在左上，杆在右下 → dx < 0
    if dx <= -TURN_MIN_DX:
        return "L", dx, sign
    if dx >= TURN_MIN_DX:
        return "R", dx, sign
    return None, dx, sign


class TurnFilter:
    def __init__(self):
        self.result = None      # 已确认的方向
        self.seen = None
        self.n = 0

    def update(self, d):
        """连续 TURN_CONFIRM 帧同一方向才确认；确认结果变化时返回 True。"""
        self.n = self.n + 1 if d == self.seen else 1
        self.seen = d
        if d is not None and self.n >= TURN_CONFIRM and d != self.result:
            self.result = d
            return True
        return False

    def text(self):
        return "TURN=" + (self.result or "-")


# ======================= 串口命令 =======================
class App:
    def __init__(self):
        self.reset()

    def reset(self):
        self.counter = Counter()
        self.turn = TurnFilter()
        self.running = DEBUG    # 比赛模式等 S 才开始颜色计数
        self.reported = False
        self.dirty = True       # 需要刷新车载屏

    def feed(self, data):
        """每个字节当一条命令，返回回复列表；换行等其他字节忽略。"""
        replies = []
        for ch in data:
            cmd = chr(ch).upper()
            if cmd == "R":
                self.reset()
                replies.append("OK RESET")
            elif cmd == "S":
                self.running = True
                replies.append("OK START")
            elif cmd == "Q":
                replies.append(self.counter.text())
            elif cmd == "T":
                replies.append(self.turn.text())
        return replies


# ======================= 显示 =======================
def draw_order(img, order):
    """左上角 128×30 区域显示顺序，车载屏只刷新这一块。"""
    img.draw_rectangle((0, 0, 128, 30), color=(0, 0, 0), fill=True)
    for i in range(4):
        label = "-"
        if i < len(order):
            img.draw_rectangle((32 * i, 21, 28, 7), color=SWATCHES[order[i]], fill=True)
            label = LABELS[order[i]]
        img.draw_string(32 * i + 6, 2, label, color=WHITE, scale=2)


def draw_color_debug(img, c, how):
    s = img.get_statistics(roi=SAMPLE)          # 先取样再画线，免得把白线算进去
    name = LABELS[c] if c >= 0 else ("MULTI" if c == MULTI else "-")
    img.draw_rectangle(SAMPLE, color=WHITE)
    img.draw_rectangle(CROI, color=WHITE)
    img.draw_string(0, img.height() - 12, "NOW=%s %s LAB=%d,%d,%d" %
                    (name, how, s.l_median(), s.a_median(), s.b_median()), color=WHITE)


def draw_turn_debug(img, d, dx, sign, confirmed):
    if sign is not None:
        img.draw_rectangle(sign.rect(), color=(255, 0, 255))
        img.draw_line(sign.x(), sign.cy(), sign.x() + sign.w(), sign.cy(), color=(255, 0, 255))
    img.draw_string(img.width() - 112, 34, "SEE=%s %+.2f" % (d or "-", dx), color=WHITE)
    img.draw_string(img.width() - 112, 46, confirmed, color=WHITE)


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
    import image
    import time
    from pyb import UART

    cam = setup_camera(sensor)
    print("MODE=%s COLOR=%s TURN=%s %s" % (MODE, COLOR_ON, TURN_ON, cam))
    lcd = None
    if USE_LCD and COLOR_ON:
        import display
        lcd = display.SPIDisplay()
    uart = UART(3, 115200, timeout_char=0)
    clock = time.clock()
    app = App()
    full_at = None

    def reply(text):
        print(text)
        uart.write(text + "\r\n")

    while True:
        clock.tick()
        img = sensor.snapshot()
        if uart.any():
            for text in app.feed(uart.read() or b""):
                reply(text)

        if TURN_ON:
            d, dx, sign = detect_turn(img, image.rgb_to_lab)
            if app.turn.update(d):
                reply(app.turn.text())
            if DEBUG:
                draw_turn_debug(img, d, dx, sign, app.turn.text())

        if COLOR_ON:
            counter = app.counter
            counting = app.running and not counter.full()
            if counting or DEBUG:
                c, how = detect_color(img)
                if counting and counter.update(c):
                    app.dirty = True
                if DEBUG:
                    draw_color_debug(img, c, how)
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
                app.counter, app.reported, app.dirty = Counter(), False, True
                full_at = None
                print("AUTO RESET")

        if DEBUG:
            img.draw_string(0, img.height() - 24, "%s FPS=%.0f %s" %
                            ("RUN" if app.running else "WAIT S", clock.fps(), cam), color=WHITE)


if __name__ == "__main__":
    main()
