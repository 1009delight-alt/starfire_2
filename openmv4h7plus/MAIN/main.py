"""星火杯赛题二视觉 · 比赛版（OpenMV4 H7 Plus）。

与 STM32 主控按《比赛程序开发交接文档》第 4 章的 $ 帧协议通信。主控指定当前任务，
相机只执行这一项识别：TURN（分岔转向牌）、COLOR（四色顺序）、PARK（车库 P 牌）。
识别函数沿用 openmv_debug/main.py；调参先在调试版里做，再把阈值抄过来。
"""

# ======================= 开关 =======================
MODE = "RACE"       # "RACE"  比赛：只听主控命令，不画调试信息，不用车载屏
                    # "DEBUG" 调试：画面叠加识别细节；未收到主控命令时按 DEBUG_TASK 运行
DEBUG_TASK = "COLOR"    # 调试模式的默认任务：IDLE / TURN / COLOR / PARK
LCD_IN_DEBUG = False    # 调试模式是否用 SPI 车载屏显示颜色顺序（比赛不用，由主控 OLED 显示）

# 串口 UART3（P4=TX，P5=RX，115200 8N1）
# 主控 → 相机：$CMD:IDLE#  $CMD:TURN#  $CMD:COLOR#  $CMD:PARK#  $CMD:RESET#  $CMD:QUERY#
#             $CMD:COLOR:R-G#（带上主控已收到的颜色，相机重启后从下一块接着数）
# 相机 → 主控：$ACK:<任务>#  $HB:<任务>,<fps>#（每 500ms）  $TURN:L#/$TURN:R#
#             $COLOR:<n>,R-G-B-Y#  $PARK:<1|0>,<cx>,<area>#
HB_PERIOD_MS = 500
RESEND_MS = 200         # 已确认的方向、满 4 色后的结果，每隔多久重发一次，防丢帧
MAX_BODY = 63           # 帧体最大字节数，超长整帧丢弃

# ======================= 摄像头 =======================
FRAME_SIZE = "QVGA"     # QQVGA / QVGA / VGA
CAMERA_ROTATE_180 = True  # 摄像头倒装时旋转 180 度；正装时改为 False
EXPOSURE_US = None      # 画面过亮发白时填一个比 EXP= 读数小的值，如 8000；None = 自动后锁定
GAIN_DB = None          # 过亮时可填 0；None = 自动后锁定

# ======================= 颜色顺序参数（TH_Y、MIN_SAT 11-26 按暖光改过，调试版未同步） =======================
TH_R = (12, 95, 25, 127, 8, 127)
TH_G = (12, 100, -128, -30, -5, 127)  # 11-26 现场绿色 A 中位数约 -53；收紧上限，避免黄色阴影混入（合成素材 76 号强反光要 -15）
TH_B = (8, 85, -35, 127, -128, -18)
TH_Y = (35, 100, -25, 24, 28, 127)   # 11-26 现场黄色 B 约 29~61，背景约 3；A 与绿色范围分开（合成素材暖光墙面 B 约 28~31）
HUE = ((335, 22), (78, 170), (190, 265), (36, 72))      # 红、绿、蓝、黄
VOTES = 8
MIN_SAT = 40            # 饱和度下限（%）；暖光墙面约 32%，低于 40 当作灰色
ROI = (72, 35, 16, 64)  # 白框 (x, y, 宽, 高)，按 160×120 填写
MIN_PIXELS = 80
MIN_W, MIN_H = 6, 10
MIN_DENSITY = 0.45
CONFIRM = 3
CLEAR = 3
REQUIRE_GAP = True

# ======================= 转向标志参数（11-26 按右前 45° 改过，调试版未同步） =======================
TH_SIGN = (0, 80, -10, 60, -128, -12)   # 深蓝底；L 上限 80：强反光处蓝牌 L 到 75、B 约 -16
TH_ARROW = (55, 100, -25, 25, -12, 40)  # 白箭头/白字；B 下限 -12 与反光蓝区分开，上限 40 容下暖光米黄字
TURN_ASPECT = (0.45, 1.4)  # 宽高比；右前 45° 斜看圆牌约 0.6~0.75
TURN_ROI = (0, 0, 160, 120)
TURN_MIN_PIXELS = 60
TURN_MIN_DX = 0.08
ARROW_INSET = 0.15      # 算箭头时牌框四边各内缩的比例
ARROW_RADIUS = 0.35     # 白块中心按牌宽、高归一后离圆心的上限（牌外白边约 0.5）
TURN_CONFIRM = 5

# ======================= P 牌参数（待实测） =======================
# 蓝底白字方牌：四角是蓝色（与圆形转向牌相反），框内有一定比例的白色笔画
TH_PARK = TH_SIGN       # 蓝底阈值，先沿用转向牌，按实物 LAB 读数再调
PARK_ROI = (0, 0, 160, 120)
PARK_MIN_PIXELS = 70    # 蓝牌最少像素（按 160×120）
PARK_MIN_SIDE = 10      # 外接矩形最短边（按 160×120），太小的远处色块不要
PARK_ASPECT = (0.40, 1.50)  # 宽高比；右前 45° 斜看约 0.55~0.65
PARK_MIN_CORNERS = 2    # 4 个内缩 1/8 的角点中至少几个是蓝色；斜看成平行四边形会丢一角，遮挡再丢一角
PARK_WHITE_RATIO = (0.08, 0.65)  # 牌中部白色像素占中部面积的比例范围；过白的是白板之类
PARK_WHITE_MIN_H = 0.5  # P 字白色外接框高度至少占牌中部高度的比例（横白条约 0.25）
PARK_WHITE_TALL = 1.2   # P 字白色外接框高宽比下限（P 约 2.2，横白条约 0.4）
PARK_CONFIRM = 4        # 连续几帧看到才上报 flag=1，防单帧误检触发入库

# ======================= 内部常量 =======================
DEBUG = MODE == "DEBUG"
AUTO_RESET_MS = 3000 if DEBUG else 0
LABELS = "RGBY"
SWATCHES = ((255, 0, 0), (0, 220, 0), (0, 80, 255), (255, 220, 0))
WHITE = (255, 255, 255)
EMPTY, MULTI = -1, -2
TASKS = ("IDLE", "TURN", "COLOR", "PARK")
K = {"QQVGA": 1, "QVGA": 2, "VGA": 4}[FRAME_SIZE]
COLOR_TH = [TH_R, TH_G, TH_B, TH_Y]
CROI = tuple(v * K for v in ROI)
_sw, _sh = max(1, CROI[2] // 4), max(1, CROI[3] // 4)
SAMPLE = (CROI[0] + (CROI[2] - _sw) // 2, CROI[1] + (CROI[3] - _sh) // 2, _sw, _sh)
TROI = tuple(v * K for v in TURN_ROI)
PROI = tuple(v * K for v in PARK_ROI)


def _diff(a, b):
    """a - b（毫秒）。板上运行时换成 time.ticks_diff，处理计数回绕。"""
    return a - b


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
        self.rearm()

    def rearm(self):
        """重新进入 COLOR 任务时调用：保留已记顺序，只清掉连续帧状态。"""
        order = getattr(self, "order", None)
        self.last = order[-1] if order else EMPTY   # 刚记入、还没离开白框的颜色
        self.seen = EMPTY       # 当前连续看到的结果
        self.n = 0
        # 是否允许记下一块。已有顺序时重新进入任务，白框里可能还是刚记过的那一块，
        # 必须先看到 CLEAR 帧空框才能记下一块，避免同一块记两次
        self.armed = not order

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

    def seq(self):
        return "-".join(LABELS[i] for i in self.order)


# ======================= 转向识别 =======================
def find_sign(img, to_lab):
    """找最大的深蓝色圆牌，返回 blob 或 None。四角是蓝色的（方牌、蓝色物块）会被排除。"""
    best = None
    for b in img.find_blobs([TH_SIGN], roi=TROI, pixels_threshold=TURN_MIN_PIXELS * K * K,
                            area_threshold=TURN_MIN_PIXELS * K * K, merge=False):
        x, y, w, h = b.x(), b.y(), b.w(), b.h()
        if not TURN_ASPECT[0] <= w / h <= TURN_ASPECT[1]:
            continue
        dx, dy = max(1, w // 10), max(1, h // 10)
        corners = ((x + dx, y + dy), (x + w - 1 - dx, y + dy),
                   (x + dx, y + h - 1 - dy), (x + w - 1 - dx, y + h - 1 - dy))
        if sum(in_th(to_lab(img.get_pixel(px, py)), TH_SIGN) for px, py in corners) > 1:
            continue
        if best is None or b.pixels() > best.pixels():
            best = b
    return best


def arrow_x(img, sign, top):
    """牌内上半或下半部分白色像素的平均 x；只算靠近圆心的部分，避开牌外白边。

    先把牌框四边各内缩 ARROW_INSET 再分上下两半，白块中心按牌的宽、高归一后离圆心
    小于 ARROW_RADIUS 才算。斜看时圆牌是竖椭圆，按宽、高分别归一才不会把白边圈进来；
    原来按整框切半、按宽度算半径，牌框差 1 像素，白边弧段就会时进时出，dx 跳到门槛附近。
    """
    x, y, w, h = sign.x(), sign.y(), sign.w(), sign.h()
    cx, cy = x + w / 2, y + h / 2
    mx, my = int(w * ARROW_INSET), int(h * ARROW_INSET)
    ix, iy, iw, ih = x + mx, y + my, max(1, w - 2 * mx), max(2, h - 2 * my)
    half = (ix, iy, iw, ih // 2) if top else (ix, iy + ih // 2, iw, ih - ih // 2)
    total = moment = 0
    for b in img.find_blobs([TH_ARROW], roi=half, pixels_threshold=2, area_threshold=2, merge=False):
        if ((b.cx() - cx) / w) ** 2 + ((b.cy() - cy) / h) ** 2 < ARROW_RADIUS ** 2:
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


# ======================= P 牌识别 =======================
def find_park(img, to_lab):
    """找最大的蓝底白字方牌。四角为蓝色才算方牌（圆形转向牌四角不是蓝色）。"""
    best = None
    for b in img.find_blobs([TH_PARK], roi=PROI, pixels_threshold=PARK_MIN_PIXELS * K * K,
                            area_threshold=PARK_MIN_PIXELS * K * K, merge=False):
        x, y, w, h = b.x(), b.y(), b.w(), b.h()
        if w < PARK_MIN_SIDE * K or h < PARK_MIN_SIDE * K:
            continue
        if not PARK_ASPECT[0] <= w / h <= PARK_ASPECT[1]:
            continue
        dx, dy = max(1, w // 8), max(1, h // 8)
        corners = ((x + dx, y + dy), (x + w - 1 - dx, y + dy),
                   (x + dx, y + h - 1 - dy), (x + w - 1 - dx, y + h - 1 - dy))
        if sum(in_th(to_lab(img.get_pixel(px, py)), TH_PARK) for px, py in corners) < PARK_MIN_CORNERS:
            continue
        ix, iy = x + w // 6, y + h // 6
        iw, ih = max(2, w - 2 * (w // 6)), max(2, h - 2 * (h // 6))
        white = 0
        wx0 = wy0 = 1 << 30
        wx1 = wy1 = -1
        for wb in img.find_blobs([TH_ARROW], roi=(ix, iy, iw, ih), pixels_threshold=3,
                                 area_threshold=3, merge=False):
            white += wb.pixels()
            wx0, wy0 = min(wx0, wb.x()), min(wy0, wb.y())
            wx1, wy1 = max(wx1, wb.x() + wb.w()), max(wy1, wb.y() + wb.h())
        if not PARK_WHITE_RATIO[0] <= white / (iw * ih) <= PARK_WHITE_RATIO[1]:
            continue
        # P 字是竖长的笔画；横白条、方块字之类不算
        if wy1 - wy0 < PARK_WHITE_MIN_H * ih or (wy1 - wy0) < PARK_WHITE_TALL * (wx1 - wx0):
            continue
        if best is None or w * h > best.w() * best.h():
            best = b
    return best


def detect_park(img, to_lab):
    """返回 (found, cx, area, blob)。cx 归一到 -100~100（右为正），area 按 160×120 折算。"""
    b = find_park(img, to_lab)
    if b is None:
        return False, 0, 0, None
    half = img.width() / 2
    cx = int(round((b.cx() - half) * 100 / half))
    cx = max(-100, min(100, cx))
    return True, cx, b.pixels() // (K * K), b


class ParkFilter:
    """连续 PARK_CONFIRM 帧看到 P 牌才确认；中断一帧就重新计数。"""

    def __init__(self):
        self.hits = 0

    def update(self, found, cx, area):
        self.hits = self.hits + 1 if found else 0
        if self.hits >= PARK_CONFIRM:
            return (1, cx, area)
        return (0, 0, 0)


# ======================= 串口协议 =======================
def frame(body):
    return "$" + body + "#"


def parse_seq(text):
    """"R-G-B" → [0, 1, 2]；1~3 个颜色（4 个时主控已锁存，不会再下发），不合法返回 None。"""
    parts = text.split("-")
    if not 1 <= len(parts) <= 3:
        return None
    out = []
    for p in parts:
        if len(p) != 1 or p not in LABELS:
            return None
        out.append(LABELS.index(p))
    return out


class Link:
    """按 $...# 拆帧：$ 重新开始收帧，# 结束，回车换行忽略，超长整帧丢弃。"""

    def __init__(self):
        self.buf = None

    def feed(self, data):
        bodies = []
        for ch in data:
            c = chr(ch) if isinstance(ch, int) else ch
            if c == "$":
                self.buf = []
            elif self.buf is None:
                continue
            elif c == "#":
                bodies.append("".join(self.buf))
                self.buf = None
            elif c in "\r\n":
                continue
            else:
                self.buf.append(c)
                if len(self.buf) > MAX_BODY:
                    self.buf = None
        return bodies


class App:
    """协议与任务状态。不依赖 OpenMV 模块，可在 PC 上测试。"""

    def __init__(self, task="IDLE"):
        self.task = task
        self.counter = Counter()
        self.turn = TurnFilter()
        self.park = (0, 0, 0)
        self.park_filter = ParkFilter()
        self.last_hb = None
        self.last_turn_tx = None
        self.last_color_tx = None

    def reset(self):
        self.__init__("IDLE")

    # ---------- 收到主控命令 ----------
    def command(self, body, fps=0, now=0):
        body = body.strip().upper()
        if not body.startswith("CMD:"):
            return []
        arg = body[4:]
        if arg == "RESET":
            self.reset()
            return [frame("ACK:IDLE")]
        if arg == "QUERY":
            out = self.result_frames(now, force=True)
            out.append(self.hb(fps, now))
            return out
        if arg.startswith("COLOR:"):
            return self.color_resume(arg[6:], now)
        if arg in TASKS:
            if arg != self.task:        # 重复的同一命令只回 ACK，没有副作用
                if arg == "TURN":
                    self.turn = TurnFilter()
                elif arg == "COLOR":
                    self.counter.rearm()
                elif arg == "PARK":
                    self.park = (0, 0, 0)
                    self.park_filter = ParkFilter()
                self.task = arg
                self.last_turn_tx = self.last_color_tx = None
            return [frame("ACK:" + arg)]
        return []

    def color_resume(self, text, now):
        """$CMD:COLOR:<已知序列>#：主控把已收到的颜色带下来（相机重启后续接，文档 4.2）。

        - 相机自己的顺序是主控序列的开头（含相机刚重启、顺序为空）：采用主控序列，
          并要求先看到空框再计下一块；
        - 主控序列是相机顺序的开头（相机领先，主控漏收了帧）：保留相机的，立即补发一次；
        - 两者矛盾：以主控为准（主控已显示在 OLED 上）。
        序列不合法时整条命令忽略。
        """
        seed = parse_seq(text)
        if seed is None:
            return []
        mine = self.counter.order
        out = []
        if mine[:len(seed)] == seed and len(mine) > len(seed):
            pass                                    # 相机领先
        elif seed != mine:
            self.counter.order = list(seed)
            self.counter.rearm()
        if self.task != "COLOR":
            self.task = "COLOR"
            self.counter.rearm()
            self.last_turn_tx = self.last_color_tx = None
        out.append(frame("ACK:COLOR"))
        if len(mine) > len(seed) and mine[:len(seed)] == seed:
            out.append(self.color_frame())
            self.last_color_tx = now
        return out

    # ---------- 结果帧 ----------
    def turn_frame(self):
        return frame("TURN:" + self.turn.result)

    def color_frame(self):
        return frame("COLOR:%d,%s" % (len(self.counter.order), self.counter.seq()))

    def park_frame(self):
        return frame("PARK:%d,%d,%d" % self.park)

    def result_frames(self, now, force=False):
        out = []
        if self.task == "TURN" and self.turn.result:
            out.append(self.turn_frame())
            self.last_turn_tx = now
        elif self.task == "COLOR" and self.counter.order:
            out.append(self.color_frame())
            self.last_color_tx = now
        elif self.task == "PARK" and force:
            out.append(self.park_frame())
        return out

    def hb(self, fps, now):
        self.last_hb = now
        return frame("HB:%s,%d" % (self.task, int(fps)))

    # ---------- 每帧识别结果 ----------
    def on_turn(self, d, now):
        changed = self.turn.update(d)
        if self.turn.result and (changed or self.last_turn_tx is None or
                                 _diff(now, self.last_turn_tx) >= RESEND_MS):
            self.last_turn_tx = now
            return [self.turn_frame()]
        return []

    def on_color(self, c, now):
        if self.counter.update(c):
            self.last_color_tx = now
            return [self.color_frame()]
        if self.counter.full() and (self.last_color_tx is None or
                                    _diff(now, self.last_color_tx) >= RESEND_MS):
            self.last_color_tx = now
            return [self.color_frame()]
        return []

    def on_park(self, found, cx, area):
        self.park = self.park_filter.update(found, cx, area)
        return [self.park_frame()]

    def periodic(self, now, fps):
        if self.last_hb is None or _diff(now, self.last_hb) >= HB_PERIOD_MS:
            return [self.hb(fps, now)]
        return []


# ======================= 调试显示 =======================
def draw_order(img, order):
    """左上角 128×30 区域显示顺序。"""
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


def draw_turn_debug(img, d, dx, sign, result):
    if sign is not None:
        img.draw_rectangle(sign.rect(), color=(255, 0, 255))
        img.draw_line(sign.x(), sign.cy(), sign.x() + sign.w(), sign.cy(), color=(255, 0, 255))
    img.draw_string(img.width() - 112, 34, "SEE=%s %+.2f" % (d or "-", dx), color=WHITE)
    img.draw_string(img.width() - 112, 46, "TURN=%s" % (result or "-"), color=WHITE)


def draw_park_debug(img, found, cx, area, blob, park):
    if blob is not None:
        img.draw_rectangle(blob.rect(), color=(0, 255, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 255))
    img.draw_string(0, img.height() - 24, "SEE=%d CX=%+d AREA=%d" % (1 if found else 0, cx, area), color=WHITE)
    img.draw_string(0, img.height() - 12, "PARK=%d,%d,%d" % park, color=WHITE)


# ======================= 主程序 =======================
def setup_camera(sensor):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, FRAME_SIZE))
    sensor.set_hmirror(CAMERA_ROTATE_180)
    sensor.set_vflip(CAMERA_ROTATE_180)
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
    global _diff
    import sensor
    import image
    import time
    from pyb import UART

    _diff = time.ticks_diff
    cam = setup_camera(sensor)
    print("MODE=%s %s" % (MODE, cam))

    lcd = None
    if DEBUG and LCD_IN_DEBUG:
        try:
            import display
            lcd = display.SPIDisplay()
        except Exception as e:          # 没接屏也要能启动
            print("LCD OFF:", e)

    uart = UART(3, 115200, timeout_char=0)
    clock = time.clock()
    app = App(DEBUG_TASK if DEBUG else "IDLE")
    link = Link()
    full_at = None

    def send(frames):
        for f in frames:
            if DEBUG or not (f.startswith("$PARK:") or f.startswith("$HB:")):
                print(f)
            uart.write(f + "\r\n")

    send([app.hb(0, time.ticks_ms())])      # 开机就绪，主控据此判定在线

    while True:
        clock.tick()
        img = sensor.snapshot()
        now = time.ticks_ms()
        fps = clock.fps()

        if uart.any():
            for body in link.feed(uart.read() or b""):
                send(app.command(body, fps, now))

        task = app.task
        if task == "TURN":
            d, dx, sign = detect_turn(img, image.rgb_to_lab)
            send(app.on_turn(d, now))
            if DEBUG:
                draw_turn_debug(img, d, dx, sign, app.turn.result)

        elif task == "COLOR":
            if app.counter.full() and not DEBUG:
                c, how = EMPTY, "FULL"          # 满 4 色后不再识别，只按周期重发结果
            else:
                c, how = detect_color(img)
            send(app.on_color(c, now))
            if DEBUG:
                draw_color_debug(img, c, how)
                draw_order(img, app.counter.order)
                if lcd:
                    lcd.write(img, roi=(0, 0, 128, 30), y=60)
                if app.counter.full():
                    if full_at is None:
                        full_at = now
                    elif AUTO_RESET_MS and time.ticks_diff(now, full_at) > AUTO_RESET_MS:
                        app.counter = Counter()
                        full_at = None
                        print("AUTO RESET")

        elif task == "PARK":
            found, cx, area, blob = detect_park(img, image.rgb_to_lab)
            send(app.on_park(found, cx, area))
            if DEBUG:
                draw_park_debug(img, found, cx, area, blob, app.park)

        send(app.periodic(now, fps))

        if DEBUG:
            img.draw_string(0, 0 if task == "PARK" else img.height() - 24,
                            "TASK=%s FPS=%.0f %s" % (task, fps, cam), color=WHITE)


if __name__ == "__main__":
    main()
