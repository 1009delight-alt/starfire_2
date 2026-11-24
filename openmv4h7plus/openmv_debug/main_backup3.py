"""星火杯四色顺序识别。摄像头朝向赛道侧面，物块依次穿过画面白框。"""

# ==================== 可调参数 ====================
# 这里是开机默认值。运行中可用串口命令临时修改（见下方"串口命令"），
# 调好后发 P 打印出全部参数，复制回这里保存，否则断电即丢失。

# 1. 摄像头和预览
FRAME_SIZE = "QVGA"      # VGA=640×480，QVGA=320×240，QQVGA=160×120；QVGA 帧率更高
WARMUP_MS = 2000         # 开机后在现场照明下等待曝光和白平衡稳定，再锁定
LOCK_EXPOSURE = True     # 同时锁定曝光，避免深浅色块经过时颜色漂移；现场太暗可改 False
# 画面过亮、颜色发白时手动指定曝光（微秒）和增益（dB）。None = 用预热后自动得到的值。
# 先看调试画面上的 EXP= 和 GAIN= 读数，再把 EXPOSURE_US 设成它的一半左右试，逐步调到颜色最鲜艳。
EXPOSURE_US = None       # 例如 8000；过小拍屏幕会出现横向滚动条纹
GAIN_DB = None           # 例如 0；越小噪点越少、画面越暗
SHOW_DEBUG = True        # IDE 显示单帧识别结果及中心小框的 LAB 中位数；跑车时可关闭
USE_LCD = False          # 接好兼容 OpenMV SPI LCD Shield 的车载屏后改为 True（比赛必须用车载屏显示！）

# 1.5 串口和按键
# 串口是 OpenMV UART3：P4=TX，P5=RX。调试时接 USB 转 TTL 模块，比赛时接 STM32，
# 都要 TX/RX 交叉并共地。IDE 照常通过 USB 看画面，两者互不影响。
# 串口命令（每条以换行结尾，串口助手里勾选"发送新行"）：
#   R              清空结果（触发模式下回到等待）
#   S              开始计数
#   Q              查询当前结果
#   P              打印全部可调参数，可直接复制回本文件
#   SET 名字 数值   临时修改参数，例如：
#                  SET CONFIRM_FRAMES 2
#                  SET BASE_ROI 70 35 18 64
#                  SET R 12 95 25 127 8 127   （R/G/B/Y 四行颜色阈值）
#                  SET HUE_Y 36 72            （HUE_R/G/B/Y 色相范围）
#                  SET SHOW_DEBUG 0           （开关量用 0/1）
# 记满四块后会主动发出 "ORDER=R-G-B-Y"。
USE_UART_TRIGGER = False # False：上电就计数（调试用）；True：收到 S 才开始（比赛接 STM32 用）
UART_BAUD = 115200
AUTO_RESET_MS = 3000     # 单独调试用：记满四块后过多少毫秒自动清空重来；0 = 关闭。
                         # ！！比赛前必须改成 0，否则裁判还没记录结果就被清掉！！
RESET_PIN = None         # 接一个按键到某引脚和 GND（如 "P9"），按下即清空结果；None 表示不用

# 2. 四种颜色。每行依次为 (L下限, L上限, A下限, A上限, B下限, B上限)。
# L 是亮度（0~100），A 由绿色到红色，B 由蓝色到黄色（两者均为 -128~127）。
# 保持四行的红、绿、蓝、黄顺序。将实物对准中心小框，结合 IDE 阈值编辑器调对应行。
# 绿和黄都可能有负 A、正 B。若黄色显示为 G，重点检查这两行的 A 边界。
THRESHOLDS = [
    (12, 95,   25, 127,    8, 127),  # 红 R
    (12, 95, -128, -33,   -5, 127),  # 绿 G
    ( 8, 85,  -35, 127, -128, -18),  # 蓝 B
    (35,100,  -32,  24,   28, 127),  # 黄 Y
]

# 3. 白框和最小色块。以下尺寸统一按 160×120 基准图填写，切换分辨率会自动换算。
# (左边距, 上边距, 宽, 高)：物块从框外经过就调位置；框太宽可能同时看到两块。
BASE_ROI = (72, 35, 16, 64)
BASE_MIN_PIXELS = 80     # 同色像素数下限，远处小色块容易漏检时可调小
BASE_MIN_AREA = 90       # 色斑外接矩形面积下限
BASE_MIN_WIDTH = 6       # 只计算落入白框内的宽和高
BASE_MIN_HEIGHT = 10
MIN_DENSITY = 0.45       # 同色像素占外接矩形的最小比例，提高可滤掉零碎噪点
CENTER_FALLBACK = True   # 大框没有合格色斑时，在中心小框的 9 个点按色相判色
CENTER_MIN_VOTES = 8     # 至少多少个点同色才接受，范围 5~9；越大越不容易被背景误触发
MIN_VALUE = 35           # 最亮 RGB 通道低于此值时忽略该点 (0~255)
MIN_SAT_PERCENT = 28     # 忽略近灰色的点，降低此值会接受更浅的色块
# 色相角范围，依次为红、绿、蓝、黄；红色跨过 360°/0°。
HUE_RANGES = [(335, 22), (78, 170), (190, 265), (36, 72)]

# 4. 计数时机
CONFIRM_FRAMES = 3       # 连续多少帧认出同色才记一块；车快时可试 2
CLEAR_FRAMES = 3         # 两块之间至少连续多少帧空框；过小容易把一块重复计数
REQUIRE_GAP = True       # True：任意两块之间都必须先出现空框，防止边缘误判多记一块；
                         # 若现场物块紧挨着、白框里看不到空隙，改 False（仅同色块之间要求空框）

# ==================== 识别逻辑 ====================

LABELS = ("R", "G", "B", "Y")
SWATCHES = ((255, 0, 0), (0, 220, 0), (0, 80, 255), (255, 220, 0))
EMPTY = -1
AMBIGUOUS = -2
SCALE = {"QQVGA": 1, "QVGA": 2, "VGA": 4}[FRAME_SIZE]


def derive():
    """由基准参数换算出当前分辨率下的实际值；修改参数后需重新调用。"""
    global ROI, MIN_PIXELS, MIN_AREA, MIN_WIDTH, MIN_HEIGHT, SAMPLE_ROI
    ROI = tuple(v * SCALE for v in BASE_ROI)
    MIN_PIXELS = BASE_MIN_PIXELS * SCALE * SCALE
    MIN_AREA = BASE_MIN_AREA * SCALE * SCALE
    MIN_WIDTH = BASE_MIN_WIDTH * SCALE
    MIN_HEIGHT = BASE_MIN_HEIGHT * SCALE
    # 中心四分之一宽高作为取色框，尽量避免混入背景。
    sample_w = max(1, ROI[2] // 4)
    sample_h = max(1, ROI[3] // 4)
    SAMPLE_ROI = (ROI[0] + (ROI[2] - sample_w) // 2, ROI[1] + (ROI[3] - sample_h) // 2,
                  sample_w, sample_h)


derive()
CAMERA_INFO = ""

# 允许 SET 修改的单值参数及类型
INT_PARAMS = ("BASE_MIN_PIXELS", "BASE_MIN_AREA", "BASE_MIN_WIDTH", "BASE_MIN_HEIGHT",
              "CENTER_MIN_VOTES", "MIN_VALUE", "MIN_SAT_PERCENT", "CONFIRM_FRAMES", "CLEAR_FRAMES")
FLOAT_PARAMS = ("MIN_DENSITY",)
BOOL_PARAMS = ("CENTER_FALLBACK", "REQUIRE_GAP", "SHOW_DEBUG")


def params_text():
    g = globals()
    lines = []
    for name in BOOL_PARAMS + INT_PARAMS + FLOAT_PARAMS:
        lines.append("%s = %r" % (name, g[name]))
    lines.append("BASE_ROI = %r" % (tuple(BASE_ROI),))
    lines.append("THRESHOLDS = [")
    for i in range(4):
        lines.append("    %r,  # %s" % (tuple(THRESHOLDS[i]), LABELS[i]))
    lines.append("]")
    lines.append("HUE_RANGES = %r" % ([tuple(h) for h in HUE_RANGES],))
    return "\n".join(lines)


def set_param(args):
    """处理 SET 命令，返回回复文字。"""
    if not args:
        return "ERR usage: SET NAME VALUE"
    name = args[0].upper()
    vals = args[1:]
    g = globals()
    try:
        if name in INT_PARAMS and len(vals) == 1:
            g[name] = int(vals[0])
        elif name in FLOAT_PARAMS and len(vals) == 1:
            g[name] = float(vals[0])
        elif name in BOOL_PARAMS and len(vals) == 1:
            g[name] = int(vals[0]) != 0
        elif name == "BASE_ROI" and len(vals) == 4:
            g["BASE_ROI"] = tuple(int(v) for v in vals)
        elif name in LABELS and len(vals) == 6:
            THRESHOLDS[LABELS.index(name)] = tuple(int(v) for v in vals)
        elif name.startswith("HUE_") and name[4:] in LABELS and len(vals) == 2:
            HUE_RANGES[LABELS.index(name[4:])] = (int(vals[0]), int(vals[1]))
        else:
            return "ERR unknown or wrong count: " + name
    except ValueError:
        return "ERR bad number"
    derive()
    return "OK " + name + " " + " ".join(vals)


def detect(img):
    """返回当前帧的颜色编号。空框和多色分别返回 EMPTY 和 AMBIGUOUS。"""
    codes = 0
    for blob in img.find_blobs(THRESHOLDS, roi=ROI, x_stride=1, y_stride=1,
                               pixels_threshold=MIN_PIXELS, area_threshold=MIN_AREA, merge=False):
        if blob.h() >= MIN_HEIGHT and blob.w() >= MIN_WIDTH and blob.density() >= MIN_DENSITY:
            # OpenMV 用 1、2、4、8 表示命中第 1~4 行阈值；同一颜色的多个色斑只算一种。
            codes |= blob.code()
    if codes == 0:
        return EMPTY
    if codes in (1, 2, 4, 8):
        return (1, 2, 4, 8).index(codes)
    return AMBIGUOUS


def center_color(img):
    """按中心小框的实际像素投票，降低固定 LAB 范围和最小面积造成的漏检。"""
    x, y, w, h = SAMPLE_ROI
    votes = [0, 0, 0, 0]
    for row in (1, 3, 5):
        for col in (1, 3, 5):
            r, g, b = img.get_pixel(x + col * w // 6, y + row * h // 6)
            high = max(r, g, b)
            delta = high - min(r, g, b)
            if not delta or high < MIN_VALUE or delta * 100 < MIN_SAT_PERCENT * high:
                continue
            if high == r:
                hue = (60 * (g - b) // delta) % 360
            elif high == g:
                hue = 120 + 60 * (b - r) // delta
            else:
                hue = 240 + 60 * (r - g) // delta
            for i, (low, upper) in enumerate(HUE_RANGES):
                if low <= hue <= upper or (low > upper and (hue >= low or hue <= upper)):
                    votes[i] += 1
                    break
    best = max(votes)
    if best >= CENTER_MIN_VOTES:
        return votes.index(best), "CENTER"
    return EMPTY, "LOW COVER" if best else "NO COLOR"


class Sequence:
    def __init__(self):
        self.order = []          # 已确认的行驶顺序，最多四个位置
        self.active = EMPTY      # 最近已记入且尚未离开白框的颜色
        self.candidate = EMPTY   # 当前连续观察到的结果
        self.frames = 0
        self.armed = True        # REQUIRE_GAP 模式下，记一块后要等空框才重新允许计数

    def update(self, color):
        if len(self.order) == 4:
            return False
        if color != self.candidate:
            self.candidate = color
            self.frames = 0
        self.frames = min(self.frames + 1, max(CONFIRM_FRAMES, CLEAR_FRAMES))
        # 稳定空框后才能再次计入同色块；不同颜色可直接交接。
        if color == EMPTY and self.frames >= CLEAR_FRAMES:
            self.active = EMPTY
            self.armed = True
        allowed = self.armed if REQUIRE_GAP else color != self.active
        if color >= 0 and allowed and self.frames >= CONFIRM_FRAMES:
            self.order.append(color)
            self.active = color
            self.armed = False
            return True
        return False

    def text(self):
        return "ORDER=" + "-".join(LABELS[i] for i in self.order)


class Controller:
    """保存计数状态并处理串口/按键命令，与硬件无关，方便电脑上测试。"""

    def __init__(self):
        self.buf = b""
        self.reset()

    def reset(self):
        self.sequence = Sequence()
        self.running = not USE_UART_TRIGGER
        self.sent = False
        self.refresh = True

    def feed(self, data):
        """喂入串口收到的字节，返回需要回复的文字列表。"""
        replies = []
        self.buf += data
        while True:
            cut = -1
            for i, ch in enumerate(self.buf):
                if ch in (10, 13):
                    cut = i
                    break
            if cut < 0:
                break
            line = self.buf[:cut].decode().strip()
            self.buf = self.buf[cut + 1:]
            if line:
                replies.append(self.command(line))
        if len(self.buf) > 80:   # 丢弃没有换行的垃圾数据
            self.buf = b""
        return replies

    def command(self, line):
        parts = line.split()
        cmd = parts[0].upper()
        if cmd == "R":
            self.reset()
            return "OK RESET" + (" (WAIT S)" if not self.running else "")
        if cmd == "S":
            self.running = True
            return "OK START"
        if cmd == "Q":
            return self.sequence.text()
        if cmd == "P":
            return params_text()
        if cmd == "SET":
            return set_param(parts[1:])
        return "ERR unknown command: " + cmd


def draw_order(img, order):
    """在固定的 128×30 区域绘制顺序，车载屏只需刷新这一区域。"""
    img.draw_rectangle((0, 0, 128, 30), color=(0, 0, 0), fill=True)
    for i in range(4):
        x = 32 * i
        label = "-"
        if i < len(order):
            color = order[i]
            img.draw_rectangle((x, 21, 28, 7), color=SWATCHES[color], fill=True)
            label = LABELS[color]
        img.draw_string(x + 6, 2, label, color=(255, 255, 255), scale=2)


def draw_debug(img, color, reason, running, fps):
    # 在绘制任何线框前取样，避免把标记颜色算进读数。
    stats = img.get_statistics(roi=SAMPLE_ROI)
    name = LABELS[color] if color >= 0 else ("MULTI" if color == AMBIGUOUS else "NONE")
    line = "NOW=%s %s LAB=%d,%d,%d" % (name, reason, stats.l_median(), stats.a_median(), stats.b_median())
    img.draw_rectangle(SAMPLE_ROI, color=(255, 255, 255))
    img.draw_string(0, img.height() - 12, line, color=(255, 255, 255))
    state = "RUN" if running else "WAIT S"
    img.draw_string(0, img.height() - 24, "%s FPS=%.1f %s" % (state, fps, CAMERA_INFO),
                    color=(255, 255, 255))


def main():
    import sensor
    import time
    from pyb import UART, Pin

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, FRAME_SIZE))
    sensor.skip_frames(time=WARMUP_MS)
    if GAIN_DB is None:
        sensor.set_auto_gain(False)
    else:
        sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.set_auto_whitebal(False)
    if EXPOSURE_US:
        sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    elif LOCK_EXPOSURE:
        sensor.set_auto_exposure(False)
    sensor.skip_frames(time=300)   # 等新曝光生效
    global CAMERA_INFO
    CAMERA_INFO = "EXP=%d GAIN=%.1f" % (sensor.get_exposure_us(), sensor.get_gain_db())
    print(CAMERA_INFO)
    lcd = None
    if USE_LCD:
        import display
        lcd = display.SPIDisplay()
    else:
        print("WARNING: USE_LCD=False, 比赛时必须在车载屏显示结果")
    uart = UART(3, UART_BAUD, timeout_char=0)
    button = Pin(RESET_PIN, Pin.IN, Pin.PULL_UP) if RESET_PIN else None
    last_button = 1
    last_press = 0
    clock = time.clock()
    ctl = Controller()

    def reply(text):
        print(text)
        uart.write(text.replace("\n", "\r\n") + "\r\n")

    if AUTO_RESET_MS:
        print("WARNING: AUTO_RESET_MS=%d, 记满后会自动清空, 比赛前改成 0" % AUTO_RESET_MS)
    full_since = None

    while True:
        clock.tick()
        img = sensor.snapshot()
        changed = False

        if uart.any():
            for text in ctl.feed(uart.read() or b""):
                reply(text)
        if button is not None:
            now = button.value()
            if last_button and not now and time.ticks_diff(time.ticks_ms(), last_press) > 300:
                last_press = time.ticks_ms()
                ctl.reset()
                reply("OK RESET (BUTTON)")
            last_button = now

        seq = ctl.sequence
        # 调试模式下未开始或记满四块仍显示 NOW，方便拿同一块反复调阈值。
        need_detect = ctl.running and len(seq.order) < 4
        if SHOW_DEBUG or need_detect:
            color = detect(img)
            reason = "BLOB" if color != EMPTY else "NO BLOB"
            if color == EMPTY and CENTER_FALLBACK:
                color, reason = center_color(img)
            if need_detect:
                changed = seq.update(color)
            if SHOW_DEBUG:
                draw_debug(img, color, reason, ctl.running, clock.fps())
        img.draw_rectangle(ROI, color=(255, 255, 255))
        draw_order(img, seq.order)
        if ctl.refresh or changed:
            print(seq.text())
            if lcd is not None:
                lcd.write(img, roi=(0, 0, 128, 30), y=60)
            ctl.refresh = False
        if len(seq.order) == 4 and not ctl.sent:
            reply(seq.text())
            ctl.sent = True
            full_since = time.ticks_ms()
        if AUTO_RESET_MS and full_since is not None and \
                time.ticks_diff(time.ticks_ms(), full_since) > AUTO_RESET_MS:
            ctl.reset()
            full_since = None
            reply("AUTO RESET")
        if len(ctl.sequence.order) < 4:
            full_since = None


if __name__ == "__main__":
    main()
