"""星火杯四色顺序识别。摄像头朝向赛道侧面，物块依次穿过画面白框。"""

# ==================== 可调参数（修改后重新运行） ====================

# 1. 摄像头和预览
FRAME_SIZE = "VGA"       # VGA=640×480，QVGA=320×240，QQVGA=160×120
WARMUP_MS = 2000         # 开机后在现场照明下等待曝光和白平衡稳定，再锁定增益和白平衡
SHOW_DEBUG = True        # IDE 显示单帧识别结果及中心小框的 LAB 中位数；跑车时可关闭
USE_LCD = False          # 接好兼容 OpenMV SPI LCD Shield 的车载屏后改为 True

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
CENTER_MIN_VOTES = 7     # 至少多少个点同色才接受，范围 5~9
MIN_VALUE = 35           # 最亮 RGB 通道低于此值时忽略该点 (0~255)
MIN_SAT_PERCENT = 28     # 忽略近灰色的点，降低此值会接受更浅的色块
# 色相角范围，依次为红、绿、蓝、黄；红色跨过 360°/0°。
HUE_RANGES = ((335, 22), (78, 170), (190, 265), (36, 72))

# 4. 计数时机
CONFIRM_FRAMES = 3       # 连续多少帧认出同色才记一块；车快时可试 2
CLEAR_FRAMES = 3         # 相邻同色块之间至少连续多少帧空框；过小容易把一块重复计数

# ==================== 识别逻辑 ====================

LABELS = ("R", "G", "B", "Y")
SWATCHES = ((255, 0, 0), (0, 220, 0), (0, 80, 255), (255, 220, 0))
EMPTY = -1
AMBIGUOUS = -2
SCALE = {"QQVGA": 1, "QVGA": 2, "VGA": 4}[FRAME_SIZE]
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
        if color >= 0 and color != self.active and self.frames >= CONFIRM_FRAMES:
            self.order.append(color)
            self.active = color
            return True
        return False


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


def draw_debug(img, color, reason):
    # 在绘制任何线框前取样，避免把标记颜色算进读数。
    stats = img.get_statistics(roi=SAMPLE_ROI)
    name = LABELS[color] if color >= 0 else ("MULTI" if color == AMBIGUOUS else "NONE")
    line = "NOW=%s %s LAB=%d,%d,%d" % (name, reason, stats.l_median(), stats.a_median(), stats.b_median())
    img.draw_rectangle(SAMPLE_ROI, color=(255, 255, 255))
    img.draw_string(0, img.height() - 12, line, color=(255, 255, 255))


def main():
    import sensor

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, FRAME_SIZE))
    sensor.skip_frames(time=WARMUP_MS)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    lcd = None
    if USE_LCD:
        import display
        lcd = display.SPIDisplay()
    sequence = Sequence()
    first_frame = True
    while True:
        img = sensor.snapshot()
        changed = False
        # 调试模式下记满四块仍显示 NOW，方便拿同一块反复调阈值。
        if SHOW_DEBUG or len(sequence.order) < 4:
            color = detect(img)
            reason = "BLOB" if color != EMPTY else "NO BLOB"
            if color == EMPTY and CENTER_FALLBACK:
                color, reason = center_color(img)
            changed = sequence.update(color)
            if SHOW_DEBUG:
                draw_debug(img, color, reason)
        img.draw_rectangle(ROI, color=(255, 255, 255))
        draw_order(img, sequence.order)
        if first_frame or changed:
            print("ORDER=" + "-".join(LABELS[i] for i in sequence.order))
            if lcd is not None:
                lcd.write(img, roi=(0, 0, 128, 30), y=60)
            first_frame = False


if __name__ == "__main__":
    main()
