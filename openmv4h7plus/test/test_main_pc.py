"""电脑上测试 openmv4h7plus/MAIN/main.py 的协议层与计数逻辑（不需要 OpenMV 硬件）。

用法（在项目根目录）：
    python -I openmv4h7plus/test/test_main_pc.py
    python -I openmv4h7plus/test/test_main_pc.py <输出目录>   # 另外生成给主控解析器联测用的字节流
"""
import importlib.util
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(here, "..", "MAIN", "main.py")
outdir = sys.argv[1] if len(sys.argv) > 1 else None
spec = importlib.util.spec_from_file_location("mvmain", path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

fails = 0


def check(cond, msg):
    global fails
    if not cond:
        fails += 1
        print("FAIL:", msg)
    else:
        print("ok  :", msg)


# ---- Link ----
L = m.Link()
check(L.feed(b"$CMD:TU") == [] and L.feed(b"RN#\r\n") == ["CMD:TURN"], "半帧拼接")
check(L.feed(b"$CMD:IDLE#$CMD:QUERY#") == ["CMD:IDLE", "CMD:QUERY"], "两帧连发")
check(L.feed(b"junk#$CMD:$CMD:PARK#") == ["CMD:PARK"], "垃圾字节与 $ 重新同步")
check(L.feed(b"$" + b"A" * 64 + b"#") == [], "超长帧丢弃")
check(L.feed(b"$" + b"A" * 63 + b"#") == ["A" * 63], "63 字节帧接受")

# ---- App: 任务切换与幂等 ----
a = m.App()
check(a.command("CMD:TURN", 20, 0) == ["$ACK:TURN#"] and a.task == "TURN", "切到 TURN 回 ACK")
for i in range(m.TURN_CONFIRM - 1):
    check(a.on_turn("L", 10 + i) == [], "第 %d 帧未确认不发" % (i + 1)) if i == 0 else a.on_turn("L", 10 + i)
out = a.on_turn("L", 100)
check(out == ["$TURN:L#"], "连续 %d 帧确认后发 $TURN:L#" % m.TURN_CONFIRM)
check(a.on_turn(None, 150) == [], "200ms 内不重发")
check(a.on_turn(None, 300) == ["$TURN:L#"], "200ms 后重发")
check(a.command("CMD:TURN", 20, 310) == ["$ACK:TURN#"] and a.turn.result == "L", "重复 TURN 命令不清结果")
q = a.command("CMD:QUERY", 25, 320)
check(q == ["$TURN:L#", "$HB:TURN,25#"], "QUERY 补发结果和心跳")

# ---- COLOR ----
check(a.command("CMD:COLOR", 20, 400) == ["$ACK:COLOR#"], "切到 COLOR")
R, G, B, Y, E = 0, 1, 2, 3, m.EMPTY
seq = []
for col in (R, G, G, Y):
    seq += [col] * m.CONFIRM + [E] * m.CLEAR
sent = []
t = 500
for c in seq:
    sent += a.on_color(c, t)
    t += 30
check(sent == ["$COLOR:1,R#", "$COLOR:2,R-G#", "$COLOR:3,R-G-G#", "$COLOR:4,R-G-G-Y#"], "四色含相邻同色逐块上报: %s" % sent)
check(a.on_color(E, t + 10) == [], "满 4 色后 200ms 内不重发")
check(a.on_color(E, t + 250) == ["$COLOR:4,R-G-G-Y#"], "满 4 色后定期重发")
check(a.command("CMD:IDLE", 0, t + 300) == ["$ACK:IDLE#"] and a.counter.seq() == "R-G-G-Y", "切 IDLE 保留顺序")
check(a.command("CMD:COLOR", 0, t + 310) == ["$ACK:COLOR#"] and len(a.counter.order) == 4, "重回 COLOR 不清顺序")

# ---- COLOR：切走再切回，白框里仍是同一块，不能重复计数 ----
d = m.App()
d.command("CMD:COLOR", 0, 0)
got = []
for k in range(m.CONFIRM):
    got += d.on_color(R, 10 * k)
check(got == ["$COLOR:1,R#"], "先记下第 1 块 R")
d.command("CMD:IDLE", 0, 100)
d.command("CMD:COLOR", 0, 110)
again = []
for k in range(m.CONFIRM * 3):
    again += d.on_color(R, 200 + 10 * k)
check(again == [] and d.counter.seq() == "R", "切回后同一块 R 不重复计数（得到 %s）" % (d.counter.seq() or "-"))
for k in range(m.CLEAR):
    d.on_color(E, 400 + 10 * k)
nxt = []
for k in range(m.CONFIRM):
    nxt += d.on_color(R, 500 + 10 * k)
check(nxt == ["$COLOR:2,R-R#"], "看到空框后，相邻同色的下一块 R 正常计入")

# ---- COLOR 续接：主控带下已收到的颜色 ----
r = m.App()                                     # 相当于相机刚重启，顺序为空
out = r.command("CMD:COLOR:R-G", 0, 0)
check(out == ["$ACK:COLOR#"] and r.task == "COLOR" and r.counter.seq() == "R-G", "重启后按主控序列 R-G 续接")
blk = []
for k in range(m.CONFIRM):
    blk += r.on_color(G, 10 * k)               # 白框里还是第 2 块 G
check(blk == [], "续接后白框里仍是上一块时不计数")
for k in range(m.CLEAR):
    r.on_color(E, 100 + 10 * k)
for col in (B, Y):
    for k in range(m.CONFIRM):
        blk += r.on_color(col, 200 + 10 * k)
    for k in range(m.CLEAR):
        r.on_color(E, 300 + 10 * k)
check(blk == ["$COLOR:3,R-G-B#", "$COLOR:4,R-G-B-Y#"], "续接后接着报第 3、4 块: %s" % blk)
ahead = m.App()
ahead.command("CMD:COLOR", 0, 0)
for col in (R, G, B):
    for k in range(m.CONFIRM):
        ahead.on_color(col, 10 * k)
    for k in range(m.CLEAR):
        ahead.on_color(E, 10 * k)
check(ahead.command("CMD:COLOR:R-G", 0, 500) == ["$ACK:COLOR#", "$COLOR:3,R-G-B#"] and ahead.counter.seq() == "R-G-B",
      "相机领先时保留自己的顺序并立即补发")
conflict = m.App()
conflict.command("CMD:COLOR", 0, 0)
for k in range(m.CONFIRM):
    conflict.on_color(B, 10 * k)
conflict.command("CMD:COLOR:R-G", 0, 100)
check(conflict.counter.seq() == "R-G", "与主控矛盾时以主控为准")
bad = m.App()
check(bad.command("CMD:COLOR:R-X", 0, 0) == [] and bad.task == "IDLE", "非法续接序列整条忽略")
check(bad.command("CMD:COLOR:R-G-B-Y", 0, 0) == [] and bad.command("CMD:COLOR:", 0, 0) == [], "4 个或空序列忽略")
again2 = m.App()
again2.command("CMD:COLOR:R", 0, 0)
check(again2.command("CMD:COLOR:R", 0, 10) == ["$ACK:COLOR#"] and again2.counter.seq() == "R", "重复续接命令无副作用")

# ---- PARK ----
a.command("CMD:PARK", 0, 0)
outs = [a.on_park(True, -12, 340) for _ in range(m.PARK_CONFIRM)]
check(all(o == ["$PARK:0,0,0#"] for o in outs[:-1]), "未满 %d 帧只报 0" % m.PARK_CONFIRM)
check(outs[-1] == ["$PARK:1,-12,340#"], "连续 %d 帧后报 $PARK:1,-12,340#" % m.PARK_CONFIRM)
check(a.on_park(False, 0, 0) == ["$PARK:0,0,0#"], "丢一帧立即回到 0")
for _ in range(m.PARK_CONFIRM):
    a.on_park(True, -12, 340)
check(a.command("CMD:QUERY", 18, 0)[0] == "$PARK:1,-12,340#", "QUERY 补发 P 牌")

# ---- RESET / 心跳 ----
check(a.command("CMD:RESET", 0, 0) == ["$ACK:IDLE#"] and a.task == "IDLE" and a.counter.order == [] and a.turn.result is None, "RESET 清空回 IDLE")
b = m.App()
check(b.periodic(0, 30) == ["$HB:IDLE,30#"], "首次心跳")
check(b.periodic(400, 30) == [], "500ms 内不发心跳")
check(b.periodic(500, 30) == ["$HB:IDLE,30#"], "500ms 发心跳")
check(b.command("XYZ") == [] and b.command("CMD:FOO") == [], "未知帧忽略")
check(b.command(" cmd:turn ") == ["$ACK:TURN#"], "命令不区分大小写、忽略空白")

# ---- 黄色现场样本及暗部试调边界（不代表完整图像识别验收） ----
for label, lab, expected in (
    ("本次黄色", (61, -11, 55), 3),
    ("此前黄色阴影", (59, -19, 29), 3),
    ("黄色暗部试调边界", (30, -28, 26), 3),
    ("现场绿色", (76, -53, 32), 1),
    ("背景B上界", (74, -3, 14), None),
):
    matches = [i for i, th in enumerate(m.COLOR_TH) if m.in_th(lab, th)]
    check(matches == ([] if expected is None else [expected]), label + "阈值匹配")

# ---- 生成 C 联测字节流 ----
def scene(name, frames):
    if outdir is None:
        return
    with open("%s/%s.bin" % (outdir, name), "wb") as f:
        f.write("".join(x + "\r\n" for x in frames).encode())

c = m.App()
s_turn = c.command("CMD:TURN", 25, 0) + [c.hb(25, 0)]
for i in range(m.TURN_CONFIRM):
    s_turn += c.on_turn("R", 10 * i)
scene("turn", s_turn)
s_color = c.command("CMD:COLOR", 25, 0)
t = 1000
for col in seq:
    s_color += c.on_color(col, t)
    t += 30
s_color += c.on_color(E, t + 300)
scene("color", s_color)
s_park = c.command("CMD:PARK", 25, 0)
for _ in range(m.PARK_CONFIRM):
    s_park += c.on_park(True, 10, 200)
scene("park", s_park)
scene("stale_turn", ["$TURN:L#"])     # 未确认 TURN 任务时的结果应被主控丢弃
print("FAILS", fails)
sys.exit(1 if fails else 0)
