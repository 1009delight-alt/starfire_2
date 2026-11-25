"""读出 DBG_LINE_ONLY 的车上循迹记录（main.c 的 g_line_log 环形缓冲）。

用法（在 stm32f407vet6/ 下，车停稳后不断电，插上 PWLink）：
    python -I scripts/line_log.py            # 读出并写 out/line_log.csv，打印摘要
    python -I scripts/line_log.py --all      # 另外逐帧打印

只读内存（commander -O connect_mode=attach，不停核、不复位）。符号地址取自
out/SkyStar/SkyStar/Debug/SkyStar.axf.map，必须与车上程序是同一次编译。
"""
import glob
import os
import re
import struct
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(ROOT, 'out', 'SkyStar', 'SkyStar', 'Debug', 'SkyStar.axf.map')
RUN = os.path.join(ROOT, 'out', 'SkyStar+SkyStar.cbuild-run.yml')
REC = 16
STOP = {0: '未停/未跑', 1: '按键', 2: '丢线超限', 3: '电机故障'}


def find_pyocd():
    pats = glob.glob(os.path.expanduser(
        r'~\.vscode\extensions\arm.vscode-cmsis-debugger-*\tools\pyocd\pyocd.exe'))
    if not pats:
        sys.exit('找不到 pyocd.exe（VS Code 的 Arm CMSIS Debugger 插件）')
    return sorted(pats)[-1]


def sym(section, obj):
    text = open(MAP, encoding='utf-8', errors='replace').read()
    m = re.search(r'(0x[0-9a-f]{8})\s+(0x[0-9a-f]{8})\s+\w+\s+RW\s+\d+\s+' + re.escape(section) + r'\s+' + re.escape(obj),
                  text)
    if not m:
        sys.exit('map 里找不到 %s（%s）：车上程序是不是 DBG_LINE_ONLY？' % (section, obj))
    return int(m.group(1), 16), int(m.group(2), 16)


def read_mem(pyocd, ranges):
    cmd = [pyocd, 'commander', '--probe', 'cmsisdap:', '--cbuild-run', RUN, '-f', '1000000',
           '-O', 'connect_mode=attach']
    for addr, size in ranges:
        cmd += ['-c', 'read8 0x%08x %d' % (addr, size)]
    out = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace').stdout
    mem = {}
    for line in out.splitlines():
        m = re.match(r'([0-9a-f]{8}):\s+((?:[0-9a-f]{2}\s+)+)', line)
        if m:
            base = int(m.group(1), 16)
            for i, b in enumerate(m.group(2).split()):
                mem[base + i] = int(b, 16)
    data = []
    for addr, size in ranges:
        if any((addr + i) not in mem for i in range(size)):
            sys.exit('读内存失败，PWLink 连上了吗？\n' + out[-800:])
        data.append(bytes(mem[addr + i] for i in range(size)))
    return data


def main():
    pyocd = find_pyocd()
    # g_line_rec 固定在 CCM RAM 开头（main.c 的 LINE_LOG_ADDR）：magic/head/count/stop + 500 帧 × 16 字节
    a_rec, n_rec = 0x10000000, 16 + 500 * REC
    a_tick, _ = sym('.bss.uwTick', 'stm32f4xx_hal.o')
    rec, tick = read_mem(pyocd, [(a_rec, n_rec), (a_tick, 4)])
    magic, head, cnt, stop = struct.unpack_from('<IIII', rec, 0)
    buf = rec[16:]
    n = len(buf) // REC
    print('单片机已运行 %.1f s' % (struct.unpack('<I', tick)[0] / 1000.0))
    if magic != 0x4C4E4C47 or head >= n or cnt > n:
        sys.exit('记录头无效：断过电（只有复位能保留记录），或车上不是这一版程序')
    start = (head - cnt) % n
    rows = []
    for k in range(cnt):
        ms, err, tl, tr, vl, vr, bits, flags = struct.unpack_from('<Ihhhhhbb', buf, ((start + k) % n) * REC)
        rows.append((ms, err / 100.0, tl, tr, vl, vr, bits & 0xFF, flags & 0xFF))
    if not rows:
        sys.exit('没有记录：这次起跑后还没跑过一拍？')
    os.makedirs(os.path.join(ROOT, 'out'), exist_ok=True)
    csv = os.path.join(ROOT, 'out', 'line_log.csv')
    t0 = rows[0][0]
    with open(csv, 'w', encoding='utf-8-sig') as f:
        f.write('t_ms,err,left_tgt,right_tgt,left_act,right_act,bits,found,split,held,lost,cross\n')
        for ms, err, tl, tr, vl, vr, bits, fl in rows:
            f.write('%d,%.2f,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d\n' % (
                ms - t0, err, tl, tr, vl, vr, ''.join('1' if bits >> i & 1 else '0' for i in range(8)),
                fl & 1, fl >> 1 & 1, fl >> 2 & 1, fl >> 3 & 1, fl >> 4 & 1))
    errs = [r[1] for r in rows]
    cross0 = sum(1 for a, b in zip(errs, errs[1:]) if (a < 0) != (b < 0) and a != 0 and b != 0)
    print('记录 %d 帧（%.1f s），停车原因：%s' % (len(rows), (rows[-1][0] - t0) / 1000.0, STOP.get(stop, stop)))
    print('误差范围 %+.2f ~ %+.2f，过零 %d 次；丢线帧 %d' % (
        min(errs), max(errs), cross0, sum(1 for r in rows if r[7] & 8)))
    print('已写 %s' % csv)
    show = rows if '--all' in sys.argv else rows[-60:]
    if show is not rows:
        print('最后 60 帧（--all 看全部）：')
    print('   t_ms  bits      err   目标 左/右    实测 左/右  标志')
    for ms, err, tl, tr, vl, vr, bits, fl in show:
        tag = ''.join(c for c, b in zip('FSHLC', range(5)) if fl >> b & 1)
        print('%7d  %s %+5.2f  %+5d %+5d  %+5d %+5d  %s' % (
            ms - t0, ''.join('1' if bits >> i & 1 else '0' for i in range(8)), err, tl, tr, vl, vr, tag))


if __name__ == '__main__':
    main()
