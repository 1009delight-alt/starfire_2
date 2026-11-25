"""读出车上运行记录（App/run_log.c，CCM RAM 0x10000000 的环形缓冲）。

纯循迹调试模式和比赛状态机（比赛、避障、找库单项）都会记：每 20 ms 一帧，最近 10 s，停车即冻结。
用法（在 stm32f407vet6/ 下，车停稳后不断电，插上 PWLink）：
    python -I scripts/line_log.py            # 读出并写 out/line_log.csv，打印摘要和最后 60 帧
    python -I scripts/line_log.py --all      # 逐帧打印全部

只读内存（commander -O connect_mode=attach，不停核、不复位）。uwTick 地址取自
out/SkyStar/SkyStar/Debug/SkyStar.axf.map（只用来显示单片机已运行多久）。
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
ADDR = 0x10000000           # run_log.h 的 RUN_LOG_ADDR
N = 500                     # RUN_LOG_N
MAGIC = 0x324E4C47          # RUN_LOG_MAGIC "GLN2"
HDR = 24
REC = 24
FMT = '<IhhhhhHBBBBH2x'
STOP = {0: '未停（还在跑或没跑过）', 1: '按键/回待机', 2: '丢线超限', 3: '电机故障', 4: '完成'}
STATE = ['IDLE', 'TRACK', 'AVOID', 'FORK', 'COLOR', 'PARK_FIND', 'PARK_REV', 'DONE', 'FAULT']
AVOID = ['转出', '斜出', '转平行', '平行', '转回', '回线']
PARK = ['前进', '转90', '倒车', '停5s']
MODE = {0: '本次上电后没跑过', 1: '纯循迹', 2: '比赛状态机'}


def find_pyocd():
    pats = glob.glob(os.path.expanduser(
        r'~\.vscode\extensions\arm.vscode-cmsis-debugger-*\tools\pyocd\pyocd.exe'))
    if not pats:
        sys.exit('找不到 pyocd.exe（VS Code 的 Arm CMSIS Debugger 插件）')
    return sorted(pats)[-1]


def sym(section, obj):
    try:
        text = open(MAP, encoding='utf-8', errors='replace').read()
    except OSError:
        return None
    m = re.search(r'(0x[0-9a-f]{8})\s+(0x[0-9a-f]{8})\s+\w+\s+RW\s+\d+\s+' + re.escape(section) + r'\s+' +
                  re.escape(obj), text)
    return int(m.group(1), 16) if m else None


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


def bits_str(b):
    return ''.join('1' if b >> i & 1 else '0' for i in range(8))


def where(state, step):
    name = STATE[state] if state < len(STATE) else ('LINE' if state == 0xFF else str(state))
    if state == 2 and step < len(AVOID):
        name += '/' + AVOID[step]
    elif state == 6 and step < len(PARK):
        name += '/' + PARK[step]
    return name


def main():
    pyocd = find_pyocd()
    ranges = [(ADDR, HDR + N * REC)]
    a_tick = sym('.bss.uwTick', 'stm32f4xx_hal.o')
    if a_tick:
        ranges.append((a_tick, 4))
    got = read_mem(pyocd, ranges)
    rec = got[0]
    if a_tick:
        print('单片机已运行 %.1f s' % (struct.unpack('<I', got[1])[0] / 1000.0))
    magic, head, cnt, stop, mode, active = struct.unpack_from('<IIIIII', rec, 0)
    if magic != MAGIC or head >= N or cnt > N:
        sys.exit('记录头无效：断过电，或车上不是这一版程序')
    if stop >= 0x08000000:
        s = read_mem(pyocd, [(stop, 24)])[0].split(b'\0')[0].decode('ascii', 'replace')
        stop_txt = '故障 ' + s
    else:
        stop_txt = STOP.get(stop, str(stop))
    rows = []
    start = (head - cnt) % N
    for k in range(cnt):
        rows.append(struct.unpack_from(FMT, rec, HDR + ((start + k) % N) * REC))
    print('模式：%s，%s' % (MODE.get(mode, mode), '正在记录' if active else '已冻结'))
    if not rows:
        sys.exit('没有记录：这次起跑后还没跑过一拍？')
    t0 = rows[0][0]
    os.makedirs(os.path.join(ROOT, 'out'), exist_ok=True)
    csv = os.path.join(ROOT, 'out', 'line_log.csv')
    with open(csv, 'w', encoding='utf-8-sig') as f:
        f.write('t_ms,state,err,left_tgt,right_tgt,left_act,right_act,sonar_mm,seg_mm,bits,found,split,held,lost,cross\n')
        for ms, err, tl, tr, vl, vr, son, bits, fl, st, sp, seg in rows:
            f.write('%d,%s,%.2f,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d\n' % (
                ms - t0, where(st, sp), err / 100.0, tl, tr, vl, vr, son, seg, bits_str(bits),
                fl & 1, fl >> 1 & 1, fl >> 2 & 1, fl >> 3 & 1, fl >> 4 & 1))
    errs = [r[1] / 100.0 for r in rows]
    print('记录 %d 帧（%.1f s），停车原因：%s' % (len(rows), (rows[-1][0] - t0) / 1000.0, stop_txt))
    print('误差范围 %+.2f ~ %+.2f；丢线帧 %d' % (min(errs), max(errs), sum(1 for r in rows if r[8] & 8)))
    seq = []
    for r in rows:
        w = where(r[9], r[10])
        if not seq or seq[-1][0] != w:
            seq.append([w, r[0] - t0])
    print('经过：' + ' → '.join('%s@%.2fs' % (w, t / 1000.0) for w, t in seq))
    print('已写 %s' % csv)
    show = rows if '--all' in sys.argv else rows[-60:]
    if show is not rows:
        print('最后 60 帧（--all 看全部）：')
    print('   t_ms  状态            bits      err   目标 左/右    实测 左/右  超声  段里程 标志')
    for ms, err, tl, tr, vl, vr, son, bits, fl, st, sp, seg in show:
        tag = ''.join(c for c, b in zip('FSHLC', range(5)) if fl >> b & 1)
        print('%7d  %-14s %s %+5.2f  %+5d %+5d  %+5d %+5d  %4d  %5d  %s' % (
            ms - t0, where(st, sp), bits_str(bits), err / 100.0, tl, tr, vl, vr, son, seg, tag))


if __name__ == '__main__':
    main()
