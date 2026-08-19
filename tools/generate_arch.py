#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the R3D three-module architecture diagram to ARCHITECTURE.png via PIL.
Pure-Python, no browser needed. Chinese text via Noto Sans CJK.
"""
from PIL import Image, ImageDraw, ImageFont

# ---------------- palette ----------------
BG       = (246, 248, 251)
PANEL    = (255, 255, 255)
INK      = (15, 23, 42)
SUB      = (81, 96, 122)
FAINT    = (138, 151, 173)
LINE     = (227, 233, 242)
LINE2    = (210, 219, 232)

HW       = (14, 124, 122);   HW_BG=(231,246,244);   HW_BD=(157,217,211)
R3       = (33, 80, 217);    R3_BG=(236,241,254);   R3_BD=(174,196,246)
LIO      = (122, 69, 214);   LIO_BG=(242,236,251);  LIO_BD=(198,176,242)
CTRL     = (217, 96, 43);    CTRL_BG=(252,239,227)
DATA     = (30, 158, 106);   DATA_BG=(228,246,238)
FILE     = (154, 106, 0);    FILE_BG=(251,241,217)
CODE     = (51, 66, 91)

W = 1780
LEFT, RIGHT = 40, 1740

# ---------------- fonts ----------------
def F(path, size):
    try: return ImageFont.truetype(path, size)
    except Exception: return None
FONT_PATHS_BOLD = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                   "/usr/share/fonts/opentype/noto/NotoSansCJK-Black.ttc"]
FONT_PATHS_REG  = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                   "/usr/share/fonts/opentype/noto/NotoSansCJK-DemiLight.ttc",
                   "/usr/share/fonts/opentype/noto/NotoSansCJK-Light.ttc",
                   "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"]
def pick(paths, size):
    for p in paths:
        f = F(p, size)
        if f: return f
    return ImageFont.load_default()
def font(sz, bold=False): return pick(FONT_PATHS_BOLD if bold else FONT_PATHS_REG, int(round(sz)))

F_H1   = font(27, True)
F_HSUB = font(13)
F_MH   = font(18, True)   # module header title
F_MHS  = font(11)         # module header subtitle
F_CT   = font(13.5, True) # card title
F_CD   = font(11.5)       # card desc
F_AR   = font(12, True)   # iface arrow label
F_LB   = font(10.5)       # iface label
F_BUS  = font(11.5)
F_BUSB = font(12, True)
F_FLT  = font(13.5, True)
F_FLI = font(11.5)
F_LEG  = font(12)
F_FOOT = font(11)

# ---------------- helpers ----------------
def rrect(d, box, rad, fill=None, outline=None, w=1):
    d.rounded_rectangle(box, radius=rad, fill=fill, outline=outline, width=w)

def txt(d, xy, s, f, fill):
    d.text(xy, s, font=f, fill=fill)

_meas_img = Image.new("RGB", (10, 10))
_md = ImageDraw.Draw(_meas_img)
def measure(d, s, f):
    b = _md.textbbox((0,0), s, font=f)
    return b[2]-b[0], b[3]-b[1]
def text_w(s, f):
    return _md.textbbox((0,0), s, font=f)[2]
def wrap_line(text, fnt, max_w):
    out, cur = [], ""
    for ch in text:
        if text_w(cur + ch, fnt) <= max_w:
            cur += ch
        else:
            out.append(cur); cur = ch
    out.append(cur)
    return out
def prep_cards(cards, inner_w):
    max_w = inner_w - 6
    res = []
    for (title, pill, lines, ac) in cards:
        fl = []
        for ln in lines:
            fl.extend(wrap_line(ln, F_CD, max_w))
        res.append((title, pill, fl, ac))
    return res

# ---------------- content data ----------------
# each component card: (title, pill, [desc lines], accent)
HW_CARDS = [
    ("电源板 MCU", "UART", [
        "串口 /dev/ttyS2 @9600 8N1，9字节定长帧",
        "帧格式: [5C][cmd][×5][0D][0A]",
        "→ Jetson: 0x31 周期遥测",
        "   (电压mV / 温度 / 关机请求 / 电量%)",
        "← Jetson: 0x32 风扇 · 0x33 关机 · 0x34 雷达",
    ], HW),
    ("Livox MID-360 激光雷达", None, [
        "非重复扫描 LiDAR + 内置 IMU",
        "由电源板继电器上 / 下电",
        "驱动持续把硬件纳秒时戳写入共享文件",
    ], HW),
    ("双路 USB 相机", None, [
        "/dev/video0 · /dev/video2",
        "原生 MJPG（V4L2 零编码直出）",
    ], HW),
    ("GPIO / LED 指示", "wiringPi", [
        "绿灯 wp1(运行) · 红灯 wp0(错误·低电)",
        "扫描态绿光 1Hz 闪烁",
        "后端 WiringPiGpio(实机) / MockGpio(开发)",
        "由 USE_WIRINGPI 宏编译期切换",
    ], HW),
    ("雷达电源继电器", None, [
        "受电源板 0x34 控制",
        "决定 Livox 雷达上 / 下电",
    ], HW),
]
R3_CARDS = [
    ("手机 App / Web 前端", "用户层", [
        "Three.js 点云渲染 (前端 index.html, 内置 r128)",
        "连接 ws://设备IP:8765/data",
        "发 startRecord / stopRecord",
        "实时收 odom · pcd · img",
    ], R3),
    ("r3d_ws_bridge", "ROS 节点", [
        "节点名 ws_sync_sender；boost::beast WS 服务端",
        ":8765/data（单线程）",
        "订阅 lio_sam: odometry / cloud_registered (BEST_EFFORT)",
        "try_sync 50ms 配对 odom↔lidar (<0.05s)",
        "每连接 50ms tick 推流（pcd 10Hz）",
        "命令分发: startRecord→写IPC+拉起ROS栈",
        "          stopRecord→关窗口+写IPC",
    ], R3),
    ("r3d_main", "设备主控·无ROS", [
        "状态机: INIT→RUN ⇄ SCAN→(ERROR/SHUTDOWN)",
        "IPC线程: 100ms 轮询 /tmp/shared_cmd.txt",
        "         → start/stop_radar",
        "驱动: UART 控电源板(雷达上下电)、GPIO 控LED",
        "      bridge_launcher 启动 ws_bridge",
        "关机: 电量<10%/电源板请求→latch",
        "      → execlp(\"shutdown -h now\")",
    ], R3),
    ("image_odom_recorder", "录制存档", [
        "订阅 /lio_sam/mapping/odometry → odom.txt",
        "双相机事件驱动抓帧，曝光时刻转墙钟µs命名",
        "触发双相机 → images/camera{0,1}/<墙钟微秒>_<timeshare UTC纳秒>.jpg",
        "fork ros2 bag record 录 /livox/imu /livox/lidar",
        "→ rosbag2/",
    ], R3),
    ("r3d_daemon", "看门狗", [
        "独立进程；r3d_main 崩溃 3s 后自动 respawn",
        "正常退出则同步退出",
    ], R3),
]
LIO_CARDS = [
    ("livox_ros_driver2", "外部包", [
        "msg_MID360_launch.py 启动；直读 Livox 硬件",
        "发布 /livox/lidar (点云) · /livox/imu",
        "同时把硬件纳秒时戳写入 ~/timeshare",
    ], LIO),
    ("lio_sam", "SLAM 核心·外部包", [
        "run.launch.py 启动；LIO 紧耦合里程计 + 因子图建图",
        "消费 /livox/lidar + /livox/imu",
        "输出 /lio_sam/mapping/odometry (全局位姿)",
        "输出 /lio_sam/mapping/cloud_registered (配准点云)",
    ], LIO),
    ("模块边界说明", None, [
        "仓库内无源码 / 无 launch / 无参数 yaml",
        "与 R3D 系统仅靠 4 个话题名解耦:",
        "odometry · cloud_registered",
        "/livox/lidar · /livox/imu",
    ], LIO),
    ("mock_publisher", "调试", [
        "无雷达时发布同话题假 odom+pcd",
        "供 headless 测试",
    ], LIO),
]

# lanes: (kind, dir, label, [lines])  dir 'L'=flow toward left, 'R'=toward right
LANE1 = [
    ("data", "L", "UART 遥测", ["电源板 0x31 帧流", "(电压/温度/电量) → r3d_main 解析"]),
    ("ctrl", "R", "UART 命令", ["r3d_main → 电源板:", "0x32 风扇·0x33 关机·0x34 雷达"]),
    ("data", "L", "相机帧", ["/dev/video0,2 MJPG", "→ image_odom_recorder 直抓"]),
]
LANE2 = [
    ("ctrl", "R", "launch 命令", ["ws_bridge 用 gnome-terminal", "+xdotool 拉起 livox/lio_sam/recorder"]),
    ("data", "L", "ROS 话题", ["/lio_sam/mapping/odometry(位姿)", "+cloud_registered(点云)→推流/存档"]),
    ("file", "L", "雷达时戳", ["~/timeshare 16B mmap", "(livox驱动写)→recorder 读, 对齐相机"]),
]

FLOWS = [
    (CTRL, "1", "开机建图控制流（跨三模块）", [
        "手机发 startRecord → ws_bridge",
        "ws_bridge 写 shared_cmd=1 ＋ 开 3 终端",
        "  拉起 livox / lio_sam / recorder",
        "r3d_main 读到 IPC → start_radar",
        "  → UART 0x34 雷达电源 ON",
        "电源板继电器给 Livox 上电, 点云吐出",
        "lio_sam 跑起来, mapping_ready 后开始推流",
    ]),
    (DATA, "2", "实时数据流（硬件 → 手机）", [
        "Livox 点云+IMU → livox_driver2 → /livox/*",
        "lio_sam 紧耦合 → odometry + cloud_registered",
        "ws_bridge 订阅 → try_sync 时间配对",
        "WS :8765/data 50ms tick 推",
        "  odom(20Hz) · pcd(10Hz)",
        "手机 Three.js 渲染点云 + 位姿轨迹",
    ]),
    (FILE, "3", "离线存档流（建图产物）", [
        "recorder 双相机事件驱动抓帧，曝光时刻转墙钟µs命名",
        "双相机起步对齐后独立计数抽帧",
        "  → images/camera{0,1}/<墙钟微秒>_<timeshare UTC纳秒>.jpg",
        "订阅 lio_sam odometry → odom.txt",
        "fork ros2 bag record → rosbag2/",
        "全部落 ~/Desktop/save_pcd/<时间戳>/",
    ]),
]

# ---------------- layout calculation ----------------
def card_height(lines):
    return 20 + len(lines) * 16 + 22

def module_height(cards, inner_w, header_h=46, pad=14, gap=11):
    h = header_h + pad
    for c in prep_cards(cards, inner_w):
        h += card_height(c[2]) + gap
    return h + pad - gap

H_HW  = module_height(HW_CARDS, 404)
H_R3  = module_height(R3_CARDS, 524)
H_LIO = module_height(LIO_CARDS, 404)
GRID_H = max(H_HW, H_R3, H_LIO)

# column x ranges
COL_HW = (40, 480)        # w 440
LANE1_X = (480, 590)       # w 110
COL_R3 = (590, 1150)       # w 560
LANE2_X = (1150, 1260)     # w 110
COL_LIO= (1260, 1700)      # w 440

TITLE_H = 96
GRID_TOP = TITLE_H + 24            # = 120
GRID_BOTTOM = GRID_TOP + GRID_H    # bottom of modules

BUS_TOP = GRID_BOTTOM + 18
BUS_H = 86
FLOWS_TOP = BUS_TOP + BUS_H + 18
FLOWS_H = 0
for fl in FLOWS:
    FLOWS_H = max(FLOWS_H, 40 + len(fl[3]) * 18 + 14)
LEG_TOP = FLOWS_TOP + FLOWS_H + 16
FOOT_TOP = LEG_TOP + 26
H_TOTAL = FOOT_TOP + 26

# ---------------- draw ----------------
img = Image.new("RGB", (W, H_TOTAL), BG)
d = ImageDraw.Draw(img)

def draw_arrow_tri(x, y, pointing, color, size=7):
    # pointing 'R' or 'L', vertical center y
    if pointing == "R":
        pts = [(x, y-size), (x, y+size), (x+size+2, y)]
    else:
        pts = [(x+size+2, y-size), (x+size+2, y+size), (x, y)]
    d.polygon(pts, fill=color)

def draw_flow_arrow(cx_left, cx_right, y, direction, color):
    # a horizontal line with a triangle head; direction 'L'/'R'
    if direction == "R":
        draw_arrow_tri(cx_right-9, y, "R", color)
        d.line([(cx_left, y), (cx_right-9, y)], fill=color, width=3)
    else:
        draw_arrow_tri(cx_left+9, y, "L", color)
        d.line([(cx_left+9, y), (cx_right, y)], fill=color, width=3)

# ---- title ----
t1 = "硬件设计 · R3D 扫描系统 · LIO-SAM"
# colorize three segments
cx = LEFT
segs = [("硬件设计", HW), (" · ", FAINT), ("R3D 扫描系统", R3), (" · ", FAINT), ("LIO-SAM", LIO)]
# center the whole title
total = sum(measure(d, s, F_H1)[0] for s,_ in segs)
cx = (W - total)//2
for s,col in segs:
    w = measure(d, s, F_H1)[0]
    txt(d, (cx, 26), s, F_H1, col)
    cx += w
sub = "手持式 3D 建图设备 — 三大模块各自业务 · 模块间数据与控制流转（基于真实代码梳理）"
sw = measure(d, sub, F_HSUB)[0]
txt(d, ((W-sw)//2, 64), sub, F_HSUB, SUB)

# ---- module cards ----
def draw_module(col_range, title, sub, accent, cards, accent_bg, accent_bd):
    x0, x1 = col_range
    # outer panel
    rrect(d, (x0, GRID_TOP, x1, GRID_TOP+GRID_H), 16, fill=PANEL, outline=LINE2, w=1)
    # header bar (top rounded)
    d.rounded_rectangle((x0, GRID_TOP, x1, GRID_TOP+46), radius=16, fill=accent)
    d.rectangle((x0, GRID_TOP+30, x1, GRID_TOP+46), fill=accent)  # square off bottom of header
    # icon square
    rrect(d, (x0+12, GRID_TOP+9, x0+12+28, GRID_TOP+9+28), 8, fill=(255,255,255,40))
    ic = title[0] if False else (title.split()[0][0] if False else "")
    # use a fixed letter
    letter = {"① 硬件设计":"H","② R3D 扫描系统":"R","③ LIO-SAM":"L"}[title]
    lw,_ = measure(d, letter, font(16, True))
    txt(d, (x0+12+(28-lw)//2, GRID_TOP+12), letter, font(16,True), (255,255,255))
    txt(d, (x0+48, GRID_TOP+7), title, F_MH, (255,255,255))
    txt(d, (x0+48, GRID_TOP+28), sub, F_MHS, (235,240,255))
    # cards
    inner_w = (x1 - x0) - 36
    prepped = prep_cards(cards, inner_w)
    cy = GRID_TOP + 46 + 14
    for (ctitle, pill, lines, ac) in prepped:
        ch = card_height(lines)
        rrect(d, (x0+12, cy, x1-12, cy+ch), 11, fill=PANEL, outline=LINE, w=1)
        # left accent stripe
        d.rectangle((x0+12, cy+10, x0+15, cy+ch-10), fill=ac)
        # title
        tx = x0+24
        txt(d, (tx, cy+9), ctitle, F_CT, INK)
        # pill
        tw = measure(d, ctitle, F_CT)[0]
        if pill:
            pw = measure(d, pill, font(10, True))[0] + 12
            px = tx + tw + 8
            rrect(d, (px, cy+9, px+pw, cy+9+18), 9, fill=ac)
            plw = measure(d, pill, font(10,True))[0]
            txt(d, (px+(pw-plw)//2, cy+11), pill, font(10,True), (255,255,255))
        # desc lines
        ly = cy + 30
        for ln in lines:
            txt(d, (tx, ly), ln, F_CD, SUB)
            ly += 16
        cy += ch + 11

draw_module(COL_HW, "① 硬件设计", "物理外设 · 电源板协议 · 传感器", HW, HW_CARDS, HW_BG, HW_BD)
draw_module(COL_R3, "② R3D 扫描系统", "控制 · 通信 · 录制存档（居中调度）", R3, R3_CARDS, R3_BG, R3_BD)
draw_module(COL_LIO, "③ LIO-SAM", "外部 ROS 包 · 定位建图核心", LIO, LIO_CARDS, LIO_BG, LIO_BD)

# ---- lanes ----
def draw_lane(lane_x, items, left_mod_name, right_mod_name):
    x0, x1 = lane_x
    n = len(items)
    avail = GRID_H - 40
    slot = avail / n
    kind_color = {"ctrl":CTRL, "data":DATA, "file":FILE}
    kind_bg    = {"ctrl":CTRL_BG, "data":DATA_BG, "file":FILE_BG}
    for i,(kind,direc,label,lines) in enumerate(items):
        cy = GRID_TOP + 20 + int(i*slot)
        ch = 22 + len(lines)*15 + 10
        # card
        rrect(d, (x0+6, cy, x1-6, cy+ch), 10, fill=PANEL, outline=LINE2, w=1)
        # top stripe
        col = kind_color[kind]
        d.rectangle((x0+6, cy, x1-6, cy+3), fill=col)
        # arrow + label row
        midy = cy + 13
        ax_left, ax_right = x0+12, x1-12
        draw_flow_arrow(ax_left+4, ax_right-4, midy, direc, col)
        lw = measure(d, label, F_AR)[0]
        txt(d, ((x0+x1-lw)//2, cy+5), label, F_AR, col)
        # lines
        ly = cy + 24
        for ln in lines:
            lw = measure(d, ln, F_LB)[0]
            txt(d, ((x0+x1-lw)//2, ly), ln, F_LB, SUB)
            ly += 15

draw_lane(LANE1_X, LANE1, "硬件", "R3D")
draw_lane(LANE2_X, LANE2, "R3D", "LIO-SAM")

# ---- bus bar (decoupling boundaries) ----
rrect(d, (LEFT, BUS_TOP, RIGHT, BUS_TOP+BUS_H), 14, fill=FILE_BG, outline=FILE, w=2)
d.line([(LEFT, BUS_TOP), (LEFT, BUS_TOP+BUS_H)], fill=FILE, width=2)
txt(d, (LEFT+16, BUS_TOP+12), "⇆  两个解耦边界", F_BUSB, FILE)
bus_lines = [
    "① 文件 IPC  /tmp/shared_cmd.txt  ({\"cmd\":1|0}，原子 tmp+rename 写)  —  r3d_ws_bridge 写 → r3d_main 读：手机命令跨进程传到设备主控，触发雷达上 / 下电。",
    "② 共享时戳  ~/timeshare  (16B mmap，int64 高/低 = Livox 纳秒)  —  livox 驱动写 → image_odom_recorder 读：让每张照片打上真实雷达硬件时间，事后与点云 / 里程计对齐。",
]
ly = BUS_TOP + 34
for ln in bus_lines:
    txt(d, (LEFT+16, ly), ln, F_BUS, (107,82,16))
    ly += 24

# ---- three flows ----
colw = (RIGHT - LEFT - 28) // 3
for i,(col, num, title, lines) in enumerate(FLOWS):
    x0 = LEFT + i*(colw+14)
    x1 = x0 + colw
    rrect(d, (x0, FLOWS_TOP, x1, FLOWS_TOP+FLOWS_H), 13, fill=PANEL, outline=LINE2, w=1)
    # number badge
    rrect(d, (x0+14, FLOWS_TOP+13, x0+14+24, FLOWS_TOP+13+24), 7, fill=col)
    nw = measure(d, num, font(13,True))[0]
    txt(d, (x0+14+(24-nw)//2, FLOWS_TOP+15), num, font(13,True), (255,255,255))
    txt(d, (x0+46, FLOWS_TOP+16), title, F_FLT, INK)
    ly = FLOWS_TOP + 46
    for ln in lines:
        if ln.strip().startswith("→") or ln.startswith("  "):
            txt(d, (x0+30, ly), ln, F_FLI, SUB)
        else:
            # numbered marker
            txt(d, (x0+16, ly), "•", F_FLI, col)
            txt(d, (x0+30, ly), ln, F_FLI, SUB)
        ly += 18

# ---- legend ----
leg_y = LEG_TOP + 4
def legend_item(x, color, label, dashed=False):
    if dashed:
        d.line([(x, leg_y+7),(x+26, leg_y+7)], fill=color, width=3)
        # make it look dashed
        d.line([(x, leg_y+7),(x+26, leg_y+7)], fill=FILE_BG, width=3)
        seg=5
        xs=x
        while xs < x+26:
            d.line([(xs, leg_y+7),(min(xs+seg,x+26), leg_y+7)], fill=color, width=3)
            xs += seg*2
    else:
        d.line([(x, leg_y+7),(x+26, leg_y+7)], fill=color, width=3)
    txt(d, (x+34, leg_y), label, F_LEG, SUB)
    return x + 34 + measure(d, label, F_LEG)[0] + 26
lx = LEFT + 40
lx = legend_item(lx, CTRL, "控制流（命令 / 触发）")
lx = legend_item(lx, DATA, "数据流（遥测 / 话题 / 媒体）")
lx = legend_item(lx, FILE, "文件 / 共享内存解耦", dashed=True)
txt(d, (lx, leg_y), "◀━━ 向左流动   ｜   ━━▶ 向右流动", F_LEG, SUB)

# ---- footer ----
foot = "R3D_cpp · 基于 src/ 与 ros2_ws/src/r3d_bringup/ 真实代码梳理 · 三大模块 = 硬件设计 / R3D 扫描系统 / LIO-SAM"
fw = measure(d, foot, F_FOOT)[0]
txt(d, ((W-fw)//2, FOOT_TOP+6), foot, F_FOOT, FAINT)

img.save("/home/bjw/code/R3D_cpp/ARCHITECTURE.png", "PNG")
print("saved ARCHITECTURE.png", img.size)
