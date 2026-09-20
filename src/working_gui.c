import socket
import threading
import re
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import matplotlib.patheffects as pe
import numpy as np
import os
import math

CHOICE    = "live"
filename  = "sensor-scan.txt"
absolute_path = os.path.dirname(os.path.abspath(__file__))
full_path     = os.path.join(absolute_path, filename)
HOST = "192.168.1.1"
PORT = 288

BG         = "#0b0d11"
PANEL      = "#111418"
BORDER     = "#1e2530"
ACCENT     = "#3cffc8"
ACCENT_DIM = "#1a7a60"
WARN       = "#f0a030"
DANGER     = "#f05050"
INFO       = "#4a9ef0"
TEXT       = "#dce0e8"
MUTED      = "#5a6070"
GRID_COLOR = "#1a2030"
ROBOT_COLOR= "#4a9ef0"

MINE_ACTIVE  = "#f05050"
MINE_DEMINED = "#40c060"
OBJ_COLOR    = "#f0a030"

GRID_STEP_CM = 6
MAP_FORWARD_CM = 96
MAP_BACK_CM = 18
MAP_HALF_WIDTH_CM = 60
CYBOT_RADIUS_CM = 4

angle_data = []
dist_data  = []
ir_data    = []
ping_data  = []

detected_objects      = []
gap_info              = {}
bump_objects          = []
_pending_active_mines = []
_obj_table_scroll = [0]
_obj_table_rows   = [0]

cybot_x   = [0.0]
cybot_y   = [0.0]
cybot_hdg = [90.0]
cybot_draw_x   = [0.0]
cybot_draw_y   = [0.0]
cybot_draw_hdg = [90.0]

status_msg   = ["IDLE"]
scan_count   = [0]
stop_flag    = False
cybot_socket = None
cybot        = None
rx_thread    = None
rx_running   = [False]

# ---------------------------------------------------------------------------
# Regexes — matched exactly to C uart output strings
# ---------------------------------------------------------------------------

# "Angle: X Deg\t IR Dist: Y cm\t Ping Dist: Z cm\t Used dist: W cm"
_RE_SCAN = re.compile(
    r"Angle:\s*(\d+)\s*Deg"
    r"[\t ]+IR Dist:\s*([\d.]+)\s*cm"
    r"[\t ]+Ping Dist:\s*([\d.]+)\s*cm"
    r"[\t ]+Used dist:\s*([\d.]+)"
)

# "Mine K/N at angle A dist D width W "   (auto mode, mid-run)
_RE_MINE_AUTO = re.compile(
    r"Mine\s+(\d+)/(\d+)\s+at angle\s+(\d+)\s+dist\s+([\d.]+)\s+width\s+([\d.]+)"
)

# "Mine K angle: A dist: D width: W "     (manual scan result)
_RE_MINE_MAN = re.compile(
    r"Mine\s+(\d+)\s+angle:\s*(\d+)\s+dist:\s*([\d.]+)\s+width:\s*([\d.]+)"
)

# "Gap angle: A\twidth: W"                (manual scan result)
# "Gap found at angle A width W"          (auto mode)
_RE_GAP = re.compile(
    r"(?:Gap angle:\s*(\d+)[\t ]+width:\s*([\d.]+)"
    r"|Gap found at angle\s+(\d+)\s+width\s+([\d.]+))"
)

# "Object angle S-E mid M dist D width W" (finish_object report)
_RE_OBJ_LINE = re.compile(
    r"Object angle\s+(\d+)-(\d+)\s+mid\s+(\d+)\s+dist\s+([\d.]+)\s+width\s+([\d.]+)"
)

# "Ignored blip S-E mid M dist D width W"  — logged by finish_object, not a detection
_RE_IGNORED_BLIP = re.compile(r"^Ignored blip\b", re.IGNORECASE)

# "Pos: x X cm, y Y cm, hdg Z deg"
_RE_POS_XY = re.compile(
    r"Pos:\s*x\s*([+-]?[\d.]+)\s*cm,\s*"
    r"y\s*([+-]?[\d.]+)\s*cm,\s*"
    r"hdg\s*([+-]?[\d.]+)\s*deg",
    re.IGNORECASE,
)
# fallback polar pos (not emitted by current C code but kept for compatibility)
_RE_POS_POLAR = re.compile(r"Pos:\s*([+-]?[\d.]+)\s*cm,\s*([+-]?[\d.]+)\s*deg")

# pose dedup: skip update if pose hasn't meaningfully changed
_POSE_DEDUP_XY_CM  = 0.05   # cm
_POSE_DEDUP_HDG_DEG = 0.1   # deg

# "Obstacle hit"
_RE_BUMP  = re.compile(r"Obstacle hit", re.IGNORECASE)

# "Boundary/hole detected"
_RE_BOUNDARY = re.compile(r"Boundary/hole detected", re.IGNORECASE)

# "Possible mine seen in only one sweep..."
_RE_POSSIBLE_MINE = re.compile(r"Possible mine seen in only one sweep", re.IGNORECASE)

# "Turning X degrees to face gap"
_RE_TURNING = re.compile(r"Turning\s+([+-]?\d+)\s+degrees", re.IGNORECASE)

# "Mine still exists..."
_RE_MINE_STILL   = re.compile(r"Mine still exists", re.IGNORECASE)

# "Mines cleared" / "No mine or gap found..." / "Gap found at angle..."
_RE_MINE_CLEARED = re.compile(
    r"(No mine or gap found|Gap found at angle|Mines [Cc]leared)", re.IGNORECASE)

# track last angle seen to detect new sweep without "Scan Start"
_last_angle = [-1]
_radar_sweeps_since_clear = [0]
_gap_clear_pose_countdown = [0]


def _smooth_cybot_pose():
    alpha = 0.28
    cybot_draw_x[0] += (cybot_x[0] - cybot_draw_x[0]) * alpha
    cybot_draw_y[0] += (cybot_y[0] - cybot_draw_y[0]) * alpha

    hdg_delta = ((cybot_hdg[0] - cybot_draw_hdg[0] + 180) % 360) - 180
    cybot_draw_hdg[0] = (cybot_draw_hdg[0] + hdg_delta * alpha) % 360


def _scan_to_world(angle_deg, distance_cm, base_x=None, base_y=None, base_hdg=None):
    if base_x is None:
        base_x = cybot_x[0]
    if base_y is None:
        base_y = cybot_y[0]
    if base_hdg is None:
        base_hdg = cybot_hdg[0]

    world_angle = base_hdg + (angle_deg - 90.0)
    rad = math.radians(world_angle)
    return (
        base_x + distance_cm * math.cos(rad),
        base_y + distance_cm * math.sin(rad),
    )


def _store_pose_from_line(line):
    m = _RE_POS_XY.search(line)
    if m:
        try:
            nx = float(m.group(1))
            ny = float(m.group(2))
            nh = float(m.group(3))
        except Exception:
            return False
        # Skip redundant spam (bot stopped, demining wait, etc.)
        if (abs(nx - cybot_x[0]) < _POSE_DEDUP_XY_CM and
                abs(ny - cybot_y[0]) < _POSE_DEDUP_XY_CM and
                abs(nh - cybot_hdg[0]) < _POSE_DEDUP_HDG_DEG):
            return True   # consumed but no update needed
        cybot_x[0], cybot_y[0], cybot_hdg[0] = nx, ny, nh
        if _gap_clear_pose_countdown[0] > 0:
            _gap_clear_pose_countdown[0] -= 1
            if _gap_clear_pose_countdown[0] == 0:
                _clear_scan_scene()
                status_msg[0] = "MOVED THROUGH GAP — GRID CLEARED"
        return True

    m = _RE_POS_POLAR.search(line)
    if m:
        try:
            dist_cm = float(m.group(1))
            deg     = float(m.group(2))
            rad     = math.radians(deg)
            nx = dist_cm * math.cos(rad)
            ny = dist_cm * math.sin(rad)
            nh = deg
        except Exception:
            return False
        if (abs(nx - cybot_x[0]) < _POSE_DEDUP_XY_CM and
                abs(ny - cybot_y[0]) < _POSE_DEDUP_XY_CM and
                abs(nh - cybot_hdg[0]) < _POSE_DEDUP_HDG_DEG):
            return True
        cybot_x[0], cybot_y[0], cybot_hdg[0] = nx, ny, nh
        if _gap_clear_pose_countdown[0] > 0:
            _gap_clear_pose_countdown[0] -= 1
            if _gap_clear_pose_countdown[0] == 0:
                _clear_scan_scene()
                status_msg[0] = "MOVED THROUGH GAP — GRID CLEARED"
        return True

    return False


def _mark_pending_mines_demined():
    for obj in _pending_active_mines:
        if obj["state"] == "active":
            obj["state"] = "demined"
    _pending_active_mines.clear()


def _clear_scan_scene():
    angle_data.clear(); dist_data.clear()
    ir_data.clear();    ping_data.clear()
    detected_objects.clear(); gap_info.clear()
    bump_objects.clear(); _pending_active_mines.clear()
    _obj_table_scroll[0] = 0
    _radar_sweeps_since_clear[0] = 0
    _gap_clear_pose_countdown[0] = 0


def _strip_command_prefix(line):
    if len(line) > 1 and line[0].islower() and line[1].isupper():
        return line[1:]
    return line


def _store_mine_detection(obj):
    detected_objects.append(obj)


def _store_object_detection(obj):
    detected_objects.append(obj)
    status_msg[0] = f"OBJECT DETECTED  angle {obj['mid_angle']:.0f}°"


def parse_and_store(line):
    line = line.strip()
    line = _strip_command_prefix(line)

    if _store_pose_from_line(line):
        return

    # "Ignored blip ..." — finish_object logged a too-small/edge object; skip silently
    if _RE_IGNORED_BLIP.match(line):
        return

    # --- scan data point ---
    m = _RE_SCAN.search(line)
    if m:
        try:
            ang  = int(m.group(1))
            ir   = float(m.group(2))
            ping = float(m.group(3))
            dist = float(m.group(4))

            if ang == 0 and _last_angle[0] > 0:
                _radar_sweeps_since_clear[0] += 1
                if _radar_sweeps_since_clear[0] >= 2:
                    angle_data.clear()
                    dist_data.clear()
                    ir_data.clear()
                    ping_data.clear()
                    _radar_sweeps_since_clear[0] = 0

            _last_angle[0] = ang
            angle_data.append(ang)
            ir_data.append(ir)
            ping_data.append(ping)
            dist_data.append(dist)
            status_msg[0] = f"SCANNING…  angle {ang}°  |  {len(angle_data)} pts"
        except Exception:
            pass
        return

    # --- object line from finish_object() ---
    m = _RE_OBJ_LINE.search(line)
    if m:
        try:
            start_a = int(m.group(1))
            end_a   = int(m.group(2))
            mid_a   = int(m.group(3))
            dist    = float(m.group(4))
            width   = float(m.group(5))
            n    = len(detected_objects) + 1
            obj  = {
                "mid_angle": mid_a,
                "distance":  dist,
                "wx":        _scan_to_world(mid_a, dist)[0],
                "wy":        _scan_to_world(mid_a, dist)[1],
                "ang_width": abs(end_a - start_a),
                "lin_width": width,
                "label":     f"OBJ {n}",
                "is_mine":   False,
                "state":     "object",
            }
            _store_object_detection(obj)
        except Exception:
            pass
        return

    # --- mine auto mode: "Mine K/N at angle A dist D width W" ---
    m = _RE_MINE_AUTO.search(line)
    if m:
        try:
            obj = {
                "mid_angle": int(m.group(3)),
                "distance":  float(m.group(4)),
                "wx":        _scan_to_world(int(m.group(3)), float(m.group(4)))[0],
                "wy":        _scan_to_world(int(m.group(3)), float(m.group(4)))[1],
                "ang_width": 0.0,
                "lin_width": float(m.group(5)),
                "label":     f"MINE {m.group(1)}/{m.group(2)}",
                "is_mine":   True,
                "state":     "active",
            }
            _store_mine_detection(obj)
            _pending_active_mines.append(obj)
        except Exception:
            pass
        return

    # --- mine manual mode: "Mine K angle: A dist: D width: W" ---
    m = _RE_MINE_MAN.search(line)
    if m:
        try:
            obj = {
                "mid_angle": int(m.group(2)),
                "distance":  float(m.group(3)),
                "wx":        _scan_to_world(int(m.group(2)), float(m.group(3)))[0],
                "wy":        _scan_to_world(int(m.group(2)), float(m.group(3)))[1],
                "ang_width": 0.0,
                "lin_width": float(m.group(4)),
                "label":     f"MINE {m.group(1)}",
                "is_mine":   True,
                "state":     "active",
            }
            _store_mine_detection(obj)
            _pending_active_mines.append(obj)
        except Exception:
            pass
        return

    # --- gap ---
    m = _RE_GAP.search(line)
    if m:
        try:
            if m.group(1) is not None:
                gap_info["angle"] = int(m.group(1))
                gap_info["width"] = float(m.group(2))
            else:
                gap_info["angle"] = int(m.group(3))
                gap_info["width"] = float(m.group(4))
        except Exception:
            pass
        return

    # --- bump / obstacle hit ---
    if _RE_BUMP.search(line):
        n = len(bump_objects) + 1
        bump_objects.append({
            "x":     cybot_x[0],
            "y":     cybot_y[0],
            "label": f"OBJ {n}",
        })
        detected_objects.append({
            "mid_angle": int(cybot_hdg[0]),
            "distance":  0.0,
            "ang_width": 0.0,
            "lin_width": 0.0,
            "label":     f"BUMP {n}",
            "is_mine":   False,
            "state":     "bump",
            "bx":        cybot_x[0],
            "by":        cybot_y[0],
        })
        return

    # --- boundary / hole ---
    if _RE_BOUNDARY.search(line):
        status_msg[0] = "BOUNDARY / HOLE DETECTED"
        return

    # --- possible mine (uncertain) ---
    if _RE_POSSIBLE_MINE.search(line):
        status_msg[0] = "POSSIBLE MINE — RE-SCAN"
        return

    # --- mine still present after demining attempt ---
    if _RE_MINE_STILL.search(line):
        status_msg[0] = "⚠ Mine still present — re-scanning"
        return

    # --- mines cleared / no mine or gap / gap found (post-demine rescan) ---
    if _RE_MINE_CLEARED.search(line):
        _mark_pending_mines_demined()
        if "gap found at angle" in line.lower():
            status_msg[0] = "MINE CLEARED — GAP FOUND"
        else:
            status_msg[0] = "MINE CLEARED — NO MINE"
        return

    # --- turning to gap ---
    m = _RE_TURNING.search(line)
    if m:
        turn_amount = int(m.group(1))
        _gap_clear_pose_countdown[0] = 2 if turn_amount != 0 else 1
        status_msg[0] = f"TURNING {m.group(1)}° TO GAP"
        return

    # --- scan end ---
    if line.upper() == "SCAN END":
        if status_msg[0].startswith("SCANNING"):
            status_msg[0] = f"SCAN COMPLETE  —  {len(detected_objects)} detection(s)"
        return

    # --- all mines found ---
    if "all mines" in line.lower() or "mines found" in line.lower():
        status_msg[0] = "ALL MINES FOUND!"
        return


def read_from_file():
    gap_info.clear()
    angle_data.clear(); dist_data.clear()
    ir_data.clear();    ping_data.clear()
    detected_objects.clear()
    bump_objects.clear()
    _pending_active_mines.clear()
    _last_angle[0] = -1
    _radar_sweeps_since_clear[0] = 0
    _obj_table_scroll[0] = 0
    cybot_x[0] = 0.0; cybot_y[0] = 0.0; cybot_hdg[0] = 90.0
    cybot_draw_x[0] = 0.0; cybot_draw_y[0] = 0.0; cybot_draw_hdg[0] = 90.0

    if not os.path.isfile(full_path):
        status_msg[0] = "FILE NOT FOUND"
        return

    status_msg[0] = "LOADING…"

    with open(full_path) as f:
        for line in f:
            parse_and_store(line)

    scan_count[0] += 1
    mines_a = sum(1 for o in detected_objects if o["is_mine"] and o.get("state") == "active")
    mines_d = sum(1 for o in detected_objects if o.get("state") == "demined")
    status_msg[0] = (
        f"SCAN #{scan_count[0]}  —  {len(angle_data)} pts  "
        f"|  {mines_a} mine(s)  |  {mines_d} demined  "
        f"|  {'GAP @'+str(gap_info['angle'])+'°' if gap_info else 'no gap'}"
    )


def init_socket():
    global cybot_socket, cybot, rx_thread
    cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cybot_socket.connect((HOST, PORT))
    cybot = cybot_socket
    if rx_thread is None or not rx_thread.is_alive():
        rx_running[0] = True
        rx_thread = threading.Thread(target=receive_loop, daemon=True)
        rx_thread.start()


def receive_loop():
    global stop_flag
    pending = ""
    try:
        with open(full_path, "w", buffering=1) as f:
            while rx_running[0]:
                chunk = cybot_socket.recv(4096)
                if not chunk:
                    status_msg[0] = "DISCONNECTED"
                    break

                pending += chunk.decode(errors="replace")
                lines = pending.splitlines(keepends=True)
                pending = ""
                if lines and not lines[-1].endswith(("\n", "\r")):
                    pending = lines.pop()

                for line in lines:
                    print(line, end="", flush=True)
                    f.write(line)
                    parse_and_store(line)

                    try:
                        fig.canvas.draw_idle()
                    except Exception:
                        pass

                    ls = line.strip().lower()
                    if "all mines" in ls or "mines found" in ls:
                        status_msg[0] = "ALL MINES FOUND!"
    except Exception as e:
        status_msg[0] = f"ERR: {e}"


gui_mode = ["IDLE"]
_held_key   = [None]
_move_timer = [None]
KEY_MAP = {"w": "w", "a": "a", "s": "s", "d": "d"}


def send_byte(ch):
    try:
        if cybot is None:
            init_socket()
        cybot_socket.sendall(ch.encode())
    except Exception as e:
        status_msg[0] = f"ERR: {e}"


def on_manual(event):
    gui_mode[0] = "MANUAL"
    status_msg[0] = "MANUAL MODE  [m]"
    if CHOICE == "live":
        try:
            if cybot is None:
                init_socket()
            send_byte("m")
        except Exception as e:
            status_msg[0] = f"CONNECT ERR: {e}"


def on_auto(event):
    gui_mode[0] = "AUTO"
    status_msg[0] = "AUTO ARMED  →  press START [z]"
    if CHOICE == "live":
        try:
            if cybot is None:
                init_socket()
            send_byte("n")
        except Exception as e:
            status_msg[0] = f"CONNECT ERR: {e}"


def on_start(event):
    if gui_mode[0] != "AUTO":
        status_msg[0] = "⚠  Switch to AUTO first  [n]"
        return
    gui_mode[0] = "RUNNING"
    status_msg[0] = "AUTO RUNNING  [z]"
    if CHOICE == "live":
        send_byte("z")


def on_scan(event):
    global stop_flag
    if CHOICE == "file":
        read_from_file()
        return
    if gui_mode[0] != "MANUAL":
        status_msg[0] = "⚠  Must be in MANUAL mode to scan"
        return
    stop_flag = False
    send_byte("i")
    status_msg[0] = "SCANNING…  [i]"


def on_stop(event):
    global stop_flag
    stop_flag = True
    gui_mode[0] = "STOPPED"
    status_msg[0] = "STOPPED  [x]"
    if CHOICE == "live":
        try:
            if cybot:
                send_byte("x")
        except Exception:
            pass

def on_clear(event):
    _clear_scan_scene()
    cybot_x[0] = 0; cybot_y[0] = 0; cybot_hdg[0] = 90
    cybot_draw_x[0] = 0; cybot_draw_y[0] = 0; cybot_draw_hdg[0] = 90
    _last_angle[0] = -1
    gui_mode[0] = "IDLE"
    status_msg[0] = "CLEARED"

def _repeat_move():
    if _held_key[0] and CHOICE == "live" and gui_mode[0] == "MANUAL":
        send_byte(_held_key[0])
        _move_timer[0] = threading.Timer(0.20, _repeat_move)
        _move_timer[0].start()

def _redraw_wasd():
    try:
        update(None)
        fig.canvas.draw_idle()
    except Exception:
        pass

def on_key_press(event):
    k = event.key
    if k in KEY_MAP:
        byte = KEY_MAP[k]
        if _held_key[0] != byte:
            _held_key[0] = byte
            if _move_timer[0]:
                _move_timer[0].cancel()
            if gui_mode[0] == "MANUAL" and CHOICE == "live":
                send_byte(byte)
                _move_timer[0] = threading.Timer(0.20, _repeat_move)
                _move_timer[0].start()
            label = {"w": "FWD", "a": "LEFT", "s": "BACK", "d": "RIGHT"}[byte]
            status_msg[0] = f"MOVE  {label}  [{k.upper()}]"
            _redraw_wasd()


def on_key_release(event):
    if event.key in KEY_MAP:
        _held_key[0] = None
        if _move_timer[0]:
            _move_timer[0].cancel()
        _redraw_wasd()

def on_scroll(event):
    if event.inaxes is not ax_obj:
        return
    max_scroll = max(0, _obj_table_rows[0] - 7)
    if event.step > 0:
        _obj_table_scroll[0] = min(max_scroll, _obj_table_scroll[0] + 1)
    else:
        _obj_table_scroll[0] = max(0, _obj_table_scroll[0] - 1)
    fig.canvas.draw_idle()

matplotlib.rcParams.update({
    "figure.facecolor": BG,
    "axes.facecolor":   PANEL,
    "text.color":       TEXT,
    "axes.labelcolor":  MUTED,
    "xtick.color":      MUTED,
    "ytick.color":      MUTED,
    "axes.edgecolor":   BORDER,
    "grid.color":       GRID_COLOR,
    "grid.linewidth":   0.5,
    "font.family":      "monospace",
    "keymap.save":      [],
})

fig = plt.figure(figsize=(14, 8), facecolor=BG)
fig.canvas.manager.set_window_title("CyBot — Sensor Dashboard")

outer = gridspec.GridSpec(
    1, 3, figure=fig,
    left=0.01, right=0.99, top=0.93, bottom=0.13,
    wspace=0.03, width_ratios=[1.6, 3.2, 1.6],
)
left_gs  = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=outer[0], hspace=0.06, height_ratios=[1, 1.4])
right_gs = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=outer[2], hspace=0.06, height_ratios=[1, 1.4])

ax_ir    = fig.add_subplot(left_gs[0])
ax_obj   = fig.add_subplot(left_gs[1])
ax_scan  = fig.add_subplot(outer[1])
ax_pos   = fig.add_subplot(right_gs[0])
ax_radar = fig.add_subplot(right_gs[1], projection="polar")

ax_wasd = fig.add_axes([0.757, 0.105, 0.235, 0.115])
ax_wasd.set_facecolor(BG)
ax_wasd.set_xlim(0, 1); ax_wasd.set_ylim(0, 1)
ax_wasd.set_xticks([]); ax_wasd.set_yticks([])
for sp in ax_wasd.spines.values():
    sp.set_edgecolor(BORDER); sp.set_linewidth(0.6)

def panel_title(ax, title, subtitle=""):
    ax.text(0.01, 1.045, title,
            transform=ax.transAxes, fontsize=9, fontweight="bold",
            color=ACCENT, va="bottom", fontfamily="monospace",
            path_effects=[pe.withStroke(linewidth=3, foreground=BG)])
    if subtitle:
        ax.text(0.99, 1.045, subtitle,
                transform=ax.transAxes, fontsize=7.5,
                color=MUTED, va="bottom", ha="right", fontfamily="monospace")


def style_axis(ax):
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
        sp.set_linewidth(0.8)
    ax.set_facecolor(PANEL)
    ax.tick_params(labelsize=7.5, length=3, width=0.5, color=BORDER)


def _obj_color(obj):
    state = obj.get("state", "active")
    if state == "demined":  return MINE_DEMINED
    if state in ("bump", "object"): return OBJ_COLOR
    if obj.get("is_mine", False): return MINE_ACTIVE
    return OBJ_COLOR


BTNS = [
    ("MANUAL  [m]",  0.05, WARN,      "#6b4810",  on_manual),
    ("SCAN  [i]",    0.19, ACCENT,    ACCENT_DIM, on_scan),
    ("AUTO  [n]",    0.38, INFO,      "#1e3a6b",  on_auto),
    ("START  [z]",   0.52, "#a0f040", "#3a5810",  on_start),
    ("STOP  [x]",    0.68, DANGER,    "#6b2020",  on_stop),
    ("CLEAR",        0.84, MUTED,     "#2a2e38",  on_clear),
]
btn_objects = []
for label, x, fg, hover, cb in BTNS:
    ax_btn = fig.add_axes([x, 0.02, 0.12, 0.07])
    ax_btn.set_facecolor(BG)
    for sp in ax_btn.spines.values():
        sp.set_edgecolor(fg); sp.set_linewidth(1.2)
    b = Button(ax_btn, label, color=BG, hovercolor=hover)
    b.label.set_color(fg); b.label.set_fontsize(7.5)
    b.label.set_fontweight("bold"); b.label.set_fontfamily("monospace")
    b.on_clicked(cb)
    btn_objects.append(b)

fig.text(0.5, 0.005,
         "move keys (manual mode):  W = fwd   A = left   S = back   D = right"
         "   |   i = scan   z = start   x = stop",
         ha="center", va="bottom", fontsize=6.5, color=MUTED, fontfamily="monospace")

fig.canvas.mpl_connect("key_press_event",   on_key_press)
fig.canvas.mpl_connect("key_release_event", on_key_release)
fig.canvas.mpl_connect("scroll_event",      on_scroll)

fig.text(0.5, 0.965, "◈  CYBOT  SENSOR  DASHBOARD  ◈",
         ha="center", va="center", fontsize=11, fontweight="bold",
         color=ACCENT, fontfamily="monospace",
         path_effects=[pe.withStroke(linewidth=4, foreground=BG)])
status_text = fig.text(0.5, 0.948, "IDLE",
                        ha="center", va="center", fontsize=8,
                        color=MUTED, fontfamily="monospace")
fig.add_artist(plt.Line2D([0.01, 0.99], [0.935, 0.935],
                           transform=fig.transFigure,
                           color=BORDER, linewidth=0.8))


def update(frame):
    status_text.set_text(status_msg[0])

    _smooth_cybot_pose()

    ax_scan.cla()
    style_axis(ax_scan)
    cx_bot, cy_bot = cybot_draw_x[0], cybot_draw_y[0]
    view_x_min = cx_bot - MAP_HALF_WIDTH_CM
    view_x_max = cx_bot + MAP_HALF_WIDTH_CM
    view_y_min = cy_bot - MAP_BACK_CM
    view_y_max = cy_bot + MAP_FORWARD_CM
    ax_scan.set_xlim(view_x_min, view_x_max)
    ax_scan.set_ylim(view_y_min, view_y_max)
    ax_scan.set_aspect("equal")
    ax_scan.grid(True, color=GRID_COLOR, lw=0.5)
    ax_scan.set_xticks(np.arange(math.floor(view_x_min / GRID_STEP_CM) * GRID_STEP_CM,
                                 math.ceil(view_x_max / GRID_STEP_CM) * GRID_STEP_CM + 1,
                                 GRID_STEP_CM))
    ax_scan.set_yticks(np.arange(math.floor(view_y_min / GRID_STEP_CM) * GRID_STEP_CM,
                                 math.ceil(view_y_max / GRID_STEP_CM) * GRID_STEP_CM + 1,
                                 GRID_STEP_CM))
    ax_scan.tick_params(labelsize=7)
    ax_scan.set_xlabel("X  (cm)", fontsize=8, color=MUTED)
    ax_scan.set_ylabel("Y  (cm)", fontsize=8, color=MUTED, labelpad=4)

    ax_scan.axhline(0, color=BORDER, lw=0.45, alpha=0.55)
    ax_scan.axvline(0, color=BORDER, lw=0.45, alpha=0.55)
    for r in [GRID_STEP_CM * n for n in (2, 4, 6, 8, 10, 12, 14, 16)]:
        ax_scan.add_patch(
            plt.Circle((cx_bot, cy_bot), r, color=GRID_COLOR, fill=False, lw=0.5, ls="--"))
        ax_scan.text(cx_bot + r * 0.707 + 1, cy_bot + r * 0.707 + 1, str(r),
                     fontsize=6.5, color=MUTED, fontfamily="monospace")

    offset = 3
    for obj in detected_objects:
        state = obj.get("state", "active")
        col   = _obj_color(obj)

        if state == "bump":
            bx = obj.get("bx", cybot_x[0])
            by = obj.get("by", cybot_y[0])
            ax_scan.scatter([bx], [by], marker="X", color=col, s=50,
                            linewidths=0.8, edgecolors=BG, zorder=4)
            ax_scan.text(bx+2, by+2, obj["label"], fontsize=5.5, color=col,
                         fontfamily="monospace", zorder=5,
                         path_effects=[pe.withStroke(linewidth=1.5, foreground=BG)])
            ax_scan.add_patch(plt.Circle((bx, by), 3, color=col, fill=False,
                              lw=0.8, ls="--", zorder=3, alpha=0.5))
        else:
            rad = math.radians(obj["mid_angle"])
            ox  = obj.get("wx", obj["distance"] * math.cos(rad))
            oy  = obj.get("wy", obj["distance"] * math.sin(rad))
            dot_offsets = [0]
            ax_scan.scatter([ox] * len(dot_offsets), [oy + dy for dy in dot_offsets],
                            marker='o', color=col, s=20, linewidths=0, zorder=3)
            ax_scan.text(ox+2, oy+max(dot_offsets)+2, obj["label"], fontsize=5.5, color=col,
                         fontfamily="monospace", zorder=5,
                         path_effects=[pe.withStroke(linewidth=1.5, foreground=BG)])
            if state == "demined":
                ax_scan.add_patch(plt.Circle((ox,oy), 4, color=MINE_DEMINED,
                                  fill=False, lw=1.2, ls="-", zorder=4, alpha=0.85))
                ax_scan.text(ox+2, oy-offset-5, "DEMINED", fontsize=5,
                             color=MINE_DEMINED, fontfamily="monospace", zorder=5,
                             fontweight="bold",
                             path_effects=[pe.withStroke(linewidth=1.5, foreground=BG)])
            elif state == "active" and obj.get("is_mine"):
                ax_scan.add_patch(plt.Circle((ox,oy), 4, color=MINE_ACTIVE,
                                  fill=False, lw=0.8, ls="--", zorder=4, alpha=0.5))
            elif state == "object":
                ax_scan.add_patch(plt.Circle((ox,oy), 3, color=OBJ_COLOR,
                                  fill=False, lw=0.8, ls="--", zorder=4, alpha=0.5))

    for bo in bump_objects:
        bx, by = bo["x"], bo["y"]
        ax_scan.scatter([bx], [by], marker="X", color=OBJ_COLOR, s=55,
                        linewidths=1.0, edgecolors=BG, zorder=6)
        ax_scan.text(bx+2, by+3, bo["label"], fontsize=5.5, color=OBJ_COLOR,
                     fontfamily="monospace", zorder=6,
                     path_effects=[pe.withStroke(linewidth=1.5, foreground=BG)])

    if gap_info:
        g_world_angle = cybot_draw_hdg[0] + (gap_info["angle"] - 90.0)
        g_rad = math.radians(g_world_angle)
        gx = cx_bot + MAP_FORWARD_CM * 0.88 * math.cos(g_rad)
        gy = cy_bot + MAP_FORWARD_CM * 0.88 * math.sin(g_rad)
        ax_scan.annotate("", xy=(gx,gy), xytext=(cx_bot,cy_bot),
                         arrowprops=dict(arrowstyle="->", color=INFO, lw=1.2, alpha=0.75))
        ax_scan.text(gx+1.5, gy+1.5, f"GAP\n{gap_info['angle']}°",
                     fontsize=5.5, color=INFO, fontfamily="monospace", zorder=5)

    bot_heading = math.radians(cybot_draw_hdg[0])
    nose_x = cx_bot + CYBOT_RADIUS_CM * 1.45 * math.cos(bot_heading)
    nose_y = cy_bot + CYBOT_RADIUS_CM * 1.45 * math.sin(bot_heading)
    ax_scan.add_patch(plt.Circle((cx_bot, cy_bot), CYBOT_RADIUS_CM,
                      facecolor=PANEL, edgecolor=ROBOT_COLOR,
                      lw=1.4, zorder=6))
    ax_scan.plot([cx_bot, nose_x], [cy_bot, nose_y],
                 color=ROBOT_COLOR, lw=1.5, zorder=7)
    ax_scan.scatter([nose_x], [nose_y], color=ROBOT_COLOR, s=14,
                    linewidths=0, zorder=7)
    ax_scan.text(cx_bot, cy_bot - CYBOT_RADIUS_CM - 3, "CYBOT", fontsize=6,
                 color=ROBOT_COLOR, fontfamily="monospace", zorder=7,
                 ha="center")

    legend_items = [
        (MINE_ACTIVE,  "Active mine"),
        (MINE_DEMINED, "Demined"),
        (OBJ_COLOR,    "Obstacle"),
    ]
    for li, (lc, ll) in enumerate(legend_items):
        lx_leg = view_x_min + 1
        ly_leg = view_y_min + 4 + li * 8
        ax_scan.scatter([lx_leg+2.5], [ly_leg], color=lc, s=22, zorder=7, linewidths=0)
        ax_scan.text(lx_leg+6, ly_leg, ll, fontsize=5.5, color=lc,
                     fontfamily="monospace", va="center", zorder=7)

    active_n  = sum(1 for o in detected_objects if o["is_mine"] and o.get("state") == "active")
    demined_n = sum(1 for o in detected_objects if o.get("state") == "demined")
    panel_title(ax_scan, "SCAN  MAP",
                f"{len(angle_data)} pts  |  {active_n} mine(s)  "
                f"|  {demined_n} demined  |  {len(bump_objects)} obj  |  grid {GRID_STEP_CM}:1")

    ax_ir.cla()
    style_axis(ax_ir)
    N_BARS = 45
    if ir_data and angle_data:
        step    = max(1, len(ir_data) // N_BARS)
        a_vals  = angle_data[::step][:N_BARS]
        ir_vals = ir_data[::step][:len(a_vals)]
        pg_vals = ping_data[::step][:len(a_vals)]
        xs = range(len(a_vals))
        ax_ir.bar(xs, pg_vals, color=WARN,   width=0.85, alpha=0.55, linewidth=0, label="Ping")
        ax_ir.bar(xs, ir_vals, color=ACCENT, width=0.85, alpha=0.80, linewidth=0, label="IR")
        ax_ir.set_xlim(-0.5, max(N_BARS-0.5, len(a_vals)-0.5))
        ts = max(1, len(a_vals) // 6)
        ax_ir.set_xticks(range(0, len(a_vals), ts))
        ax_ir.set_xticklabels([f"{a_vals[i]}°" for i in range(0, len(a_vals), ts)], fontsize=6.5)
        top = max(max(ir_vals, default=0), max(pg_vals, default=0))
        ax_ir.set_ylim(0, top * 1.25 if top > 0 else 5)
        ax_ir.legend(fontsize=6, loc="upper right",
                     facecolor=PANEL, edgecolor=BORDER, labelcolor=TEXT)
    else:
        ax_ir.text(0.5, 0.5, "NO DATA", ha="center", va="center",
                   fontsize=8, color=MUTED, transform=ax_ir.transAxes,
                   fontfamily="monospace")
        ax_ir.set_xticks([])
    ax_ir.set_yticks([])
    ax_ir.spines["top"].set_visible(False)
    ax_ir.spines["right"].set_visible(False)
    panel_title(ax_ir, "IR  vs  PING", "dist (cm) vs angle")

    ax_obj.cla()
    style_axis(ax_obj)
    ax_obj.set_xlim(0,1); ax_obj.set_ylim(0,1)
    ax_obj.set_xticks([]); ax_obj.set_yticks([])
    ax_obj.spines["top"].set_visible(False)
    ax_obj.spines["right"].set_visible(False)
    active_n  = sum(1 for o in detected_objects if o["is_mine"] and o.get("state") == "active")
    demined_n = sum(1 for o in detected_objects if o.get("state") == "demined")
    object_n  = sum(1 for o in detected_objects if not o["is_mine"] and o.get("state") != "bump")
    panel_title(ax_obj, "MINES / OBJECTS",
                f"{active_n} active  {demined_n} demined  {object_n} object(s)")

    COL_X   = [0.02, 0.32, 0.58, 0.80]
    COL_LBL = ["NAME", "DIST", "ANGLE", "WIDTH"]
    for cx_c, cl in zip(COL_X, COL_LBL):
        ax_obj.text(cx_c, 0.92, cl, fontsize=6.5, color=MUTED,
                    fontfamily="monospace", va="top", fontweight="bold")
    ax_obj.axhline(0.88, color=BORDER, lw=0.8, xmin=0.01, xmax=0.99)

    rows = []
    for obj in reversed(detected_objects):
        state = obj.get("state", "active")
        badge = " ✓" if state=="demined" else (" !" if state=="bump" else "")
        rows.append({
            "cols": [
                obj["label"] + badge,
                f"{obj['distance']:.1f}" if obj["distance"] > 0 else "—",
                f"{obj['mid_angle']:.0f}°",
                f"{obj['lin_width']:.1f}" if obj["lin_width"] > 0 else "—",
            ],
            "color": _obj_color(obj),
        })

    if gap_info:
        rows.insert(0, {
            "cols": ["GAP", "—", f"{gap_info['angle']}°", f"{gap_info['width']:.1f}"],
            "color": INFO,
        })

    _obj_table_rows[0] = len(rows)
    _obj_table_scroll[0] = min(_obj_table_scroll[0], max(0, len(rows) - 7))

    if not rows:
        ax_obj.text(0.5, 0.50, "—  no detections  —",
                    ha="center", va="center", fontsize=8, color=MUTED,
                    fontfamily="monospace")
    else:
        CONTENT_TOP = 0.84
        LEGEND_BOT  = 0.11
        MAX_ROWS    = 7
        available   = CONTENT_TOP - LEGEND_BOT
        visible_rows = rows[_obj_table_scroll[0]:_obj_table_scroll[0] + MAX_ROWS]
        row_h       = min(available / max(len(visible_rows), 1), 0.115)
        for idx, row in enumerate(visible_rows):
            y_row = CONTENT_TOP - idx * row_h - row_h * 0.15
            col = row["color"]
            for cx_c, val in zip(COL_X, row["cols"]):
                ax_obj.text(cx_c, y_row, val, fontsize=7.5, color=col,
                            fontfamily="monospace", va="top", clip_on=True)
            if idx < len(visible_rows) - 1:
                ax_obj.axhline(y_row - row_h*0.82, color=BORDER, lw=0.3,
                               xmin=0.01, xmax=0.99)
        if len(rows) > MAX_ROWS:
            start_n = _obj_table_scroll[0] + 1
            end_n   = _obj_table_scroll[0] + len(visible_rows)
            ax_obj.text(0.98, 0.885, f"{start_n}-{end_n}/{len(rows)}",
                        fontsize=5.5, color=MUTED, fontfamily="monospace",
                        va="bottom", ha="right")

    ax_obj.axhline(0.10, color=BORDER, lw=0.5, xmin=0.01, xmax=0.99)
    for li, (lc, ll) in enumerate([(MINE_ACTIVE,"active"),(MINE_DEMINED,"demined ✓"),(OBJ_COLOR,"obstacle !")]):
        ax_obj.text(COL_X[li], 0.055, ll, fontsize=5.5, color=lc,
                    fontfamily="monospace", va="bottom")

    ax_pos.cla()
    style_axis(ax_pos)
    ax_pos.set_xticks([]); ax_pos.set_yticks([])
    ax_pos.spines["top"].set_visible(False)
    ax_pos.spines["right"].set_visible(False)
    ax_pos.set_xlim(0,1); ax_pos.set_ylim(0,1)
    panel_title(ax_pos, "CYBOT  INFO")

    active_mines_n  = sum(1 for o in detected_objects if o["is_mine"] and o.get("state")=="active")
    demined_mines_n = sum(1 for o in detected_objects if o.get("state")=="demined")
    fields = [
        ("MODE",      gui_mode[0]),
        ("SCAN PTS",  str(len(angle_data))),
        ("MINES",     f"{active_mines_n} active"),
        ("DEMINED",   str(demined_mines_n)),
        ("OBSTACLES", str(len(bump_objects))),
        ("GAP",       f"{gap_info['angle']}° / {gap_info['width']:.1f}cm"
                       if gap_info else "—"),
    ]
    for i, (lbl, val) in enumerate(fields):
        y_f = 0.88 - i * 0.140
        ax_pos.text(0.03, y_f, lbl, fontsize=7.5, color=MUTED,
                    fontfamily="monospace", va="center")
        ax_pos.text(0.97, y_f, val, fontsize=8, color=TEXT,
                    fontfamily="monospace", va="center", ha="right", fontweight="bold")
        if i < len(fields) - 1:
            ax_pos.axhline(y_f - 0.063, color=BORDER, lw=0.4, xmin=0.02, xmax=0.98)

    ax_radar.cla()
    ax_radar.set_facecolor(PANEL)
    ax_radar.set_thetamin(0)
    ax_radar.set_thetamax(180)
    ax_radar.set_theta_zero_location("E")
    ax_radar.set_theta_direction(1)
    ax_radar.set_rmax(30)
    ax_radar.grid(True, color=GRID_COLOR, linewidth=0.5)
    ax_radar.set_yticklabels([])
    ax_radar.tick_params(colors=MUTED, labelsize=6.5)
    ax_radar.set_xticks(np.linspace(0, np.pi, 7))
    ax_radar.set_xticklabels(["0°","30°","60°","90°","120°","150°","180°"],
                              fontsize=6.5, color=MUTED, fontfamily="monospace")
    ax_radar.spines["polar"].set_edgecolor(BORDER)
    ax_radar.spines["polar"].set_linewidth(0.8)

    if angle_data and dist_data:
        sweep_ranges = []
        sweep_start = 0
        for i in range(1, min(len(angle_data), len(dist_data))):
            if angle_data[i] < angle_data[i - 1]:
                sweep_ranges.append((sweep_start, i))
                sweep_start = i
        sweep_ranges.append((sweep_start, min(len(angle_data), len(dist_data))))

        for sweep_i, (start, end) in enumerate(sweep_ranges):
            if end - start < 2:
                continue
            is_latest_sweep = sweep_i == len(sweep_ranges) - 1
            rads  = np.radians(angle_data[start:end])
            dists = np.array(dist_data[start:end], dtype=float)
            ax_radar.plot(rads, dists,
                          color=ACCENT if is_latest_sweep else INFO,
                          linewidth=1.8 if is_latest_sweep else 1.2,
                          alpha=0.9 if is_latest_sweep else 0.35,
                          zorder=3 if is_latest_sweep else 2)
        for obj in detected_objects:
            if obj.get("state") == "bump":
                continue
            col = _obj_color(obj)
            if obj["is_mine"]:
                marker = "o" if obj.get("state") == "demined" else "x"
                ax_radar.plot(math.radians(obj["mid_angle"]),
                              min(obj["distance"], 30),
                              marker, color=col, ms=6, mew=1.4, zorder=5)
            else:
                ax_radar.plot(math.radians(obj["mid_angle"]),
                              min(obj["distance"], 30),
                              "o", color=col, ms=5, mew=0, zorder=5)

    panel_title(ax_radar, "RADAR  VIEW", "polar  0°–180°")

    ax_wasd.cla()
    ax_wasd.set_facecolor(BG)
    ax_wasd.set_xlim(0,1); ax_wasd.set_ylim(0,1)
    ax_wasd.set_xticks([]); ax_wasd.set_yticks([])
    for sp in ax_wasd.spines.values():
        sp.set_edgecolor(BORDER); sp.set_linewidth(0.6)

    in_manual = (gui_mode[0] == "MANUAL")
    held      = _held_key[0]

    KEY_POS = {
        "w": (0.50, 0.62, "W", "FWD"),
        "a": (0.18, 0.18, "A", "LEFT"),
        "s": (0.50, 0.18, "S", "BACK"),
        "d": (0.82, 0.18, "D", "RIGHT"),
    }

    for byte, (kx, ky, lbl, hint) in KEY_POS.items():
        is_held = (held == byte)
        key_fg  = ACCENT     if is_held else (MUTED  if in_manual else BORDER)
        key_bg  = ACCENT_DIM if is_held else PANEL
        box_ec  = ACCENT     if is_held else (BORDER if in_manual else "#0f1015")
        lw      = 1.5        if is_held else 0.8
        box = plt.matplotlib.patches.FancyBboxPatch(
            (kx-0.10, ky-0.13), 0.20, 0.26,
            boxstyle="round,pad=0.015", linewidth=lw,
            edgecolor=box_ec, facecolor=key_bg,
            transform=ax_wasd.transData, zorder=3,
        )
        ax_wasd.add_patch(box)
        ax_wasd.text(kx, ky+0.04, lbl, ha="center", va="center", fontsize=10,
                     fontweight="bold", fontfamily="monospace", color=key_fg, zorder=4)
        ax_wasd.text(kx, ky-0.06, hint, ha="center", va="center", fontsize=5,
                     fontfamily="monospace",
                     color=key_fg if is_held else BORDER, zorder=4)

    mode_color = ACCENT if in_manual else MUTED
    mode_label = ("MANUAL" if in_manual else ("AUTO" if gui_mode[0]=="AUTO" else gui_mode[0]))
    ax_wasd.text(0.5, 0.96, f"MOVE  KEYS  —  {mode_label}",
                 ha="center", va="top", fontsize=6.5,
                 color=mode_color, fontfamily="monospace", fontweight="bold")


ani = FuncAnimation(fig, update, interval=100,
                    cache_frame_data=False, save_count=300)

if CHOICE == "file":
    read_from_file()

plt.show()