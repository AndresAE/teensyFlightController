import serial
import time
import re
from collections import deque
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


# ============================================================
# SERIAL STREAM
# ============================================================

def open_serial(port="COM5", baud=115200):
    ser = serial.Serial(port, baud, timeout=0)
    return ser


# ============================================================
# PARSER
# ============================================================

def parse_group_line(line):
    if ":" not in line:
        return None, {}

    group, rest = line.split(":", 1)
    group = group.strip()

    pairs = re.split(r",\s*", rest.strip())
    data = {}

    for p in pairs:
        if "=" not in p:
            continue
        key, val = p.split("=", 1)
        key = key.strip()
        val = val.strip()

        try:
            if "." in val:
                val = float(val)
            else:
                val = int(val)
        except ValueError:
            pass

        data[key] = val

    return group, data


# ============================================================
# READ 5-LINE TELEMETRY BLOCK
# ============================================================

def read_lines_from_serial(ser):
    lines = []
    # read all available lines (non-blocking because timeout=0)
    while True:
        raw = ser.readline()
        if not raw:
            break
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        if line:
            lines.append((time.time(), line))
    return lines


def read_message_block_from_lines(lines):
    # single telemetry block may span multiple groups; return one block per 5-group block
    blocks = []
    block = {}
    count = 0
    for t, line in lines:
        group, data = parse_group_line(line)
        if group:
            block[group] = data
            block["timestamp"] = t
            count += 1
        if count >= 5:
            blocks.append(block)
            block = {}
            count = 0
    return blocks


# ============================================================
# HISTORY BUFFERS (ROLLING)
# ============================================================

MAX_POINTS = 2000

# numeric rolling deques (fast, preallocated behavior)
history = {
    "t": deque(maxlen=MAX_POINTS),
    "ax": deque(maxlen=MAX_POINTS),
    "ay": deque(maxlen=MAX_POINTS),
    "az": deque(maxlen=MAX_POINTS),
    "roll": deque(maxlen=MAX_POINTS),
    "pitch": deque(maxlen=MAX_POINTS),
    "yaw": deque(maxlen=MAX_POINTS),
    "pb": deque(maxlen=MAX_POINTS),
    "ny": deque(maxlen=MAX_POINTS),
    "nz": deque(maxlen=MAX_POINTS),
    "throttle": deque(maxlen=MAX_POINTS),
    "lat": deque(maxlen=MAX_POINTS),
    "lon": deque(maxlen=MAX_POINTS),
}


def append_block(block):
    t = block.get("timestamp", time.time())
    imu = block.get("imu", {})
    gdc = block.get("gdc", {})
    gps = block.get("gps", {})

    history["t"].append(t)
    history["ax"].append(imu.get("ax_fps2", np.nan))
    history["ay"].append(imu.get("ay_fps2", np.nan))
    history["az"].append(imu.get("az_fps2", np.nan))
    history["roll"].append(imu.get("roll_d", np.nan))
    history["pitch"].append(imu.get("pitch_d", np.nan))
    history["yaw"].append(imu.get("yaw_d", np.nan))
    history["pb"].append(gdc.get("pbCmd_dps", np.nan))
    history["ny"].append(gdc.get("nyCmd_B_fps2", np.nan))
    history["nz"].append(gdc.get("nzCmd_B_fps2", np.nan))
    history["throttle"].append(gdc.get("throttleCmd_pct", np.nan))
    history["lat"].append(gps.get("latG_d", np.nan))
    history["lon"].append(gps.get("lon_d", np.nan))


# ============================================================
# REAL-TIME PLOT SETUP
# ============================================================

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Telemetry")
win.resize(1000, 800)

ax_imu = win.addPlot(row=0, col=0, title="IMU Accelerations (fps²)")
ax_imu.showGrid(x=True, y=True)
ax_imu.addLegend()
# distinct colors for ax, ay, az
imu_colors = ("r", "g", "b")
imu_lines = [ax_imu.plot(pen=pg.mkPen(color=c, width=1), name=l) for l, c in zip(("ax", "ay", "az"), imu_colors)]

ax_euler = win.addPlot(row=1, col=0, title="Euler Angles")
ax_euler.showGrid(x=True, y=True)
ax_euler.addLegend()
# distinct colors for roll, pitch, yaw
euler_colors = ("m", "y", "c")
euler_lines = [ax_euler.plot(pen=pg.mkPen(color=c, width=1), name=l) for l, c in zip(("roll", "pitch", "yaw"), euler_colors)]

ax_gdc = win.addPlot(row=2, col=0, title="GDC Commands")
ax_gdc.showGrid(x=True, y=True)
ax_gdc.addLegend()
# distinct colors for gdc lines: pb, ny, nz, throttle
gdc_colors = ("#d62728", "#ff7f0e", "#2ca02c", "#9467bd")
gdc_lines = [ax_gdc.plot(pen=pg.mkPen(color=c, width=1), name=l) for l, c in zip(("pbCmd_dps", "nyCmd", "nzCmd", "throttle"), gdc_colors)]

ax_gps = win.addPlot(row=3, col=0, title="GPS Track")
ax_gps.showGrid(x=True, y=True)
gps_scatter = pg.ScatterPlotItem(size=6, brush=pg.mkBrush(100, 100, 255, 150))
ax_gps.addItem(gps_scatter)

frame_count = 0
Y_RECALC_PERIOD = 10  # recalc y-limits every N frames
TIME_WINDOW_S = 10.0  # fixed time window (seconds) to show


def update():
    global frame_count
    # read all available lines from serial
    lines = read_lines_from_serial(stream_ser)
    if not lines:
        return
    blocks = read_message_block_from_lines(lines)
    for b in blocks:
        append_block(b)

    t = np.array(history["t"])
    if t.size == 0:
        return

    t0 = t[-1] - TIME_WINDOW_S

    ax_vals = np.array(history["ax"])
    ay_vals = np.array(history["ay"])
    az_vals = np.array(history["az"])

    # update curves (pyqtgraph expects x,y arrays)
    for line, vals in zip(imu_lines, (ax_vals, ay_vals, az_vals)):
        line.setData(t, vals)
    ax_imu.setXRange(max(t[0], t0), t[-1], padding=0)

    roll = np.array(history["roll"])
    pitch = np.array(history["pitch"])
    yaw = np.array(history["yaw"])
    for line, vals in zip(euler_lines, (roll, pitch, yaw)):
        line.setData(t, vals)
    ax_euler.setXRange(max(t[0], t0), t[-1], padding=0)

    pb = np.array(history["pb"])
    ny = np.array(history["ny"])
    nz = np.array(history["nz"])
    throttle = np.array(history["throttle"])
    for line, vals in zip(gdc_lines, (pb, ny, nz, throttle)):
        line.setData(t, vals)
    ax_gdc.setXRange(max(t[0], t0), t[-1], padding=0)

    lat = np.array(history["lat"])
    lon = np.array(history["lon"])
    if lat.size:
        pts = np.column_stack((lon, lat))
        spots = [{'pos': p} for p in pts]
        gps_scatter.setData(spots)

    # recalc y-limits periodically
    if frame_count % Y_RECALC_PERIOD == 0:
        for ax, arrays in ((ax_imu, (ax_vals, ay_vals, az_vals)),
                           (ax_euler, (roll, pitch, yaw)),
                           (ax_gdc, (pb, ny, nz, throttle))):
            stacked = np.vstack([a for a in arrays])
            if np.isfinite(stacked).any():
                vmin = np.nanmin(stacked)
                vmax = np.nanmax(stacked)
                if vmin == vmax:
                    vmin -= 0.5
                    vmax += 0.5
                ax.setYRange(vmin, vmax, padding=0)

    frame_count += 1


# ============================================================
# MAIN
# ============================================================

# open serial port used by update()
stream_ser = open_serial("COM6", 9600)

# use a Qt timer to call update periodically
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # ms

if __name__ == "__main__":
    app.exec_()
