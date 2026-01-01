import serial
import time
import re
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# SERIAL STREAM
# ============================================================

def serial_stream(port="COM5", baud=115200):
    ser = serial.Serial(port, baud, timeout=0.1)
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            yield time.time(), line


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

def read_message_block(stream):
    block = {}
    for _ in range(5):
        t, line = next(stream)
        group, data = parse_group_line(line)
        if group:
            block[group] = data
            block["timestamp"] = t
    return block


# ============================================================
# HISTORY BUFFERS (ROLLING)
# ============================================================

MAX_POINTS = 500

history = {
    "t": [],
    "imu": [],
    "gps": [],
    "gdc": [],
}

def append_block(block):
    history["t"].append(block["timestamp"])
    history["imu"].append(block.get("imu", {}))
    history["gps"].append(block.get("gps", {}))
    history["gdc"].append(block.get("gdc", {}))

    for k in history:
        if len(history[k]) > MAX_POINTS:
            history[k] = history[k][-MAX_POINTS:]


# ============================================================
# REAL-TIME PLOT SETUP
# ============================================================

fig, (ax_imu, ax_euler, ax_gdc, ax_gps) = plt.subplots(4, 1, figsize=(10, 14))


def update(frame):
    block = read_message_block(stream)
    append_block(block)

    t = history["t"]

    # ---------------- IMU STRIP CHART ----------------
    ax_vals = [d.get("ax_fps2") for d in history["imu"]]
    ay_vals = [d.get("ay_fps2") for d in history["imu"]]
    az_vals = [d.get("az_fps2") for d in history["imu"]]

    ax_imu.cla()
    ax_imu.plot(t, ax_vals, label="ax")
    ax_imu.plot(t, ay_vals, label="ay")
    ax_imu.plot(t, az_vals, label="az")
    ax_imu.set_title("IMU Accelerations (fps²)")
    ax_imu.set_ylabel("Accel")
    ax_imu.grid(True)
    ax_imu.legend()

    # ---------------- EULER ANGLES ----------------
    roll = [d.get("roll_d") for d in history["imu"]]
    pitch = [d.get("pitch_d") for d in history["imu"]]
    yaw = [d.get("yaw_d") for d in history["imu"]]

    ax_euler.cla()
    ax_euler.plot(t, roll, label="roll (deg)")
    ax_euler.plot(t, pitch, label="pitch (deg)")
    ax_euler.plot(t, yaw, label="yaw (deg)")
    ax_euler.set_title("Euler Angles")
    ax_euler.set_ylabel("Degrees")
    ax_euler.grid(True)
    ax_euler.legend()

    # ---------------- GDC COMMANDS ----------------
    pb = [d.get("pbCmd_dps") for d in history["gdc"]]
    ny = [d.get("nyCmd_B_fps2") for d in history["gdc"]]
    nz = [d.get("nzCmd_B_fps2") for d in history["gdc"]]
    throttle = [d.get("throttleCmd_pct") for d in history["gdc"]]

    ax_gdc.cla()
    ax_gdc.plot(t, pb, label="pbCmd_dps")
    ax_gdc.plot(t, ny, label="nyCmd_B_fps2")
    ax_gdc.plot(t, nz, label="nzCmd_B_fps2")
    ax_gdc.plot(t, throttle, label="throttleCmd_pct")
    ax_gdc.set_title("GDC Commands")
    ax_gdc.set_ylabel("Commands")
    ax_gdc.grid(True)
    ax_gdc.legend()

    # ---------------- GPS SCATTER ----------------
    lat = [d.get("latG_d") for d in history["gps"]]
    lon = [d.get("lon_d") for d in history["gps"]]

    ax_gps.cla()
    ax_gps.scatter(lon, lat, s=8, alpha=0.6)
    ax_gps.set_title("GPS Track")
    ax_gps.set_xlabel("Longitude")
    ax_gps.set_ylabel("Latitude")
    ax_gps.grid(True)

    return []


# ============================================================
# MAIN
# ============================================================

stream = serial_stream("COM6", 9600)

ani = FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
