import serial
import threading
import queue
import time
import struct
import traceback
from collections import deque
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


# ============================================================
# SERIAL STREAM
# ============================================================

def open_serial(port="COM5", baud=57600):
    ser = serial.Serial(port, baud, timeout=0)
    return ser


# ============================================================
# BINARY PACKET READER
# ============================================================

HEADER = b'\xAB\xCD'
STRUCT_FORMAT = "<d bb 9d 11d H"  # "<d bb 17d bb 11d H"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


packet_queue = queue.Queue()
serial_stop_event = threading.Event()


def serial_reader_thread(port, baud, q, stop_event):
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception:
        print(f"serial_reader_thread: could not open {port} @{baud}")
        return
    while not stop_event.is_set():
        try:
            # sync to header
            while True:
                b = ser.read(1)
                if not b:
                    break
                if b == HEADER[1:2]:
                    b2 = ser.read(1)
                    if b2 == HEADER[0:1]:
                        break
            if not b:
                continue
            # read payload
            payload = ser.read(STRUCT_SIZE)
            if len(payload) != STRUCT_SIZE:
                continue
            data_without_crc = payload[:-2]
            received_crc = int.from_bytes(payload[-2:], "little")
            computed_crc = crc16_ccitt(data_without_crc)
            if received_crc != computed_crc:
                continue
            pkt = struct.unpack(STRUCT_FORMAT, payload)
            q.put((time.time(), pkt))
        except Exception:
            continue
    try:
        ser.close()
    except Exception:
        pass


def drain_packet_queue():
    packets = []
    while True:
        try:
            item = packet_queue.get_nowait()
        except queue.Empty:
            break
        packets.append(item)
    return packets


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
    "gx": deque(maxlen=MAX_POINTS),
    "gy": deque(maxlen=MAX_POINTS),
    "gz": deque(maxlen=MAX_POINTS),
    "eff0": deque(maxlen=MAX_POINTS),
    "eff1": deque(maxlen=MAX_POINTS),
    "eff2": deque(maxlen=MAX_POINTS),
    "eff3": deque(maxlen=MAX_POINTS),
}


def append_packet(timestamp, pkt):
    # Unpack according to STRUCT_FORMAT: "<d bb 9d 11d H"
    # pkt: met_s, stp_ms, phs, ax, ay, az, gx, gy, gz, roll, pitch, yaw, [11 unused], crc
    met_s = pkt[0]
    stp_ms = pkt[1]
    phs = pkt[2]
    ax_fps2 = pkt[3]
    ay_fps2 = pkt[4]
    az_fps2 = pkt[5]
    gx_dps = pkt[6]
    gy_dps = pkt[7]
    gz_dps = pkt[8]
    roll_d = pkt[9]
    pitch_d = pkt[10]
    yaw_d = pkt[11]
    eff0 = pkt[19]
    eff1 = pkt[20]
    eff2 = pkt[21]
    eff3 = pkt[22]
    # pkt[12:23] are the 11 unused doubles
    # pkt[23] is the CRC

    history["t"].append(timestamp)
    history["ax"].append(ax_fps2)
    history["ay"].append(ay_fps2)
    history["az"].append(az_fps2)
    history["gx"].append(gx_dps)
    history["gy"].append(gy_dps)
    history["gz"].append(gz_dps)
    history["roll"].append(roll_d)
    history["pitch"].append(pitch_d)
    history["yaw"].append(yaw_d)
    history["eff0"].append(eff0)
    history["eff1"].append(eff1)
    history["eff2"].append(eff2)
    history["eff3"].append(eff3)

# ============================================================
# REAL-TIME PLOT SETUP
# ============================================================


app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Telemetry")
win.resize(1100, 900)

# Left column: Euler, Gyros, Accelerations (strip charts)
ax_euler = win.addPlot(row=0, col=0, title="Euler Angles (deg)")
ax_euler.showGrid(x=True, y=True)
ax_euler.addLegend()
euler_colors = ("r", "g", "b")
euler_lines = [ax_euler.plot(pen=pg.mkPen(color=c, width=1), name=l)
               for l, c in zip(("roll", "pitch", "yaw"), euler_colors)]

ax_gyro = win.addPlot(row=1, col=0, title="Gyros (deg/s)")
ax_gyro.showGrid(x=True, y=True)
ax_gyro.addLegend()
gyro_colors = ("r", "g", "b")
gyro_lines = [ax_gyro.plot(pen=pg.mkPen(color=c, width=1), name=l)
              for l, c in zip(("gx", "gy", "gz"), gyro_colors)]

ax_accel = win.addPlot(row=2, col=0, title="Accelerations (fps²)")
ax_accel.showGrid(x=True, y=True)
ax_accel.addLegend()
accel_colors = ("r", "g", "b")
accel_lines = [ax_accel.plot(pen=pg.mkPen(color=c, width=1), name=l)
               for l, c in zip(("ax", "ay", "az"), accel_colors)]

ax_eff = win.addPlot(row=3, col=0, title="Effectors ([])")
ax_eff.showGrid(x=True, y=True)
ax_eff.addLegend()
eff_colors = ("r", "g", "b", "m")
eff_lines = [ax_eff.plot(pen=pg.mkPen(color=c, width=1), name=l)
             for l, c in zip(("eff0", "eff1", "eff2", "eff3"), eff_colors)]

# # Right column: GPS lat/lon/alt (strip charts) and GDC commands
# ax_lat = win.addPlot(row=0, col=1, title="Latitude (deg)")
# ax_lat.showGrid(x=True, y=True)
# lat_line = ax_lat.plot(pen=pg.mkPen(color="#003f5c", width=1), name="lat")

# ax_lon = win.addPlot(row=1, col=1, title="Longitude (deg)")
# ax_lon.showGrid(x=True, y=True)
# lon_line = ax_lon.plot(pen=pg.mkPen(color="#2f4b7c", width=1), name="lon")

# ax_alt = win.addPlot(row=2, col=1, title="Altitude (ft)")
# ax_alt.showGrid(x=True, y=True)
# alt_line = ax_alt.plot(pen=pg.mkPen(color="#665191", width=1), name="alt")

# ax_gdc = win.addPlot(row=3, col=1, title="GDC Commands")
# ax_gdc.showGrid(x=True, y=True)
# ax_gdc.addLegend()
# gdc_colors = ("#d62728", "#ff7f0e", "#2ca02c", "#9467bd")
# gdc_lines = [ax_gdc.plot(pen=pg.mkPen(color=c, width=1), name=l) for l, c in zip(
#     ("pbCmd_dps", "nyCmd", "nzCmd", "throttle"), gdc_colors)]

frame_count = 0
Y_RECALC_PERIOD = 10  # recalc y-limits every N frames
TIME_WINDOW_S = 10.0  # fixed time window (seconds) to show
updating = False
skip_count = 0


def update():
    global frame_count, updating, skip_count
    if updating:
        skip_count += 1
        return
    updating = True
    try:
        # read all available packets from background reader queue
        packets = drain_packet_queue()
        for t, pkt in packets:
            append_packet(t, pkt)

        t = np.array(history["t"])
        # If we don't have any telemetry yet, set axes to a moving window
        if t.size == 0:
            now = time.time()
            t0 = now - TIME_WINDOW_S
            # set x-range on all axes so the UI keeps moving even with no data
            # , ax_gdc, ax_lat, ax_lon, ax_alt):
            for axp in (ax_accel, ax_gyro, ax_euler, ax_eff):
                axp.setXRange(t0, now, padding=0)
            # diagnostic queue size occasionally
            if frame_count % 200 == 0:
                try:
                    print(
                        f"no-data: queue_size={packet_queue.qsize()} skip_count={skip_count}")
                except Exception:
                    pass
            frame_count += 1
            return
        else:
            t0 = time.time() - TIME_WINDOW_S
        # accelerations
        ax_vals = np.array(history["ax"])
        ay_vals = np.array(history["ay"])
        az_vals = np.array(history["az"])
        for line, vals in zip(accel_lines, (ax_vals, ay_vals, az_vals)):
            line.setData(t, vals)
        ax_accel.setXRange(max(t[0], t0), t[-1], padding=0)

        # gyros
        gx = np.array(history["gx"])
        gy = np.array(history["gy"])
        gz = np.array(history["gz"])
        for line, vals in zip(gyro_lines, (gx, gy, gz)):
            line.setData(t, vals)
        ax_gyro.setXRange(max(t[0], t0), t[-1], padding=0)

        # euler
        roll = np.array(history["roll"])
        pitch = np.array(history["pitch"])
        yaw = np.array(history["yaw"])
        for line, vals in zip(euler_lines, (roll, pitch, yaw)):
            line.setData(t, vals)
        ax_euler.setXRange(max(t[0], t0), t[-1], padding=0)

        # eff
        eff0_ = np.array(history["eff0"])
        eff1_ = np.array(history["eff1"])
        eff2_ = np.array(history["eff2"])
        eff3_ = np.array(history["eff3"])
        for line, vals in zip(eff_lines, (eff0_, eff1_, eff2_, eff3_)):
            line.setData(t, vals)
        ax_eff.setXRange(max(t[0], t0), t[-1], padding=0)

        # # GDC commands
        # pb = np.array(history["pb"])
        # ny = np.array(history["ny"])
        # nz = np.array(history["nz"])
        # throttle = np.array(history["throttle"])
        # for line, vals in zip(gdc_lines, (pb, ny, nz, throttle)):
        #     line.setData(t, vals)
        # ax_gdc.setXRange(max(t[0], t0), t[-1], padding=0)

        # # GPS lat/lon/alt as strip charts
        # lat = np.array(history["lat"])
        # lon = np.array(history["lon"])
        # alt = np.array(history["alt"])
        # lat_line.setData(t, lat)
        # lon_line.setData(t, lon)
        # alt_line.setData(t, alt)
        # ax_lat.setXRange(max(t[0], t0), t[-1], padding=0)
        # ax_lon.setXRange(max(t[0], t0), t[-1], padding=0)
        # ax_alt.setXRange(max(t[0], t0), t[-1], padding=0)

        # recalc y-limits periodically for all multi-line axes
        if frame_count % Y_RECALC_PERIOD == 0:
            axes_and_arrays = [
                (ax_accel, (ax_vals, ay_vals, az_vals)),
                (ax_gyro, (gx, gy, gz)),
                (ax_euler, (roll, pitch, yaw)),
                (ax_eff, (eff0_, eff1_, eff2_, eff3_)),
                # (ax_gdc, (pb, ny, nz, throttle)),
                # (ax_lat, (lat,)),
                # (ax_lon, (lon,)),
                # (ax_alt, (alt,)),
            ]
            for axp, arrays in axes_and_arrays:
                try:
                    # ensure arrays are same-length and non-empty
                    arrays = [np.asarray(a) for a in arrays]
                    if not arrays:
                        continue
                    sizes = [a.size for a in arrays]
                    if not any(sizes):
                        continue
                    # if sizes differ, skip y autoscale for that axis this frame
                    if len(set(sizes)) != 1:
                        continue
                    stacked = np.vstack(arrays)
                    if np.isfinite(stacked).any():
                        vmin = np.nanmin(stacked)
                        vmax = np.nanmax(stacked)
                        if vmin == vmax:
                            vmin -= 0.5
                            vmax += 0.5
                        axp.setYRange(vmin, vmax, padding=0)
                except Exception:
                    print("y-range calc error:")
                    traceback.print_exc()

        frame_count += 1
    except Exception:
        print("update error:")
        traceback.print_exc()
    finally:
        updating = False


# ============================================================
# MAIN
# ============================================================

# start background serial reader thread (reads serial and enqueues packets)
serial_thread = threading.Thread(target=serial_reader_thread, args=(
    "COM3", 57600, packet_queue, serial_stop_event), daemon=True)
serial_thread.start()

# ensure background thread stops when the Qt app exits


def _stop_serial():
    serial_stop_event.set()


app.aboutToQuit.connect(_stop_serial)

# use a Qt timer to call update periodically
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # ms

if __name__ == "__main__":
    app.exec_()
