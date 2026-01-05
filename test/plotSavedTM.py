import matplotlib.pyplot as plt
from datetime import datetime

log_file = "tm/telemetry_log.txt"

# Storage
t = []
ax = []; ay = []; az = []
gx = []; gy = []; gz = []
gps_lat = []; gps_lon = []

def parse_timestamp(ts):
    return datetime.fromisoformat(ts)

def parse_keyvals(parts):
    """Convert ['ax_fps2=-0.76', 'ay_fps2=-0.38', ...] into a dict."""
    out = {}
    for p in parts:
        if "=" in p:
            k, v = p.split("=", 1)
            out[k.strip()] = v.strip()
    return out

ii = 0
with open(log_file, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        # Split timestamp and the rest
        ts_str, rest = line.split(",", 1)
        ts = parse_timestamp(ts_str)

        # Split group and keyvals
        group, kv_str = rest.split(":", 1)
        group = group.strip().lower()
        kv_parts = [p.strip() for p in kv_str.split(",")]
        kv = parse_keyvals(kv_parts)

        print(ii)

        # -------------------------
        # IMU
        # -------------------------
        if group == "imu":
            t.append(ts)
            ax.append(float(kv["ax_fps2"]))
            ay.append(float(kv["ay_fps2"]))
            az.append(float(kv["az_fps2"]))
            gx.append(float(kv["gx_dps"]))
            gy.append(float(kv["gy_dps"]))
            gz.append(float(kv["gz_dps"]))

        # -------------------------
        # GPS
        # -------------------------
        elif group == "gps":
            gps_lat.append(float(kv["latG_d"]))
            gps_lon.append(float(kv["lon_d"]))
        
        ii += 1

# Convert timestamps to seconds
if t:
    t0 = t[0]
    t_sec = [(ti - t0).total_seconds() for ti in t]
else:
    t_sec = []

# -----------------------------
# Plot IMU accelerations
# -----------------------------
plt.figure(figsize=(10,5))
plt.plot(t_sec, ax, label="ax")
plt.plot(t_sec, ay, label="ay")
plt.plot(t_sec, az, label="az")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (fps²)")
plt.title("IMU Accelerations")
plt.legend()
plt.grid(True)

# -----------------------------
# Plot IMU gyros
# -----------------------------
plt.figure(figsize=(10,5))
plt.plot(t_sec, gx, label="gx")
plt.plot(t_sec, gy, label="gy")
plt.plot(t_sec, gz, label="gz")
plt.xlabel("Time (s)")
plt.ylabel("Gyro (deg/s)")
plt.title("IMU Gyros")
plt.legend()
plt.grid(True)

# -----------------------------
# GPS scatter
# -----------------------------
plt.figure(figsize=(6,6))
plt.scatter(gps_lat, gps_lon, s=5)
plt.xlabel("Latitude")
plt.ylabel("Longitude")
plt.title("GPS Track")
plt.grid(True)

plt.show()