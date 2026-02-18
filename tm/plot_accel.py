import re
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

# Read the latest telemetry log
log_file = "tm/data/telemetry_log_20260209_220232.txt"

timestamps = []
ax_data = []
ay_data = []
az_data = []

with open(log_file, 'r') as f:
    for line in f:
        # Look for IMU lines
        if 'imu:' in line:
            # Extract timestamp
            try:
                ts_str = line.split(',')[0]
                ts = datetime.fromisoformat(ts_str)
                timestamps.append(ts)
                
                # Extract ax, ay, az values
                ax_match = re.search(r'roll_d=([^,]+)', line)
                ay_match = re.search(r'pitch_d=([^,]+)', line)
                az_match = re.search(r'yaw_d=([^,]+)', line)
                
                if ax_match and ay_match and az_match:
                    ax_data.append(float(ax_match.group(1)))
                    ay_data.append(float(ay_match.group(1)))
                    az_data.append(float(az_match.group(1)))
            except ValueError:
                continue

# Convert timestamps to relative time (seconds from start)
if timestamps:
    start_time = timestamps[0]
    rel_time = [(t - start_time).total_seconds() for t in timestamps]
    
    # Create the plot
    plt.figure(figsize=(12, 6))
    plt.plot(rel_time, ax_data, label='ax (ft/s²)', linewidth=1.5)
    plt.plot(rel_time, ay_data, label='ay (ft/s²)', linewidth=1.5)
    plt.plot(rel_time, az_data, label='az (ft/s²)', linewidth=1.5)
    
    plt.xlabel('Time (seconds)')
    plt.ylabel('Acceleration (ft/s²)')
    plt.title('Accelerometer Data (ax, ay, az)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('accel_plot.png', dpi=150)
    plt.show()
    
    print(f"Plotted {len(ax_data)} acceleration samples")
    print(f"Time range: {rel_time[0]:.2f}s to {rel_time[-1]:.2f}s")
