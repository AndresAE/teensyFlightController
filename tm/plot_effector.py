"""
Plot effector control positions over time from telemetry log.
"""
import re
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def parse_telemetry_log(log_file):
    """Parse telemetry log and extract APT (actuator) effector data."""
    timestamps = []
    time_offset = None
    effC_0_list = []
    effC_1_list = []
    effC_2_list = []
    effC_3_list = []

    with open(log_file, 'r') as f:
        for line in f:
            # Look for APT (actuator) lines
            if 'apt:' in line:
                # Extract datetime from start of line
                timestamp_str = line.split(',')[0]
                try:
                    dt = datetime.fromisoformat(timestamp_str)
                    if time_offset is None:
                        time_offset = dt
                    elapsed_sec = (dt - time_offset).total_seconds()
                    timestamps.append(elapsed_sec)
                except ValueError:
                    continue

                # Extract effector control values
                eff0_m = re.search(r'effC_0:([-\d.]+)', line)
                eff1_m = re.search(r'effC_1:([-\d.]+)', line)
                eff2_m = re.search(r'effC_2:([-\d.]+)', line)
                eff3_m = re.search(r'effC_3:([-\d.]+)', line)

                # Use NaN if values are missing
                eff0 = float(eff0_m.group(1)) if eff0_m else float('nan')
                eff1 = float(eff1_m.group(1)) if eff1_m else float('nan')
                eff2 = float(eff2_m.group(1)) if eff2_m else float('nan')
                eff3 = float(eff3_m.group(1)) if eff3_m else float('nan')

                effC_0_list.append(eff0)
                effC_1_list.append(eff1)
                effC_2_list.append(eff2)
                effC_3_list.append(eff3)

    return (
        np.array(timestamps),
        np.array(effC_0_list),
        np.array(effC_1_list),
        np.array(effC_2_list),
        np.array(effC_3_list),
    )


def plot_effector(log_file):
    """Parse and plot effector control positions."""
    print(f"Parsing {log_file}...")
    t, eff0, eff1, eff2, eff3 = parse_telemetry_log(log_file)

    if len(t) == 0:
        print("No APT (effector) data found in log file.")
        return

    print(f"Found {len(t)} effector samples")

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Effector Control Positions', fontsize=14)

    # Plot each effector channel
    axes[0, 0].plot(t, eff0, 'b-', linewidth=1.5)
    axes[0, 0].set_ylabel('Position')
    axes[0, 0].set_title('Effector 0')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t, eff1, 'g-', linewidth=1.5)
    axes[0, 1].set_ylabel('Position')
    axes[0, 1].set_title('Effector 1')
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(t, eff2, 'r-', linewidth=1.5)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Position')
    axes[1, 0].set_title('Effector 2')
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(t, eff3, 'orange', linewidth=1.5)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Position')
    axes[1, 1].set_title('Effector 3')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Find the most recent telemetry log
    log_dir = Path(__file__).parent / 'data'
    log_files = sorted(log_dir.glob(
        'telemetry_log_20260303_173831.txt'), reverse=True)

    if not log_files:
        print(f"No telemetry logs found in {log_dir}")
        exit(1)

    latest_log = log_files[0]
    print(f"Using telemetry log: {latest_log.name}")
    plot_effector(str(latest_log))
