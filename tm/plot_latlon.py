"""
Plot latitude and longitude from telemetry log
"""
import re
import matplotlib.pyplot as plt
from pathlib import Path

def parse_telemetry_log(log_file):
    """Parse telemetry log and extract GPS data (lat/lon), vECEF components, and HAE"""
    latitudes = []
    longitudes = []
    vX = []
    vY = []
    vZ = []
    heights = []

    with open(log_file, 'r') as f:
        for line in f:
            # Look for GPS lines
            if 'gps:' in line:
                lat_match = re.search(r'latG_d=([-\d.]+)', line)
                lon_match = re.search(r'lon_d=([-\d.]+)', line)
                if not (lat_match and lon_match):
                    continue

                lat = float(lat_match.group(1))
                lon = float(lon_match.group(1))

                vx_m = re.search(r'vECEF_X_fps=([-\d.]+)', line)
                vy_m = re.search(r'vECEF_Y_fps=([-\d.]+)', line)
                vz_m = re.search(r'vECEF_Z_fps=([-\d.]+)', line)
                hae_m = re.search(r'HAE_ft=([-\d.]+)', line)

                # Use NaN if components are missing
                vx = float(vx_m.group(1)) if vx_m else float('nan')
                vy = float(vy_m.group(1)) if vy_m else float('nan')
                vz = float(vz_m.group(1)) if vz_m else float('nan')
                hae = float(hae_m.group(1)) if hae_m else float('nan')

                latitudes.append(lat)
                longitudes.append(lon)
                vX.append(vx)
                vY.append(vy)
                vZ.append(vz)
                heights.append(hae)

    return latitudes, longitudes, vX, vY, vZ, heights

def plot_latlon(log_file):
    """Plot latitude/longitude trajectory, vECEF components, and HAE"""
    lat, lon, vX, vY, vZ, hae = parse_telemetry_log(log_file)

    if not lat or not lon:
        print("No GPS data found in log file")
        return

    print(f"Found {len(lat)} GPS points")
    print(f"Latitude range: {min(lat):.6f} to {max(lat):.6f}")
    print(f"Longitude range: {min(lon):.6f} to {max(lon):.6f}")

    # Create a GridSpec: left column for trajectory, right column 4 stacked plots
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(4, 2, width_ratios=[3, 1], hspace=0.4, wspace=0.3)

    ax_traj = fig.add_subplot(gs[:, 0])
    ax_vx = fig.add_subplot(gs[0, 1])
    ax_vy = fig.add_subplot(gs[1, 1])
    ax_vz = fig.add_subplot(gs[2, 1])
    ax_hae = fig.add_subplot(gs[3, 1])

    # Trajectory plot
    ax_traj.plot(lon, lat, 'b-', linewidth=1, alpha=0.7)
    ax_traj.plot(lon[0], lat[0], 'go', markersize=8, label='Start')
    ax_traj.plot(lon[-1], lat[-1], 'r^', markersize=8, label='End')
    ax_traj.set_xlabel('Longitude (degrees)')
    ax_traj.set_ylabel('Latitude (degrees)')
    ax_traj.set_title('GPS Trajectory')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend()
    ax_traj.axis('equal')

    indices = range(len(lat))

    # vECEF X
    ax_vx.plot(indices, vX, 'C0-', linewidth=1)
    ax_vx.set_title('vECEF_X_fps')
    ax_vx.set_xlabel('Sample Index')
    ax_vx.set_ylabel('fps')
    ax_vx.grid(True, alpha=0.3)

    # vECEF Y
    ax_vy.plot(indices, vY, 'C1-', linewidth=1)
    ax_vy.set_title('vECEF_Y_fps')
    ax_vy.set_xlabel('Sample Index')
    ax_vy.set_ylabel('fps')
    ax_vy.grid(True, alpha=0.3)

    # vECEF Z
    ax_vz.plot(indices, vZ, 'C2-', linewidth=1)
    ax_vz.set_title('vECEF_Z_fps')
    ax_vz.set_xlabel('Sample Index')
    ax_vz.set_ylabel('fps')
    ax_vz.grid(True, alpha=0.3)

    # HAE (Height Above Ellipsoid)
    ax_hae.plot(indices, hae, 'C3-', linewidth=1)
    ax_hae.set_title('HAE_ft')
    ax_hae.set_xlabel('Sample Index')
    ax_hae.set_ylabel('ft')
    ax_hae.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Use the telemetry log from the tm directory
    log_file = Path(__file__).parent / 'data/telemetry_log_20260209_220232.txt'

    if log_file.exists():
        plot_latlon(log_file)
    else:
        print(f"Log file not found: {log_file}")
