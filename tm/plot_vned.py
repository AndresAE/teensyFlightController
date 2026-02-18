"""
Plot velocity in NED frame (converted from ECEF using lat/lon)
"""
import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def parse_telemetry_log(log_file):
    """Parse telemetry log and extract GPS data"""
    latitudes = []
    longitudes = []
    vX = []
    vY = []
    vZ = []

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

                # Use NaN if components are missing
                vx = float(vx_m.group(1)) if vx_m else float('nan')
                vy = float(vy_m.group(1)) if vy_m else float('nan')
                vz = float(vz_m.group(1)) if vz_m else float('nan')

                latitudes.append(lat)
                longitudes.append(lon)
                vX.append(vx)
                vY.append(vy)
                vZ.append(vz)

    return np.array(latitudes), np.array(longitudes), np.array(vX), np.array(vY), np.array(vZ)


def ecef_to_ned_rotation(lat_deg, lon_deg):
    """
    Create rotation matrix from ECEF to NED coordinates.
    
    Args:
        lat_deg: Latitude in degrees
        lon_deg: Longitude in degrees
    
    Returns:
        3x3 rotation matrix (dC_e2n)
    """
    lat_rad = np.radians(lat_deg)
    lon_rad = np.radians(lon_deg)
    
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)
    
    # Rotation matrix from ECEF to NED
    C_e2n = np.array([
        [-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
        [-sin_lat * sin_lon,  cos_lon, -cos_lat * sin_lon],
        [cos_lat,             0,        -sin_lat]
    ])
    
    return C_e2n


def ecef_to_ned_velocity(vx_ecef, vy_ecef, vz_ecef, lat_deg, lon_deg):
    """
    Transform velocity from ECEF to NED frame.
    
    Args:
        vx_ecef, vy_ecef, vz_ecef: ECEF velocity components
        lat_deg, lon_deg: Latitude and longitude in degrees
    
    Returns:
        (vN, vE, vD): NED velocity components
    """
    v_ecef = np.array([vx_ecef, vy_ecef, vz_ecef])
    C_e2n = ecef_to_ned_rotation(lat_deg, lon_deg)
    v_ned = C_e2n @ v_ecef
    return v_ned[0], v_ned[1], v_ned[2]


def plot_vned(log_file):
    """Plot velocity in NED frame"""
    lat, lon, vX, vY, vZ = parse_telemetry_log(log_file)

    if len(lat) == 0:
        print("No GPS data found in log file")
        return

    print(f"Found {len(lat)} GPS points")

    # Transform ECEF velocities to NED for each point
    vN = np.zeros_like(vX)
    vE = np.zeros_like(vY)
    vD = np.zeros_like(vZ)

    for i in range(len(lat)):
        vN[i], vE[i], vD[i] = ecef_to_ned_velocity(vX[i], vY[i], vZ[i], lat[i], lon[i])

    # Print statistics
    valid_vN = vN[~np.isnan(vN)]
    valid_vE = vE[~np.isnan(vE)]
    valid_vD = vD[~np.isnan(vD)]
    
    if len(valid_vN) > 0:
        print(f"vN range: {np.min(valid_vN):.2f} to {np.max(valid_vN):.2f} fps")
        print(f"vE range: {np.min(valid_vE):.2f} to {np.max(valid_vE):.2f} fps")
        print(f"vD range: {np.min(valid_vD):.2f} to {np.max(valid_vD):.2f} fps")

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    indices = np.arange(len(lat))

    # Plot 1: vN, vE, vD vs time
    axes[0, 0].plot(indices, vN, 'r-', label='North', linewidth=1, alpha=0.7)
    axes[0, 0].plot(indices, vE, 'g-', label='East', linewidth=1, alpha=0.7)
    axes[0, 0].plot(indices, vD, 'b-', label='Down', linewidth=1, alpha=0.7)
    axes[0, 0].set_xlabel('Sample Index')
    axes[0, 0].set_ylabel('Velocity (fps)')
    axes[0, 0].set_title('NED Velocity Components vs Time')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    # Plot 2: Velocity magnitude
    v_mag = np.sqrt(vN**2 + vE**2 + vD**2)
    axes[0, 1].plot(indices, v_mag, 'k-', linewidth=1)
    axes[0, 1].set_xlabel('Sample Index')
    axes[0, 1].set_ylabel('Velocity Magnitude (fps)')
    axes[0, 1].set_title('Total Velocity Magnitude')
    axes[0, 1].grid(True, alpha=0.3)

    # Plot 3: vN and vE trajectory (horizontal plane)
    axes[1, 0].plot(vE, vN, 'b-', linewidth=1, alpha=0.7)
    axes[1, 0].plot(vE[0], vN[0], 'go', markersize=8, label='Start')
    axes[1, 0].plot(vE[-1], vN[-1], 'r^', markersize=8, label='End')
    axes[1, 0].set_xlabel('East velocity (fps)')
    axes[1, 0].set_ylabel('North velocity (fps)')
    axes[1, 0].set_title('Horizontal Velocity (E-N plane)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    axes[1, 0].axis('equal')

    # Plot 4: North and Down velocity
    axes[1, 1].plot(indices, vN, 'r-', label='North', linewidth=1, alpha=0.7)
    axes[1, 1].plot(indices, vD, 'b-', label='Down', linewidth=1, alpha=0.7)
    axes[1, 1].set_xlabel('Sample Index')
    axes[1, 1].set_ylabel('Velocity (fps)')
    axes[1, 1].set_title('North and Down Velocity Components')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Use the telemetry log from the tm directory
    log_file = Path(__file__).parent / 'data/telemetry_log_20260209_220232.txt'

    if log_file.exists():
        plot_vned(log_file)
    else:
        print(f"Log file not found: {log_file}")
