import numpy as np


class earth:
    def __init__(self):
        # inputs
        # Number of iterations for latitude calculation in ECEF to LLA conversion
        self.nIterLat = 10

        # Earth Data
        self.rEarth_ft = 20925524.9  # Earth's radius in feet
        self.e = 0.0818191908426  # Earth's eccentricity
        # Angular velocity of Earth in inertial frame (rad/s)
        self.wEI_E_rps = np.array([0.0, 0.0, 7.2722e-5])

    def lla_to_ecef(self, lla_ddft):
        # Placeholder for LLA to ECEF conversion
        lat_rad = np.radians(lla_ddft[0])  # Latitude in radians
        lon_rad = np.radians(lla_ddft[1])  # Longitude in radians
        sin_lat = np.sin(lat_rad)
        cos_lat = np.cos(lat_rad)
        sin_lon = np.sin(lon_rad)
        cos_lon = np.cos(lon_rad)
        # Prime vertical radius of curvature at the given latitude (ft)
        N = self.primeVerticalRadiusOfCurvature(lla_ddft[0])
        x_ft = (N + lla_ddft[2]) * cos_lat * cos_lon
        y_ft = (N + lla_ddft[2]) * cos_lat * sin_lon
        z_ft = (N * (1.0 - self.e**2) + lla_ddft[2]) * sin_lat
        return np.array([x_ft, y_ft, z_ft])  # ECEF coordinates (ft)

    def ecef_to_lla(self, xBE_E_ft):
        # Placeholder for ECEF to LLA conversion
        # This is a complex conversion and typically requires iterative methods, but we'll use a simple approximation for testing purposes.
        # Longitude in degrees
        lon_d = np.degrees(np.arctan2(xBE_E_ft[1], xBE_E_ft[0]))
        nIter = 0  # Iteration counter
        hae_ft = 0.0  # Initial guess for height above ellipsoid
        n_ft = self.rEarth_ft  # Initial guess for prime vertical radius of curvature

        while (nIter < self.nIterLat):
            latG_d = np.degrees(np.arctan((xBE_E_ft[2]) / (np.sqrt(xBE_E_ft[0] * xBE_E_ft[0] + xBE_E_ft[1] * xBE_E_ft[1]) * (
                1 - (self.e**2 * n_ft) / (n_ft + hae_ft)))))  # Latitude in degrees
            n_ft = self.primeVerticalRadiusOfCurvature(latG_d)
            hae_ft = np.sqrt(xBE_E_ft[0] * xBE_E_ft[0] + xBE_E_ft[1]
                             * xBE_E_ft[1]) / np.cos(np.radians(latG_d)) - n_ft
            nIter += 1

        # LLA coordinates (degrees, degrees, feet)
        return np.array([latG_d, lon_d, hae_ft])

    def ned_angular_velocity(self, lla_ddft, vBE_B_fps, G2B):
        # Placeholder for NED angular velocity calculation
        # Convert body velocity to NED velocity (ft/s)
        vNED_fps = G2B.transpose() @ vBE_B_fps
        # Angular velocity in NED frame (rad/s)
        wNEDx_rps = vNED_fps[1] / \
            (self.primeVerticalRadiusOfCurvature(lla_ddft[0]) + lla_ddft[2])
        # Angular velocity in NED frame (rad/s)
        wNEDy_rps = - \
            vNED_fps[0] / \
            (self.meridionalRadiusOfCurvature(lla_ddft[0]) + lla_ddft[2])
        # Angular velocity in NED frame (rad/s)
        wNEDz_rps = -wNEDx_rps * np.tan(np.radians(lla_ddft[0]))
        # Angular velocity in NED frame (rad/s)
        wNED_G_rps = np.array([wNEDx_rps, wNEDy_rps, wNEDz_rps])
        # Convert NED angular velocity to body frame (rad/s)
        wNED_B_rps = G2B @ wNED_G_rps
        return wNED_B_rps  # Angular velocity in body frame (rad/s)

    def meridionalRadiusOfCurvature(self, latG_d):
        # Placeholder for meridional radius of curvature calculation
        # Meridional radius of curvature (ft)
        M = self.rEarth_ft * (1 - self.e**2) / (1 - self.e **
                                                2 * np.sin(np.radians(latG_d))**2)**(3/2)
        return M  # Meridional radius of curvature at the given latitude (ft)

    def primeVerticalRadiusOfCurvature(self, latG_d):
        # Placeholder for prime vertical radius of curvature calculation
        # Prime vertical radius of curvature (ft)
        N = self.rEarth_ft / \
            np.sqrt(1 - self.e**2 * np.sin(np.radians(latG_d))**2)
        # Prime vertical radius of curvature at the given latitude (ft)
        return N
