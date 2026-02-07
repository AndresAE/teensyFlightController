import time
import numpy as np

class Simulator:
    def __init__(self):
        self.dt_s = 0.01  # Time step in seconds
        self.nIterLat = 10  # Number of iterations for latitude calculation in ECEF to LLA conversion

        # inputs
        self.f_B_lbf = np.array([0.0, 0.0, 0.0])  # Force in body frame (lbf)
        self.m_B_CG_ftlb = np.array([0.0, 0.0, 0.0])  # Moment about center of gravity in body frame (ft-lbf)
        self.mass_slug = 1.0  # Mass in slugs
        self.j_slugft2 = np.array([[1.0, 0.0, 0.0],   # Inertia matrix (slug-ft^2)
                                  [0.0, 1.0, 0.0],
                                    [0.0, 0.0, 1.0]])
        
        # initial states
        self.lla_ddft = np.array([0.0, 0.0, 0.0])  # Latitude (deg), Longitude (deg), Altitude (ft)
        self.vBE_B_fps = np.array([0.0, 0.0, 0.0])  # Velocity in body frame (ft/s)
        self.euler_d = np.array([0.0, 0.0, 0.0])  # Euler angles (roll, pitch, yaw in degrees)
        self.wBG_B_rps = np.array([0.0, 0.0, 0.0])  # Angular velocity in body frame (rad/s)
        self.lonEI_d = 0.0  # Longitude in Earth inertial frame (deg)

        # Earth Data
        self.rEarth_ft = 20925524.9  # Earth's radius in feet
        self.e = 0.0818191908426  # Earth's eccentricity
        self.wEI_E_rps = np.array([0.0, 0.0, 7.2722e-5])  # Angular velocity of Earth in inertial frame (rad/s)

        self.update()  # Initialize intermediate variables

    def step(self):
        # Call this method every loop to advance the simulation by one timestep.
        self.update()

        # integrate
        self.xBE_E_ft += self.vBE_E_fps * self.dt_s  # Update position in ECEF frame (ft)
        self.vBE_B_fps += self.aBE_B_fps2 * self.dt_s  # Update velocity in body frame (ft/s)
        self.wBI_B_rps += self.wBIdot_B_rps2 * self.dt_s  # Update angular velocity in body frame (rad/s)
        self.q += self.qDot * self.dt_s  # Update quaternion (assuming small angles for simplicity)
        self.lonEI_d = self.wEI_E_rps[2] * self.dt_s  # Update longitude due to Earth's rotation (deg)

        # post-processing
        B2E = self.quaternion_to_dcm(self.q)  # Update Euler angles (degrees)
        self.lla_ddft = self.ecef_to_lla(self.xBE_E_ft)  # Update LLA coordinates (degrees, degrees, feet)
        self.euler_d = self.dcm_to_euler(B2E.transpose() @ self.ned2ecefTransformation(self.lla_ddft))  # Update Euler angles (degrees)
        self.wBG_B_rps = self.wBI_B_rps - self.E2B @ self.wEI_E_rps - self.wNED_rps  # Update angular velocity in body frame (rad/s)

    def update(self):
        # preprocessing
        self.E2B = self.ecef2bodyTransformation(self.lla_ddft, self.euler_d)  # Transformation from ECEF to body frame
        self.G2B = self.ned2bodyTransformation(self.euler_d)  # Transformation from NED to body frame
        self.xBE_E_ft = self.lla_to_ecef(self.lla_ddft)  # Position in ECEF frame (ft)
        self.aBI_B_fps2 = self.f_B_lbf / self.mass_slug  # Acceleration in body frame (ft/s^2)
        self.vBE_E_fps =  self.E2B.transpose() @ self.vBE_B_fps
        self.wNED_rps = self.ned_angular_velocity(self.lla_ddft, self.vBE_B_fps, self.G2B)  # Angular velocity due to motion in NED frame (rad/s)
        self.wBI_B_rps = self.wBG_B_rps + self.E2B @ self.wEI_E_rps + self.wNED_rps  # Total angular velocity in body frame (rad/s)
        self.q = self.dcm_to_quaternion(self.E2B.transpose())  # Quaternion representation of orientation
        
        self.aBE_B_fps2 = self.aBI_B_fps2 - self.E2B @ np.cross(self.wEI_E_rps, np.cross(self.wEI_E_rps, self.xBE_E_ft)) + np.cross(self.vBE_B_fps, self.wBI_B_rps + self.E2B @ self.wEI_E_rps) # Acceleration in body frame (ft/s^2)
        self.wBIdot_B_rps2 = np.linalg.inv(self.j_slugft2) @ (self.m_B_CG_ftlb - np.cross(self.wBI_B_rps, self.j_slugft2 @ self.wBI_B_rps))  # Angular acceleration in body frame (rad/s^2)
        pqrQuaternion_rps = self.wBI_B_rps - self.E2B @ self.wEI_E_rps
        self.qDot = 0.5 * np.array([[0, -pqrQuaternion_rps[0], -pqrQuaternion_rps[1], -pqrQuaternion_rps[2]],
                                    [pqrQuaternion_rps[0], 0, pqrQuaternion_rps[2], -pqrQuaternion_rps[1]],
                                    [pqrQuaternion_rps[1], -pqrQuaternion_rps[2], 0, pqrQuaternion_rps[0]],
                                    [pqrQuaternion_rps[2], pqrQuaternion_rps[1], -pqrQuaternion_rps[0], 0]]) @ self.q  # Quaternion derivative (assuming initial quaternion is [1, 0, 0, 0])
        
    def lla_to_ecef(self, lla_ddft):
        # Placeholder for LLA to ECEF conversion
        lat_rad = np.radians(lla_ddft[0])  # Latitude in radians
        lon_rad = np.radians(lla_ddft[1])  # Longitude in radians
        sin_lat = np.sin(lat_rad)
        cos_lat = np.cos(lat_rad)
        sin_lon = np.sin(lon_rad)
        cos_lon = np.cos(lon_rad)
        N = self.primeVerticalRadiusOfCurvature(lla_ddft[0])  # Prime vertical radius of curvature at the given latitude (ft)
        x_ft = (N + lla_ddft[2]) * cos_lat * cos_lon
        y_ft = (N + lla_ddft[2]) * cos_lat * sin_lon
        z_ft = (N * (1.0 - self.e**2) + lla_ddft[2]) * sin_lat
        return np.array([x_ft, y_ft, z_ft])  # ECEF coordinates (ft)

    def ecef_to_lla(self, xBE_E_ft):
        # Placeholder for ECEF to LLA conversion
        # This is a complex conversion and typically requires iterative methods, but we'll use a simple approximation for testing purposes.
        lon_d = np.degrees(np.arctan2(xBE_E_ft[1], xBE_E_ft[0]))  # Longitude in degrees
        nIter = 0 # Iteration counter
        hae_ft = 0.0 # Initial guess for height above ellipsoid
        n_ft = self.rEarth_ft # Initial guess for prime vertical radius of curvature

        while (nIter < self.nIterLat):
            latG_d = np.degrees(np.arctan((xBE_E_ft[2]) / (np.sqrt(xBE_E_ft[0] * xBE_E_ft[0] + xBE_E_ft[1] * xBE_E_ft[1]) * (1 - (self.e**2 * n_ft) / (n_ft + hae_ft)))))  # Latitude in degrees
            n_ft = self.primeVerticalRadiusOfCurvature(latG_d)
            hae_ft = np.sqrt(xBE_E_ft[0] * xBE_E_ft[0] + xBE_E_ft[1] * xBE_E_ft[1]) / np.cos(np.radians(latG_d)) - n_ft
            nIter += 1

        return np.array([latG_d, lon_d, hae_ft])  # LLA coordinates (degrees, degrees, feet)
    
    def ecef2bodyTransformation(self, lla_ddft, euler_d):
        # Placeholder for ECEF to body frame conversion
        ecef2ned = self.ecef2nedTransformation(lla_ddft)  # Get the ECEF to NED transformation matrix
        ned2body = self.ned2bodyTransformation(euler_d)  # Get the NED to body transformation matrix
        ecef2body = ned2body @ ecef2ned  # Combined transformation from ECEF to body frame
        return ecef2body  # Return the transformation matrix

    def ecef2nedTransformation(self, lla_ddft):
        # Placeholder for ECEF to NED frame conversion
        ned2ecef = self.ned2ecefTransformation(lla_ddft)  # Get the NED to ECEF transformation matrix
        return ned2ecef.transpose()  # Return the transformation matrix (ECEF to NED)
    
    def ned2ecefTransformation(self, lla_ddft):
        # Placeholder for ECEF to NED frame conversion
        ned2ecef = np.array([[-np.sin(np.radians(lla_ddft[0])) * np.cos(np.radians(lla_ddft[1])), -np.sin(np.radians(lla_ddft[1])), -np.cos(np.radians(lla_ddft[0])) * np.cos(np.radians(lla_ddft[1]))],
                             [-np.sin(np.radians(lla_ddft[0])) * np.sin(np.radians(lla_ddft[1])), np.cos(np.radians(lla_ddft[1])), -np.cos(np.radians(lla_ddft[0])) * np.sin(np.radians(lla_ddft[1]))],
                             [np.cos(np.radians(lla_ddft[0])), 0, -np.sin(np.radians(lla_ddft[0]))]])
        return ned2ecef  # Return the transformation matrix

    def body2nedTransformation(self, euler_d):
        # Placeholder for body to NED frame conversion
        ned2body = self.ned2bodyTransformation(euler_d)  # Get the NED to body transformation matrix
        return ned2body.transpose()  # Return the transformation matrix (body to NED)

    def ned2bodyTransformation(self, euler_d):
        # Placeholder for NED to body frame conversion
        cr = np.cos(np.radians(euler_d[0]))  # Roll cosine
        sr = np.sin(np.radians(euler_d[0]))  # Roll sine
        cp = np.cos(np.radians(euler_d[1]))  # Pitch cosine
        sp = np.sin(np.radians(euler_d[1]))  # Pitch sine
        cy = np.cos(np.radians(euler_d[2]))  # Yaw cosine
        sy = np.sin(np.radians(euler_d[2]))  # Yaw sine

        ned2body = np.array([
            [ cp*cy,                     cp*sy,                    -sp     ],
            [ sr*sp*cy - cr*sy,         sr*sp*sy + cr*cy,         sr*cp   ],
            [ cr*sp*cy + sr*sy,         cr*sp*sy - sr*cy,         cr*cp   ]
        ])

        return ned2body  # Return the transformation matrix

    def euler_to_quaternion(self, euler_d):
        # Placeholder for Euler to quaternion conversion
        roll_r = np.radians(euler_d[0])  # Roll in radians
        pitch_r = np.radians(euler_d[1])  # Pitch in radians
        yaw_r = np.radians(euler_d[2])  # Yaw in radians

        cy = np.cos(yaw_r * 0.5)
        sy = np.sin(yaw_r * 0.5)
        cp = np.cos(pitch_r * 0.5)
        sp = np.sin(pitch_r * 0.5)
        cr = np.cos(roll_r * 0.5)
        sr = np.sin(roll_r * 0.5)
        qw = cy*cp*cr + sy*sp*sr
        qx = cy*cp*sr - sy*sp*cr
        qy = cy*sp*cr + sy*cp*sr
        qz = sy*cp*cr - cy*sp*sr

        return np.array([qw, qx, qy, qz])  # Quaternion (w, x, y, z)
    
    def quaternion_to_euler(self, q):
        """
        q: array-like of shape (..., 4) in [w, x, y, z] order
        returns Euler angles (roll, pitch, yaw) in degrees
        using aerospace ZYX convention.
        """
        q = np.asarray(q)
        w, x, y, z = q[...,0], q[...,1], q[...,2], q[...,3]

        # Roll (x-axis rotation)
        t0 = 2.0 * (w*x + y*z)
        t1 = 1.0 - 2.0 * (x*x + y*y)
        roll = np.arctan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = 2.0 * (w*y - z*x)
        t2 = np.clip(t2, -1.0, 1.0)   # numerical safety
        pitch = np.arcsin(t2)

        # Yaw (z-axis rotation)
        t3 = 2.0 * (w*z + x*y)
        t4 = 1.0 - 2.0 * (y*y + z*z)
        yaw = np.arctan2(t3, t4)

        # Convert to degrees
        return np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])  # Euler angles (roll, pitch, yaw in degrees)
    
    def dcm_to_quaternion(self, dcm):
        """
        Convert a 3x3 direction cosine matrix (DCM) to a quaternion (w, x, y, z).
        C must be a proper rotation matrix.
        """
        C = np.asarray(dcm)
        tr = np.trace(C)

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (C[2,1] - C[1,2]) / S
            y = (C[0,2] - C[2,0]) / S
            z = (C[1,0] - C[0,1]) / S

        else:
            # Find the largest diagonal element
            if (C[0,0] > C[1,1]) and (C[0,0] > C[2,2]):
                S = np.sqrt(1.0 + C[0,0] - C[1,1] - C[2,2]) * 2.0
                w = (C[2,1] - C[1,2]) / S
                x = 0.25 * S
                y = (C[0,1] + C[1,0]) / S
                z = (C[0,2] + C[2,0]) / S

            elif C[1,1] > C[2,2]:
                S = np.sqrt(1.0 + C[1,1] - C[0,0] - C[2,2]) * 2.0
                w = (C[0,2] - C[2,0]) / S
                x = (C[0,1] + C[1,0]) / S
                y = 0.25 * S
                z = (C[1,2] + C[2,1]) / S

            else:
                S = np.sqrt(1.0 + C[2,2] - C[0,0] - C[1,1]) * 2.0
                w = (C[1,0] - C[0,1]) / S
                x = (C[0,2] + C[2,0]) / S
                y = (C[1,2] + C[2,1]) / S
                z = 0.25 * S

        q = np.array([w, x, y, z])
        return q / np.linalg.norm(q)
    
    def quaternion_to_dcm(self, q):
        """
        Convert a quaternion (w, x, y, z) to a 3x3 direction cosine matrix (DCM).
        The input quaternion should be normalized.
        """
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]
        dcm = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
            [2*(x*y + z*w),           1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
            [2*(x*z - y*w),           2*(y*z + x*w),         1 - 2*(x**2 + y**2)]
        ])
        return dcm


    def ned_angular_velocity(self, lla_ddft, vBE_B_fps, G2B):
        # Placeholder for NED angular velocity calculation
        vNED_fps = G2B.transpose() @ vBE_B_fps  # Convert body velocity to NED velocity (ft/s)
        wNEDx_rps = vNED_fps[1] / (self.primeVerticalRadiusOfCurvature(lla_ddft[0]) + lla_ddft[2])  # Angular velocity in NED frame (rad/s)
        wNEDy_rps = -vNED_fps[0] / (self.meridionalRadiusOfCurvature(lla_ddft[0]) + lla_ddft[2])  # Angular velocity in NED frame (rad/s)
        wNEDz_rps = -wNEDx_rps * np.tan(np.radians(lla_ddft[0]))  # Angular velocity in NED frame (rad/s)
        wNED_G_rps = np.array([wNEDx_rps, wNEDy_rps, wNEDz_rps])  # Angular velocity in NED frame (rad/s)
        wNED_B_rps = G2B @ wNED_G_rps  # Convert NED angular velocity to body frame (rad/s)
        return wNED_B_rps  # Angular velocity in body frame (rad/s)
    
    def meridionalRadiusOfCurvature(self, latG_d):
        # Placeholder for meridional radius of curvature calculation
        M = self.rEarth_ft * (1 - self.e**2) / (1 - self.e**2 * np.sin(np.radians(latG_d))**2)**(3/2)  # Meridional radius of curvature (ft)
        return M  # Meridional radius of curvature at the given latitude (ft)

    def primeVerticalRadiusOfCurvature(self, latG_d):
        # Placeholder for prime vertical radius of curvature calculation
        N = self.rEarth_ft / np.sqrt(1 - self.e**2 * np.sin(np.radians(latG_d))**2)  # Prime vertical radius of curvature (ft)
        return N  # Prime vertical radius of curvature at the given latitude (ft)
    
    def dcm_to_euler(self, C):
        """
        Convert a 3x3 DCM to ZYX Euler angles (roll, pitch, yaw).
        Returns (roll, pitch, yaw) in radians.
        """
        C = np.asarray(C)

        # Pitch (θ)
        pitch = -np.arcsin(np.clip(C[0,2], -1.0, 1.0))

        # Handle gimbal lock
        if abs(C[0,2]) > 0.999999:
            # Gimbal lock: yaw and roll are coupled
            yaw = np.arctan2(-C[1,0], C[1,1])
            roll = 0.0
        else:
            # Roll (φ)
            roll = np.arctan2(C[1,2], C[2,2])

            # Yaw (ψ)
            yaw = np.arctan2(C[0,1], C[0,0])

        return np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])  # Euler angles (roll, pitch, yaw in degrees)

    def report(self):
        return f"state = {self.state:.3f}"
