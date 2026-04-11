import numpy as np


class common:
    def __init__(self):
        self.r2d = 180/np.pi  # radians to degrees conversion factor
        self.d2r = np.pi/180  # degrees to radians conversion factor
        self.g_fps2 = 32.174  # gravitational acceleration in ft/s^2
        self.ft2in = 12  # foot to inch
        self.in2ft = 1/12  # inch to foot
        self.ft2m = 1/3.28084  # ft to meter
        self.m2ft = 3.28084  # meter to foot
        self.slug2kg = 14.593902937  # slug to kilogram
        self.kg2slug = 0.0685217659  # kilogram to slug
        self.lbf2n = 4.44822  # pound force to newton
        self.n2lbf = 1 / 4.44822  # newton to pound force

    def ecef2bodyTransformation(self, lla_ddft, euler_d):
        # Placeholder for ECEF to body frame conversion
        # Get the ECEF to NED transformation matrix
        ecef2ned = self.ecef2nedTransformation(lla_ddft)
        # Get the NED to body transformation matrix
        ned2body = self.ned2bodyTransformation(euler_d)
        ecef2body = ned2body @ ecef2ned  # Combined transformation from ECEF to body frame
        return ecef2body  # Return the transformation matrix

    def ecef2nedTransformation(self, lla_ddft):
        # Placeholder for ECEF to NED frame conversion
        # Get the NED to ECEF transformation matrix
        ned2ecef = self.ned2ecefTransformation(lla_ddft)
        return ned2ecef.transpose()  # Return the transformation matrix (ECEF to NED)

    def ned2ecefTransformation(self, lla_ddft):
        # Placeholder for ECEF to NED frame conversion
        ned2ecef = np.array([[-np.sin(np.radians(lla_ddft[0])) * np.cos(np.radians(lla_ddft[1])), -np.sin(np.radians(lla_ddft[1])), -np.cos(np.radians(lla_ddft[0])) * np.cos(np.radians(lla_ddft[1]))],
                             [-np.sin(np.radians(lla_ddft[0])) * np.sin(np.radians(lla_ddft[1])), np.cos(np.radians(
                                 lla_ddft[1])), -np.cos(np.radians(lla_ddft[0])) * np.sin(np.radians(lla_ddft[1]))],
                             [np.cos(np.radians(lla_ddft[0])), 0, -np.sin(np.radians(lla_ddft[0]))]])
        return ned2ecef  # Return the transformation matrix

    def body2nedTransformation(self, euler_d):
        # Placeholder for body to NED frame conversion
        # Get the NED to body transformation matrix
        ned2body = self.ned2bodyTransformation(euler_d)
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
            [cp*cy,                     cp*sy,                    -sp],
            [sr*sp*cy - cr*sy,         sr*sp*sy + cr*cy,         sr*cp],
            [cr*sp*cy + sr*sy,         cr*sp*sy - sr*cy,         cr*cp]
        ])

        return ned2body  # Return the transformation matrix

    def stab2bodyTransformation(self, alpha_d):
        """Stability Axes to Body."""
        ca = np.cos(alpha_d*self.d2r)
        sa = np.sin(alpha_d*self.d2r)

        stab2body = np.array([
            [ca,  0.0, -sa],
            [0.0,  1.0, 0.0],
            [sa,  0.0,  ca]
        ])

        return stab2body

    def body2stabTransformation(self, alpha_d):
        """Body Axes to Stability."""
        body2stab = np.transpose(self.stab2bodyTransformation(alpha_d))
        return body2stab

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
        w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]

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
        # Euler angles (roll, pitch, yaw in degrees)
        return np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

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
            x = (C[2, 1] - C[1, 2]) / S
            y = (C[0, 2] - C[2, 0]) / S
            z = (C[1, 0] - C[0, 1]) / S

        else:
            # Find the largest diagonal element
            if (C[0, 0] > C[1, 1]) and (C[0, 0] > C[2, 2]):
                S = np.sqrt(1.0 + C[0, 0] - C[1, 1] - C[2, 2]) * 2.0
                w = (C[2, 1] - C[1, 2]) / S
                x = 0.25 * S
                y = (C[0, 1] + C[1, 0]) / S
                z = (C[0, 2] + C[2, 0]) / S

            elif C[1, 1] > C[2, 2]:
                S = np.sqrt(1.0 + C[1, 1] - C[0, 0] - C[2, 2]) * 2.0
                w = (C[0, 2] - C[2, 0]) / S
                x = (C[0, 1] + C[1, 0]) / S
                y = 0.25 * S
                z = (C[1, 2] + C[2, 1]) / S

            else:
                S = np.sqrt(1.0 + C[2, 2] - C[0, 0] - C[1, 1]) * 2.0
                w = (C[1, 0] - C[0, 1]) / S
                x = (C[0, 2] + C[2, 0]) / S
                y = (C[1, 2] + C[2, 1]) / S
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

    def dcm_to_euler(self, C):
        """
        Convert a 3x3 DCM to ZYX Euler angles (roll, pitch, yaw).
        Returns (roll, pitch, yaw) in radians.
        """
        C = np.asarray(C)

        # Pitch (θ)
        pitch = -np.arcsin(np.clip(C[0, 2], -1.0, 1.0))

        # Handle gimbal lock
        if abs(C[0, 2]) > 0.999999:
            # Gimbal lock: yaw and roll are coupled
            yaw = np.arctan2(-C[1, 0], C[1, 1])
            roll = 0.0
        else:
            # Roll (φ)
            roll = np.arctan2(C[1, 2], C[2, 2])

            # Yaw (ψ)
            yaw = np.arctan2(C[0, 1], C[0, 0])

        # Euler angles (roll, pitch, yaw in degrees)
        return np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])
