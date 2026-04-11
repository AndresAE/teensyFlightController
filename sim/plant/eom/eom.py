import numpy as np
from common.common import common
from plant.earth.earth import earth


class eom:
    def __init__(self):
        self.dt_s = 0.01  # Time step in seconds
        # Number of iterations for latitude calculation in ECEF to LLA conversion
        self.nIterLat = 10

        # inputs
        self.f_B_lbf = np.array([0.0, 0.0, 0.0])  # Force in body frame (lbf)
        # Moment about center of gravity in body frame (ft-lbf)
        self.m_B_CG_ftlb = np.array([0.0, 0.0, 0.0])
        self.mass_slug = 1.0  # Mass in slugs
        self.j_slugft2 = np.array([[1.0, 0.0, 0.0],   # Inertia matrix (slug-ft^2)
                                  [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]])

        # initial states
        # Latitude (deg), Longitude (deg), Altitude (ft)
        self.lla_ddft = np.array([0.0, 0.0, 0.0])
        # Velocity in body frame (ft/s)
        self.vBE_B_fps = np.array([0.0, 0.0, 0.0])
        # Euler angles (roll, pitch, yaw in degrees)
        self.euler_d = np.array([0.0, 0.0, 0.0])
        # Angular velocity in body frame (rad/s)
        self.wBG_B_rps = np.array([0.0, 0.0, 0.0])
        self.lonEI_d = 0.0  # Longitude in Earth inertial frame (deg)

        # external classes
        self.cmn = common()
        self.earth = earth()

        self.update()  # Initialize intermediate variables

    def step(self):
        # Call this method every loop to advance the simulation by one timestep.
        self.update()

        # integrate
        # Update position in ECEF frame (ft)
        self.xBE_E_ft += self.vBE_E_fps * self.dt_s
        # Update velocity in body frame (ft/s)
        self.vBE_B_fps += self.aBE_B_fps2 * self.dt_s
        # Update angular velocity in body frame (rad/s)
        self.wBI_B_rps += self.wBIdot_B_rps2 * self.dt_s
        # Update quaternion (assuming small angles for simplicity)
        self.q += self.qDot * self.dt_s
        # Update longitude due to Earth's rotation (deg)
        self.lonEI_d = self.earth.wEI_E_rps[2] * self.dt_s

        # post-processing
        # Update Euler angles (degrees)
        B2E = self.cmn.quaternion_to_dcm(self.q)
        # Update LLA coordinates (degrees, degrees, feet)
        self.lla_ddft = self.earth.ecef_to_lla(self.xBE_E_ft)
        self.euler_d = self.cmn.dcm_to_euler(B2E.transpose(
        ) @ self.cmn.ned2ecefTransformation(self.lla_ddft))  # Update Euler angles (degrees)
        self.wBG_B_rps = self.wBI_B_rps - self.E2B @ self.earth.wEI_E_rps - \
            self.wNED_rps  # Update angular velocity in body frame (rad/s)

    def update(self):
        # preprocessing
        # Transformation from ECEF to body frame
        self.E2B = self.cmn.ecef2bodyTransformation(
            self.lla_ddft, self.euler_d)
        # Transformation from NED to body frame
        self.G2B = self.cmn.ned2bodyTransformation(self.euler_d)
        self.xBE_E_ft = self.earth.lla_to_ecef(
            self.lla_ddft)  # Position in ECEF frame (ft)
        # Acceleration in body frame (ft/s^2)
        self.aBI_B_fps2 = self.f_B_lbf / self.mass_slug
        self.vBE_E_fps = self.E2B.transpose() @ self.vBE_B_fps
        # Angular velocity due to motion in NED frame (rad/s)
        self.wNED_rps = self.earth.ned_angular_velocity(
            self.lla_ddft, self.vBE_B_fps, self.G2B)
        self.wBI_B_rps = self.wBG_B_rps + self.E2B @ self.earth.wEI_E_rps + \
            self.wNED_rps  # Total angular velocity in body frame (rad/s)
        # Quaternion representation of orientation
        self.q = self.cmn.dcm_to_quaternion(self.E2B.transpose())

        self.aBE_B_fps2 = self.aBI_B_fps2 - self.E2B @ np.cross(self.earth.wEI_E_rps, np.cross(self.earth.wEI_E_rps, self.xBE_E_ft)) + np.cross(
            self.vBE_B_fps, self.wBI_B_rps + self.E2B @ self.earth.wEI_E_rps)  # Acceleration in body frame (ft/s^2)
        self.wBIdot_B_rps2 = np.linalg.inv(self.j_slugft2) @ (self.m_B_CG_ftlb - np.cross(
            self.wBI_B_rps, self.j_slugft2 @ self.wBI_B_rps))  # Angular acceleration in body frame (rad/s^2)
        pqrQuaternion_rps = self.wBI_B_rps - self.E2B @ self.earth.wEI_E_rps
        self.qDot = 0.5 * np.array([[0, -pqrQuaternion_rps[0], -pqrQuaternion_rps[1], -pqrQuaternion_rps[2]],
                                    [pqrQuaternion_rps[0], 0,
                                        pqrQuaternion_rps[2], -pqrQuaternion_rps[1]],
                                    [pqrQuaternion_rps[1], -pqrQuaternion_rps[2],
                                        0, pqrQuaternion_rps[0]],
                                    [pqrQuaternion_rps[2], pqrQuaternion_rps[1], -pqrQuaternion_rps[0], 0]]) @ self.q  # Quaternion derivative (assuming initial quaternion is [1, 0, 0, 0])

    def lsim(self, sys, u, t, x0=None):
        """
        sys: tuple (A, B, C, D)
        u: input array, shape (len(t), m)
        t: time array
        x0: initial state (optional)
        """
        A, B, C, D = sys
        # A = np.asarray(A)
        # B = np.asarray(B)
        # C = np.asarray(C)
        # D = np.asarray(D)

        n = A.shape[0]          # state dimension
        m = B.shape[1]          # input dimension
        p = C.shape[0]          # output dimension

        if x0 is None:
            x = np.zeros(n)
        else:
            x = np.asarray(x0).reshape(-1)

        dt = np.diff(t)
        N = len(t)

        x_hist = np.zeros((N, n))
        y_hist = np.zeros((N, p))

        for k in range(N):
            uk = np.atleast_1d(u[k])
            x_hist[k] = x.transpose()
            y_hist[k] = C @ x + D @ uk

            if k == N - 1:
                break

            # RK4 integration
            def f(xi, ui):
                return A @ xi + B @ ui

            k1 = f(x, uk)
            k2 = f(x + 0.5 * dt[k] * k1, uk)
            k3 = f(x + 0.5 * dt[k] * k2, uk)
            k4 = f(x + dt[k] * k3, uk)

            x = x + (dt[k] / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        return t, y_hist, x_hist

    def report(self):
        return f"state = {self.state:.3f}"
