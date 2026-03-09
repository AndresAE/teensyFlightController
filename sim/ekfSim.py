import numpy as np
from matplotlib import pyplot as plt
import math


def geodeticToBody(phi_d, theta_d, psi_d):
    phi = np.deg2rad(phi_d)
    theta = np.deg2rad(theta_d)
    psi = np.deg2rad(psi_d)
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(phi), -np.sin(phi)],
                    [0, np.sin(phi), np.cos(phi)]])
    R_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                    [0, 1, 0],
                    [-np.sin(theta), 0, np.cos(theta)]])
    R_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
    R = R_z @ R_y @ R_x  # body to earth rotation matrix
    return R


def getF(x, u, dt_s, dx):
    xPert0 = f(x, u, dt_s)
    n = len(x)
    F = np.zeros((n, n))
    for ii in range(n):
        xPert = x.copy()
        xPert[ii] += dx
        xOut = f(xPert, u, dt_s)
        F[:, ii] = (xOut - xPert0) / dx
    return F


def filteredDerivative(x, xPrev, dt, cutoffFreq, filtered_dxPrev):
    # Compute the raw derivative
    dx = (x - xPrev) / dt
    # Compute the low-pass filter coefficient (alpha)
    alpha = (2 * np.pi * cutoffFreq * dt) / (2 * np.pi * cutoffFreq * dt + 1)
    # Apply the low-pass filter
    filtered_dx = alpha * dx + (1 - alpha) * filtered_dxPrev
    return filtered_dx


def f(x, u, dt_s):
    xPrev_ft = x[0]
    yPrev_ft = x[1]
    zPrev_ft = x[2]
    vxPrev_ft = x[3]
    vyPrev_ft = x[4]
    vzPrev_ft = x[5]
    phiPrev_d = x[6]
    thetaPrev_d = x[7]
    psiPrev_d = x[8]

    ax_fps2 = u[0]
    ay_fps2 = u[1]
    az_fps2 = u[2]
    p_dps = u[3]
    q_dps = u[4]
    r_dps = u[5]

    phi_d = phiPrev_d + (p_dps + np.tan(np.deg2rad(thetaPrev_d)) * (q_dps * np.sin(
        np.deg2rad(phiPrev_d)) + r_dps * np.cos(np.deg2rad(phiPrev_d)))) * dt_s
    theta_d = thetaPrev_d + (q_dps * np.cos(np.deg2rad(phiPrev_d)) -
                             r_dps * np.sin(np.deg2rad(phiPrev_d))) * dt_s
    psi_d = psiPrev_d + (q_dps * np.sin(np.deg2rad(phiPrev_d)) + r_dps *
                         np.cos(np.deg2rad(phiPrev_d))) * dt_s / np.cos(np.deg2rad(thetaPrev_d))

    E2B = geodeticToBody(phi_d, theta_d, psi_d)
    u_new = np.concatenate([E2B.T @ np.array([u[0], u[1], u[2]]), [0, 0, 0]])
    ax_fps2 = u_new[0]
    ay_fps2 = u_new[1]
    az_fps2 = u_new[2]

    x_ft = xPrev_ft + vxPrev_ft * dt_s + 0.5 * ax_fps2 * dt_s**2
    y_ft = yPrev_ft + vyPrev_ft * dt_s + 0.5 * ay_fps2 * dt_s**2
    z_ft = zPrev_ft + vzPrev_ft * dt_s + 0.5 * az_fps2 * dt_s**2
    vx_ft = vxPrev_ft + ax_fps2 * dt_s
    vy_ft = vyPrev_ft + ay_fps2 * dt_s
    vz_ft = vzPrev_ft + az_fps2 * dt_s

    xOut = np.array([x_ft, y_ft, z_ft, vx_ft, vy_ft,
                    vz_ft, phi_d, theta_d, psi_d])
    return xOut


# plant
v_E_fps = np.array([0, 20, 0])
x_E_fps = np.array([0, 0, 0])
euler_d = np.array([0, 0, 50])
a_E_fps = np.array([0, 5, 0])

# estimation
x = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
nStates = len(x)
u = np.array([0, 0, 0, 0, 0, 0])
dx = 0.1
xGPSprev = np.array([0, 0, 0])
vGPS = np.array([0, 0, 0])
H = np.hstack([np.eye(6), np.zeros((6, 3))])
P = np.eye(nStates)

Q = 100 * np.eye(nStates)
R = 0.01 * np.eye(6)

# simulation
dt_s = 0.01
tEnd_s = 100
t_s = np.arange(0, tEnd_s, dt_s)
nSteps = len(t_s)
out_xRec = np.zeros((nSteps, nStates))
out_xRec_E_fps = np.zeros((nSteps, 3))
out_vRec_E_fps = np.zeros((nSteps, 3))
out_eulerRec_d = np.zeros((nSteps, 3))

for ii in range(nSteps):
    # plant integration
    x_E_fps = x_E_fps + v_E_fps * dt_s + 0.5 * a_E_fps * dt_s ** 2
    v_E_fps = v_E_fps + a_E_fps * dt_s
    E2B = geodeticToBody(euler_d[0], euler_d[1], euler_d[2])

    # measurement
    u = np.concatenate([E2B @ a_E_fps, [0, 0, 0]])
    xGPS = x_E_fps

    # Kalman Filter
    # predict
    F = getF(x, u, dt_s, dx)
    x = f(x, u, dt_s)
    P = F @ P @ F.T + Q

    # update every 0.1 s
    # if ii % 10 == 0:
    vGPS = filteredDerivative(xGPS, xGPSprev, 0.01, 1, vGPS)
    xGPSprev = xGPS
    z = np.concatenate([xGPS, vGPS])
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(nStates) - K @ H) @ P

    out_xRec[ii, :] = x
    out_xRec_E_fps[ii, :] = x_E_fps
    out_vRec_E_fps[ii, :] = v_E_fps
    out_eulerRec_d[ii, :] = euler_d

# Plotting
# plt.figure()
# plt.plot(t_s, out_xRec[:, 0])
# plt.plot(t_s, out_xRec_E_fps[:, 0])
# plt.xlabel('time [s]')
# plt.ylabel('x [ft]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 1])
# plt.plot(t_s, out_xRec_E_fps[:, 1])
# plt.xlabel('time [s]')
# plt.ylabel('y [ft]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 2])
# plt.plot(t_s, out_xRec_E_fps[:, 2])
# plt.xlabel('time [s]')
# plt.ylabel('z [ft]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 3])
# plt.plot(t_s, out_vRec_E_fps[:, 0])
# plt.xlabel('time [s]')
# plt.ylabel('vx [fps]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 4])
# plt.plot(t_s, out_vRec_E_fps[:, 1])
# plt.xlabel('time [s]')
# plt.ylabel('vy [fps]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 5])
# plt.plot(t_s, out_vRec_E_fps[:, 2])
# plt.xlabel('time [s]')
# plt.ylabel('vz [fps]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 6])
# plt.plot(t_s, out_eulerRec_d[:, 0])
# plt.xlabel('time [s]')
# plt.ylabel('phi [d]')
# plt.grid(True)
# plt.show()

# plt.figure()
# plt.plot(t_s, out_xRec[:, 7])
# plt.plot(t_s, out_eulerRec_d[:, 1])
# plt.xlabel('time [s]')
# plt.ylabel('theta [d]')
# plt.grid(True)
# plt.show()

plt.figure()
plt.plot(t_s, out_xRec[:, 8])
plt.plot(t_s, out_eulerRec_d[:, 2])
plt.xlabel('time [s]')
plt.ylabel('psi [d]')
plt.grid(True)
plt.show()
