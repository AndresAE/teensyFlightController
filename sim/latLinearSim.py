# vscode-format: off
import numpy as np
import matplotlib.pyplot as plt
from plant.eom.eom import eom
from common.common import common

cmn = common()
eomObj = eom()

# lateral
v_fps = 220.1
rho_slugft3 = 0.00205
qBar_psf = 0.5*rho_slugft3*v_fps**2
Sref_ft2 = 174
xRef_ft = 36
mass_slug = 2650/32.2
I_xx_slugft2 = 948
I_yy_slugft2 = 1346
I_zz_slugft2 = 1967
I_xz_slugft2 = 0
theta_d = 0
alpha_d = 0
gamma_d = 0
g_fps2 = 32.174

Cy_beta = -0.393
Cy_p = -0.075
Cy_r = 0.214
Cy_ail = 0
Cy_rud = 0.187
Cr_beta = -0.0923
Cr_p = -0.484
Cr_r = 0.0798
Cr_ail = 0.229
Cr_rud = 0.0147
Cn_beta = 0.0587
Cn_p = -0.0278
Cn_r = -0.0937
Cn_ail = -0.0216
Cn_rud = -0.0645

inertia_B_slugft2 = np.array(
    [[I_xx_slugft2, 0, -I_xz_slugft2], [0, I_yy_slugft2, 0], [-I_xz_slugft2, 0, I_zz_slugft2]])
inertia_S_slugft2 = inertia_B_slugft2
invInertia_S_slugft2 = np.linalg.inv(inertia_S_slugft2)

Y_beta = qBar_psf * Sref_ft2 * Cy_beta / mass_slug
Y_p = qBar_psf * Sref_ft2 * xRef_ft * Cy_p / (2 * mass_slug * v_fps)
Y_r = qBar_psf * Sref_ft2 * xRef_ft * Cy_r / (2 * mass_slug * v_fps)
Y_ail = qBar_psf * Sref_ft2 * Cy_ail / mass_slug
Y_rud = qBar_psf * Sref_ft2 * Cy_rud / mass_slug

tmp_beta = qBar_psf * Sref_ft2 * xRef_ft * \
    invInertia_S_slugft2 @ np.array([[Cr_beta], [0], [Cn_beta]])
tmp_p = (qBar_psf * Sref_ft2 * xRef_ft * xRef_ft / (2 * v_fps)) * \
    invInertia_S_slugft2 @ np.array([[Cr_p], [0], [Cn_p]])
tmp_r = (qBar_psf * Sref_ft2 * xRef_ft * xRef_ft / (2 * v_fps)) * \
    invInertia_S_slugft2 @ np.array([[Cr_r], [0], [Cn_r]])
tmp_ail = qBar_psf * Sref_ft2 * xRef_ft * \
    invInertia_S_slugft2 @ np.array([[Cr_ail], [0], [Cn_ail]])
tmp_rud = qBar_psf * Sref_ft2 * xRef_ft * \
    invInertia_S_slugft2 @ np.array([[Cr_rud], [0], [Cn_rud]])

Rp_beta = tmp_beta[0][0]
Rp_p = tmp_p[0][0]
Rp_r = tmp_r[0][0]
Rp_ail = tmp_ail[0][0]
Rp_rud = tmp_rud[0][0]
Np_beta = tmp_beta[2][0]
Np_p = tmp_p[2][0]
Np_r = tmp_r[2][0]
Np_ail = tmp_ail[2][0]
Np_rud = tmp_rud[2][0]

A = np.array([[Y_beta/v_fps, g_fps2*np.cos(np.deg2rad(theta_d))/v_fps, Y_p/v_fps, (Y_r/v_fps-1)],
              [0, 0, np.cos(np.deg2rad(gamma_d))/np.cos(np.deg2rad(theta_d)),
               np.sin(np.deg2rad(gamma_d))/np.cos(np.deg2rad(theta_d))],
              [Rp_beta, 0, Rp_p, Rp_r],
              [Np_beta, 0, Np_p, Np_r]])
B = np.array([[Y_ail/v_fps, Y_rud/v_fps],
              [0, 0],
              [Rp_ail, Rp_rud],
              [Np_ail, Np_rud]])
C = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [Y_beta, 0, 0, 0]])
D = np.array([[0, 0],
              [0, 0],
              [0, 0],
              [Y_ail, Y_rud]])

olSys = (A, B, C, D)
t_s = np.arange(0, 10 + 0.01, 0.01)
u = np.zeros((len(t_s), 2))
u[t_s > 1, 1] = 1/57.3

t_out, y_out, x_out = eomObj.lsim(olSys, u, t_s)

plt.figure()
plt.plot(t_s, y_out[:, 1]*57.3)
plt.xlabel('Time (s)')
plt.ylabel('Roll Angle (deg)')
plt.show()
plt.figure()
plt.plot(t_s, y_out[:, 2]*57.3)
plt.xlabel('Time (s)')
plt.ylabel('Roll Rate (deg/s)')
plt.show()
plt.figure()
plt.plot(t_s, y_out[:, 0]*57.3)
plt.xlabel('Time (s)')
plt.ylabel('Beta (deg)')
plt.show()
