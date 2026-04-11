# vscode-format: off
import numpy as np
import matplotlib.pyplot as plt
from plant.eom.eom import eom
from common.common import common
from sim.plant.aerodynamics.aeroModelingTools import aeroModeling
from plant.mass.massModeling import massModeling

cmn = common()
eomObj = eom()


class controller():
    def __init__(self):
        self.pCmd_errorInt = 0
        self.qCmd_errorInt = 0
        self.rCmd_errorInt = 0

    def step(self, x, u_cmd, dt_s):
        ka_phi = 0.5
        ka_ny = 5
        kd_p = 0.5  # 2.43
        ki_p = 1
        kd_r = -0.5  # 1.75
        ki_r = 1

        pitchLimit = 30
        ka_nz = -1.5
        kd_q = 1  # 1.58
        ki_q = 1

        mixing = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        phi_d = x[3]
        ny_B_fps2 = x[4]
        nz_B_fps2 = x[5]

        p_B_dps = x[0]
        q_B_dps = x[1]
        r_B_dps = x[2]

        phiCmd_d = u_cmd[0]
        nzCmd_B_fps2 = u_cmd[1]
        nyCmd_B_fps2 = u_cmd[2]

        # middle loop
        pCmd_B_dps = ka_phi * (phiCmd_d - phi_d)
        qCmd_B_dps = ka_nz * (nzCmd_B_fps2 - nz_B_fps2)
        rCmd_B_dps = ka_ny * (nyCmd_B_fps2 - ny_B_fps2)

        pCmd_error_dps = pCmd_B_dps - p_B_dps
        qCmd_error_dps = qCmd_B_dps - q_B_dps
        rCmd_error_dps = rCmd_B_dps - r_B_dps

        self.pCmd_errorInt += pCmd_error_dps * dt_s
        self.qCmd_errorInt += qCmd_error_dps * dt_s
        self.rCmd_errorInt += rCmd_error_dps * dt_s

        # inner loop
        rollCommand = kd_p * (ki_p * self.pCmd_errorInt - p_B_dps)
        pitchCommand = kd_q * (ki_q * self.qCmd_errorInt - q_B_dps)
        yawCommand = kd_r * (ki_r * self.rCmd_errorInt - r_B_dps)

        # control allocation
        u = cmn.d2r * \
            mixing @ np.array([[rollCommand], [pitchCommand], [yawCommand]])
        return u


aeroMdl = aeroModeling()
massMdl = massModeling()

# lateral
v_fps = 30
rho_slugft3 = 0.00238
qBar_psf = 0.5*rho_slugft3*v_fps**2
mass_slug = 1.8/32.2
theta_d = 0
alpha_d = 0
gamma_d = 0
g_fps2 = cmn.g_fps2

xcg_ft = 11.25*cmn.in2ft
zcg_ft = 2*cmn.in2ft
l_fus_ft = 3
d_fus_ft = 0.3

cRoot_wing_ft = 7.5*cmn.in2ft
cTip_wing_ft = 7.5*cmn.in2ft
b_wing_ft = 47*cmn.in2ft
sweepLE_wing_d = 0
xLE_wing_ft = 9.5*cmn.in2ft
ybStart_wing_ft = 0.7
ybEnd_wing_ft = 0.9
tau_wing = 0.25

cRoot_ht_ft = 7.5*cmn.in2ft
cTip_ht_ft = 7.5*cmn.in2ft
b_ht_ft = 47*cmn.in2ft
sweepLE_ht_d = 0
xLE_ht_ft = 30.5*cmn.in2ft
ybStart_ht_ft = 0.7
ybEnd_ht_ft = 0.9
tau_ht = 0.25

cRoot_vt_ft = 5*cmn.in2ft
cTip_vt_ft = 3*cmn.in2ft
b_vt_ft = 8*cmn.in2ft
sweepLE_vt_d = 20
xLE_vt_ft = 30.5*cmn.in2ft
ybStart_vt_ft = 0.7
ybEnd_vt_ft = 0.9
tau_vt = 0.25

s_wing_ft2 = aeroMdl.planformArea(b_wing_ft, cRoot_wing_ft, cTip_wing_ft)
s_ht_ft2 = aeroMdl.planformArea(b_ht_ft, cRoot_ht_ft, cTip_ht_ft)
s_vt_ft2 = aeroMdl.planformArea(b_vt_ft, cRoot_vt_ft, cTip_vt_ft)

aspectRatio_wing = aeroMdl.aspectRatio(b_wing_ft, s_wing_ft2)
aspectRatio_ht = aeroMdl.aspectRatio(b_ht_ft, s_ht_ft2)
aspectRatio_vt = aeroMdl.aspectRatio(b_vt_ft, s_vt_ft2)

taper_wing = aeroMdl.taperRatio(cRoot_wing_ft, cTip_wing_ft)
taper_ht = aeroMdl.taperRatio(cRoot_ht_ft, cTip_ht_ft)
taper_vt = aeroMdl.taperRatio(cRoot_vt_ft, cTip_vt_ft)

CLa_wing = aeroMdl.polhamus(aspectRatio_wing)
CLa_ht = aeroMdl.polhamus(aspectRatio_ht)
CLa_vt = aeroMdl.polhamus(aspectRatio_vt)

mac_wing_ft = aeroMdl.meanAerodynamicChord(cRoot_wing_ft, taper_wing)
mac_ht_ft = aeroMdl.meanAerodynamicChord(cRoot_ht_ft, taper_ht)
mac_vt_ft = aeroMdl.meanAerodynamicChord(cRoot_vt_ft, taper_vt)

yMac_wing_ft = aeroMdl.meanAerodynamicChordSpanwise(
    b_wing_ft, cRoot_wing_ft, taper_wing)
yMac_ht_ft = aeroMdl.meanAerodynamicChordSpanwise(
    b_ht_ft, cRoot_ht_ft, taper_ht)
yMac_vt_ft = aeroMdl.meanAerodynamicChordSpanwise(
    b_vt_ft, cRoot_vt_ft, taper_vt)

xac_wing_ft = (xLE_wing_ft + yMac_wing_ft *
               np.tan(np.deg2rad(sweepLE_wing_d)) + 0.25*mac_wing_ft)
xac_ht_ft = (xLE_ht_ft + yMac_ht_ft *
             np.tan(np.deg2rad(sweepLE_ht_d)) + 0.25*mac_ht_ft)
xac_vt_ft = (xLE_vt_ft + yMac_vt_ft *
             np.tan(np.deg2rad(sweepLE_vt_d)) + 0.25*mac_vt_ft)

Xv = xac_vt_ft - xcg_ft
Zv = yMac_vt_ft - zcg_ft

Cy_beta_w = 0
Cy_beta_b = -CLa_vt*l_fus_ft*d_fus_ft/s_wing_ft2
Cy_beta_vt = -CLa_vt * s_vt_ft2 / s_wing_ft2

I_xx_slugft2 = massMdl.axialMassMomentInertiaCylinder(
    mass_slug, cRoot_wing_ft)
I_yy_slugft2 = massMdl.longitudinalMassMomentInertiaCylinder(
    mass_slug, cRoot_wing_ft, b_wing_ft)
I_zz_slugft2 = I_yy_slugft2
I_xz_slugft2 = 0

Cy_beta = Cy_beta_w + Cy_beta_vt + Cy_beta_b
Cy_p = 2 * Cy_beta_vt * (Zv*np.cos(cmn.d2r*alpha_d) -
                         Xv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft
Cy_r = -2 * Cy_beta_vt * (Xv*np.cos(cmn.d2r*alpha_d) +
                          Zv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft
Cy_ail = 0
Cy_rud = CLa_vt * s_vt_ft2 / s_wing_ft2 * tau_vt

Cr_beta = - CLa_vt * s_vt_ft2 / s_wing_ft2 * (Zv*np.cos(cmn.d2r*alpha_d) -
                                              Xv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft
Cr_p = (2*Cy_beta_vt * (Zv/b_wing_ft)**2)
Cr_r = -2*Cy_beta_vt * (Xv*np.cos(cmn.d2r*alpha_d) +
                        Zv*np.sin(cmn.d2r*alpha_d)) * (Zv*np.cos(cmn.d2r*alpha_d) +
                                                       Xv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft**2
Cr_ail = 0.229
Cr_rud = Cy_rud * (Zv*np.cos(cmn.d2r*alpha_d) -
                   Xv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft

Cn_beta = (CLa_vt * s_vt_ft2 / s_wing_ft2) * (Xv*np.cos(cmn.d2r*alpha_d) +
                                              Zv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft
Cn_p = -2*Cy_beta_vt * (Xv*np.cos(cmn.d2r*alpha_d) +
                        Zv*np.sin(cmn.d2r*alpha_d)) * (Zv*np.cos(cmn.d2r*alpha_d) -
                                                       Xv*np.sin(cmn.d2r*alpha_d) - Zv) / b_wing_ft**2
Cn_r = 2*Cy_beta_vt*(Xv*np.cos(cmn.d2r*alpha_d) +
                     Zv*np.sin(cmn.d2r*alpha_d)) / b_wing_ft**2
Cn_ail = -0.0216
Cn_rud = -CLa_vt * s_vt_ft2 / s_wing_ft2 * tau_vt * (Xv*np.cos(cmn.d2r*alpha_d) +
                                                     Zv*np.sin(cmn.d2r*alpha_d))/b_wing_ft

inertia_B_slugft2 = np.array(
    [[I_xx_slugft2, 0, -I_xz_slugft2], [0, I_yy_slugft2, 0], [-I_xz_slugft2, 0, I_zz_slugft2]])
inertia_S_slugft2 = inertia_B_slugft2
invInertia_S_slugft2 = np.linalg.inv(inertia_S_slugft2)

Y_beta = qBar_psf * s_wing_ft2 * Cy_beta / mass_slug
Y_p = qBar_psf * s_wing_ft2 * b_wing_ft * Cy_p / (2 * mass_slug * v_fps)
Y_r = qBar_psf * s_wing_ft2 * b_wing_ft * Cy_r / (2 * mass_slug * v_fps)
Y_ail = qBar_psf * s_wing_ft2 * Cy_ail / mass_slug
Y_rud = qBar_psf * s_wing_ft2 * Cy_rud / mass_slug

tmp_beta = qBar_psf * s_wing_ft2 * b_wing_ft * \
    invInertia_S_slugft2 @ np.array([[Cr_beta], [0], [Cn_beta]])
tmp_p = (qBar_psf * s_wing_ft2 * b_wing_ft * b_wing_ft / (2 * v_fps)) * \
    invInertia_S_slugft2 @ np.array([[Cr_p], [0], [Cn_p]])
tmp_r = (qBar_psf * s_wing_ft2 * b_wing_ft * b_wing_ft / (2 * v_fps)) * \
    invInertia_S_slugft2 @ np.array([[Cr_r], [0], [Cn_r]])
tmp_ail = qBar_psf * s_wing_ft2 * b_wing_ft * \
    invInertia_S_slugft2 @ np.array([[Cr_ail], [0], [Cn_ail]])
tmp_rud = qBar_psf * s_wing_ft2 * b_wing_ft * \
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
              [0, 0, 0, 1],
              [Y_beta, 0, 0, 0]])
D = np.array([[0, 0],
              [0, 0],
              [0, 0],
              [0, 0],
              [Y_ail, Y_rud]])

t_s = np.arange(0, 10 + 0.01, 0.01)
fcs = controller()
beta = []
phi_d = []
p_rps = []
r_rps = []
ny_fps2 = []
aileon_d = []
rudder_d = []
gdcCmd = np.array([0, 0, 0])
fcsIn = np.array([0, 0, 0, 0, 0, 0])
x = np.array([[0.0], [0.0], [0.0], [0.0]])
dt_s = 0.01
for i in range(len(t_s)):
    if t_s[i] > 1:
        gdcCmd = np.array([0, 0, 5])

    uOut = fcs.step(fcsIn, gdcCmd, 0.01)
    u = np.array([[uOut[0][0]], [uOut[2][0]]])
    xDot = A @ x + B @ u
    x += xDot * dt_s
    y = C @ x + D @ u

    fcsIn = np.array([y[2][0]*cmn.r2d, 0, y[3][0] *
                     cmn.r2d, y[1][0]*cmn.r2d, y[4][0], 0])

    beta.append(y[0][0])
    phi_d.append(y[1][0]*cmn.r2d)
    p_rps.append(y[2][0])
    r_rps.append(y[3][0])
    ny_fps2.append(y[4][0])
    aileon_d.append(u[0][0]*cmn.r2d)
    rudder_d.append(u[1][0]*cmn.r2d)

plt.figure()
plt.plot(t_s, beta)
plt.xlabel('Time (s)')
plt.ylabel('Beta (deg)')
plt.show()
plt.figure()
plt.plot(t_s, phi_d)
plt.xlabel('Time (s)')
plt.ylabel('phi (d)')
plt.show()
plt.figure()
plt.plot(t_s, p_rps)
plt.xlabel('Time (s)')
plt.ylabel('P (deg/s)')
plt.show()
plt.figure()
plt.plot(t_s, r_rps)
plt.xlabel('Time (s)')
plt.ylabel('Roll Rate (deg/s)')
plt.show()
plt.figure()
plt.plot(t_s, ny_fps2)
plt.xlabel('Time (s)')
plt.ylabel('Ny (fps^2)')
plt.show()
plt.figure()
plt.plot(t_s, aileon_d)
plt.xlabel('Time (s)')
plt.ylabel('Aileon (deg)')
plt.show()
plt.figure()
plt.plot(t_s, rudder_d)
plt.xlabel('Time (s)')
plt.ylabel('Rudder (deg)')
plt.show()
