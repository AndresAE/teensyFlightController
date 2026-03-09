from plant.aerodynamics.aeroModeling import aeroModeling
from plant.mass.massModeling import massModeling
from plant.eom.eom import eom
import matplotlib.pyplot as plt
import numpy as np
from common.common import common

cmn = common()
aeroMdl = aeroModeling()
massMdl = massModeling()
eomObj = eom()


class controller():
    def __init__(self):
        self.pCmd_errorInt = 0
        self.qCmd_errorInt = 0
        self.rCmd_errorInt = 0
        self.rollCommandPrev = 0
        self.pitchCommandPrev = 0
        self.yawCommandPrev = 0
        self.saturated = False

    def step(self, x, u_cmd, dt_s):
        ka_phi = 5
        ka_ny = 50
        kd_p = 1  # 2.43
        ki_p = 1/100
        kd_r = -1  # 1.75
        ki_r = 2

        pitchLimit = 30
        ka_nz = -1.5
        kd_q = 0.5  # 1.58
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
        nzCmd_B_fps2 = u_cmd[2]
        nyCmd_B_fps2 = u_cmd[1]

        # middle loop
        pCmd_B_dps = ka_phi * (phiCmd_d - phi_d)
        qCmd_B_dps = ka_nz * (nzCmd_B_fps2 - nz_B_fps2)
        rCmd_B_dps = ka_ny * (nyCmd_B_fps2 - ny_B_fps2)

        pCmd_error_dps = pCmd_B_dps - p_B_dps
        qCmd_error_dps = qCmd_B_dps - q_B_dps
        rCmd_error_dps = rCmd_B_dps - r_B_dps

        if (self.saturated) and (np.sign(self.pitchCommandPrev) == np.sign(qCmd_error_dps)):
            self.pCmd_errorInt = 0
            self.qCmd_errorInt = (
                self.pitchCommandPrev / kd_q + q_B_dps) / ki_q
            self.rCmd_errorInt = 0
            self.pCmd_errorInt += pCmd_error_dps * dt_s
            self.qCmd_errorInt += qCmd_error_dps * dt_s
            self.rCmd_errorInt += rCmd_error_dps * dt_s
        else:
            self.pCmd_errorInt += pCmd_error_dps * dt_s
            self.qCmd_errorInt += qCmd_error_dps * dt_s
            self.rCmd_errorInt += rCmd_error_dps * dt_s

        # inner loop
        rollCommand = kd_p * (ki_p * self.pCmd_errorInt - p_B_dps)
        pitchCommand = kd_q * (ki_q * self.qCmd_errorInt - q_B_dps)
        yawCommand = kd_r * (ki_r * self.rCmd_errorInt - r_B_dps)

        if pitchCommand > pitchLimit:
            pitchCommand = pitchLimit
            self.saturated = True
        elif pitchCommand < -pitchLimit:
            pitchCommand = -pitchLimit
            self.saturated = True
        else:
            self.saturated = False

        self.rollCommandPrev = rollCommand
        self.pitchCommandPrev = pitchCommand
        self.yawCommandPrev = yawCommand

        # control allocation
        u = -cmn.d2r * \
            mixing @ np.array([[rollCommand], [pitchCommand], [yawCommand]])
        return u, qCmd_B_dps


# long
v_fps = 30
rho_slugft3 = 0.00238
qBar_psf = 0.5*rho_slugft3*v_fps**2

# estimates
weight_lb = 1.5
xcg_ft = 11.25*cmn.in2ft

cRoot_wing_ft = 7.5*cmn.in2ft
cTip_wing_ft = 7.5*cmn.in2ft
b_wing_ft = 47*cmn.in2ft
sweepLE_wing_d = 0
xLE_wing_ft = 9.5*cmn.in2ft
tau_wing = 0.25

cRoot_ht_ft = 5*cmn.in2ft
cTip_ht_ft = 5*cmn.in2ft
b_ht_ft = 16.5*cmn.in2ft
sweepLE_ht_d = 0
xLE_ht_ft = 30.5*cmn.in2ft
ybStart_ht_ft = 0.7
ybEnd_ht_ft = 0.9
tau_ht = 1.5/5

s_wing_ft2 = aeroMdl.planformArea(b_wing_ft, cRoot_wing_ft, cTip_wing_ft)
taper_wing = aeroMdl.taperRatio(cRoot_wing_ft, cTip_wing_ft)
aspectRatio_wing = aeroMdl.aspectRatio(b_wing_ft, s_wing_ft2)
CLa_wing = aeroMdl.polhamus(aspectRatio_wing)
mac_wing_ft = aeroMdl.meanAerodynamicChord(cRoot_wing_ft, taper_wing)
yMac_wing_ft = aeroMdl.meanAerodynamicChordSpanwise(
    b_wing_ft, cRoot_wing_ft, taper_wing)
xacHat_wing_ft = (xLE_wing_ft + yMac_wing_ft *
                  np.tan(np.deg2rad(sweepLE_wing_d)) + 0.25*mac_wing_ft) / mac_wing_ft
Sref_ft2 = s_wing_ft2
xRef_ft = mac_wing_ft
xcgHat = xcg_ft / mac_wing_ft

s_ht_ft2 = aeroMdl.planformArea(b_ht_ft, cRoot_ht_ft, cTip_ht_ft)
taper_ht = aeroMdl.taperRatio(cRoot_ht_ft, cTip_ht_ft)
aspectRatio_ht = aeroMdl.aspectRatio(b_ht_ft, s_ht_ft2)
CLa_ht = aeroMdl.polhamus(aspectRatio_ht)
mac_ht_ft = aeroMdl.meanAerodynamicChord(cRoot_ht_ft, taper_ht)
yMac_ht_ft = aeroMdl.meanAerodynamicChordSpanwise(
    b_ht_ft, cRoot_ht_ft, taper_ht)
xacHat_ht_ft = (xLE_ht_ft + yMac_ht_ft *
                np.tan(np.deg2rad(sweepLE_ht_d)) + 0.25*mac_ht_ft) / mac_wing_ft
CMq = -2 * CLa_ht * (s_ht_ft2/s_wing_ft2) * (xcgHat-xacHat_ht_ft)**2

mass_slug = weight_lb / 32.2
I_yy_slugft2 = massMdl.longitudinalMassMomentInertiaCylinder(
    mass_slug, xRef_ft, b_wing_ft)

CL_alpha = CLa_wing + CLa_ht * (s_ht_ft2/s_wing_ft2)
CL_delta = CLa_ht * (s_ht_ft2/s_wing_ft2) * tau_ht

CM_alpha = CLa_wing * (xcgHat - xacHat_wing_ft) + CLa_ht * \
    (s_ht_ft2/s_wing_ft2) * (xcgHat - xacHat_ht_ft)
CM_delta = CLa_ht * (s_ht_ft2/s_wing_ft2) * tau_ht * (xcgHat - xacHat_ht_ft)

Cz_alpha = -CL_alpha
Cm_alpha = CM_alpha
Cm_delta = CM_delta
Cz_delta = -CL_delta
Cm_q = CMq

Z_alpha = qBar_psf * Sref_ft2 * Cz_alpha / mass_slug
M_alpha = qBar_psf * Sref_ft2 * xRef_ft * Cm_alpha / I_yy_slugft2
M_q = qBar_psf * Sref_ft2 * xRef_ft * \
    xRef_ft * Cm_q / (I_yy_slugft2 * 2 * v_fps)
Z_delta = qBar_psf * Sref_ft2 * Cz_delta / mass_slug
M_delta = qBar_psf * Sref_ft2 * xRef_ft * Cm_delta / I_yy_slugft2

A = np.array([[Z_alpha/v_fps, 1],
              [M_alpha, M_q]])
B = np.array([[Z_delta/v_fps],
              [M_delta]])
C = np.array([[1, 0],
              [0, 1],
              [Z_alpha, 0]])
D = np.array([[0],
              [0],
              [Z_delta]])

t_s = np.arange(0, 10 + 0.01, 0.01)
fcs = controller()
alpha_d = []
q_dps = []
nz_fps2 = []
elevator_d = []
intPitch = []
qCmdOut = []
uFCSOut = []
satOut = []
gdcCmd = np.array([0, 0, 0])
fcsIn = np.array([0, 0, 0, 0, 0, 0])
x = np.array([[0.0], [0.0]])
dt_s = 0.01
for i in range(len(t_s)):
    if (t_s[i] > 1) and (t_s[i] < 5):
        gdcCmd = np.array([0, 0, -10])
        # dele = -5*cmn.d2r
    elif t_s[i] >= 5:
        gdcCmd = np.array([0, 0, 0])
        # dele = 0
    else:
        gdcCmd = np.array([0, 0, 0])
        # dele = 0

    uOut, qCmd_B_dps = fcs.step(fcsIn, gdcCmd, 0.01)
    u = np.array([[uOut[1][0]]])
    # u = np.array([[dele]])
    xDot = A @ x + B @ u
    x += xDot * dt_s
    y = C @ x + D @ u

    fcsIn = np.array([0, y[1][0]*cmn.r2d, 0, 0, 0, y[2][0]])

    alpha_d.append(y[0][0]*cmn.r2d)
    q_dps.append(y[1][0]*cmn.r2d)
    nz_fps2.append(y[2][0])
    elevator_d.append(uOut[1][0]*cmn.r2d)
    intPitch.append(fcs.qCmd_errorInt)
    satOut.append(fcs.saturated)
    uFCSOut.append(uOut[1][0]*cmn.r2d)
    qCmdOut.append(qCmd_B_dps)

plt.figure()
plt.plot(t_s, alpha_d)
plt.xlabel('Time (s)')
plt.ylabel('Alpha (deg)')
plt.show()
plt.figure()
plt.plot(t_s, nz_fps2)
plt.xlabel('Time (s)')
plt.ylabel('Nz (fps^2)')
plt.show()
plt.figure()
plt.plot(t_s, elevator_d)
plt.xlabel('Time (s)')
plt.ylabel('Elevator (deg)')
plt.show()
plt.figure()
plt.plot(t_s, intPitch)
plt.xlabel('Time (s)')
plt.ylabel('int pitch')
plt.show()
# plt.figure()
# plt.plot(t_s, uFCSOut)
# plt.xlabel('Time (s)')
# plt.ylabel('pitch FCS [d]')
plt.show()
plt.figure()
plt.plot(t_s, qCmdOut)
plt.plot(t_s, q_dps)
plt.xlabel('Time (s)')
plt.ylabel('pitch rate')
plt.show()
plt.figure()
plt.plot(t_s, satOut)
plt.xlabel('Time (s)')
plt.ylabel('sat')
plt.show()
