from sim.plant.aerodynamics.aeroModelingTools import aeroModeling
from plant.mass.massModeling import massModeling
from plant.eom.eom import eom
import matplotlib.pyplot as plt
import numpy as np

aeroMdl = aeroModeling()
massMdl = massModeling()
eomObj = eom()

# long
v_fps = 50
rho_slugft3 = 0.00238
qBar_psf = 0.5*rho_slugft3*v_fps**2

# estimates
weight_lb = 1.5
xcg_ft = 11.25/12

cRoot_wing_ft = 7.5/12
cTip_wing_ft = 7.5/12
b_wing_ft = 47/12
sweepLE_wing_d = 0
xLE_wing_ft = 9.5/12
ybStart_wing_ft = 0.7
ybEnd_wing_ft = 0.9
tau_wing = 0.25

cRoot_ht_ft = 5/12
cTip_ht_ft = 5/12
b_ht_ft = 16.5/12
sweepLE_ht_d = 0
xLE_ht_ft = 30.5/12
ybStart_ht_ft = 0.7
ybEnd_ht_ft = 0.9
tau_ht = 1.5/5

s_wing_ft2 = 0.5*(cRoot_wing_ft+cTip_wing_ft)*b_wing_ft
taper_wing = cTip_wing_ft/cRoot_wing_ft
aspectRatio_wing = b_wing_ft**2/s_wing_ft2
CLa_wing = aeroMdl.polhamus(aspectRatio_wing)
mac_wing_ft = aeroMdl.meanAerodynamicChord(cRoot_wing_ft, taper_wing)
yMac_wing_ft = aeroMdl.meanAerodynamicChordSpanwise(
    b_wing_ft, cRoot_wing_ft, taper_wing)
xacHat_wing_ft = (xLE_wing_ft + yMac_wing_ft *
                  np.tan(np.deg2rad(sweepLE_wing_d)) + 0.25*mac_wing_ft) / mac_wing_ft
Sref_ft2 = s_wing_ft2
xRef_ft = mac_wing_ft
xcgHat = xcg_ft / mac_wing_ft

s_ht_ft2 = 0.5*(cRoot_ht_ft+cTip_ht_ft)*b_ht_ft
taper_ht = cTip_ht_ft/cRoot_ht_ft
aspectRatio_ht = b_ht_ft**2/s_ht_ft2
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

olSys = (A, B, C, D)
t_s = np.arange(0, 10 + 0.01, 0.01)
u = np.zeros_like(t_s)
u[t_s > 1] = -30/57.3

t_out, y_out, x_out = eomObj.lsim(olSys, u.transpose(), t_s.transpose())
plt.figure()
plt.plot(t_out, y_out[:, 0]*57.3)
plt.show()
plt.figure()
plt.plot(t_out, y_out[:, 2])
plt.show()

Ka = -0.5
Kq = -0.2
Ki = 0.1

Acl = np.array([[A[0, 0], A[0, 1], 0, B[0, 0]],
                [A[1, 0], A[1, 1], 0, B[1, 0]],
                [-Ka*C[2, 0], -1, 0, -Ka*D[2, 0]],
                [-Kq*A[1, 0]-Kq*Ki*Ka*C[2, 0], -Kq*A[1, 1]-Kq*Ki, 0, - Kq*B[1, 0]-Kq*Ki*Ka*D[2, 0]]])

Bcl = np.array([[0],
                [0],
                [Ka],
                [Kq*Ki*Ka]])

Ccl = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [C[2, 0], 0, 0, D[2, 0]],
                [0, 0, 0, 1]])

Dcl = np.array([[0],
                [0],
                [0],
                [0]])

clSys = (Acl, Bcl, Ccl, Dcl)
t_s = np.arange(0, 10 + 0.01, 0.01)
u = np.zeros_like(t_s)
u[t_s > 1] = -5

t_out, y_out, x_out = eomObj.lsim(clSys, u.transpose(), t_s.transpose())
plt.figure()
plt.plot(t_s, y_out[:, 2])
plt.show()
plt.figure()
plt.plot(t_s, y_out[:, 0]*57.3)
plt.show()
plt.figure()
plt.plot(t_s, y_out[:, 1])
plt.show()
plt.figure()
plt.plot(t_s, y_out[:, 3]*57.3)
plt.show()
