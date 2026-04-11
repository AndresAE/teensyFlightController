
import numpy as np
import avlwrapper as avl
from common.common import common
from plant.earth.atmosphere import atmosphere
from matplotlib import pyplot as plt

cmn = common()
atm = atmosphere()


def show_treffz(session):
    """function for showing the Trefftz plot."""
    if 'gs_bin' in session.config.settings:
        images = session.save_trefftz_plots()
        for img in images:
            avl.show_image(img)
    else:
        for idx, _ in enumerate(session.cases):
            session.show_trefftz_plot(idx+1)  # cases start from 1


b_aileronRoot_m = 7.0*cmn.in2ft*cmn.ft2m
b_aileronTip_m = 9.75*cmn.in2ft*cmn.ft2m+b_aileronRoot_m
b_wingTip_m = 6.5*cmn.in2ft*cmn.ft2m+b_aileronTip_m
c_aileronRoot_m = 7.5*cmn.in2ft*cmn.ft2m
c_aileronTip_m = 7.5*cmn.in2ft*cmn.ft2m
c_wingTip_m = 7.5*cmn.in2ft*cmn.ft2m
xLE_wing_m = 9.5*cmn.in2ft*cmn.ft2m
zLE_wing_m = 3.5*cmn.in2ft*cmn.ft2m
c_hingeRoot_m = 5.75*cmn.in2ft*cmn.ft2m
c_hingeTip_m = 5.75*cmn.in2ft*cmn.ft2m

s_wing_m2 = b_wingTip_m*(c_aileronRoot_m+c_wingTip_m)
wing_taper = c_wingTip_m / c_aileronRoot_m
wing_mac_m = ((2 * c_aileronRoot_m/3) *
              (1 + wing_taper+wing_taper**2) /
              (1+wing_taper))

wing_root_le_pnt = avl.Point(xLE_wing_m, 0, zLE_wing_m)
wing_aileronRoot_le_pnt = avl.Point(xLE_wing_m, b_aileronRoot_m, zLE_wing_m)
wing_aileronTip_le_pnt = avl.Point(xLE_wing_m, b_aileronTip_m, zLE_wing_m)
wing_tip_le_pnt = avl.Point(xLE_wing_m, b_wingTip_m, zLE_wing_m)

aileron = avl.Control(name='aileron',
                      gain=1.0,
                      x_hinge=c_hingeRoot_m/c_aileronRoot_m,
                      duplicate_sign=-1)

root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                           chord=c_aileronRoot_m,
                           airfoil=avl.NacaAirfoil('2414'))
aileronRoot_section = avl.Section(leading_edge_point=wing_aileronRoot_le_pnt,
                                  chord=c_aileronRoot_m,
                                  airfoil=avl.NacaAirfoil('2410'),
                                  controls=[aileron])
aileronTip_section = avl.Section(leading_edge_point=wing_aileronTip_le_pnt,
                                 chord=c_aileronTip_m,
                                 airfoil=avl.NacaAirfoil('2410'),
                                 controls=[aileron])
tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                          chord=c_wingTip_m,
                          airfoil=avl.NacaAirfoil('2410'))

# y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
wing = avl.Surface(name='wing',
                   n_chordwise=12,
                   chord_spacing=avl.Spacing.cosine,
                   n_spanwise=20,
                   span_spacing=avl.Spacing.cosine,
                   y_duplicate=0.0,
                   sections=[root_section, aileronRoot_section, aileronTip_section, tip_section])

cRoot_ht_m = 5.0*cmn.in2ft*cmn.ft2m
cTip_ht_m = 4.0*cmn.in2ft*cmn.ft2m
b_ht_m = 16.0*cmn.in2ft*cmn.ft2m
c_hingeRootElevator_m = 3.5*cmn.in2ft*cmn.ft2m
c_hingeTipElevator_m = 3.0*cmn.in2ft*cmn.ft2m
xLE_ht_m = 30.0*cmn.in2ft*cmn.ft2m
zLE_ht_m = 1.75*cmn.in2ft*cmn.ft2m
sweepLE_ht_r = np.radians(5.0)

elevator = avl.Control(name='elevator',
                       gain=1.0,
                       x_hinge=c_hingeRootElevator_m/cRoot_ht_m,
                       duplicate_sign=1)

ht_root_le_pnt = avl.Point(xLE_ht_m, 0, zLE_ht_m)
ht_tip_le_pnt = avl.Point(x=ht_root_le_pnt.x + 0.5*b_ht_m*np.tan(sweepLE_ht_r),
                          y=0.5*b_ht_m,
                          z=ht_root_le_pnt.z)

root_section = avl.Section(leading_edge_point=ht_root_le_pnt,
                           chord=cRoot_ht_m,
                           airfoil=avl.NacaAirfoil('0010'),
                           controls=[elevator])
tip_section = avl.Section(leading_edge_point=ht_tip_le_pnt,
                          chord=cTip_ht_m,
                          airfoil=avl.NacaAirfoil('0010'),
                          controls=[elevator])
horizontal_tail = avl.Surface(name='horizontal_tail',
                              n_chordwise=12,
                              chord_spacing=avl.Spacing.cosine,
                              n_spanwise=20,
                              span_spacing=avl.Spacing.cosine,
                              y_duplicate=0.0,
                              sections=[root_section, tip_section])

# vertical tail
cRoot_vt_m = 5.5*cmn.in2ft*cmn.ft2m
cTip_vt_m = 3.5*cmn.in2ft*cmn.ft2m
b_vt_m = 6.0*cmn.in2ft*cmn.ft2m
c_hingeRootRudder_m = 4.5*cmn.in2ft*cmn.ft2m
c_hingeTipRudder_m = 2.65*cmn.in2ft*cmn.ft2m
xLE_vt_m = 29.0*cmn.in2ft*cmn.ft2m
zLE_vt_m = 2.0*cmn.in2ft*cmn.ft2m
sweepLE_vt_r = np.radians(25.0)

rudder = avl.Control(name='rudder',
                     gain=1.0,
                     x_hinge=c_hingeRootRudder_m/cRoot_vt_m,
                     duplicate_sign=1)

vt_root_le_pnt = avl.Point(xLE_vt_m, 0, zLE_vt_m)
vt_tip_le_pnt = avl.Point(x=vt_root_le_pnt.x + 0.5*b_vt_m*np.tan(sweepLE_vt_r),
                          y=0,
                          z=zLE_vt_m+b_vt_m)

root_section = avl.Section(leading_edge_point=vt_root_le_pnt,
                           chord=cRoot_vt_m,
                           airfoil=avl.NacaAirfoil('0010'),
                           controls=[rudder])
tip_section = avl.Section(leading_edge_point=vt_tip_le_pnt,
                          chord=cTip_vt_m,
                          airfoil=avl.NacaAirfoil('0010'),
                          controls=[rudder])
vertical_tail = avl.Surface(name='vertical_tail',
                            n_chordwise=12,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=20,
                            span_spacing=avl.Spacing.cosine,
                            sections=[root_section, tip_section])

velocity_mps = 20.0*cmn.ft2m
atm.altitudeMSL_ft = 0.0
a_fps = atm.speed_of_sound()
mach = velocity_mps / (a_fps * cmn.ft2m)
rho_kgm2 = atm.air_density() * cmn.slug2kg / (cmn.ft2m**3)
weight_n = 1.8 * cmn.lbf2n

# calculate the m.a.c. leading edge location
ref_pnt = avl.Point(x=12*cmn.in2ft*cmn.ft2m,
                    y=0.0, z=2.5*cmn.in2ft*cmn.ft2m)

aircraft = avl.Aircraft(name='aircraft',
                        reference_area=s_wing_m2,
                        reference_chord=wing_mac_m,
                        reference_span=b_wingTip_m*2,
                        reference_point=ref_pnt,
                        surfaces=[wing, horizontal_tail, vertical_tail])

# create a session with only the geometry
session = avl.Session(geometry=aircraft)

session.show_geometry()

alphas_d = np.arange(-10, 21, 5)
betas_d = np.arange(-10, 11, 5)

coef = np.zeros((len(alphas_d), len(betas_d), 6))

for ii in range(len(alphas_d)):
    caseList = []
    for jj in range(len(betas_d)):
        caseList.append(avl.Case(name='baseline',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas_d[ii],
                                 beta=betas_d[jj],
                                 elevator=0.0,
                                 rudder=0.0,
                                 aileron=0.0))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(betas_d)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coef[ii, kk, 0] = body['CDtot']
        coef[ii, kk, 1] = body['CYtot']
        coef[ii, kk, 2] = body['CLtot']
        coef[ii, kk, 3] = body['Cltot']
        coef[ii, kk, 4] = body['Cmtot']
        coef[ii, kk, 5] = body['Cntot']

eles_d = np.arange(-30, 31, 10)
coefEle = np.zeros((len(alphas_d), len(eles_d), 6))

# elevator
for ii in range(len(alphas_d)):
    caseList = []
    for jj in range(len(eles_d)):
        caseList.append(avl.Case(name='elevator',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas_d[ii],
                                 beta=0.0,
                                 elevator=eles_d[jj],
                                 rudder=0.0,
                                 aileron=0.0))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(eles_d)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coefEle[ii, kk, 0] = body['CDtot']
        coefEle[ii, kk, 1] = body['CYtot']
        coefEle[ii, kk, 2] = body['CLtot']
        coefEle[ii, kk, 3] = body['Cltot']
        coefEle[ii, kk, 4] = body['Cmtot']
        coefEle[ii, kk, 5] = body['Cntot']

# rudder
ruds_d = np.arange(-30, 31, 5)
coefRud = np.zeros((len(alphas_d), len(ruds_d), 6))

for ii in range(len(alphas_d)):
    caseList = []
    for jj in range(len(ruds_d)):
        caseList.append(avl.Case(name='rudder',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas_d[ii],
                                 beta=0.0,
                                 elevator=0.0,
                                 rudder=ruds_d[jj],
                                 aileron=0.0))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(ruds_d)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coefRud[ii, kk, 0] = body['CDtot']
        coefRud[ii, kk, 1] = body['CYtot']
        coefRud[ii, kk, 2] = body['CLtot']
        coefRud[ii, kk, 3] = body['Cltot']
        coefRud[ii, kk, 4] = body['Cmtot']
        coefRud[ii, kk, 5] = body['Cntot']

# aileron
ails_d = np.arange(-30, 31, 5)
coefAil = np.zeros((len(alphas_d), len(ails_d), 6))

for ii in range(len(alphas_d)):
    caseList = []
    for jj in range(len(ails_d)):
        caseList.append(avl.Case(name='aileron',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas_d[ii],
                                 beta=0.0,
                                 elevator=0.0,
                                 rudder=0.0,
                                 aileron=-ails_d[jj]))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(ails_d)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coefAil[ii, kk, 0] = body['CDtot']
        coefAil[ii, kk, 1] = body['CYtot']
        coefAil[ii, kk, 2] = body['CLtot']
        coefAil[ii, kk, 3] = body['Cltot']
        coefAil[ii, kk, 4] = body['Cmtot']
        coefAil[ii, kk, 5] = body['Cntot']

# roll rate
rollRate = 0.001
coefRollRate = np.zeros((len(alphas_d), 6))

for ii in range(len(alphas_d)):
    caseList = avl.Case(name='rollRate',
                             mach=mach,
                             velocity=velocity_mps,
                             density=rho_kgm2,
                             roll_rate=rollRate,
                             pitch_rate=0.0,
                             yaw_rate=0.0,
                             alpha=alphas_d[ii],
                             beta=0.0,
                             elevator=0.0,
                             rudder=0.0,
                             aileron=0.0)

    session = avl.Session(geometry=aircraft, cases=[caseList])
    result = session.run_all_cases()

    case_data = result[1]
    body = case_data["Totals"]

    coefRollRate[ii, 0] = body['CDtot']
    coefRollRate[ii, 1] = body['CYtot']
    coefRollRate[ii, 2] = body['CLtot']
    coefRollRate[ii, 3] = body['Cltot']
    coefRollRate[ii, 4] = body['Cmtot']
    coefRollRate[ii, 5] = body['Cntot']

# pitch rate
pitchRate = 0.001
coefPitchRate = np.zeros((len(alphas_d), 6))

for ii in range(len(alphas_d)):
    caseList = avl.Case(name='pitchRate',
                             mach=mach,
                             velocity=velocity_mps,
                             density=rho_kgm2,
                             roll_rate=0.0,
                             pitch_rate=pitchRate,
                             yaw_rate=0.0,
                             alpha=alphas_d[ii],
                             beta=0.0,
                             elevator=0.0,
                             rudder=0.0,
                             aileron=0.0)

    session = avl.Session(geometry=aircraft, cases=[caseList])
    result = session.run_all_cases()

    case_data = result[1]
    body = case_data["Totals"]

    coefPitchRate[ii, 0] = body['CDtot']
    coefPitchRate[ii, 1] = body['CYtot']
    coefPitchRate[ii, 2] = body['CLtot']
    coefPitchRate[ii, 3] = body['Cltot']
    coefPitchRate[ii, 4] = body['Cmtot']
    coefPitchRate[ii, 5] = body['Cntot']

# yaw rate
yawRate = 0.001
coefYawRate = np.zeros((len(alphas_d), 6))

for ii in range(len(alphas_d)):
    caseList = avl.Case(name='yawRate',
                             mach=mach,
                             velocity=velocity_mps,
                             density=rho_kgm2,
                             roll_rate=0.0,
                             pitch_rate=0.0,
                             yaw_rate=yawRate,
                             alpha=alphas_d[ii],
                             beta=0.0,
                             elevator=0.0,
                             rudder=0.0,
                             aileron=0.0)

    session = avl.Session(geometry=aircraft, cases=[caseList])
    result = session.run_all_cases()

    case_data = result[1]
    body = case_data["Totals"]

    coefYawRate[ii, 0] = body['CDtot']
    coefYawRate[ii, 1] = body['CYtot']
    coefYawRate[ii, 2] = body['CLtot']
    coefYawRate[ii, 3] = body['Cltot']
    coefYawRate[ii, 4] = body['Cmtot']
    coefYawRate[ii, 5] = body['Cntot']

# plt.figure()
# for kk in range(len(ruds)):
#     plt.plot(alphas, coefRollRate[:, 1])
# plt.show()

# plt.figure()
# for kk in range(len(ruds)):
#     plt.plot(alphas, coefPitchRate[:, 1])
# plt.show()

# plt.figure()
# for kk in range(len(ruds)):
#     plt.plot(alphas, coefYawRate[:, 1])
# plt.show()

# plt.figure()
# for kk in range(len(ruds)):
#     plt.plot(alphas, coefAil[:, kk, 1])
# plt.show()

# plt.figure()
# for kk in range(len(ruds)):
#     plt.plot(alphas, coefAil[:, kk, 3])
# plt.show()

# plt.figure()
# for kk in range(len(ruds)):
#     plt.plot(alphas, coefAil[:, kk, 5])
# plt.show()

# plt.figure()
# for kk in range(len(alphas)):
#     plt.plot(betas, coef[kk, :, 1])
# plt.show()

# plt.figure()
# for kk in range(len(alphas)):
#     plt.plot(betas, coef[kk, :, 3])
# plt.show()

# plt.figure()
# for kk in range(len(alphas)):
#     plt.plot(betas, coef[kk, :, 5])
# plt.show()

# increments
zeroBetaIdx = np.where(betas_d == 0.0)[0]
incEle = np.zeros((len(alphas_d), len(eles_d), 6))
incRud = np.zeros((len(alphas_d), len(ruds_d), 6))
incAil = np.zeros((len(alphas_d), len(ails_d), 6))
derivRollRate = np.zeros((len(alphas_d), 6))
derivPitchRate = np.zeros((len(alphas_d), 6))
derivYawRate = np.zeros((len(alphas_d), 6))

for kk in range(0, 6):
    for jj in range(len(eles_d)):
        incEle[:, jj, kk] = coefEle[:, jj, kk] - \
            np.transpose(coef[:, zeroBetaIdx, kk])

    for jj in range(len(ruds_d)):
        incRud[:, jj, kk] = coefRud[:, jj, kk] - \
            np.transpose(coef[:, zeroBetaIdx, kk])

    for jj in range(len(ails_d)):
        incAil[:, jj, kk] = coefAil[:, jj, kk] - \
            np.transpose(coef[:, zeroBetaIdx, kk])

    derivRollRate[:, kk] = (coefRollRate[:, kk] -
                            np.transpose(coef[:, zeroBetaIdx, kk])) / rollRate
    derivPitchRate[:, kk] = (coefPitchRate[:, kk] -
                             np.transpose(coef[:, zeroBetaIdx, kk])) / pitchRate
    derivYawRate[:, kk] = (coefYawRate[:, kk] -
                           np.transpose(coef[:, zeroBetaIdx, kk])) / yawRate


aeroModel = {
    "sRef_ft2": s_wing_m2 * cmn.m2ft**2,
    "cRef_ft": wing_mac_m * cmn.m2ft,
    "bRef_ft": b_wingTip_m*2 * cmn.m2ft,
    "MRC_B_ft": np.array([-ref_pnt.x * cmn.m2ft, ref_pnt.y * cmn.m2ft, -ref_pnt.z * cmn.m2ft]),
    "alphaBP_d": alphas_d,
    "betaBP_d": betas_d,
    "aileronBP_d": ails_d,
    "elevatorBP_d": eles_d,
    "rudderBP_d": ruds_d,
    "baseline": coef,
    "aileronInc": incAil,
    "elevatorInc": incEle,
    "rudderInc": incRud,
    "rollRateDeriv": derivRollRate,
    "pitchRateDeriv": derivPitchRate,
    "yawRateDeriv": derivYawRate,
}

np.savez("aero_model.npz", **aeroModel)
