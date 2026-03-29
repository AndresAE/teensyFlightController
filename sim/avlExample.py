
import numpy as np
import avlwrapper as avl
from common.common import common
from plant.earth.atmosphere import atmosphere

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


b_wing_m = 47*cmn.in2ft*cmn.ft2m
cRoot_wing_m = 7.5*cmn.in2ft*cmn.ft2m
cTip_wing_m = 6.0*cmn.in2ft*cmn.ft2m
xLE_wing_m = 9.5*cmn.in2ft*cmn.ft2m
sweepLE_wing_r = np.radians(0)
wing_dihedral_r = np.radians(0)

s_wing_m2 = b_wing_m*(cRoot_wing_m+cTip_wing_m)/2
wing_taper = cTip_wing_m / cRoot_wing_m
wing_mac_m = ((2 * cRoot_wing_m/3) *
              (1 + wing_taper+wing_taper**2) /
              (1+wing_taper))

wing_root_le_pnt = avl.Point(xLE_wing_m, 0, 3*cmn.in2ft*cmn.ft2m)
wing_tip_le_pnt = avl.Point(x=xLE_wing_m + 0.5 * b_wing_m * np.tan(sweepLE_wing_r),
                            y=0.5 * b_wing_m,
                            z=0.5 * b_wing_m * np.tan(wing_dihedral_r)+3*cmn.in2ft*cmn.ft2m)
wing_tip2_le_pnt = avl.Point(x=xLE_wing_m + 0.5 * b_wing_m * np.tan(sweepLE_wing_r),
                             y=b_wing_m,
                             z=0.5 * b_wing_m * np.tan(wing_dihedral_r)+3*cmn.in2ft*cmn.ft2m)

root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                           chord=cRoot_wing_m,
                           airfoil=avl.NacaAirfoil('2414'))
tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                          chord=cTip_wing_m,
                          airfoil=avl.NacaAirfoil('2410'))
tip2_section = avl.Section(leading_edge_point=wing_tip2_le_pnt,
                           chord=cTip_wing_m*0.5,
                           airfoil=avl.NacaAirfoil('2410'))

# y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
wing = avl.Surface(name='wing',
                   n_chordwise=12,
                   chord_spacing=avl.Spacing.cosine,
                   n_spanwise=20,
                   span_spacing=avl.Spacing.cosine,
                   y_duplicate=0.0,
                   sections=[root_section, tip_section, tip2_section])

cRoot_ht_m = 5*cmn.in2ft*cmn.ft2m
cTip_ht_m = 3*cmn.in2ft*cmn.ft2m
b_ht_m = 16.5*cmn.in2ft*cmn.ft2m
xLE_ht_m = 30.5*cmn.in2ft*cmn.ft2m
sweepLE_ht_r = np.radians(10)
ht_dihedral_r = np.radians(0)

elevator = avl.Control(name='elevator',
                       gain=1.0,
                       x_hinge=0.6,
                       duplicate_sign=1)

ht_root_le_pnt = avl.Point(xLE_ht_m, 0, 0)
ht_tip_le_pnt = avl.Point(x=ht_root_le_pnt.x + 0.5*cRoot_ht_m*np.tan(sweepLE_ht_r),
                          y=0.5*b_ht_m,
                          z=ht_root_le_pnt.z + 0.5*cRoot_ht_m*np.tan(ht_dihedral_r))

root_section = avl.Section(leading_edge_point=ht_root_le_pnt,
                           chord=cRoot_ht_m,
                           airfoil=avl.NacaAirfoil('0012'),
                           controls=[elevator])
tip_section = avl.Section(leading_edge_point=ht_tip_le_pnt,
                          chord=cTip_ht_m,
                          airfoil=avl.NacaAirfoil('0012'),
                          controls=[elevator])
horizontal_tail = avl.Surface(name='horizontal_tail',
                              n_chordwise=12,
                              chord_spacing=avl.Spacing.cosine,
                              n_spanwise=20,
                              span_spacing=avl.Spacing.cosine,
                              y_duplicate=0.0,
                              sections=[root_section, tip_section])

# vertical tail
cRoot_vt_m = 5*cmn.in2ft*cmn.ft2m
cTip_vt_m = 3*cmn.in2ft*cmn.ft2m
b_vt_m = 9*cmn.in2ft*cmn.ft2m
xLE_vt_m = 27*cmn.in2ft*cmn.ft2m
sweepLE_vt_r = np.radians(20)

rudder = avl.Control(name='rudder',
                     gain=1.0,
                     x_hinge=0.6,
                     duplicate_sign=1)

vt_root_le_pnt = avl.Point(xLE_vt_m, 0, 0)
vt_tip_le_pnt = avl.Point(x=vt_root_le_pnt.x + 0.5*cRoot_vt_m*np.tan(sweepLE_vt_r),
                          y=0,
                          z=b_vt_m)

root_section = avl.Section(leading_edge_point=vt_root_le_pnt,
                           chord=cRoot_vt_m,
                           airfoil=avl.NacaAirfoil('0012'),
                           controls=[rudder])
tip_section = avl.Section(leading_edge_point=vt_tip_le_pnt,
                          chord=cTip_vt_m,
                          airfoil=avl.NacaAirfoil('0012'),
                          controls=[rudder])
vertical_tail = avl.Surface(name='vertical_tail',
                            n_chordwise=12,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=20,
                            span_spacing=avl.Spacing.cosine,
                            sections=[root_section, tip_section])

velocity_mps = 20*cmn.ft2m
atm.altitudeMSL_ft = 0.0
a_fps = atm.speed_of_sound()
mach = velocity_mps / (a_fps * cmn.ft2m)
rho_kgm2 = atm.air_density() * cmn.slug2kg / (cmn.ft2m**3)

# calculate the m.a.c. leading edge location
ref_pnt = avl.Point(x=13*cmn.in2ft*cmn.ft2m,
                    y=0.0, z=0.0)

aircraft = avl.Aircraft(name='aircraft',
                        reference_area=s_wing_m2,
                        reference_chord=wing_mac_m,
                        reference_span=b_wing_m,
                        reference_point=ref_pnt,
                        surfaces=[wing, horizontal_tail, vertical_tail])

# create a session with only the geometry
session = avl.Session(geometry=aircraft)

# check if we have ghostscript
if 'gs_bin' in session.config.settings:
    img = session.save_geometry_plot()[0]
    avl.show_image(img)
else:
    session.show_geometry()

alphas = np.arange(-10, 21, 5)
betas = np.arange(-10, 11, 5)

coef = np.zeros((len(alphas), len(betas), 6))

for ii in range(len(alphas)):
    caseList = []
    for jj in range(len(betas)):
        caseList.append(avl.Case(name='baseline',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas[ii],
                                 beta=betas[jj],
                                 elevator=0.0,
                                 rudder=0))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(betas)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coef[ii, kk, 0] = body['CDtot']
        coef[ii, kk, 1] = body['CYtot']
        coef[ii, kk, 2] = body['CLtot']
        coef[ii, kk, 3] = body['Cltot']
        coef[ii, kk, 4] = body['Cmtot']
        coef[ii, kk, 5] = body['Cntot']

eles = np.arange(-30, 31, 5)
coefEle = np.zeros((len(alphas), len(eles), 6))

# elevator
for ii in range(len(alphas)):
    caseList = []
    for jj in range(len(eles)):
        caseList.append(avl.Case(name='elevator',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas[ii],
                                 beta=0,
                                 elevator=eles[jj],
                                 rudder=0))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(eles)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coefEle[ii, kk, 0] = body['CDtot']
        coefEle[ii, kk, 1] = body['CYtot']
        coefEle[ii, kk, 2] = body['CLtot']
        coefEle[ii, kk, 3] = body['Cltot']
        coefEle[ii, kk, 4] = body['Cmtot']
        coefEle[ii, kk, 5] = body['Cntot']

# rudder
ruds = np.arange(-30, 31, 5)
coefRud = np.zeros((len(alphas), len(ruds), 6))

for ii in range(len(alphas)):
    caseList = []
    for jj in range(len(ruds)):
        caseList.append(avl.Case(name='rudder',
                                 mach=mach,
                                 velocity=velocity_mps,
                                 density=rho_kgm2,
                                 roll_rate=0.0,
                                 pitch_rate=0.0,
                                 yaw_rate=0.0,
                                 alpha=alphas[ii],
                                 beta=0,
                                 elevator=0,
                                 rudder=ruds[jj]))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    for kk in range(len(ruds)):
        case_data = result[kk+1]
        body = case_data["Totals"]

        coefRud[ii, kk, 0] = body['CDtot']
        coefRud[ii, kk, 1] = body['CYtot']
        coefRud[ii, kk, 2] = body['CLtot']
        coefRud[ii, kk, 3] = body['Cltot']
        coefRud[ii, kk, 4] = body['Cmtot']
        coefRud[ii, kk, 5] = body['Cntot']

# roll rate
rollRate = 0.01
coefRollRate = np.zeros((len(alphas), 6))
caseList = []

for ii in range(len(alphas)):
    caseList.append(avl.Case(name='rollRate',
                             mach=mach,
                             velocity=velocity_mps,
                             density=rho_kgm2,
                             roll_rate=rollRate,
                             pitch_rate=0.0,
                             yaw_rate=0.0,
                             alpha=alphas[ii],
                             beta=0,
                             elevator=0,
                             rudder=0.0))

    session = avl.Session(geometry=aircraft, cases=caseList)
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
pitchRate = 0.01
coefPitchRate = np.zeros((len(alphas), 6))
caseList = []

for ii in range(len(alphas)):
    caseList.append(avl.Case(name='pitchRate',
                             mach=mach,
                             velocity=velocity_mps,
                             density=rho_kgm2,
                             roll_rate=0.0,
                             pitch_rate=pitchRate,
                             yaw_rate=0.0,
                             alpha=alphas[ii],
                             beta=0.0,
                             elevator=0.0,
                             rudder=0.0))

    session = avl.Session(geometry=aircraft, cases=caseList)
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
yawRate = 0.01
coefYawRate = np.zeros((len(alphas), 6))
caseList = []

for ii in range(len(alphas)):
    caseList.append(avl.Case(name='yawRate',
                             mach=mach,
                             velocity=velocity_mps,
                             density=rho_kgm2,
                             roll_rate=0.0,
                             pitch_rate=0.0,
                             yaw_rate=yawRate,
                             alpha=alphas[ii],
                             beta=0.0,
                             elevator=0.0,
                             rudder=0.0))

    session = avl.Session(geometry=aircraft, cases=caseList)
    result = session.run_all_cases()

    case_data = result[1]
    body = case_data["Totals"]

    coefYawRate[ii, 0] = body['CDtot']
    coefYawRate[ii, 1] = body['CYtot']
    coefYawRate[ii, 2] = body['CLtot']
    coefYawRate[ii, 3] = body['Cltot']
    coefYawRate[ii, 4] = body['Cmtot']
    coefYawRate[ii, 5] = body['Cntot']

a = 1
