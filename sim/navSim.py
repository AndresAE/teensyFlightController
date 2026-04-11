import numpy as np
from plant.eom import eom
import matplotlib.pyplot as plt
from plant.aerodynamics.aeroModel import aeroModel
from plant.earth.atmosphere import atmosphere
from fcs import fcs

if __name__ == "__main__":
    sim = eom()
    aeroMdl = aeroModel()
    atmMdl = atmosphere()
    apt = fcs()
    sim.dt_s = 0.01  # Set the time step to 20 ms
    # Initial LLA (San Francisco, 1000 ft altitude)
    sim.lla_ddft = np.array([0, 0, 0.0])
    # Initial velocity in body frame (ft/s)
    sim.vBE_B_fps = np.array([40.0, 0.0, 5.0])
    # Initial orientation (roll, pitch, yaw in degrees)
    sim.euler_d = np.array([0.0, 5.0, 0.0])
    sim.wBG_B_rps = -sim.earth.ned_angular_velocity(sim.lla_ddft, sim.vBE_B_fps, sim.cmn.ned2bodyTransformation(
        sim.euler_d))-sim.E2B @ sim.earth.wEI_E_rps  # Initial angular velocity in body frame (rad/s) to counteract NED rotation
    sim.update()  # Update the simulation to initialize intermediate variables

    # Apply gravity force in body frame (lbf)
    sim.f_B_lbf = np.array([0.0, 0.0, 0.0])

    t_sim_s = np.array([])  # Time array for plotting
    alpha = np.array([])  # Latitude array for plotting
    beta = np.array([])  # Longitude array for plotting
    hae_ft = np.array([])  # Height above ellipsoid array for plotting
    vel = np.array([])
    p_dps = np.array([])
    ail_d = np.array([])
    ele_d = np.array([])
    rud_d = np.array([])
    nz_g = np.array([])
    ny_g = np.array([])
    t_s = 0.0
    while t_s < 5.0:  # Run the simulation for 10 seconds
        atmMdl.altitudeMSL_ft = sim.lla_ddft[2]
        rho_slugft2 = atmMdl.air_density()

        aeroMdl.v_fps = np.linalg.norm(sim.vBE_B_fps)
        aeroMdl.alpha_d = np.degrees(
            np.atan2(sim.vBE_B_fps[2], sim.vBE_B_fps[0]))
        aeroMdl.beta_d = np.degrees(np.asin(sim.vBE_B_fps[1]/aeroMdl.v_fps))
        aeroMdl.qBar_PSF = 0.5*rho_slugft2*aeroMdl.v_fps**2
        aeroMdl.aileron_d = apt.aileron_d
        aeroMdl.elevator_d = apt.elevator_d
        aeroMdl.rudder_d = apt.rudder_d
        aeroMdl.p_rps = sim.wBG_B_rps[0]
        aeroMdl.q_rps = sim.wBG_B_rps[1]
        aeroMdl.r_rps = sim.wBG_B_rps[2]
        aeroMdl.cg_B_ft = np.array([-12/12, 0, -2/12])

        aeroMdl.update()

        fg_B_lbf = np.transpose(
            sim.G2B@np.array([[0.0], [0.0], [1.8]]))

        sim.f_B_lbf = aeroMdl.f_B_lbf + fg_B_lbf[0]
        sim.m_B_CG_ftlb = aeroMdl.m_CG_B_ftlbf
        sim.mass_slug = 1.8 / 32.174
        ixx_slugft2 = 0.5*0.625*(5/12)**2
        iyy_slugft2 = (1/12)*0.625*3**2
        izz_slugft2 = (1/12)*0.625*3**2
        sim.j_slugft2 = np.array(
            [[ixx_slugft2, 0, 0], [0, iyy_slugft2, 0], [0, 0, izz_slugft2]])

        sim.step()  # Advance the simulation by one time step

        apt.nzC_B_fps2 = -32.174
        if t_s > 2:
            apt.nyC_B_fps2 = 5.0
        else:
            apt.nyC_B_fps2 = 0.0
        apt.pC_B_dps = 0.0
        apt.wBI_B_dps = sim.wBI_B_rps*sim.cmn.r2d
        apt.specificForce_fps2 = aeroMdl.f_B_lbf / sim.mass_slug
        apt.step()

        # print(
        #     f"HAE (ft): {sim.lla_ddft[2]:.2f}, Velocity (ft/s): {sim.vBE_B_fps[0]:.2f}, Lat (deg): {sim.lla_ddft[0]:.6f}, Pitch (deg): {sim.euler_d[1]:.2f}")
        t_sim_s = np.append(t_sim_s, t_s)
        beta = np.append(beta, aeroMdl.beta_d)
        alpha = np.append(alpha, aeroMdl.alpha_d)
        hae_ft = np.append(hae_ft, sim.lla_ddft[2])
        vel = np.append(vel, aeroMdl.v_fps)
        ail_d = np.append(ail_d, aeroMdl.aileron_d)
        ele_d = np.append(ele_d, aeroMdl.elevator_d)
        rud_d = np.append(rud_d, aeroMdl.rudder_d)
        nz_g = np.append(nz_g, apt.specificForce_fps2[2])
        ny_g = np.append(ny_g, apt.specificForce_fps2[1])
        p_dps = np.append(p_dps, sim.wBG_B_rps[0]*57.3)
        t_s += sim.dt_s

    fig = plt.figure()
    plt.plot(t_sim_s, ail_d)
    plt.title('ail')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, ele_d)
    plt.title('ele')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, rud_d)
    plt.title('rud')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, p_dps)
    plt.title('p_dps')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, beta)
    plt.title('beta')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, ny_g)
    plt.title('ny')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, nz_g)
    plt.title('nz')
    plt.show()

    fig = plt.figure()
    plt.plot(t_sim_s, alpha)
    plt.title('alpha')
    plt.show()

    # fig = plt.figure()
    # plt.plot(t_sim_s, hae_ft)
    # plt.show()

    # fig = plt.figure()
    # plt.plot(t_sim_s, vel)
    # plt.show()
