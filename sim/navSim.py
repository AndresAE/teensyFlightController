import numpy as np
from sim.plant.eom.eom import Simulator
import matplotlib.pyplot as plt

if __name__ == "__main__":
    sim = Simulator()
    sim.dt_s = 0.01  # Set the time step to 20 ms
    # Initial LLA (San Francisco, 1000 ft altitude)
    sim.lla_ddft = np.array([0, 0, 40000.0])
    # Initial velocity in body frame (ft/s)
    sim.vBE_B_fps = np.array([80.0, 0.0, 0.0])
    # Initial orientation (roll, pitch, yaw in degrees)
    sim.euler_d = np.array([0.0, 0.0, 0.0])
    sim.wBG_B_rps = -sim.ned_angular_velocity(sim.lla_ddft, sim.vBE_B_fps, sim.ned2bodyTransformation(
        sim.euler_d))-sim.E2B @ sim.wEI_E_rps  # Initial angular velocity in body frame (rad/s) to counteract NED rotation
    sim.update()  # Update the simulation to initialize intermediate variables

    # Apply gravity force in body frame (lbf)
    sim.f_B_lbf = np.array([0.0, -500.0, 0.0])

    t_sim_s = np.array([])  # Time array for plotting
    latG_d = np.array([])  # Latitude array for plotting
    lonG_d = np.array([])  # Longitude array for plotting
    hae_ft = np.array([])  # Height above ellipsoid array for plotting
    t_s = 0.0
    while t_s < 100.0:  # Run the simulation for 10 seconds
        sim.step()  # Advance the simulation by one time step
        print(
            f"HAE (ft): {sim.lla_ddft[2]:.2f}, Velocity (ft/s): {sim.vBE_B_fps[0]:.2f}, Lat (deg): {sim.lla_ddft[0]:.6f}, Pitch (deg): {sim.euler_d[1]:.2f}")
        t_sim_s = np.append(t_sim_s, t_s)
        latG_d = np.append(latG_d, sim.lla_ddft[0])
        lonG_d = np.append(lonG_d, sim.lla_ddft[1])
        hae_ft = np.append(hae_ft, sim.lla_ddft[2])
        t_s += sim.dt_s

    # Plotting the results
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(lonG_d, latG_d, hae_ft)
    ax.set_xlabel('Longitude (deg)')
    ax.set_ylabel('Latitude (deg)')
    ax.set_zlabel('Height above Ellipsoid (ft)')
    ax.set_title('3D Trajectory of the Vehicle')
    plt.show()
