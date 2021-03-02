# --------------------------------------------------------
# Wind-Aware Guidance Logic Simulation
# Created by Thomas Stastny <tstastny@ethz.ch>, 03/2021
# --------------------------------------------------------

# pybind
import sys
sys.path.append('build')
import cpp_functions as cpp

# numpy
import numpy as np

# modules
import pysrc.simulation_env as se


# ----------------------------------------------------------------------------------------------------------------------
# simulation setup

# define UAV initial state
uav = se.Aircraft(np.array([0.0, 0.0]),     # position
                  15.0,                     # airspeed
                  0.0,                      # heading
                  0.0,                      # roll
                  1.0,                      # airspeed time constant
                  0.5)                      # roll time constant

# paths
loit1 = cpp.Loiter([0.0, 0.0],  # position
                   100.0,       # radius
                   -1)          # counter clockwise
path_list = [loit1]

# disturbances
wind = se.Wind(np.array([0.0, 0.0]))

# control
# ...

# timing
t_sim = 30                      # simulation time [s]
dt_sim = 0.01                   # simulation time step [s]
dt_ctrl = np.min(0.1, dt_sim)   # control time step [s]


# ----------------------------------------------------------------------------------------------------------------------
# allocation TODO: move all this to a class

n_sim = int(t_sim /dt_sim) + 1
sim_data = {'t': np.array([ i *dt_sim for i in range(n_sim)])}

# aircraft states
sim_data['pos'] = np.zeros([2, n_sim])
sim_data['airsp'] = np.zeros(n_sim)
sim_data['heading'] = np.zeros(n_sim)
sim_data['roll'] = np.zeros(n_sim)

# aircraft outputs
sim_data['air_vel'] = np.zeros([2, n_sim])
sim_data['ground_vel'] = np.zeros([2, n_sim])

# aircraft controls
sim_data['roll_ref'] = np.zeros(n_sim)
sim_data['airsp_ref'] = np.zeros(n_sim)

# environment
sim_data['wind'] = np.zeros([2, n_sim])


# ----------------------------------------------------------------------------------------------------------------------
# simulate TODO: move all this to a class

for k in range(1, n_sim):

    if sim_data['t'][k] % dt_ctrl == 0:
        # generate control references

        roll_ref = 0.0
        airsp_ref = 15.0
        control = (roll_ref, airsp_ref)

    # integrate (simple euler)
    uav.update_state(uav.state + dt_sim * uav.d_state_dt(control, wind.vel()))

    # logs

    # aircraft states
    sim_data['pos'][:, k] = uav.pos()
    sim_data['airsp'] = uav.airspeed
    sim_data['heading'] = uav.heading
    sim_data['roll'] = uav.roll

    # aircraft outputs
    sim_data['air_vel'][:, k] = uav.air_vel
    sim_data['ground_vel'][:, k] = uav.ground_vel()

    # aircraft controls
    sim_data['roll_ref'] = control[0]
    sim_data['airsp_ref'] = control[1]

    # environment
    sim_data['wind'] = wind.vel()


# ----------------------------------------------------------------------------------------------------------------------
# plot TODO: move all this to a class



