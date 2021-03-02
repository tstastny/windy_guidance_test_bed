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
import pysrc.plotting as pl

# ----------------------------------------------------------------------------------------------------------------------
# simulation setup

# define UAV initial state
uav = se.Aircraft(np.array([0.0, 50.0]),    # position
                  10.0,                     # airspeed
                  0.0,                      # heading
                  0.0,                      # roll
                  1.0,                      # airspeed time constant
                  0.5)                      # roll time constant

# paths
line1 = cpp.Line([0.0, 0.0],  # position
                 [1.0, 0.0])  # orientation

# disturbances
wind = se.Wind(np.array([0.0, 0.0]))

# control
# ...

# timing TODO: organize in config dict
t_sim = 10                          # simulation time [s]
dt_sim = 0.01                       # simulation time step [s]
dt_ctrl = np.min([0.1, dt_sim])     # control time step [s]


# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# allocation TODO: move all this to a class

n_sim = int(t_sim /dt_sim) + 1
sim_data = {'time': np.array([ i *dt_sim for i in range(n_sim)])}

sim_data['aircraft states'] = {
    'pos': np.zeros([2, n_sim]),
    'airsp': np.zeros(n_sim),
    'heading': np.zeros(n_sim),
    'roll': np.zeros(n_sim),
}

sim_data['aircraft outputs'] = {
    'air_vel': np.zeros([2, n_sim]),
    'ground_vel': np.zeros([2, n_sim])
}

sim_data['aircraft controls'] = {
    'roll_ref': np.zeros(n_sim),
    'airsp_ref': np.zeros(n_sim)
}

sim_data['environment'] = {
    'wind_vel': np.zeros([2, n_sim])
}


# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# simulate TODO: move all this to a class

for k in range(n_sim):

    # evaluate path ----------------------------------------------------------------------------------------------------
    line1.updateState(uav.pos(), uav.vel(wind.vel))

    # control the aircraft ---------------------------------------------------------------------------------------------
    if sim_data['time'][k] % dt_ctrl == 0 or k == 0:

        roll_ref = 0.0
        airsp_ref = 15.0
        control = (roll_ref, airsp_ref)

    # output -----------------------------------------------------------------------------------------------------------

    # aircraft states
    sim_data['aircraft states']['pos'][:, k] = uav.pos()
    sim_data['aircraft states']['airsp'][k] = uav.airspeed()
    sim_data['aircraft states']['heading'][k] = uav.heading()
    sim_data['aircraft states']['roll'][k] = uav.roll()

    # aircraft outputs
    sim_data['aircraft outputs']['air_vel'][:, k] = uav.air_vel
    sim_data['aircraft outputs']['ground_vel'][:, k] = uav.vel(wind.vel)

    # aircraft controls
    sim_data['aircraft controls']['roll_ref'][k] = control[0]
    sim_data['aircraft controls']['airsp_ref'][k] = control[1]

    # environment
    sim_data['environment']['wind_vel'][:, k] = wind.vel

    # integrate (simple euler) -----------------------------------------------------------------------------------------
    uav.update_state(uav.state + dt_sim * uav.d_state_dt(control, wind.vel))


# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# plot
# TODO: move all this to a class
# TODO: make selectable (based on dict groups)

# matplotlib
import matplotlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
plt.rcParams.update({"text.usetex": True})
plt.rcParams.update({'legend.frameon': True,
                     'legend.framealpha': 1,
                     'legend.facecolor': 'white',
                     'axes.edgecolor': 'black'})

# ----------------------------------------------------------------------------------------------------------------------
# aircraft states plots

# position plot
fig_pos2d = plt.figure()
ax_pos2d = fig_pos2d.add_subplot(aspect='equal', xlabel='East [m]', ylabel='North [m]')
ax_pos2d.grid()
sz_marker = 5

# line
path_color = [0.5, 0.5, 0.5]
line_pos = line1.pos()
line_upt = line1.upt()
len_upt = 5.0
ax_pos2d.scatter(line_pos[1], line_pos[0], s=sz_marker, marker='o', color=path_color)
ax_pos2d.quiver(line_pos[1], line_pos[0], line_upt[1] * len_upt, line_upt[0] * len_upt, color=path_color)
ax_pos2d.plot(line_pos[1] + np.array([0.0, line_upt[1] * 1000.0]),
              line_pos[0] + np.array([0.0, line_upt[0] * 1000.0]),  # TODO: truncation function for line plot
              color=path_color,
              label='Path')

# aircraft position
ax_pos2d.scatter(sim_data['aircraft states']['pos'][1, 0],
                 sim_data['aircraft states']['pos'][0, 0],
                 s=sz_marker, marker='^', color='tab:green')  # start position
ax_pos2d.scatter(sim_data['aircraft states']['pos'][1, -1],
                 sim_data['aircraft states']['pos'][0, -1],
                 s=sz_marker, marker='s', color='tab:red')  # end position
ax_pos2d.plot(sim_data['aircraft states']['pos'][1],
              sim_data['aircraft states']['pos'][0],
              color='tab:blue',
              label='Aircraft')

ax_pos2d.legend()

# axis limits
bb_margin = 5
pos_min, pos_max = pl.position_bounding_box(np.concatenate((sim_data['aircraft states']['pos'][1], np.array([line_pos[1]]))),
                                            np.concatenate((sim_data['aircraft states']['pos'][0], np.array([line_pos[0]]))))
ax_pos2d.set(xlim=[pos_min[0]-bb_margin, pos_max[0]+bb_margin],
             ylim=[pos_min[1]-bb_margin, pos_max[1]+bb_margin])

plt.show()

