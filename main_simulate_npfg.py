# --------------------------------------------------------
# Nonlinear Path Following Guidance Simulation
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
uav = se.Aircraft(np.array([0.0, 10.0]),    # position
                  10.0,                     # airspeed
                  np.deg2rad(-90.0),        # heading
                  0.0,                      # roll
                  1.0,                      # airspeed time constant
                  0.5)                      # roll time constant

# paths
line1 = cpp.Line([0.0, 0.0],  # position
                 [1.0, 0.0])  # orientation

# disturbances
wind = se.Wind(np.array([0.0, 0.0]))

# control
npfg = cpp.NPFG()
# airspeed reference compensation
airspeed_nom = 10.0
airspeed_max = 16.0
min_ground_speed_g = 0.0
npfg.enableFeedForwardAirVelRef(True)
npfg.enableMinGroundSpeed(False)
npfg.enableTrackKeeping(False)
npfg.enableWindExcessRegulation(False)
npfg.setAirspeedMax(airspeed_max)
npfg.setAirspeedNom(airspeed_nom)
npfg.setMinGroundSpeed(min_ground_speed_g)
npfg.setMinGroundSpeedEMax(6.0)
npfg.setNominalHeadingRate(9.81 * np.tan(np.deg2rad(35.0)) / airspeed_nom)
npfg.setNTEFraction(0.5)
# tuning
npfg.setPeriod(30)
npfg.setDamping(0.25)
roll_lim = np.deg2rad(35.0)
# other params
npfg.setWindRatioBuf(0.1)
npfg.enableBackwardsSolution(False)

# timing TODO: organize in config dict
t_sim = 10                          # simulation time [s]
dt_sim = 0.01                       # simulation time step [s]
dt_ctrl = np.min([0.1, dt_sim])     # control time step [s]
ctrl_interval = int(dt_ctrl / dt_sim)

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# allocation TODO: move all this to a class

n_sim = int(t_sim /dt_sim) + 1
sim_data = {'time': np.array([ i *dt_sim for i in range(n_sim)])}

sim_data['aircraft states'] = {
    'position': np.zeros([2, n_sim]),
    'airspeed': np.zeros(n_sim),
    'heading': np.zeros(n_sim),
    'roll': np.zeros(n_sim),
}

sim_data['aircraft outputs'] = {
    'air vel': np.zeros([2, n_sim]),
    'ground vel': np.zeros([2, n_sim]),
    'ground speed': np.zeros(n_sim)
}

sim_data['aircraft controls'] = {
    'roll ref': np.zeros(n_sim),
    'airspeed ref': np.zeros(n_sim)
}

sim_data['environment'] = {
    'wind vel': np.zeros([2, n_sim]),
    'wind speed': np.zeros(n_sim),
    'wind dir': np.zeros(n_sim)
}

sim_data['path following'] = {
    'track error': np.zeros(n_sim),
    'track error bound': np.zeros(n_sim)
}


# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# simulate TODO: move all this to a class

for k in range(n_sim):

    # evaluate path ----------------------------------------------------------------------------------------------------
    line1.updateState(uav.pos(), uav.vel(wind.vel))  # REMEMBER: update must be called before control step

    # control the aircraft ---------------------------------------------------------------------------------------------
    if np.mod(k, ctrl_interval) == 0 or k == 0:

        npfg.setPathCurvature(line1.getCurvature())
        npfg.evaluate(uav.pos(), uav.vel(wind.vel), wind.vel,
                      line1.getClosestPoint(), line1.upt(), line1.getTrackError())
        lat_accel = npfg.getLateralAccel()
        roll_ref = np.clip(np.arctan(lat_accel / uav.ONE_G), -roll_lim, roll_lim)  # TODO: encapsulate
        airspeed_ref = npfg.getAirspeedRef()
        control = (roll_ref, airspeed_ref)

    # output -----------------------------------------------------------------------------------------------------------

    # aircraft states
    sim_data['aircraft states']['position'][:, k] = uav.pos()
    sim_data['aircraft states']['airspeed'][k] = uav.airspeed()
    sim_data['aircraft states']['heading'][k] = uav.heading()
    sim_data['aircraft states']['roll'][k] = uav.roll()

    # aircraft outputs
    sim_data['aircraft outputs']['air vel'][:, k] = uav.air_vel
    sim_data['aircraft outputs']['ground vel'][:, k] = uav.vel(wind.vel)
    sim_data['aircraft outputs']['ground speed'][k] = uav.gsp(wind.vel)

    # aircraft controls
    sim_data['aircraft controls']['roll ref'][k] = control[0]
    sim_data['aircraft controls']['airspeed ref'][k] = control[1]

    # environment
    sim_data['environment']['wind vel'][:, k] = wind.vel
    sim_data['environment']['wind speed'][k] = wind.speed()
    sim_data['environment']['wind dir'][k] = wind.dir()

    # path following
    sim_data['path following']['track error'][k] = line1.getTrackError()
    sim_data['path following']['track error bound'][k] = npfg.getTrackErrorBound()

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
from matplotlib import colors
plt.style.use('seaborn-whitegrid')
plt.rcParams.update({"text.usetex": True})
plt.rcParams.update({'legend.frameon': True,
                     'legend.framealpha': 1,
                     'legend.facecolor': 'white',
                     'axes.edgecolor': 'black'})

ref_color = [0.5, 0.5, 0.5]
state_color = 'tab:blue'

# ----------------------------------------------------------------------------------------------------------------------
# 2D position plot

fig_pos2d = plt.figure()
fig_pos2d.suptitle('2D Position')
ax_pos2d = fig_pos2d.add_subplot(aspect='equal', xlabel='East [m]', ylabel='North [m]')
ax_pos2d.grid()
sz_marker = 5

# line
line_pos = line1.pos()
line_upt = line1.upt()
len_upt = 5.0
ax_pos2d.scatter(line_pos[1], line_pos[0], s=sz_marker, marker='o', color=ref_color)
ax_pos2d.quiver(line_pos[1], line_pos[0], line_upt[1] * len_upt, line_upt[0] * len_upt, color=ref_color)
ax_pos2d.plot(line_pos[1] + np.array([0.0, line_upt[1] * 1000.0]),
              line_pos[0] + np.array([0.0, line_upt[0] * 1000.0]),  # TODO: truncation function for line plot
              color=ref_color,
              label='Path')

# aircraft position
ax_pos2d.scatter(sim_data['aircraft states']['position'][1, 0],
                 sim_data['aircraft states']['position'][0, 0],
                 s=sz_marker, marker='^', color='tab:green')  # start position
ax_pos2d.scatter(sim_data['aircraft states']['position'][1, -1],
                 sim_data['aircraft states']['position'][0, -1],
                 s=sz_marker, marker='s', color='tab:red')  # end position
ax_pos2d.plot(sim_data['aircraft states']['position'][1],
              sim_data['aircraft states']['position'][0],
              color=state_color,
              label='Aircraft')

ax_pos2d.legend()

# axis limits
bb_margin = 5
pos_min, pos_max = pl.position_bounding_box(
    np.concatenate((sim_data['aircraft states']['position'][1], np.array([line_pos[1]]))),
    np.concatenate((sim_data['aircraft states']['position'][0], np.array([line_pos[0]]))))
ax_pos2d.set(xlim=[pos_min[0]-bb_margin, pos_max[0]+bb_margin],
             ylim=[pos_min[1]-bb_margin, pos_max[1]+bb_margin])

# ----------------------------------------------------------------------------------------------------------------------
# aircraft states plot

fig_states = plt.figure()
fig_states.suptitle('Aircraft States')
grid_states = plt.GridSpec(4, 1)

# airspeed plot ----------------------------------------
ax_airsp = fig_states.add_subplot(grid_states[0], xticklabels=[], ylabel='Airspeed [m/s]')
ax_airsp.plot(sim_data['time'], sim_data['aircraft controls']['airspeed ref'], color=ref_color, label='Reference')
ax_airsp.plot(sim_data['time'], sim_data['aircraft states']['airspeed'], color=state_color, label='State')

# floating point precision was making this disgusting to plot
max_airsp_ref = np.max(sim_data['aircraft controls']['airspeed ref'])
min_airsp_ref = np.min(sim_data['aircraft controls']['airspeed ref'])
range_airsp_ref = max_airsp_ref - min_airsp_ref
max_airsp = np.max(sim_data['aircraft states']['airspeed'])
min_airsp = np.min(sim_data['aircraft states']['airspeed'])
range_airsp = max_airsp - min_airsp
if range_airsp_ref < 0.1 and range_airsp < 0.1:
    ax_airsp.set(ylim=[np.min([min_airsp, min_airsp_ref])-0.1, np.max([max_airsp, max_airsp_ref])+0.1])

ax_airsp.legend()

# heading plot ----------------------------------------
ax_heading = fig_states.add_subplot(grid_states[1], xticklabels=[], ylabel='Heading [deg]')
ax_heading.plot(sim_data['time'], np.rad2deg(sim_data['aircraft states']['heading']), color=state_color)

# roll plot ----------------------------------------
ax_roll = fig_states.add_subplot(grid_states[2], xticklabels=[], ylabel='Roll [deg]')
ax_roll.plot(sim_data['time'], np.rad2deg(sim_data['aircraft controls']['roll ref']), color=ref_color)
ax_roll.plot(sim_data['time'], np.rad2deg(sim_data['aircraft states']['roll']), color=state_color)

# track error plot ----------------------------------------
ax_te = fig_states.add_subplot(grid_states[3], xlabel='Time [s]', ylabel='Track Error [m]')
ax_te.plot([sim_data['time'][0], sim_data['time'][-1]], [0.0, 0.0], color=ref_color)
ax_te.plot(sim_data['time'], sim_data['path following']['track error bound'], color=ref_color, linestyle='-.')
ax_te.plot(sim_data['time'], sim_data['path following']['track error'], color=state_color)

# ----------------------------------------------------------------------------------------------------------------------
# speeds plot

fig_speeds = plt.figure()
fig_speeds.suptitle('Speeds')
grid_speeds = plt.GridSpec(4, 1)

# speeds plot ----------------------------------------
alpha_white = 0.5
c_airsp = 'tab:orange'
c_airsp_ref = np.asarray(colors.to_rgba('tab:orange')) * alpha_white + (1-alpha_white) * np.ones(4)
c_gsp = 'black'
c_wind = 'lightskyblue'

ax_speeds = fig_speeds.add_subplot(grid_speeds[:-1], xticklabels=[], ylabel='Speed [m/s]')
ax_speeds.plot(sim_data['time'], sim_data['aircraft outputs']['ground speed'], color=c_gsp, label='Ground Sp.')
ax_speeds.plot(sim_data['time'], sim_data['aircraft controls']['airspeed ref'], color=c_airsp_ref, label='Airsp. Ref')
ax_speeds.plot(sim_data['time'], sim_data['aircraft states']['airspeed'], color=c_airsp, label='Airspeed')
ax_speeds.plot(sim_data['time'], sim_data['environment']['wind speed'], color=c_wind, label='Wind Sp.')
ax_speeds.legend()

# heading plot ----------------------------------------
ax_wind_dir = fig_speeds.add_subplot(grid_speeds[-1], xticklabels=[], ylabel='Wind Dir. [deg]')
ax_wind_dir.plot(sim_data['time'], np.rad2deg(sim_data['environment']['wind dir']), color=state_color)

plt.show()

