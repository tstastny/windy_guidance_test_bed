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
tc_roll = 0.5
uav = se.Aircraft(np.array([-20.0, 0.0]),    # position
                  12.0,                     # airspeed
                  np.deg2rad(-90.0),        # heading
                  0.0,                      # roll
                  1.0,                      # airspeed time constant
                  tc_roll)                      # roll time constant

# paths
loit1 = cpp.Loiter([0.0, 0.0],  # position
                   20.0,        # radius
                   1)           # dir

# disturbances
wind = se.Wind(np.array([0.0, 0.0]))

# control
npfg = cpp.NPFG()
# airspeed reference compensation
airspeed_nom = 12.0
airspeed_max = 12.0
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
period_0 = 30
damping_0 = 0.25
npfg.setPeriod(period_0)
npfg.setDamping(damping_0)
roll_lim = np.deg2rad(45.0)
# other params
npfg.setWindRatioBuf(0.1)
npfg.enableBackwardsSolution(False)

# timing TODO: organize in config dict
t_sim = 180                              # simulation time [s]
dt_sim = 0.01                           # simulation time step [s]
dt_ctrl = np.min([0.1, dt_sim])         # control time step [s]
ctrl_interval = int(dt_ctrl / dt_sim)
dt_est = np.min([0.01, dt_sim])         # estimation time step [s]
est_interval = int(dt_est / dt_sim)

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# allocation TODO: move all this to a class

n_sim = int(t_sim /dt_sim) + 1
sim_data = {'time': np.array([i *dt_sim for i in range(n_sim)])}

sim_data['tuning'] = {
    'max nom period': np.zeros(n_sim),
    'min nom period': np.zeros(n_sim),
    'max turn rate': np.zeros(n_sim),
    'no wind turn rate': np.zeros(n_sim),
    'nom period': np.zeros(n_sim)
}

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

sim_data['aircraft estimates'] = {
    'wind vel': np.zeros([2, n_sim]),
    'wind speed': np.zeros(n_sim),
    'wind dir': np.zeros(n_sim)
}

sim_data['environment'] = {
    'wind vel': np.zeros([2, n_sim]),
    'wind speed': np.zeros(n_sim),
    'wind dir': np.zeros(n_sim)
}

sim_data['path following'] = {
    'track error': np.zeros(n_sim),
    'track error bound': np.zeros(n_sim),
    'ff accel': np.zeros(n_sim),
    'feas': np.zeros(n_sim),
    'feas0': np.zeros(n_sim)
}


# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# simulate TODO: move all this to a class

d_airspeed = 0  # error in airspeed measurement

# init
wind.update(0)
wind_vel_est = wind.vel
# simulate
for k in range(n_sim):

    # update env
    wind.update(sim_data['time'][k])

    # estimate states --------------------------------------------------------------------------------------------------
    if np.mod(k, est_interval) == 0 or k == 0:

        # propagate (note: at k=0 no delta) -- assuming something similar to EKF tuned for 1.0m/s/s wind noise
        # wind_vel_est = (wind.vel - wind_vel_est) / 1.0 * est_interval * dt_sim + wind_vel_est
        ground_vel = uav.vel(wind.vel)
        wind_vel_est = ground_vel - (uav.airspeed() + d_airspeed) * (ground_vel - wind.vel) / np.linalg.norm(ground_vel - wind.vel)
        # wind_vel_est[0] = 0
        # wind_vel_est[1] = 0

    # control the aircraft ---------------------------------------------------------------------------------------------
    if np.mod(k, ctrl_interval) == 0 or k == 0:

        # evaluate path
        loit1.updateState(uav.pos(), uav.vel(wind.vel))  # assume we know the ground vel..

        # evaluate guidance logic
        npfg.setPathCurvature(loit1.getCurvature())
        npfg.evaluate(uav.pos(), uav.vel(wind.vel), wind_vel_est,
                      loit1.getClosestPoint(), loit1.getUnitTangent(), loit1.getTrackError())
        lat_accel = npfg.getLateralAccel()
        roll_ref = np.clip(np.arctan(lat_accel / uav.ONE_G), -roll_lim, roll_lim)  # TODO: encapsulate
        airspeed_ref = npfg.getAirspeedRef()
        control = (roll_ref, airspeed_ref)

    # output -----------------------------------------------------------------------------------------------------------

    # tuning bounds
    nowind_turn_rate = uav.airspeed() * loit1.getCurvature()
    crit_wind_factor = 2.0 * (1 - np.sqrt(1.0 - wind.speed() / uav.airspeed()))
    sim_data['tuning']['max nom period'][k] = \
        4.0 * np.pi * damping_0 / (nowind_turn_rate * crit_wind_factor) if damping_0 < 0.7071 else np.pi / (damping_0 * nowind_turn_rate * crit_wind_factor)
    sim_data['tuning']['min nom period'][k] = \
        2.0 * np.pi * (np.sqrt(damping_0**2 * (nowind_turn_rate * crit_wind_factor * tc_roll)**2 + nowind_turn_rate * crit_wind_factor * tc_roll) \
        + damping_0 * (nowind_turn_rate * crit_wind_factor * tc_roll - 1)) / nowind_turn_rate / crit_wind_factor \
        if nowind_turn_rate * crit_wind_factor > 0.0 else np.pi * tc_roll / damping_0
    sim_data['tuning']['max turn rate'][k] = \
        8.0 * damping_0**2 / (crit_wind_factor * tc_roll * (4.0 * damping_0**2 + 1.0)) if damping_0 < 0.7071 else \
        (4.0 * damping_0**2 + 1.0) / (8.0 * crit_wind_factor * tc_roll * damping_0**2)
    sim_data['tuning']['no wind turn rate'][k] = nowind_turn_rate
    sim_data['tuning']['nom period'][k] = period_0

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

    # aircraft estimates
    sim_data['aircraft estimates']['wind vel'][:, k] = wind_vel_est
    sim_data['aircraft estimates']['wind speed'][k] = np.linalg.norm(wind_vel_est)
    sim_data['aircraft estimates']['wind dir'][k] = np.arctan2(wind_vel_est[1], wind_vel_est[0]) if sim_data['aircraft estimates']['wind speed'][k] > 1.0e-1 else np.nan

    # environment
    sim_data['environment']['wind vel'][:, k] = wind.vel
    sim_data['environment']['wind speed'][k] = wind.speed()
    sim_data['environment']['wind dir'][k] = wind.dir()

    # path following
    loit1.updateState(uav.pos(), uav.vel(wind.vel))  # get ground truth values
    sim_data['path following']['track error'][k] = loit1.getTrackError()
    sim_data['path following']['track error bound'][k] = npfg.getTrackErrorBound()
    sim_data['path following']['ff accel'][k] = npfg.getLateralAccelCurvAdj()
    sim_data['path following']['feas'][k] = npfg.getBearingFeas()
    sim_data['path following']['feas0'][k] = npfg.getBearingFeas0()

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
sz_marker = 10

# loiter
threesixty = np.linspace(-np.pi, np.pi, 101)
loit_pos = loit1.pos()
ax_pos2d.plot(loit_pos[1] + loit1.radius() * np.sin(threesixty),
              loit_pos[0] + loit1.radius() * np.cos(threesixty),
              color=ref_color,
              label='Path')

# aircraft position
ax_pos2d.plot(sim_data['aircraft states']['position'][1],
              sim_data['aircraft states']['position'][0],
              color=state_color,
              label='Aircraft')
ax_pos2d.scatter(sim_data['aircraft states']['position'][1, 0],
                 sim_data['aircraft states']['position'][0, 0],
                 s=sz_marker, marker='^', color='tab:green')  # start position
ax_pos2d.scatter(sim_data['aircraft states']['position'][1, -1],
                 sim_data['aircraft states']['position'][0, -1],
                 s=sz_marker, marker='s', color='tab:red')  # end position

ax_pos2d.legend()


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
# ax_roll = fig_states.add_subplot(grid_states[2], xticklabels=[], ylabel='FF Lat Accel [m/s2]')
# ax_roll.plot(sim_data['time'], np.rad2deg(sim_data['path following']['ff accel']), color=ref_color)

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
c_wind_est = 'lightskyblue'
c_wind = np.asarray(colors.to_rgba('lightskyblue')) * alpha_white + (1-alpha_white) * np.ones(4)

ax_speeds = fig_speeds.add_subplot(grid_speeds[:-1], xticklabels=[], ylabel='Speed [m/s]')
ax_speeds.plot(sim_data['time'], sim_data['aircraft outputs']['ground speed'], color=c_gsp, label='Ground Sp.')
ax_speeds.plot(sim_data['time'], sim_data['aircraft controls']['airspeed ref'], color=c_airsp_ref, label='Airsp. Ref')
ax_speeds.plot(sim_data['time'], sim_data['aircraft states']['airspeed'], color=c_airsp, label='Airspeed')
ax_speeds.plot(sim_data['time'], sim_data['environment']['wind speed'], color=c_wind, label='Wind Sp. Est.')
ax_speeds.plot(sim_data['time'], sim_data['aircraft estimates']['wind speed'], color=c_wind_est, label='Wind Sp.')
ax_speeds.legend()

# wind dir plot ----------------------------------------
ax_wind_dir = fig_speeds.add_subplot(grid_speeds[-1], xticklabels=[], ylabel='Wind Dir. [deg]')
ax_wind_dir.plot(sim_data['time'], np.rad2deg(sim_data['environment']['wind dir']), color=ref_color)
ax_wind_dir.plot(sim_data['time'], np.rad2deg(sim_data['aircraft estimates']['wind dir']), color=state_color)

# ----------------------------------------------------------------------------------------------------------------------
# tuning plot

fig_tuning = plt.figure()
fig_tuning.suptitle('Tuning')
grid_tuning = plt.GridSpec(3, 1)

# period plot ----------------------------------------
ax_period = fig_tuning.add_subplot(grid_tuning[0], xticklabels=[], ylabel='Period [s]')
ax_period.plot(sim_data['time'], sim_data['tuning']['max nom period'], linestyle='-.', color=ref_color, label='Max. Nom. Period')
ax_period.plot(sim_data['time'], sim_data['tuning']['min nom period'], color=ref_color, label='Min. Nom. Period')
ax_period.plot(sim_data['time'], sim_data['tuning']['nom period'], color=state_color, label='Nom. Period')
ax_period.legend()

# turn rate plot ----------------------------------------
ax_tr = fig_tuning.add_subplot(grid_tuning[1], xticklabels=[], ylabel=' (no wind) Turn Rate [deg/s]')
ax_tr.plot(sim_data['time'], np.rad2deg(sim_data['tuning']['max turn rate']), color=ref_color, label='Max. Turn Rate')
ax_tr.plot(sim_data['time'], np.rad2deg(sim_data['tuning']['no wind turn rate']), color=state_color, label='Turn Rate')
ax_tr.legend()

# turn rate plot ----------------------------------------
ax_feas = fig_tuning.add_subplot(grid_tuning[2], xticklabels=[], ylabel='feas')
ax_feas.plot(sim_data['time'], sim_data['path following']['feas'], color=ref_color, label='feas')
ax_feas.plot(sim_data['time'], sim_data['path following']['feas0'], color=state_color, label='feas0')

plt.show()

