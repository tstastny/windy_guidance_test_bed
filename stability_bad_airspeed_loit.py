# pybind
import sys
sys.path.append('build')
import cpp_functions as cpp

# numpy
import numpy as np

# scipy
from scipy import optimize

# modules
import pysrc.simulation_env as se
import pysrc.plotting as pl


def eval_npfg(track_error, loit1, npfg, ground_vel, wind_vel_est):

    pos = np.array([0, track_error.item()])

    # evaluate path
    loit1.updateState(pos, ground_vel)  # assume we know the ground vel..

    # evaluate guidance logic
    npfg.setPathCurvature(loit1.getCurvature())
    npfg.evaluate(pos, ground_vel, wind_vel_est,
                  loit1.getClosestPoint(), loit1.upt(), loit1.getTrackError())
    return npfg.getLateralAccel()


# paths
line1 = cpp.Line([0.0, 0.0],  # position
                 [1.0, 0.0])  # orientation

# control
npfg = cpp.NPFG()
# airspeed reference compensation
airspeed_nom = 10.0
airspeed_max = 10.0
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
npfg.setPeriod(10)
npfg.setDamping(0.25)
roll_lim = np.deg2rad(35.0)
# other params
npfg.setWindRatioBuf(0.1)
npfg.enableBackwardsSolution(False)

# ...
bearing_vec = np.array([1, 0])
airspeed = 10.0

len_alpW = 11
alpW_list = np.linspace(0, 1, len_alpW)

len_lP = 181
lP_list = np.linspace(-np.pi, np.pi, len_lP)

d_airspeed = -2  # error in airspeed measurement

track_error_sol = np.zeros([len_alpW, len_lP])
accel_out = np.zeros([len_alpW, len_lP])
for i in range(len_alpW):

    wind_speed_0 = airspeed * alpW_list[i]

    for j in range(len_lP):

        wind_dir_0 = lP_list[j]
        wind_vel_0 = wind_speed_0 * np.array([np.cos(wind_dir_0), np.sin(wind_dir_0)])

        # ground truth steady state config
        wind_cross_bearing = (wind_vel_0[0] * bearing_vec[1] - wind_vel_0[1] * bearing_vec[0]).item()
        airsp_dot_bearing = np.sqrt(np.max([airspeed * airspeed - wind_cross_bearing * wind_cross_bearing, 0.0]))
        ground_vel = bearing_vec * (airsp_dot_bearing + wind_vel_0.dot(bearing_vec))
        air_vel_0 = ground_vel - wind_vel_0

        # estimation (assume we have good heading and gorund vel, but flawed airspeed)
        wind_vel_est = ground_vel - (airspeed + d_airspeed) * air_vel_0 / np.linalg.norm(air_vel_0)

        # find off-nominal steady state condition
        # accel_out[i, j] = eval_npfg(np.array([0.]), line1, npfg, ground_vel, wind_vel_0)
        sol = optimize.root(eval_npfg, 0.0, args=(line1, npfg, ground_vel, wind_vel_est,))

        if sol.success:
            track_error_sol[i, j] = sol.x
        else:
            track_error_sol[i, j] = np.nan


# plotting -------------------------------------------------------------------------------------------------------------

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

cmap = plt.cm.get_cmap('viridis', len_alpW)

fig = plt.figure()

ax = fig.add_subplot()
# ax = fig.add_subplot(111, projection='3d')

ax.set_title(r'Steady State Track Error')
ax.set_ylabel(r'Track Error, $e$ [m]')
ax.set_xlabel(r'Wind Dir. $\lambda_P$ [deg]')
for i in range(len_alpW):
    ax.plot(np.rad2deg(lP_list), track_error_sol[i, :], color=cmap(i))
    # ax.plot(np.rad2deg(lP_list), accel_out[i, :], color=cmap(i))

ax.set_xlim([np.rad2deg(lP_list[0]), np.rad2deg(lP_list[-1])])
ax.set_ylim([np.nanmin(track_error_sol), np.nanmax(track_error_sol)])
# ax.set_ylim([np.nanmin(accel_out), np.nanmax(accel_out)])
ax.set_ylim([-15, 15])

dummy_array = np.array([[0, 1]])
dummy_plot = ax.imshow(dummy_array, cmap='viridis')
dummy_plot.set_visible(False)
fig.colorbar(dummy_plot, ax=ax, label=r'$\alpha_{W,0}$')
#
ax.set_aspect('auto')

plt.show()

