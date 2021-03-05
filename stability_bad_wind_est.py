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


def eval_npfg(track_error, line1, npfg, ground_vel, wind_vel_est):

    pos = np.array([0, track_error.item()])

    # evaluate path
    line1.updateState(pos, ground_vel)  # assume we know the ground vel..

    # evaluate guidance logic
    npfg.setPathCurvature(line1.getCurvature())
    npfg.evaluate(pos, ground_vel, wind_vel_est,
                  line1.getClosestPoint(), line1.upt(), line1.getTrackError())
    return npfg.getLateralAccel()


# paths
line1 = cpp.Line([0.0, 0.0],  # position
                 [1.0, 0.0])  # orientation

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

# ...
bearing_vec = np.array([1, 0])
airspeed = 10.0

len_alpW = 11
alpW_list = np.linspace(0, 1, len_alpW)

len_lP = 181
lP_list = np.linspace(-np.pi, np.pi, len_lP)

d_wind_speed = 2
d_wind_dir = np.deg2rad(-15)

track_error_sol = np.zeros([len_alpW, len_lP])
len_k = 101
# accel_sol = np.zeros([len_k, len_lP, len_alpW])
accel_sol = np.zeros([len_alpW, len_lP, len_k])
k_list = np.linspace(-50, 50, len_k)
for i in range(len_alpW):

    wind_speed_0 = airspeed * alpW_list[i]

    if wind_speed_0 + d_wind_speed < 0:
        d_wind_speed_cnstr = 0
    else:
        d_wind_speed_cnstr = d_wind_speed

    for j in range(len_lP):

        wind_dir_0 = lP_list[j]
        wind_vel_0 = wind_speed_0 * np.array([np.cos(wind_dir_0), np.sin(wind_dir_0)])

        # ground truth steady state config
        wind_cross_bearing = (wind_vel_0[0] * bearing_vec[1] - wind_vel_0[1] * bearing_vec[0]).item()
        airsp_dot_bearing = np.sqrt(np.max([airspeed * airspeed - wind_cross_bearing * wind_cross_bearing, 0.0]))
        ground_vel = bearing_vec * (airsp_dot_bearing + wind_vel_0.dot(bearing_vec))

        # estimation
        wind_vel_est = (wind_speed_0 + d_wind_speed_cnstr) * np.array([np.cos(wind_dir_0 + d_wind_dir), np.sin(wind_dir_0 + d_wind_dir)])

        # find off-nominal steady state condition
        sol = optimize.root(eval_npfg, 0.5, args=(line1, npfg, ground_vel, wind_vel_est,))

        if sol.success:
            track_error_sol[i, j] = sol.x
        else:
            track_error_sol[i, j] = np.nan
        # track_error_sol[i, j] = 0.0

        # for k in range(len_k):
        #     # accel_sol[k, j, i] = eval_npfg(k_list[k], line1, npfg, ground_vel, wind_vel_est)
        #     accel_sol[i, j, k] = eval_npfg(k_list[k], line1, npfg, ground_vel, wind_vel_est)

        # if not sol.success:
        #     print('sol ', i, ',', j, ' failed')


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

ax.set_title(r'Steady State Track Error, $\Delta v_W$={{}} m/s'.format(d_wind_speed))
ax.set_ylabel(r'Track Error, $e$ [m]')
ax.set_xlabel(r'Wind Dir. $\lambda_P$ [deg]')
for i in range(len_alpW):
    ax.plot(np.rad2deg(lP_list), track_error_sol[i, :], color=cmap(i))
#     # ax.plot(np.rad2deg(lP_list), accel_sol[i, :], color=cmap(i))
#     for k in range(len_k):
#         ax.plot(np.rad2deg(lP_list), accel_sol[i, :, k], color=cmap(i))
# for j in range(8, 9, 1):#range(len_alpW):
#     for i in range(len_alpW):
#         ax.plot(k_list, accel_sol[i, j, :], color=cmap(i))
# X, Y = np.meshgrid(np.rad2deg(lP_list), k_list)
# aa = 1 * (np.abs(accel_sol[:, :, 5]) < 0.5)
# ax.plot_surface(X, Y, accel_sol[:, :, 8])
# ax.plot_surface(X, Y, aa)
#
ax.set_xlim([np.rad2deg(lP_list[0]), np.rad2deg(lP_list[-1])])
ax.set_ylim([np.nanmin(track_error_sol), np.nanmax(track_error_sol)])
ax.set_ylim([-15, 15])
#
# plot dummy data to have stand alone colorbar
dummy_array = np.array([[0, 1]])
dummy_plot = ax.imshow(dummy_array, cmap='viridis')
dummy_plot.set_visible(False)
fig.colorbar(dummy_plot, ax=ax, label=r'$\alpha_{W,0}$')
#
ax.set_aspect('auto')

plt.show()

