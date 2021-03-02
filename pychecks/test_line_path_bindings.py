# Created by Thomas Stastny <tstastny@ethz.ch>, 03/2021

# pybind
import sys
sys.path.append('build')
import cpp_functions as cpp

# numpy
import numpy as np

# matplotlib
import matplotlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
plt.rcParams.update({"text.usetex": True})
plt.rcParams.update({'legend.frameon': True,
                     'legend.framealpha': 1,
                     'legend.facecolor': 'white',
                     'axes.edgecolor': 'black'})


# line 1 at pos (0.0, 0.0) with direction (1.0, 0.0)
l1 = cpp.Line([0.0, 0.0], [3.0, -1.0])

# arbitrary vehicle positions
veh_pos_array = np.array([[north*2 for north in range(0, 100, 10)], [east-50 for east in range(0, 100, 10)]])
vpa_len = veh_pos_array.shape[1]

# vehicle velocity
veh_vel = np.array([15.0, 0.0])

# allocate plotting lists
clpt_array = np.array([[0.0] * vpa_len, [0.0] * vpa_len])
upt_array = np.array([[0.0] * vpa_len, [0.0] * vpa_len])
terr_list = [0.0] * vpa_len
curv_list = [0.0] * vpa_len

# test the vehicle positions
for i in range(vpa_len):
    l1.updateState([veh_pos_array[0, i], veh_pos_array[1, i]], veh_vel)  # input: pos, vel
    clpt_array[:, i] = l1.getClosestPoint()
    upt_array[:, i] = l1.getUnitTangent()
    terr_list[i] = l1.getTrackError()


# ----------------------------------------------------------------------------------------------------------------------
# plotting

fig, ax = plt.subplots()

# unit path tangents
qx = clpt_array[1]
qy = clpt_array[0]
qu = upt_array[1]
qv = upt_array[0]
q = ax.quiver(qx, qy, qu, qv)

# closest points
ax_cp = ax.scatter(clpt_array[1], clpt_array[0], color='tab:green', label=r'closest point')

# vehicle positions
ax_vp = ax.scatter(veh_pos_array[1], veh_pos_array[0], color='tab:blue', label=r'vehicle position')

# track errors
for i in range(vpa_len):
    terr_vec = terr_list[i] * np.array([-upt_array[1, i], upt_array[0, i]])  # +90deg rotation to get path normal
    if i == 0:
        ax_te = ax.plot([clpt_array[1, i], clpt_array[1, i]+terr_vec[1]], [clpt_array[0, i], clpt_array[0, i]+terr_vec[0]], color='tab:red', label=r'track errors')
    else:
        ax.plot([clpt_array[1, i], clpt_array[1, i]+terr_vec[1]], [clpt_array[0, i], clpt_array[0, i]+terr_vec[0]], color='tab:red')

ax.legend()
ax.set_ylabel(r'North [m]')
ax.set_xlabel(r'East [m]')
ax.set_aspect('equal')

plt.show()

