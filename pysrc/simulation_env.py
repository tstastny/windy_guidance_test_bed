# -------------------------------------------------------
# (Simple) Two-Dimensional Simulation Environment
# Collection of functions and objects
# Created by Thomas Stastny <tstastny@ethz.ch>, 03/2021
# --------------------------------------------------------

# packages
import numpy as np

# TODO:
# separate into helpers (functions) py, aircraft py (for aircraft and any derived classes), and disturbance py
# (for varying wind models etc)


#
# Function to wrap angle to [-pi,pi)
#
def wrap_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


#
# Aircraft object handling vehicle dynamics and state integration
#
class Aircraft:

    # constants
    ONE_G = 9.81  # acceleration of gravity [m/s^2]

    # enumerate
    I_POS = 0
    I_AIRSP = 2
    I_HEADING = 3
    I_ROLL = 4

    def __init__(self,
                 pos=np.array([0.0, 0.0]),      # position (north, east) [m]
                 airspeed=15.0,                 # airspeed [m/s]
                 heading=0.0,                   # heading angle [rad]
                 roll=0.0,                      # roll angle [rad]
                 tc_airspeed=1.0,               # airspeed time constant [s]
                 tc_roll=0.5):                  # roll angle time constant [s]
        self.state = np.array([pos[0], pos[1], airspeed, wrap_pi(heading), np.clip(roll, -np.pi/2, np.pi/2)])
        self.air_vel = self.state[self.I_AIRSP] * np.array([np.cos(self.state[self.I_HEADING]),
                                                            np.sin(self.state[self.I_HEADING])])
        self.tc_airspeed = tc_airspeed
        self.tc_roll = tc_roll

    #
    # dynamics
    #

    def d_state_dt(self, control, wind_vel):

        roll_ref, airsp_ref = control

        d_pos_dt = self.vel(wind_vel)
        d_airspeed_dt = (airsp_ref - self.state[self.I_AIRSP]) / self.tc_airspeed
        d_heading_dt = self.ONE_G * np.tan(self.state[self.I_ROLL]) / self.state[self.I_AIRSP]
        d_roll_dt = (roll_ref - self.state[self.I_ROLL]) / self.tc_roll

        return np.concatenate((d_pos_dt, np.array([d_airspeed_dt, d_heading_dt, d_roll_dt])))

    def update_state(self, state):

        self.state[self.I_POS:self.I_POS+2] = state[self.I_POS:self.I_POS+2]
        self.state[self.I_AIRSP] = state[self.I_AIRSP]
        self.state[self.I_HEADING] = wrap_pi(state[self.I_HEADING])
        self.state[self.I_ROLL] = np.clip(state[self.I_ROLL], -np.pi/2, np.pi/2)

        self.air_vel = self.state[self.I_AIRSP] * np.array([np.cos(self.state[self.I_HEADING]),
                                                            np.sin(self.state[self.I_HEADING])])

    #
    # getters
    #

    def pos(self):
        return self.state[self.I_POS:self.I_POS+2]

    def airspeed(self):
        return self.state[self.I_AIRSP]

    def heading(self):
        return self.state[self.I_HEADING]

    def roll(self):
        return self.state[self.I_ROLL]

    def vel(self, wind_vel):
        return self.air_vel + wind_vel  # ground velocity

    def gsp(self, wind_vel):
        return np.linalg.norm(self.air_vel + wind_vel)


#
# Wind object
#
class Wind:

    def __init__(self, vel=np.array([0.0, 0.0])):
        self.vel = vel

    def update(self, t):

        # constant
        pass

        # # variable
        # speed_0 = 3.0
        # dir_0 = np.deg2rad(-45.0)
        # speed_t = np.max([speed_0 + 5 * np.sin(2 * np.pi * t / 50) * np.cos(2 * np.pi * t / 30), 0.0])
        # dir_t = dir_0 + 0.5 * np.sin(2 * np.pi * t / 20) * np.cos(2 * np.pi * t / 35)
        # self.vel[0] = speed_t * np.cos(dir_t)
        # self.vel[1] = speed_t * np.sin(dir_t)

    def speed(self):
        return np.linalg.norm(self.vel)

    def dir(self):
        return np.arctan2(self.vel[1], self.vel[0]) if self.speed() > 1.0e-1 else np.nan
