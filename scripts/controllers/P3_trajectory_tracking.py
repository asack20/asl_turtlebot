import typing as T

import numpy as np
from numpy import linalg

V_PREV_THRES = 0.0001
DIST_THRES = 0.25

class TrajectoryTracker:
    """ Trajectory tracking controller using differential flatness """
    def __init__(self, kpx: float, kpy: float, kdx: float, kdy: float,
                 V_max: float = 0.5, om_max: float = 1) -> None:
        self.kpx = kpx
        self.kpy = kpy
        self.kdx = kdx
        self.kdy = kdy

        self.V_max = V_max
        self.om_max = om_max

        self.coeffs = np.zeros(8) # Polynomial coefficients for x(t) and y(t) as
                                  # returned by the differential flatness code

    def reset(self) -> None:
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

    def load_traj(self, times: np.ndarray, traj: np.ndarray) -> None:
        """ Loads in a new trajectory to follow, and resets the time """
        self.reset()
        self.traj_times = times
        self.traj = traj

    def get_desired_state(self, t: float) -> T.Tuple[np.ndarray, np.ndarray, np.ndarray,
                                                     np.ndarray, np.ndarray, np.ndarray]:
        """
        Input:
            t: Current time
        Output:
            x_d, xd_d, xdd_d, y_d, yd_d, ydd_d: Desired state and derivatives
                at time t according to self.coeffs
        """
        x_d = np.interp(t,self.traj_times,self.traj[:,0])
        y_d = np.interp(t,self.traj_times,self.traj[:,1])
        xd_d = np.interp(t,self.traj_times,self.traj[:,3])
        yd_d = np.interp(t,self.traj_times,self.traj[:,4])
        xdd_d = np.interp(t,self.traj_times,self.traj[:,5])
        ydd_d = np.interp(t,self.traj_times,self.traj[:,6])

        return x_d, xd_d, xdd_d, y_d, yd_d, ydd_d

    def compute_control(self, x: float, y: float, th: float, t: float) -> T.Tuple[float, float, bool]:
        """
        Inputs:
            x,y,th: Current state
            t: Current time
        Outputs:
            V, om: Control actions
        """
        errorOutput = False

        dt = t - self.t_prev
        x_d, xd_d, xdd_d, y_d, yd_d, ydd_d = self.get_desired_state(t)

        ########## Code starts here ##########
        if np.abs(self.V_prev) < V_PREV_THRES:
            if self.V_prev >= 0:
                self.V_prev = V_PREV_THRES
            else:
                self.V_prev = -V_PREV_THRES
        if (x_d - x) > DIST_THRES or (y_d) - y > DIST_THRES:
            errorOutput = True

        x_dot = self.V_prev * np.cos(th)
        y_dot = self.V_prev * np.sin(th)

        u = np.array([xdd_d + self.kpx*(x_d - x) + self.kdx*(xd_d - x_dot),
                     ydd_d + self.kpy*(y_d - y) + self.kdy*(yd_d - y_dot)])

        J = np.array([[np.cos(th), -self.V_prev * np.sin(th)],
                      [np.sin(th), self.V_prev * np.cos(th)]])

        sol = np.linalg.solve(J, u)

        om = sol[1]
        V = self.V_prev + sol[0]*dt

        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        return V, om, errorOutput
