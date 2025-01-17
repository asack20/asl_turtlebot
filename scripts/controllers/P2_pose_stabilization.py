import typing as T

import numpy as np
from utils import wrapToPi

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1
DIST_THRES = 0.25

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1: float, k2: float, k3: float,
                 V_max: float = 0.5, om_max: float = 1) -> None:
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.V_max = V_max
        self.om_max = om_max

    def load_goal(self, x_g: float, y_g: float, th_g: float) -> None:
        """ Loads in a new goal position """
        self.x_g = x_g
        self.y_g = y_g
        self.th_g = th_g

    def compute_control(self, x: float, y: float, th: float, t: float) -> T.Tuple[float, float, bool]:
        """
        Inputs:
            x,y,th: Current state
            t: Current time (you shouldn't need to use this)
        Outputs:
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
        errorOutput = False
        rho = np.sqrt(np.square(self.x_g - x) + np.square(self.y_g - y))
        if rho > DIST_THRES:
            errorOutput = True
        
        beta = np.arctan2((self.y_g - y), (self.x_g - x))

        alpha = wrapToPi(beta - th)

        delta = wrapToPi(beta - self.th_g)

        V = self.k1 * rho * np.cos(alpha)
        om = (self.k2 * alpha) + (self.k1 * np.sinc(alpha / np.pi) * np.cos(alpha)) * (alpha + self.k3 * delta)
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        #if rho < RHO_THRES and alpha <= ALPHA_THRES and delta <= DELTA_THRES:
         #   V = 0.0
          #  om = 0.0
        return V, om, errorOutput
