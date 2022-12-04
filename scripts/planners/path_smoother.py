import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, k, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        k (int): The degree of the spline fit.
            For this assignment, k should equal 3 (see documentation for
            scipy.interpolate.splrep)
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        t_smoothed (np.array [N]): Associated trajectory times
        traj_smoothed (np.array [N,7]): Smoothed trajectory
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    assert(path and k > 2 and k < len(path))
    ########## Code starts here ##########
    # Hint 1 - Determine nominal time for each point in the path using V_des
    # Hint 2 - Use splrep to determine cubic coefficients that best fit given path in x, y
    # Hint 3 - Use splev to determine smoothed paths. The "der" argument may be useful.

    ########## Code ends here ##########
    path = np.array(path)
    t_nominal = np.zeros((path.shape[0]))
    for i in range(1, path.shape[0]):
        t_nominal[i] = np.linalg.norm(path[i,:] - path[i-1,:]) / V_des + t_nominal[i-1]

    tck_x = scipy.interpolate.splrep(t_nominal, path[:, 0], s = alpha, k=k)
    tck_y = scipy.interpolate.splrep(t_nominal, path[:, 1], s = alpha, k=k )

    t_smoothed = np.arange(0, t_nominal[-1], dt)

    x_d = scipy.interpolate.splev(t_smoothed, tck_x, der=0)
    xd_d = scipy.interpolate.splev(t_smoothed, tck_x, der=1)
    xdd_d = scipy.interpolate.splev(t_smoothed, tck_x, der=2)
    y_d = scipy.interpolate.splev(t_smoothed, tck_y, der=0)
    yd_d = scipy.interpolate.splev(t_smoothed, tck_y, der=1)
    ydd_d = scipy.interpolate.splev(t_smoothed, tck_y, der=2)
    theta_d = np.arctan2(yd_d, xd_d)
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return t_smoothed, traj_smoothed
