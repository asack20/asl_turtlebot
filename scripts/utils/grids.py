import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles, radius=0.09):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.radius = radius

    def is_free(self, x):
        """Verifies that point is not inside any obstacles"""
        for obs in self.obstacles:
            inside = False
            for dim in range(len(x)):
                if x[dim] + self.radius > obs[0][dim] and x[dim] - self.radius < obs[1][dim]:
                    inside = True
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.5, radius=0.09):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = np.reshape(np.asarray(probs), (height, width))
        self.window_size = window_size
        self.thresh = thresh
        self.radius = radius

    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def is_free(self, state, thresh=0.2):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        x, y = self.snap_to_grid(state)
        x_hi, y_hi = self.snap_to_grid((state[0]+self.radius, state[1]+self.radius))
        x_lo, y_lo = self.snap_to_grid((state[0]+self.radius, state[1]+self.radius))

        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        grid_x_hi = int((x_hi - self.origin_x) / self.resolution)
        grid_x_lo = int((x_lo - self.origin_x) / self.resolution)
        grid_y_hi = int((y_hi - self.origin_y) / self.resolution)
        grid_y_lo = int((y_lo - self.origin_y) / self.resolution)

        half_size = int(round((self.window_size-1)/2))
        grid_x_lower = max(0, grid_x_lo - half_size)
        grid_y_lower = max(0, grid_y_lo - half_size)
        grid_x_upper = min(self.width, grid_x_hi + half_size + 1)
        grid_y_upper = min(self.height, grid_y_hi + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(self.probs.shape[0]):
            for j in range(self.probs.shape[1]):
                # convert i to (x,y)
                x = j * self.resolution + self.origin_x
                y = i * self.resolution + self.origin_y
                if not self.is_free((x,y)):
                    pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')
        plt.xlim([self.origin_x, self.width * self.resolution + self.origin_x])
        plt.ylim([self.origin_y, self.height * self.resolution + self.origin_y])
