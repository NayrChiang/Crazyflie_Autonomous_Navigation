import numpy as np
from .graph_search import graph_search


class WorldTraj(object):
    """
    """

    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.
        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.
        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)
        """
        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.2, 0.2, 0.2])
        self.margin = 0.5
        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1, 3))  # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        self.points = self.generate_waypoints(self.path)
        print('Number of Waypoints: ', len(self.points))

        self.v = 8
        self.N = len(self.points)
        self.d = np.linalg.norm(self.points[: self.N - 1] - self.points[1: self.N], axis=1).reshape((-1, 1))
        self.t = self.d / self.v
        self.t[0] = 2 * self.t[0]
        self.t[-1] = 2 * self.t[-1]
        self.t = self.t * np.sqrt(1.65 / self.t)
        self.T = np.vstack((np.zeros(1), np.cumsum(self.t, axis=0))).flatten()

    def generate_waypoints(self, path):
        N = len(path)
        if N < 3:
            return path

        d_max = 0
        n = 0
        for i in range(1, N - 1):
            delta = np.linalg.norm(np.cross((path[0] - path[i]), (path[-1] - path[0]))) / np.linalg.norm(path[-1] - path[0])
            if delta > d_max:
                n = i
                d_max = delta

        if d_max > 0.90:
            left = self.generate_waypoints(path[:n + 1])
            right = self.generate_waypoints(path[n:])
            return np.vstack((left[:-1], right))
        else:
            if np.linalg.norm(path[-1] - path[0]) > 1.8:
                return np.vstack((path[0], path[N // 2], path[-1]))
            else:
                return np.vstack((path[0], path[-1]))

    # Minimum Snap
    def min_snap(self, t):
        t_f = np.sum(self.t)
        x = np.zeros((5, 3))
        n = 8 * (self.N - 1)
        A = np.zeros((n, n))
        b = np.zeros((n, 3))

        for i in range(0, self.N - 1):
            b[8 * i + 3] = self.points[i]
            b[8 * i + 4] = self.points[i + 1]

        for ti, i in zip(self.t, range(0, len(self.t))):
            sub_m = np.array([
                [0, 0, 0, 0, 0, 0, 0, 1],
                [ti ** 7, ti ** 6, ti ** 5, ti ** 4, ti ** 3, ti ** 2, ti, 1],
                [7 * ti ** 6, 6 * ti ** 5, 5 * ti ** 4, 4 * ti ** 3, 3 * ti ** 2, 2 * ti, 1, 0],
                [42 * ti ** 5, 30 * ti ** 4, 20 * ti ** 3, 12 * ti ** 2, 6 * ti, 2, 0, 0],
                [210 * ti ** 4, 120 * ti ** 3, 60 * ti ** 2, 24 * ti, 6, 0, 0, 0],
                [840 * ti ** 3, 360 * ti ** 2, 120 * ti, 24, 0, 0, 0, 0],
                [2520 * ti ** 2, 720 * ti, 120, 0, 0, 0, 0, 0],
                [5040 * ti, 720, 0, 0, 0, 0, 0, 0]], dtype=object)
            A[0, 6] = 1
            A[1, 5] = 2
            A[2, 4] = 6
            # A[3, 3] = 24

            if i == len(self.t) - 1:
                A[8 * i + 3: 8 * i + 11, 8 * i: 8 * i + 8] = sub_m[: 5, :]
            else:
                A[8 * i + 5, 8 * i + 14] = -1
                A[8 * i + 6, 8 * i + 13] = -2
                A[8 * i + 7, 8 * i + 12] = -6
                A[8 * i + 8, 8 * i + 11] = -24
                A[8 * i + 9, 8 * i + 10] = -120
                A[8 * i + 10, 8 * i + 9] = -720
                A[8 * i + 3: 8 * i + 11, 8 * i: 8 * i + 8] = sub_m

        c_i = np.linalg.solve(A, b)

        if t > t_f:
            x[0] = self.points[-1]
        else:
            seg_n = np.where(self.T > t)[0][0] - 1
            t_i = self.T[seg_n]
            dt = t - t_i
            c = c_i[8 * seg_n: 8 * (seg_n + 1)]
            x = np.array([
                [dt ** 7, dt ** 6, dt ** 5, dt ** 4, dt ** 3, dt ** 2, dt, 1],
                [7 * dt ** 6, 6 * dt ** 5, 5 * dt ** 4, 4 * dt ** 3, 3 * dt ** 2, 2 * dt, 1, 0],
                [42 * dt ** 5, 30 * dt ** 4, 20 * dt ** 3, 12 * dt ** 2, 6 * dt, 2, 0, 0],
                [210 * dt ** 4, 120 * dt ** 3, 60 * dt ** 2, 24 * dt, 6, 0, 0, 0],
                [840 * dt ** 3, 360 * dt ** 2, 120 * dt, 24, 0, 0, 0, 0]]) @ c
        return x

    def update(self, t):
        """
        PRIMARY METHOD
        Given the present time, return the desired flat output and derivatives.
        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        x_matrix = self.min_snap(t)

        x = x_matrix[0]
        x_dot = x_matrix[1]
        x_ddot = x_matrix[2]
        x_dddot = x_matrix[3]
        x_ddddot = x_matrix[4]

        # STUDENT CODE END
        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}
        return flat_output
