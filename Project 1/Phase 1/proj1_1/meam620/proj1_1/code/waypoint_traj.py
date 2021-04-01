import numpy as np


class WaypointTraj(object):
    """
    Relevant state variables:
        self.points: (N, 3) array of N waypoint coordinates in 3D
        self.I_hat: (N - 1, 3) array of unit vectors describing direction of travel for all N - 1 segments
        self.vel_des: (N - 1, 3) array of velocity vectors in direction of travel for all N - 1  segments
        self.T: (N - 1, 2) array where first column is time duration of each segment and second column is start time of
                each segment
    """

    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """
        # Save waypoints and unit direction between waypoints to state
        self.points = points
        self.I_hat = np.zeros((points[:, 0].size - 1, points[0, :].size))

        # Compute direction of travel for each segment between waypoints and save to state
        diff = self.points[1:] - self.points[:-1]
        dist = np.linalg.norm(diff, ord=2, axis=1, keepdims=True)
        self.I_hat = diff / dist

        # Set desired velocity vector
        vel = 1.5
        self.vel_des = vel * self.I_hat

        # Time duration (column 0) and start time (column 1) of each segment
        self.T = np.zeros((points[:, 0].size - 1, 2))
        self.T[:, 0] = (dist / vel).T
        self.T[1:, 1] = np.cumsum(self.T[0:-1, 0], axis=0)

    def update(self, t):
        """
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
        # Position and velocity
        x = np.zeros((3,))
        x_dot = np.zeros((3,))

        # Yaw, yaw rate, acceleration, jerk, and snap always zero
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0
        if self.points.shape[0] == 1:
            x = self.points[-1, :]
            x_dot = np.array([0, 0, 0])

            flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                           'yaw': yaw, 'yaw_dot': yaw_dot}
            return flat_output

        # Time duration is within trajectory
        if t <= np.sum(self.T[:, 0]):
            # print('test1')
            ele = np.where(np.logical_or(self.T[:, 1] < t, self.T[:, 1] == t))
            # Set velocity and position state
            xdot = self.vel_des[ele[0][-1], :]
            x = self.points[ele[0][-1], :] + self.vel_des[ele[0][-1], :] * (t - self.T[ele[0][-1], 1])

        # Time is greater than duration of full trajectory
        if t > np.sum(self.T[:, 0]):
            # print('test2')
            x = self.points[-1, :]
            x_dot = np.array([0, 0, 0])

        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}

        # print(flat_output['x'])
        # print(flat_output['x_dot'])

        return flat_output
