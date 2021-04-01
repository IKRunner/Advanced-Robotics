"""
This file is not used for grading at all, and you should modify it any way you find useful.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

from flightsim.animate import animate
from flightsim.simulate import Quadrotor, simulate
from flightsim.world import World
from flightsim.axes3ds import Axes3Ds
from flightsim.crazyflie_params import quad_params
from flightsim import hover_traj
from pathlib import Path
import inspect

import waypoint_traj
import se3_control

# This object defines the quadrotor dynamical model and should not be changed.
quadrotor = Quadrotor(quad_params)

# You will complete the implementation of the SE3Control object.
my_se3_control = se3_control.SE3Control(quad_params)

# # This simple hover trajectory is useful for tuning control gains.
my_traj = hover_traj.HoverTraj()


# You will complete the implementation of the WaypointTraj object. It should
# work for any list of 3D coordinates, such as this example:
# x_rect = np.array([
#         [-2.0, -1.0, 0.0],
#         [2.0, -1.0, 0.0],
#         [2.0, 1.0, 0.0],
#         [-2.0, 1.0, 0.0],
#         [-2.0, -1.0, 0.0]])

# z_rect = np.array([
#         [0.0, -2.0, -1.0],
#         [0.0, 2.0, -1.0],
#         [0.0, 2.0, 1.0],
#         [0.0, -2.0, 1.0],
#         [0.0, -2.0, -1.0]])

points = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 3.0, 0.0],
        [2.0, 3.0, 1.0],
        [0.0, 2.0, 2.0],
        [0.0, 0.0, 2.0]])
#
#
# my_traj = waypoint_traj.WaypointTraj(points)

# Set simulation parameters.

'''
# Load the test example.
filename = 'custom_z_step_pos.json'
print("Loading: " + filename)
file = Path(inspect.getsourcefile(lambda: 0)).parent.resolve() / '..' / 'util' / filename
world = World.from_file(file) 
initial_state = world.world['initial_state']  # Initial state of quadrotor
points = world.world['points'] # Desired trajectory

t_final = 10
my_traj = waypoint_traj.WaypointTraj(points)
'''



# You may use the initial condition and a simple hover trajectory to examine the
# step response of your controller to an initial disturbance in position or
# orientation.



'''
# Initial angle
q = Rotation.from_euler('y', -40, degrees=True).as_quat()

r = Rotation.from_quat(q)
t = r.as_euler('ZXY')
'''

w = 2
world = World.empty((-w, w, -w, w, -w, w))
t_final = 20
my_traj = waypoint_traj.WaypointTraj(points)
q = np.array([0, 0, 0, 1])
initial_state = {'x': np.array([0, 0, 0]),
                 'v': np.zeros(3, ),
                 'q': q,  # [i,j,k,w]
                 'w': np.zeros(3, )}

# Perform simulation.
#
# This function performs the numerical simulation.  It returns arrays reporting
# the quadrotor state, the control outputs calculated by your controller, and
# the flat outputs calculated by you trajectory.

print('Simulate.')
(time, state, control, flat, exit) = simulate(initial_state,
                                              quadrotor,
                                              my_se3_control,
                                              my_traj,
                                              t_final,
                                              False)
print(exit.value)

# Plot Results
#
# You will need to make plots to debug your controllers and tune your gains.
# Here are some example of plots that may be useful.

# Position and Velocity vs. Time
(fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Position vs Time')
x = state['x']
x_des = flat['x']
ax = axes[0]
ax.plot(time, x_des[:, 0], 'r', time, x_des[:, 1], 'g', time, x_des[:, 2], 'b')
ax.plot(time, x[:, 0], 'r.', time, x[:, 1], 'g.', time, x[:, 2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('position, m')
ax.grid('major')
ax.set_title('Position')
v = state['v']
v_des = flat['x_dot']
ax = axes[1]
ax.plot(time, v_des[:, 0], 'r', time, v_des[:, 1], 'g', time, v_des[:, 2], 'b')
ax.plot(time, v[:, 0], 'r.', time, v[:, 1], 'g.', time, v[:, 2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('velocity, m/s')
ax.set_xlabel('time, s')
ax.grid('major')

# Orientation and Angular Velocity vs. Time
(fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Orientation vs Time')
q_des = control['cmd_q']
q = state['q']
ax = axes[0]
ax.plot(time, q_des[:, 0], 'r', time, q_des[:, 1], 'g', time, q_des[:, 2], 'b', time, q_des[:, 3], 'k')
ax.plot(time, q[:, 0], 'r.', time, q[:, 1], 'g.', time, q[:, 2], 'b.', time, q[:, 3], 'k.')
ax.legend(('i', 'j', 'k', 'w'))
ax.set_ylabel('quaternion')
ax.set_xlabel('time, s')
ax.grid('major')
w = state['w']
ax = axes[1]
ax.plot(time, w[:, 0], 'r.', time, w[:, 1], 'g.', time, w[:, 2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('angular velocity, rad/s')
ax.set_xlabel('time, s')
ax.grid('major')

# Commands vs. Time
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='Commands vs Time')
s = control['cmd_motor_speeds']
ax = axes[0]
ax.plot(time, s[:, 0], 'r.', time, s[:, 1], 'g.', time, s[:, 2], 'b.', time, s[:, 3], 'k.')
ax.legend(('1', '2', '3', '4'))
ax.set_ylabel('motor speeds, rad/s')
ax.grid('major')
ax.set_title('Commands')
M = control['cmd_moment']
ax = axes[1]
ax.plot(time, M[:, 0], 'r.', time, M[:, 1], 'g.', time, M[:, 2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('moment, N*m')
ax.grid('major')
T = control['cmd_thrust']
ax = axes[2]
ax.plot(time, T, 'k.')
ax.set_ylabel('thrust, N')
ax.set_xlabel('time, s')
ax.grid('major')

# 3D Paths
fig = plt.figure('3D Path')
ax = Axes3Ds(fig)
world.draw(ax)
ax.plot3D(state['x'][:, 0], state['x'][:, 1], state['x'][:, 2], 'b.')
ax.plot3D(flat['x'][:, 0], flat['x'][:, 1], flat['x'][:, 2], 'k')

# Euler angles
r1 = Rotation.from_quat(q_des)
r2 = Rotation.from_quat(q)
angles_des = r1.as_euler('ZXY')
angles = r2.as_euler('ZXY')


'''
# Z-step response
fig, ax = plt.subplots()
ax.plot(time, x_des[:, 2], label='Desired Z-position')
ax.plot(time, x[:, 2], label='Quadrotor Z-position')
ax.legend()
ax.legend(loc='upper right')
ax.set_ylabel('position, m')
ax.set_xlabel('time, s')
# ax.set_ylim(-0.01, 0.01)
ax.grid('major')
ax.set_title('Position')
fig.savefig('Plots/Z_step_pos.png')

# Y-step response
fig, ax = plt.subplots()
ax.plot(time, x_des[:, 1], label='Desired Y-position')
ax.plot(time, x[:, 1], label='Quadrotor Y-position')
ax.legend()
ax.legend(loc='upper right')
ax.set_ylabel('position, m')
ax.set_xlabel('time, s')
# ax.set_ylim(-0.01, 0.01)
ax.grid('major')
ax.set_title('Position')
fig.savefig('Plots/Y_step_pos.png')

# X-step response
fig, ax = plt.subplots()
ax.plot(time, x_des[:, 0],label='Desired X-position')
ax.plot(time, x[:, 0], label='Quadrotor X-position')
plt.legend()
ax.legend(loc='upper right')
ax.set_ylabel('position, m')
ax.set_xlabel('time, s')
# ax.set_ylim(-0.01, 0.01)
ax.grid('major')
ax.set_title('Position')
fig.savefig('Plots/X_step_pos.png')


# Yaw error (Euler)
fig, ax = plt.subplots()
ax.plot(time, angles_des[:, 0], label='Desired Yaw')
ax.plot(time, angles[:, 0], label='Quadrotor Yaw')
ax.set_title('Yaw Step Response')
plt.legend()
ax.legend(loc='upper right')
ax.set_ylabel('rad')
ax.set_xlabel('time, s')
# ax.set_ylim(-0.02, 0.01)
ax.grid('major')
fig.savefig('Plots/Yaw_step_pos.png')


# Roll error (Euler)
fig, ax = plt.subplots()
ax.plot(time, angles_des[:, 1], label='Desired Roll')
ax.plot(time, angles[:, 1], label='Quadrotor Roll')
ax.set_title('Roll Step Response')
ax.legend()
ax.legend(loc='upper right')
ax.set_ylabel('rad')
ax.set_xlabel('time, s')
# ax.set_ylim(-0.01, 0.01)
ax.grid('major')
fig.savefig('Plots/Roll_step_pos.png')

# Pitch error (Euler)
fig, ax = plt.subplots()
ax.plot(time, angles_des[:, 2], label='Desired Pitch')
ax.plot(time, angles[:, 2],label='Quadrotor Pitch')
ax.set_title('Pitch Step Response')
plt.legend()
ax.legend(loc='upper right')
ax.set_ylabel('rad')
ax.set_xlabel('time, s')
# ax.set_ylim(-0.01, 0.0025)
ax.grid('major')
fig.savefig('Plots/Pitch_step_pos.png')


'''




# Animation (Slow)
# Instead of viewing the animation live, you may provide a .mp4 filename to save.
R = Rotation.from_quat(state['q']).as_matrix()
ani = animate(time, state['x'], R, world=world, filename=None)
plt.show()
