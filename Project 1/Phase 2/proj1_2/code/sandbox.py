import inspect
import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import time

from flightsim.axes3ds import Axes3Ds
from flightsim.world import World

from proj1_2.code.occupancy_map import OccupancyMap
from proj1_2.code.graph_search import graph_search

# Choose a test example file. You should write your own example files too!
filename = [('custom_world_2.json', False), ('custom_world_2(1).json', False), ('custom_world_2(2).json', False),
            ('custom_world_2(3).json', False), ('custom_world_2(4).json', False), ('custom_world_2.json', True),
            ('custom_world_2(1).json', True), ('custom_world_2(2).json', True), ('custom_world_2(3).json', True),
            ('custom_world_2(4).json', True)]

margin_vals = [0.9, 0.7, 0.5, 0.3, 0.2]
nodes = np.zeros((2, 5))
i = 0
for f in filename:
    if i == 5:
        i = 0
    # Load the test example.
    file = Path(inspect.getsourcefile(lambda: 0)).parent.resolve() / '..' / 'util' / f[0]
    world = World.from_file(file)          # World boundary and obstacles.
    resolution = world.world['resolution'] # (x,y,z) resolution of discretization, shape=(3,).
    margin = world.world['margin']         # Scalar spherical robot size or safety margin.
    start  = world.world['start']          # Start point, shape=(3,)
    goal   = world.world['goal']           # Goal point, shape=(3,)
    # Run your code and return the path.
    #############################
    # fig = plt.figure()
    # ax = Axes3Ds(fig)
    # world.draw(ax)
    # ax.plot([start[0]], [start[1]], [start[2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='none')
    # ax.plot([goal[0]],  [goal[1]],  [goal[2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='none')
    # ax.set_title('Custom World')
    # plt.show()
    #########################
    start_time = time.time()
    path, node_expanded = graph_search(world, resolution, margin, start, goal, f[1])
    end_time = time.time()

    # Astar
    if f[1]:
        nodes[0, i] = node_expanded

    # Dijkstra
    if not f[1]:
        nodes[1, i] = node_expanded

    # Print results.
    print()
    print('Total number of nodes expanded:', node_expanded)
    print(f'Solved in {end_time-start_time:.2f} seconds')
    if path is not None:
        number_points = path.shape[0]
        length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
        print(f'The discrete path has {number_points} points.')
        print(f'The path length is {length:.2f} meters.')
    else:
        print('No path found.')
    # Draw the world, start, and goal.
    '''
    fig = plt.figure()
    ax = Axes3Ds(fig)
    world.draw(ax)
    ax.plot([start[0]], [start[1]], [start[2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='none')
    ax.plot([goal[0]],  [goal[1]],  [goal[2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='none')
    ax.set_title('Custom World')

    # Plot your path on the true World.
    if path is not None:
        world.draw_line(ax, path, color='blue')
        world.draw_points(ax, path, color='blue')
    '''
    '''
    # For debugging, you can visualize the path on the provided occupancy map.
    # Very, very slow! Comment this out if you're not using it.
    fig = plt.figure()
    ax = Axes3Ds(fig)
    oc = OccupancyMap(world, resolution, margin)
    oc.draw(ax)
    ax.plot([start[0]], [start[1]], [start[2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='none')
    ax.plot([goal[0]],  [goal[1]],  [goal[2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='none')
    if path is not None:
        world.draw_line(ax, path, color='blue')
        world.draw_points(ax, path, color='blue')
    '''
    i += 1
ax = plt.axes()
ax.plot(margin_vals, nodes[0, :], marker='o', c='b', label='A-star')
ax.plot(margin_vals, nodes[1, :], marker='o', c='r', label='Dijkstra')
plt.title('Path Planning Performance')
plt.legend()
ax.legend(loc='upper right')
ax.set_xlabel('Map Resolution')
ax.set_ylabel('No. Nodes Expanded')
plt.show()
