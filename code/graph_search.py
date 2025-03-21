from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    if astar is True:
        heuristic_cost = 1.2    # A*
    else:
        heuristic_cost = 1.2    # Dijkstra
    # Initialization
    cost_c = 1
    queue = []
    heappush(queue, (0, start_index))
    cost = {start_index: 0}
    parent = {start_index: None}

    # Search Queue
    while queue:
        _, current_node = heappop(queue)
        if current_node == goal_index:
            break
        # Generate Neighbor
        neighbor_nodes = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if dx != 0 or dy != 0 or dz != 0:
                        neighbor_nodes.append((current_node[0] + dx, current_node[1] + dy, current_node[2] + dz))
        # Update Cost
        for neighbor_node in neighbor_nodes:
            if occ_map.is_occupied_index(neighbor_node):
                updated_cost = cost[current_node] + 1000000
            else:
                updated_cost = (cost[current_node] + cost_c * np.linalg.norm(np.array(current_node) - np.array(neighbor_node)))

            if neighbor_node not in cost or updated_cost < cost[neighbor_node]:
                priority = (updated_cost + heuristic_cost * np.linalg.norm(np.array(neighbor_node) - np.array(goal_index)))
                heappush(queue, (priority, neighbor_node))
                cost[neighbor_node] = updated_cost
                parent[neighbor_node] = current_node

    # Generate Path
    nodes_expanded = len(parent)
    path = goal
    goal_path = goal_index

    if goal_index not in parent:
        path = None

    while goal_path != start_index:
        goal_path = parent[goal_path]
        nodes_expanded += 1
        inter_path = occ_map.index_to_metric_center(goal_path)
        path = np.vstack([inter_path, path])
    path = np.vstack([start, path])

    # Return a tuple (path, nodes_expanded)
    return path, nodes_expanded
