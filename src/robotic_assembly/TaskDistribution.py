# Import necessary modules from Rhino and Grasshopper
import Rhino.Geometry as rg                       # Provides geometric types and operations for Rhino
from Grasshopper import DataTree                  # Allows creation and manipulation of DataTrees in Grasshopper
from Grasshopper.Kernel.Data import GH_Path       # Represents a path in a DataTree

# Inputs:
# EdgesTree: DataTree of Line objects representing truss edges
# RobotPointA: Point3d representing Robot A's base location
# RobotPointB: Point3d representing Robot B's base location

# Initialize output DataTrees for assembly order and task allocation
assembly_order = DataTree[object]()               # Will store the edges in the order they should be assembled
task_allocation = DataTree[str]()                 # Will store the robot assignment for each edge

# Initialize lists to collect all distances from edge midpoints to each robot
distances_A = []                                  # Distances from edges to Robot A
distances_B = []                                  # Distances from edges to Robot B

# First pass: Calculate distances from each edge's midpoint to both robots to determine maximum distances
for path in EdgesTree.Paths:
    edges = EdgesTree.Branch(path)                # Get all edges in the current branch
    for edge in edges:
        mid_point = edge.PointAt(0.5)             # Calculate the midpoint of the edge
        dist_to_A = mid_point.DistanceTo(RobotPointA)  # Distance to Robot A
        dist_to_B = mid_point.DistanceTo(RobotPointB)  # Distance to Robot B
        distances_A.append(dist_to_A)             # Collect distances for Robot A
        distances_B.append(dist_to_B)             # Collect distances for Robot B

# Determine the maximum distances to each robot for normalization purposes
max_dist_A = max(distances_A) if distances_A else 1.0  # Avoid division by zero
max_dist_B = max(distances_B) if distances_B else 1.0

# Second pass: Process each branch to assign edges to robots based on proximity
for path in EdgesTree.Paths:
    edges = EdgesTree.Branch(path)                # Get edges in the current branch
    num_edges = len(edges)                        # Number of edges in this branch
    assembly_branch = []                          # Will hold the assembly order for this branch
    allocation_branch = []                        # Will hold the task allocation for this branch

    # Collect detailed data about each edge
    edge_data = []
    for idx, edge in enumerate(edges):
        mid_point = edge.PointAt(0.5)             # Midpoint of the edge
        dist_to_A = mid_point.DistanceTo(RobotPointA)  # Absolute distance to Robot A
        dist_to_B = mid_point.DistanceTo(RobotPointB)  # Absolute distance to Robot B
        rel_dist_to_A = dist_to_A / max_dist_A    # Normalized distance to Robot A
        rel_dist_to_B = dist_to_B / max_dist_B    # Normalized distance to Robot B
        edge_data.append({
            'index': idx,                         # Original index of the edge
            'edge': edge,                         # The edge geometry
            'dist_to_A': dist_to_A,               # Absolute distance to Robot A
            'dist_to_B': dist_to_B,               # Absolute distance to Robot B
            'rel_dist_to_A': rel_dist_to_A,       # Relative distance to Robot A
            'rel_dist_to_B': rel_dist_to_B        # Relative distance to Robot B
        })

    # Special handling for the first branch
    if path.Indices[-1] == 0:
        # Determine which robot is closer overall to the edges in this branch
        total_rel_dist_A = sum(e['rel_dist_to_A'] for e in edge_data)
        total_rel_dist_B = sum(e['rel_dist_to_B'] for e in edge_data)
        if total_rel_dist_A <= total_rel_dist_B:
            closer_robot = '0'                    # Robot A is closer
            farther_robot = '1'                   # Robot B is farther
        else:
            closer_robot = '1'                    # Robot B is closer
            farther_robot = '0'                   # Robot A is farther

        # Assign edges to robots
        # Closer robot gets two edges farthest from its base
        # Farther robot gets one edge closest to its base
        if closer_robot == '0':
            edge_data.sort(key=lambda e: e['dist_to_A'], reverse=True)
            closer_edges = edge_data[:2]          # Two edges for Robot A
            farther_edges = [e for e in edge_data if e not in closer_edges]
            farther_edge = min(farther_edges, key=lambda e: e['dist_to_B'])  # Edge for Robot B
        else:
            edge_data.sort(key=lambda e: e['dist_to_B'], reverse=True)
            closer_edges = edge_data[:2]          # Two edges for Robot B
            farther_edges = [e for e in edge_data if e not in closer_edges]
            farther_edge = min(farther_edges, key=lambda e: e['dist_to_A'])  # Edge for Robot A

        # Prepare the list of assigned edges with their corresponding robot
        assigned_edges = []
        for e in closer_edges:
            assigned_edges.append((e['index'], closer_robot, e['edge']))
        assigned_edges.append((farther_edge['index'], farther_robot, farther_edge['edge']))

        # Arrange edges to alternate robots for the first two edges if necessary
        assigned_edges.sort(key=lambda x: x[0])   # Sort by original index to maintain order
        if assigned_edges[0][1] == assigned_edges[1][1]:
            # Swap second and third edges if first two are assigned to the same robot
            assigned_edges[1], assigned_edges[2] = assigned_edges[2], assigned_edges[1]

        # Add assigned edges to the output branches
        for idx, robot, edge in assigned_edges:
            assembly_branch.append(edge)
            allocation_branch.append(robot)
    else:
        # For other branches, assign each edge to the closest robot
        for e in edge_data:
            idx = e['index']
            edge = e['edge']
            if e['rel_dist_to_A'] <= e['rel_dist_to_B']:
                robot = '0'                        # Assign to Robot A
            else:
                robot = '1'                        # Assign to Robot B
            assembly_branch.append(edge)
            allocation_branch.append(robot)

        # Note: Edges are kept in their original order to maintain assembly sequence

    # Add the assembly order and task allocation for this branch to the output DataTrees
    assembly_order.AddRange(assembly_branch, path)
    task_allocation.AddRange(allocation_branch, path)

# Outputs:
# assembly_order: DataTree of edges in the determined assembly order
# task_allocation: DataTree of strings indicating which robot ('0' or '1') is assigned to each edge
