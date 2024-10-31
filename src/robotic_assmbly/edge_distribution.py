import Rhino.Geometry as rg
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path

# Inputs:
# EdgesTree - DataTree of Line objects representing truss edges
# RobotPointA - Point3d representing Robot A's base location
# RobotPointB - Point3d representing Robot B's base location

# Initialize outputs
assembly_order = DataTree[object]()
task_allocation = DataTree[str]()

# Collect all distances to determine the maximum distance for each robot
distances_A = []
distances_B = []

for path in EdgesTree.Paths:
    edges = EdgesTree.Branch(path)
    for edge in edges:
        mid_point = edge.PointAt(0.5)
        dist_to_A = mid_point.DistanceTo(RobotPointA)
        dist_to_B = mid_point.DistanceTo(RobotPointB)
        distances_A.append(dist_to_A)
        distances_B.append(dist_to_B)

max_dist_A = max(distances_A) if distances_A else 1.0
max_dist_B = max(distances_B) if distances_B else 1.0

# Process each branch
for path in EdgesTree.Paths:
    edges = EdgesTree.Branch(path)
    num_edges = len(edges)
    assembly_branch = []
    allocation_branch = []

    # Collect edge data
    edge_data = []
    for idx, edge in enumerate(edges):
        mid_point = edge.PointAt(0.5)
        dist_to_A = mid_point.DistanceTo(RobotPointA)
        dist_to_B = mid_point.DistanceTo(RobotPointB)
        rel_dist_to_A = dist_to_A / max_dist_A
        rel_dist_to_B = dist_to_B / max_dist_B
        edge_data.append({
            'index': idx,
            'edge': edge,
            'dist_to_A': dist_to_A,
            'dist_to_B': dist_to_B,
            'rel_dist_to_A': rel_dist_to_A,
            'rel_dist_to_B': rel_dist_to_B
        })

    if path.Indices[-1] == 0:
        # Branch {*;0}
        # Determine which robot is closer to the total of all edges
        total_rel_dist_A = sum(e['rel_dist_to_A'] for e in edge_data)
        total_rel_dist_B = sum(e['rel_dist_to_B'] for e in edge_data)
        if total_rel_dist_A <= total_rel_dist_B:
            closer_robot = '0'
            farther_robot = '1'
        else:
            closer_robot = '1'
            farther_robot = '0'

        # Assign edges to robots
        # Closer robot gets two edges (farthest from its base point)
        # Farther robot gets one edge (closest to its base point)
        if closer_robot == '0':
            edge_data.sort(key=lambda e: e['dist_to_A'], reverse=True)
            closer_edges = edge_data[:2]
            farther_edges = [e for e in edge_data if e not in closer_edges]
            farther_edge = min(farther_edges, key=lambda e: e['dist_to_B'])
        else:
            edge_data.sort(key=lambda e: e['dist_to_B'], reverse=True)
            closer_edges = edge_data[:2]
            farther_edges = [e for e in edge_data if e not in closer_edges]
            farther_edge = min(farther_edges, key=lambda e: e['dist_to_A'])

        # Assign to assembly order and task allocation
        assigned_edges = []
        for e in closer_edges:
            assigned_edges.append((e['index'], closer_robot, e['edge']))
        assigned_edges.append((farther_edge['index'], farther_robot, farther_edge['edge']))

        # Arrange edges to alternate for the first two edges
        assigned_edges.sort(key=lambda x: x[0])  # Sort by original index
        if assigned_edges[0][1] == assigned_edges[1][1]:
            # If first two edges are assigned to the same robot, swap the second with the third
            assigned_edges[1], assigned_edges[2] = assigned_edges[2], assigned_edges[1]

        # Add to outputs
        for idx, robot, edge in assigned_edges:
            assembly_branch.append(edge)
            allocation_branch.append(robot)
    else:
        # Branch {*;1}
        # Assign each edge to the relative closest robot
        for e in edge_data:
            idx = e['index']
            edge = e['edge']
            if e['rel_dist_to_A'] <= e['rel_dist_to_B']:
                robot = '0'
            else:
                robot = '1'
            assembly_branch.append(edge)
            allocation_branch.append(robot)

        # For each robot, sort their assigned edges by decreasing absolute distance
        # Since we need to maintain assembly order, we can sort the entire branch if necessary
        # For now, we keep the order as is, assuming the original order is acceptable

    # Add branches to outputs
    assembly_order.AddRange(assembly_branch, path)
    task_allocation.AddRange(allocation_branch, path)

# Outputs:
# AssemblyOrder - DataTree of edges in assembly order
# TaskAllocation - DataTree of strings indicating which robot ('0' or '1') is assigned to each edge
