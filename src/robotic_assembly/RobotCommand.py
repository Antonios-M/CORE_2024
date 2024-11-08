import Rhino.Geometry as rg
import Grasshopper
from Grasshopper.Kernel.Data import GH_Path

# Inputs:
# target_planes - DataTree of Planes in assembly order
# approach_planes - DataTree of Planes corresponding to target_planes (optional)
# task_allocation - DataTree of integers (0 or 1) indicating robot assignment (0 for A, 1 for B)
# pick_up_planes_A - List of Planes for Robot A's pick-up points
# pick_up_planes_B - List of Planes for Robot B's pick-up points
# pause_command - Command to pause the robot (optional)
# waiting_command - Command for waiting state (optional)
# home_plane_A - Plane for Robot A's home position (constant for each set)
# home_plane_B - Plane for Robot B's home position (constant for each set)
# guide_planes_A - List of optional guide planes for Robot A (constant for each set)
# guide_planes_B - List of optional guide planes for Robot B (constant for each set)

# Initialize output data trees
robotA_commands = Grasshopper.DataTree[object]()
robotB_commands = Grasshopper.DataTree[object]()
robotA_status = Grasshopper.DataTree[str]()
robotB_status = Grasshopper.DataTree[str]()

# Initialize gripper states and last edge counters for both robots
gripper_state = {'A': 0, 'B': 0}
last_edge_counter = {'A': None, 'B': None}

# Initialize edge counter and pickup plane indices
edge_counter = 0
pickup_index = {'A': 0, 'B': 0}

# Get the total number of pickup planes for each robot
num_pickup_planes_A = len(pick_up_planes_A) if pick_up_planes_A else 1
num_pickup_planes_B = len(pick_up_planes_B) if pick_up_planes_B else 1

# Process each branch of the target_planes DataTree
for path in target_planes.Paths:
    # Extract planes, approaches, and allocations from the current branch
    planes = target_planes.Branch(path)
    approaches = approach_planes.Branch(path) if approach_planes and approach_planes.PathExists(path) else [None] * len(planes)
    allocations = task_allocation.Branch(path)
    num_edges = len(planes)
    
    # Prepare lists to store actions per robot for the current branch
    actions_A = []
    actions_B = []

    # Process each edge (target plane) in the current branch
    for idx in range(num_edges):
        edge_counter += 1
        plane = planes[idx]
        approach = approaches[idx] if idx < len(approaches) else None
        robot = allocations[idx]
        assigned_robot = 'A' if robot == 0 else 'B'
        other_robot = 'B' if assigned_robot == 'A' else 'A'
        last_edge_counter[assigned_robot] = edge_counter

        # Determine pickup plane and home plane based on assigned robot
        if assigned_robot == 'A':
            pickup_plane = pick_up_planes_A[pickup_index['A'] % num_pickup_planes_A] if num_pickup_planes_A > 0 else None
            home_plane = home_plane_A
            guide_plane = guide_planes_A[0] if guide_planes_A else None
            pickup_index['A'] += 1
        else:
            pickup_plane = pick_up_planes_B[pickup_index['B'] % num_pickup_planes_B] if num_pickup_planes_B > 0 else None
            home_plane = home_plane_B
            guide_plane = guide_planes_B[0] if guide_planes_B else None
            pickup_index['B'] += 1

        # Define the sequence of steps for the assigned robot in the required order
        steps_robot = [
            ("Move to home plane", home_plane if home_plane else "No home plane provided"),
            ("Move to pick up", pickup_plane if pickup_plane else "No pickup plane provided"),
            ("Return to home plane", home_plane if home_plane else "No home plane provided"),
            ("Move to guide plane", guide_plane if guide_plane else "No guide plane provided"),
        ]

        if approach:
            steps_robot.append((f"Move to approach plane {edge_counter}", approach))

        steps_robot.append((f"Move to target {edge_counter}", plane))

        if approach:
            steps_robot.append((f"Return to approach plane {edge_counter}", approach))

        steps_robot.append(("Return to guide plane", guide_plane if guide_plane else "No guide plane provided"))

        steps_robot.append(("Return to home plane", home_plane if home_plane else "No home plane provided"))

        # Add steps to the corresponding robot's action list
        if assigned_robot == 'A':
            actions_A.extend(steps_robot)
        else:
            actions_B.extend(steps_robot)

        # Create synchronized waiting steps for the other robot
        waiting_status = f"Holding edge {last_edge_counter[other_robot]}" if gripper_state[other_robot] == 1 else "Waiting for other robot"
        waiting_steps = [(waiting_status, waiting_command)] * len(steps_robot)

        if other_robot == 'A':
            actions_A.extend(waiting_steps)
        else:
            actions_B.extend(waiting_steps)

    # Synchronize actions between robots and add them to the output data trees
    total_steps = max(len(actions_A), len(actions_B))
    for i in range(total_steps):
        sub_path = GH_Path(path).AppendElement(i)
        # Add Robot A actions and statuses
        if i < len(actions_A):
            status_A, command_A = actions_A[i]
        else:
            status_A, command_A = "Waiting for other robot", waiting_command
        robotA_status.Add(status_A, sub_path)
        robotA_commands.Add(command_A, sub_path)
        
        # Add Robot B actions and statuses
        if i < len(actions_B):
            status_B, command_B = actions_B[i]
        else:
            status_B, command_B = "Waiting for other robot", waiting_command
        robotB_status.Add(status_B, sub_path)
        robotB_commands.Add(command_B, sub_path)

# Outputs
RobotA_Command = robotA_commands
RobotB_Command = robotB_commands
RobotA_Status = robotA_status
RobotB_Status = robotB_status












