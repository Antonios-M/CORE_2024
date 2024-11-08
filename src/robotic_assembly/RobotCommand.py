import Rhino.Geometry as rg
import Grasshopper
from Grasshopper.Kernel.Data import GH_Path

# Inputs:
# target_planes - DataTree of Planes in assembly order
# approach_planes - DataTree of Planes corresponding to target_planes (optional)
# task_allocation - DataTree of integers (0 or 1) indicating robot assignment (0 for A, 1 for B)
# pick_up_planes_A - List of Planes for Robot A's pick-up points
# pick_up_planes_B - List of Planes for Robot B's pick-up points
# pickup_approach_planes_A - List of Planes for Robot A's pickup approach points (optional)
# pickup_approach_planes_B - List of Planes for Robot B's pickup approach points (optional)
# pause_command - Command to pause the robot (optional)
# waiting_command - Command for waiting state (optional)
# home_plane_A - Plane for Robot A's home position (optional)
# home_plane_B - Plane for Robot B's home position (optional)

# Ensure inputs are set to the correct access and type hints in the Grasshopper Python component

# Initialize output data trees
robotA_commands = Grasshopper.DataTree[object]()
robotB_commands = Grasshopper.DataTree[object]()
robotA_status = Grasshopper.DataTree[str]()
robotB_status = Grasshopper.DataTree[str]()

# Initialize gripper states and last edge counters
gripper_state = {'A': 0, 'B': 0}
last_edge_counter = {'A': None, 'B': None}

# Initialize edge counter
edge_counter = 0

# Initialize pickup plane indices for each robot
pickup_index = {'A': 0, 'B': 0}

# Get the total number of pickup planes for each robot
num_pickup_planes_A = len(pick_up_planes_A) if pick_up_planes_A else 1
num_pickup_planes_B = len(pick_up_planes_B) if pick_up_planes_B else 1

# Get the total number of pickup approach planes for each robot
num_pickup_approach_planes_A = len(pickup_approach_planes_A) if pickup_approach_planes_A else 0
num_pickup_approach_planes_B = len(pickup_approach_planes_B) if pickup_approach_planes_B else 0

# Process each branch
for path in target_planes.Paths:
    planes = target_planes.Branch(path)
    approaches = approach_planes.Branch(path) if approach_planes and approach_planes.PathExists(path) else [None]*len(planes)
    allocations = task_allocation.Branch(path)
    num_edges = len(planes)
    
    # Check if the branch is {*;0}
    if path.Indices[-1] == 0:
        # Special handling for branch {*;0}
        
        # Collect indices of edges placed by each robot
        indices_by_robot = {'A': [], 'B': []}
        for idx, alloc in enumerate(allocations):
            robot_name = 'A' if alloc == 0 else 'B'
            indices_by_robot[robot_name].append(idx)
        
        # Prepare variables to keep track of steps
        max_steps = 0  # To synchronize steps between robots
        # Lists to store actions per robot
        actions_A = []
        actions_B = []
        
        # Process the first two edges
        for idx in range(2):
            edge_counter += 1
            plane = planes[idx]
            approach = approaches[idx] if idx < len(approaches) else None
            robot = allocations[idx]
            assigned_robot = 'A' if robot == 0 else 'B'
            other_robot = 'B' if assigned_robot == 'A' else 'A'
            last_edge_counter[assigned_robot] = edge_counter
            
            # Select the pickup plane for the assigned robot
            if assigned_robot == 'A':
                pickup_plane_list = pick_up_planes_A
                pickup_approach_list = pickup_approach_planes_A
                num_pickup_planes = num_pickup_planes_A
                num_pickup_approach_planes = num_pickup_approach_planes_A
                if num_pickup_planes > 0:
                    pickup_plane = pickup_plane_list[pickup_index['A'] % num_pickup_planes]
                else:
                    pickup_plane = None  # Handle empty list case
                if num_pickup_approach_planes > 0:
                    pickup_approach_plane = pickup_approach_list[pickup_index['A'] % num_pickup_approach_planes]
                else:
                    pickup_approach_plane = None
                pickup_index['A'] += 1
            else:
                pickup_plane_list = pick_up_planes_B
                pickup_approach_list = pickup_approach_planes_B
                num_pickup_planes = num_pickup_planes_B
                num_pickup_approach_planes = num_pickup_approach_planes_B
                if num_pickup_planes > 0:
                    pickup_plane = pickup_plane_list[pickup_index['B'] % num_pickup_planes]
                else:
                    pickup_plane = None  # Handle empty list case
                if num_pickup_approach_planes > 0:
                    pickup_approach_plane = pickup_approach_list[pickup_index['B'] % num_pickup_approach_planes]
                else:
                    pickup_approach_plane = None
                pickup_index['B'] += 1
            
            # Steps for the assigned robot
            steps_robot = []
            # Step 1: Move to pickup approach plane (if provided)
            if pickup_approach_plane is not None:
                steps_robot.append(("Move to pickup approach", pickup_approach_plane))
            # Step 2: Move to pick up
            if pickup_plane is not None:
                steps_robot.append(("Move to pick up", pickup_plane))
            else:
                steps_robot.append(("Move to pick up", "No pickup plane provided"))
            # Step 3: Close gripper
            steps_robot.append(("Close gripper", gripper_close_command))
            gripper_state[assigned_robot] = 1
            # Step 4: Move to approach plane (if provided)
            if approach is not None:
                steps_robot.append((f"Move to approach plane {edge_counter}", approach))
            # Step 5: Move to target plane
            steps_robot.append((f"Move to target {edge_counter}", plane))
            # Step 6 onwards: Holding edge
            steps_robot.append((f"Holding edge {edge_counter}", waiting_command))
            # Add steps to robot's action list
            if assigned_robot == 'A':
                actions_A.extend(steps_robot)
            else:
                actions_B.extend(steps_robot)
            
            # For the other robot, add waiting steps to synchronize
            num_steps = len(steps_robot)
            waiting_status = f"Holding edge {last_edge_counter[other_robot]}" if gripper_state[other_robot] == 1 else "Waiting for other robot"
            waiting_steps = [(waiting_status, waiting_command)] * num_steps
            if other_robot == 'A':
                actions_A.extend(waiting_steps)
            else:
                actions_B.extend(waiting_steps)
            
            max_steps = max(max_steps, len(actions_A), len(actions_B))
        
        # After first two edges, both robots pause for connection
        if pause_command is not None:
            pause_step = ("Pause for connection", pause_command)
            actions_A.append(pause_step)
            actions_B.append(pause_step)
            max_steps += 1
        
        # Process the third edge if it exists
        if num_edges > 2:
            idx = 2  # Index of the third edge
            edge_counter += 1
            plane = planes[idx]
            approach = approaches[idx] if idx < len(approaches) else None
            robot = allocations[idx]
            assigned_robot = 'A' if robot == 0 else 'B'
            other_robot = 'B' if assigned_robot == 'A' else 'A'
            last_edge_counter[assigned_robot] = edge_counter
            
            # Select the pickup plane for the assigned robot
            if assigned_robot == 'A':
                pickup_plane_list = pick_up_planes_A
                pickup_approach_list = pickup_approach_planes_A
                num_pickup_planes = num_pickup_planes_A
                num_pickup_approach_planes = num_pickup_approach_planes_A
                if num_pickup_planes > 0:
                    pickup_plane = pickup_plane_list[pickup_index['A'] % num_pickup_planes]
                else:
                    pickup_plane = None
                if num_pickup_approach_planes > 0:
                    pickup_approach_plane = pickup_approach_list[pickup_index['A'] % num_pickup_approach_planes]
                else:
                    pickup_approach_plane = None
                pickup_index['A'] += 1
            else:
                pickup_plane_list = pick_up_planes_B
                pickup_approach_list = pickup_approach_planes_B
                num_pickup_planes = num_pickup_planes_B
                num_pickup_approach_planes = num_pickup_approach_planes_B
                if num_pickup_planes > 0:
                    pickup_plane = pickup_plane_list[pickup_index['B'] % num_pickup_planes]
                else:
                    pickup_plane = None
                if num_pickup_approach_planes > 0:
                    pickup_approach_plane = pickup_approach_list[pickup_index['B'] % num_pickup_approach_planes]
                else:
                    pickup_approach_plane = None
                pickup_index['B'] += 1
            
            # Steps for the assigned robot
            steps_robot = []
            # Step 1: Move to pickup approach plane (if provided)
            if pickup_approach_plane is not None:
                steps_robot.append(("Move to pickup approach", pickup_approach_plane))
            # Step 2: Move to pick up
            if pickup_plane is not None:
                steps_robot.append(("Move to pick up", pickup_plane))
            else:
                steps_robot.append(("Move to pick up", "No pickup plane provided"))
            # Step 3: Close gripper
            steps_robot.append(("Close gripper", gripper_close_command))
            gripper_state[assigned_robot] = 1
            # Step 4: Move to approach plane (if provided)
            if approach is not None:
                steps_robot.append((f"Move to approach plane {edge_counter}", approach))
            # Step 5: Move to target plane
            steps_robot.append((f"Move to target {edge_counter}", plane))
            # Step 6: Pause for connection
            if pause_command is not None:
                steps_robot.append(("Pause for connection", pause_command))
            # Step 7: Open gripper
            steps_robot.append(("Open gripper", gripper_open_command))
            gripper_state[assigned_robot] = 0
            # Step 8: Move back to approach plane (if provided)
            if approach is not None:
                steps_robot.append((f"Move to approach plane {edge_counter}", approach))
            # Step 9: Move to home plane (if provided)
            home_plane = home_plane_A if assigned_robot == 'A' else home_plane_B
            if home_plane is not None:
                steps_robot.append(("Moving to home plane", home_plane))
            # Add steps to robot's action list
            if assigned_robot == 'A':
                actions_A.extend(steps_robot)
            else:
                actions_B.extend(steps_robot)
            
            # For the other robot, continue holding edge until pause for connection
            other_steps = []
            for i, (step, cmd) in enumerate(steps_robot):
                if step == "Pause for connection" and pause_command is not None:
                    other_steps.append(("Pause for connection", pause_command))
                else:
                    if gripper_state[other_robot] == 1:
                        other_steps.append((f"Holding edge {last_edge_counter[other_robot]}", waiting_command))
                    else:
                        other_steps.append(("Waiting for other robot", waiting_command))
            if other_robot == 'A':
                actions_A.extend(other_steps)
            else:
                actions_B.extend(other_steps)
            
            max_steps = max(max_steps, len(actions_A), len(actions_B))
        
        # After the third edge and pause, both robots can open grippers and move home
        # Open gripper for the robot that was holding edge
        for robot in ['A', 'B']:
            if gripper_state[robot] == 1:
                gripper_state[robot] = 0
                steps = []
                # Open gripper
                steps.append(("Open gripper", gripper_open_command))
                # Move back to approach plane (if provided)
                if indices_by_robot[robot]:
                    last_edge_idx = indices_by_robot[robot][-1]
                    approach = approaches[last_edge_idx] if last_edge_idx < len(approaches) else None
                    if approach is not None:
                        steps.append((f"Move to approach plane {last_edge_counter[robot]}", approach))
                else:
                    approach = None
                # Move to home plane (if provided)
                home_plane = home_plane_A if robot == 'A' else home_plane_B
                if home_plane is not None:
                    steps.append(("Moving to home plane", home_plane))
                # Add steps to robot's action list
                if robot == 'A':
                    actions_A.extend(steps)
                else:
                    actions_B.extend(steps)
                max_steps = max(max_steps, len(actions_A), len(actions_B))
        
        # Synchronize steps between robots
        total_steps = max(len(actions_A), len(actions_B))
        for i in range(total_steps):
            sub_path = GH_Path(path)
            sub_path = sub_path.AppendElement(i)
            # Robot A actions
            if i < len(actions_A):
                status_A, command_A = actions_A[i]
            else:
                status_A = "Waiting for other robot"
                command_A = waiting_command
            robotA_status.Add(status_A, sub_path)
            robotA_commands.Add(command_A, sub_path)
            # Robot B actions
            if i < len(actions_B):
                status_B, command_B = actions_B[i]
            else:
                status_B = "Waiting for other robot"
                command_B = waiting_command
            robotB_status.Add(status_B, sub_path)
            robotB_commands.Add(command_B, sub_path)
    else:
        # Handle other branches as before
        for idx in range(num_edges):
            plane = planes[idx]
            approach = approaches[idx] if idx < len(approaches) else None
            robot = allocations[idx]
            assigned_robot = 'A' if robot == 0 else 'B'
            other_robot = 'B' if assigned_robot == 'A' else 'A'
            edge_counter += 1
            last_edge_counter[assigned_robot] = edge_counter
            
            # Select the pickup plane for the assigned robot
            if assigned_robot == 'A':
                pickup_plane_list = pick_up_planes_A
                pickup_approach_list = pickup_approach_planes_A
                num_pickup_planes = num_pickup_planes_A
                num_pickup_approach_planes = num_pickup_approach_planes_A
                if num_pickup_planes > 0:
                    pickup_plane = pickup_plane_list[pickup_index['A'] % num_pickup_planes]
                else:
                    pickup_plane = None
                if num_pickup_approach_planes > 0:
                    pickup_approach_plane = pickup_approach_list[pickup_index['A'] % num_pickup_approach_planes]
                else:
                    pickup_approach_plane = None
                pickup_index['A'] += 1
            else:
                pickup_plane_list = pick_up_planes_B
                pickup_approach_list = pickup_approach_planes_B
                num_pickup_planes = num_pickup_planes_B
                num_pickup_approach_planes = num_pickup_approach_planes_B
                if num_pickup_planes > 0:
                    pickup_plane = pickup_plane_list[pickup_index['B'] % num_pickup_planes]
                else:
                    pickup_plane = None
                if num_pickup_approach_planes > 0:
                    pickup_approach_plane = pickup_approach_list[pickup_index['B'] % num_pickup_approach_planes]
                else:
                    pickup_approach_plane = None
                pickup_index['B'] += 1
            
            # Steps for the assigned robot
            steps_robot = []
            # Step 1: Move to pickup approach plane (if provided)
            if pickup_approach_plane is not None:
                steps_robot.append(("Move to pickup approach", pickup_approach_plane))
            # Step 2: Move to pick up
            if pickup_plane is not None:
                steps_robot.append(("Move to pick up", pickup_plane))
            else:
                steps_robot.append(("Move to pick up", "No pickup plane provided"))
            # Step 3: Close gripper
            steps_robot.append(("Close gripper", gripper_close_command))
            gripper_state[assigned_robot] = 1
            # Step 4: Move to approach plane (if provided)
            if approach is not None:
                steps_robot.append((f"Move to approach plane {edge_counter}", approach))
            # Step 5: Move to target plane
            steps_robot.append((f"Move to target {edge_counter}", plane))
            # Step 6: Pause for connection (if provided)
            if pause_command is not None:
                steps_robot.append(("Pause for connection", pause_command))
            # Step 7: Open gripper
            steps_robot.append(("Open gripper", gripper_open_command))
            gripper_state[assigned_robot] = 0
            # Step 8: Move back to approach plane (if provided)
            if approach is not None:
                steps_robot.append((f"Move to approach plane {edge_counter}", approach))
            # Step 9: Move to home plane (if provided)
            home_plane = home_plane_A if assigned_robot == 'A' else home_plane_B
            if home_plane is not None:
                steps_robot.append(("Moving to home plane", home_plane))
            
            # Steps for the other robot
            other_steps = []
            for step, cmd in steps_robot:
                if step == "Pause for connection" and pause_command is not None:
                    other_steps.append(("Pause for connection", pause_command))
                else:
                    if gripper_state[other_robot] == 1:
                        other_steps.append((f"Holding edge {last_edge_counter[other_robot]}", waiting_command))
                    else:
                        other_steps.append(("Waiting for other robot", waiting_command))
            
            # Add steps to data trees
            for i in range(len(steps_robot)):
                sub_path = GH_Path(path)
                sub_path = sub_path.AppendElement(idx)
                sub_path = sub_path.AppendElement(i)
                # Assigned robot
                if assigned_robot == 'A':
                    robotA_status.Add(steps_robot[i][0], sub_path)
                    robotA_commands.Add(steps_robot[i][1], sub_path)
                    robotB_status.Add(other_steps[i][0], sub_path)
                    robotB_commands.Add(other_steps[i][1], sub_path)
                else:
                    robotB_status.Add(steps_robot[i][0], sub_path)
                    robotB_commands.Add(steps_robot[i][1], sub_path)
                    robotA_status.Add(other_steps[i][0], sub_path)
                    robotA_commands.Add(other_steps[i][1], sub_path)

# Outputs:
RobotA_Command = robotA_commands
RobotB_Command = robotB_commands
RobotA_Status = robotA_status
RobotB_Status = robotB_status
