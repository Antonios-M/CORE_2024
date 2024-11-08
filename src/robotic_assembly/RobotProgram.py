# Import necessary modules
import robolink as rl          # RoboDK API for communication with RoboDK software
import robodk as rdk           # RoboDK modeling and simulation library
import robodk.robodialogs as dialogs  # RoboDK module for dialog boxes
import Rhino.Geometry as rg    # Rhino.Geometry for geometric types and operations
import math                    # Math module for mathematical functions

# -------------------- Configuration Parameters --------------------
# Define the length of a cycle, i.e., the number of planes per cycle
cycle_length = 9  # You can modify this value as needed

# Global variables for robot motion parameters
speed_ms = 0.3          # Default speed in meters per second
accel_mss = 3.0         # Default acceleration in meters per second squared
blend_radius_m = 0.001  # Default blend radius in meters (for smooth motion between points)

# Initialize variables to store results and outputs
SuccessMessage = ""                   # String to accumulate success and error messages
TargetPoses = []                      # List to store target poses
Program = ""                          # Name of the RoboDK program created
TCPOrientations = []                  # List to store TCP orientations for each plane
TargetPointCoordinates = []           # List to store coordinates of target points
PlaneExtra = []                       # List to store planes that could not be reached

def plane_to_pose(plane):
    """
    Converts a Rhino plane object to a RoboDK pose matrix.

    Parameters:
    plane (rg.Plane): The Rhino plane to convert.

    Returns:
    rdk.Mat: A 4x4 transformation matrix representing the pose in RoboDK.
    """
    # Create a RoboDK matrix using the plane's axes and origin
    pose = rdk.Mat([
        [plane.XAxis.X, plane.YAxis.X, plane.ZAxis.X, plane.OriginX],
        [plane.XAxis.Y, plane.YAxis.Y, plane.ZAxis.Y, plane.OriginY],
        [plane.XAxis.Z, plane.YAxis.Z, plane.ZAxis.Z, plane.OriginZ],
        [0, 0, 0, 1]
    ])
    return pose

def format_tcp_orientation(plane, milestone):
    """
    Formats the TCP (Tool Center Point) orientation for logging purposes.

    Parameters:
    plane (rg.Plane): The plane representing the TCP orientation.
    milestone (str): A string indicating the current milestone or step.

    Returns:
    str: A formatted string with the TCP orientation.
    """
    x_axis = plane.XAxis
    y_axis = plane.YAxis
    z_axis = plane.ZAxis
    return f"TCP orientation at {milestone}:\n  X-axis: {x_axis}\n  Y-axis: {y_axis}\n  Z-axis: {z_axis}"

def format_target_coordinates(plane, milestone):
    """
    Formats the target point coordinates for logging purposes.

    Parameters:
    plane (rg.Plane): The plane representing the target point.
    milestone (str): A string indicating the current milestone or step.

    Returns:
    str: A formatted string with the target coordinates.
    """
    return f"{plane.OriginX}, {plane.OriginY}, {plane.OriginZ}"

# -------------------- Input Parameters --------------------
# Define the input parameters required for the script.
# Replace the following example inputs with your actual data before running the script.

# Example Inputs (Uncomment and replace with actual values)
# RobotName = "YourRobotName"  # Name of the robot in RoboDK
# PlanesList = [rg.Plane(...) for _ in range(63)]  # List of Rhino planes representing target positions
# UpdateRoboDK = True          # Boolean flag to update the RoboDK program
# StopOrder = [1, 3]           # List of step positions within the cycle where the program should pause (0-based index)
# SetSpeedOrder = [1, 3]       # List of step positions within the cycle where speed changes occur
# SetSpeedValue = [0.5, 0.7]   # Corresponding list of speed values in meters per second for the speed changes

# -------------------- Input Validation --------------------
# Function to validate that all necessary inputs are provided and correct

def validate_inputs():
    """
    Validates the input parameters required for the script to run.

    Returns:
    bool: True if all inputs are valid, False otherwise.
    """
    global SuccessMessage
    # List of required variables
    required_vars = ['RobotName', 'PlanesList', 'UpdateRoboDK']
    for var in required_vars:
        if var not in globals() or not globals()[var]:
            SuccessMessage = f"Error: {var} is required."
            return False
    
    # Validate that PlanesList is a non-empty list of Rhino Plane objects
    if not isinstance(PlanesList, list) or not PlanesList:
        SuccessMessage = "Error: PlanesList must be a non-empty list of Rhino Plane objects."
        return False
    if any(not isinstance(plane, rg.Plane) for plane in PlanesList):
        SuccessMessage = "Error: All elements in PlanesList must be Rhino Plane objects."
        return False
    
    # Validate StopOrder if it is provided
    if 'StopOrder' in globals() and StopOrder is not None:
        if not isinstance(StopOrder, list) or not all(isinstance(so, int) and 0 <= so < cycle_length for so in StopOrder):
            SuccessMessage = f"Error: StopOrder must be a list of non-negative integers within the cycle index range (0 to {cycle_length -1})."
            return False
    
    # Validate SetSpeedOrder and SetSpeedValue if they are provided
    if 'SetSpeedOrder' in globals() and SetSpeedOrder is not None:
        if not isinstance(SetSpeedOrder, list) or not all(isinstance(sso, int) and 0 <= sso < cycle_length for sso in SetSpeedOrder):
            SuccessMessage = f"Error: SetSpeedOrder must be a list of non-negative integers within the cycle index range (0 to {cycle_length -1})."
            return False
        if 'SetSpeedValue' not in globals() or not isinstance(SetSpeedValue, list) or not all(isinstance(sv, (int, float)) for sv in SetSpeedValue):
            SuccessMessage = "Error: SetSpeedValue must be a list of numerical values."
            return False
        if len(SetSpeedValue) < len(SetSpeedOrder):
            SuccessMessage = "Error: SetSpeedValue must have at least as many elements as SetSpeedOrder."
            return False
    
    return True

# -------------------- Main Execution --------------------
# Proceed only if the input validation passes
if validate_inputs():
    # Initialize a connection to RoboDK
    RDK = rl.Robolink()
    SuccessMessage = "Connected to RoboDK."
    
    # Retrieve the robot item from RoboDK by its name
    robot = RDK.Item(RobotName, rl.ITEM_TYPE_ROBOT)
    # Check if the robot item is valid
    if not robot.Valid():
        # If the robot is not found, append an error message
        SuccessMessage += f"\nError: Could not find '{RobotName}' in RoboDK."
    else:
        # If the robot is found, append a success message
        SuccessMessage += f"\nFound robot '{robot.Name()}' in RoboDK."
        # Get the robot's base (parent item)
        robot_base = robot.Parent()
        # Get the tool attached to the robot
        tool = robot.getLink(rl.ITEM_TYPE_TOOL)
        # Check if both base and tool are valid
        if not robot_base.Valid() or not tool.Valid():
            SuccessMessage += "\nCannot proceed without a valid base and tool."
        else:
            # Check if a program with the given name already exists in RoboDK
            existing_program = RDK.Item("MoveThroughPlanesProgram", rl.ITEM_TYPE_PROGRAM)
            if existing_program.Valid():
                # If it exists, delete it to replace with a new one
                existing_program.Delete()
            # Define the program name
            program_name = "MoveThroughPlanesProgram"
            # Create a new program in RoboDK associated with the robot
            program = RDK.AddProgram(program_name, robot)
            # Set the reference frame and tool for the program
            program.setFrame(robot_base)
            program.setTool(tool)
            # Set default speed and acceleration for the program
            program.setSpeed(speed_ms)          # Default speed
            program.setAcceleration(accel_mss)  # Default acceleration
            SuccessMessage += f"\nProgram '{program_name}' created."

            # Create a dictionary to map specific steps in the cycle to speed values
            speed_order_to_value = {}
            if 'SetSpeedOrder' in globals() and SetSpeedOrder is not None:
                for idx, step in enumerate(SetSpeedOrder):
                    if idx < len(SetSpeedValue):
                        # Map the step index to the corresponding speed value
                        speed_order_to_value[step] = SetSpeedValue[idx]
                    else:
                        # This case should not occur due to earlier validation
                        speed_order_to_value[step] = speed_ms  # Use default speed

            # Loop through each plane in the PlanesList
            for idx, plane in enumerate(PlanesList):
                plane_number = idx  # Current plane index (0-based)
                # Determine the current step within the cycle
                step_in_cycle = plane_number % cycle_length

                # Check if the current step requires a stop
                if 'StopOrder' in globals() and StopOrder is not None:
                    if step_in_cycle in StopOrder:
                        # Display a dialog to the user to confirm continuation
                        user_choice = dialogs.ShowMessageOkCancel(
                            f"Process stopped at Plane {plane_number + 1}. Click OK to continue.", "User Confirmation")
                        if not user_choice:
                            # If user cancels, append message and break the loop
                            SuccessMessage += f"\nProcess cancelled by the user at Plane {plane_number + 1}."
                            break  # Exit the loop
                        # Optionally, display a popup in RoboDK
                        program.RunInstruction("popup('Stopped','Press Ok to move on to the next step',True,False,blocking=True)", True)
                        SuccessMessage += f"\nProcess stopped at Plane {plane_number + 1}."

                # Check if the current step requires a speed change
                if 'SetSpeedOrder' in globals() and SetSpeedOrder is not None:
                    if step_in_cycle in SetSpeedOrder:
                        # Retrieve the new speed value for this step
                        speed_value = speed_order_to_value.get(step_in_cycle, speed_ms)
                        # Set the new speed in the program
                        program.setSpeed(speed_value)
                        SuccessMessage += f"\nSpeed set to {speed_value} m/s at Plane {plane_number + 1} (Step {step_in_cycle})."

                # Convert the Rhino plane to a RoboDK pose matrix in the robot's coordinate system
                target_pose_robot = plane_to_pose(plane)
                # Get the pose of the robot's reference frame
                ref_frame_pose = robot_base.Pose()
                # Calculate the target pose relative to the reference frame
                target_pose_relative = ref_frame_pose.inv() * target_pose_robot

                # Create a new target in RoboDK for the current plane
                target_name = f"Plane_{plane_number + 1}"
                # Add the target under the robot base with the robot as the associated item
                target = RDK.AddTarget(target_name, robot_base, robot)
                # Set the target as a Cartesian target (as opposed to joint target)
                target.setAsCartesianTarget()
                # Set the pose of the target relative to the reference frame
                target.setPose(target_pose_relative)

                try:
                    # Attempt to move the robot linearly (MoveL) to the target pose
                    robot.MoveL(target_pose_relative, False)
                    # Add a linear movement instruction to the program
                    program.MoveL(target)
                    SuccessMessage += f"\nPlane {plane_number + 1} reachable with linear movement relative to reference frame."
                except Exception as e:
                    # If linear movement fails, try joint movement (MoveJ)
                    try:
                        robot.MoveJ(target_pose_relative, False)
                        program.MoveJ(target)
                        SuccessMessage += f"\nPlane {plane_number + 1} reachable with joint movement after linear movement failed."
                    except Exception as e_joint:
                        # If both movements fail, log the error and add the plane to PlaneExtra
                        SuccessMessage += f"\nPlane {plane_number + 1} unreachable with both linear and joint movements. Error: {str(e_joint)}"
                        PlaneExtra.append(plane)

                # Log the TCP orientation and target coordinates for the current plane
                TCPOrientations.append(format_tcp_orientation(plane, f"Plane {plane_number + 1}"))
                TargetPointCoordinates.append(format_target_coordinates(plane, f"Plane {plane_number + 1}"))

            # Store the program name and append a success message
            Program = program_name
            SuccessMessage += f"\nProgram '{program_name}' updated successfully."

# -------------------- Output Results --------------------
# At the end of the script, print the accumulated success and error messages
print(SuccessMessage)
# Optionally, print or log other results if required
# For example:
# print("TCP Orientations:", TCPOrientations)
# print("Target Coordinates:", TargetPointCoordinates)
# print("Unreachable Planes:", PlaneExtra)
