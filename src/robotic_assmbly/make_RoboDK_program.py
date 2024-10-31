import robolink as rl    # RoboDK API for controlling and communicating with RoboDK     
import robodk as rdk     # RoboDK tools, such as matrix operations and other utilities
import robodk.robodialogs as dialogs  # Import the dialog functions from RoboDK
import Rhino.Geometry as rg  # Import Rhino.Geometry module as rg
import math

# === Inputs ===
# RobotName: Name of the robot in RoboDK (e.g., "UR5")
# UpdateRoboDK: Boolean toggle to execute the script and update RoboDK when True
# PlanesList: List of Rhino Plane objects (each plane contains both target point and orientation)
# SetSpeedOrder: List of indices where speed commands should be inserted in the program
# SetSpeedValue: List of speed values corresponding to each SetSpeedOrder index
# StopOrder: List of indices where stop commands should be inserted in the program

# Initialize global variables for speed, acceleration, and blending radius
global speed_ms, accel_mss, blend_radius_m
speed_ms = None
accel_mss = None
blend_radius_m = None

# Initialize outputs
SuccessMessage = ""
TargetPoses = []  # List of converted poses
Program = ""
TCPOrientations = []
TargetPointCoordinates = []
PlaneExtra = []  # Unreachable planes for other robots to attempt

def plane_to_pose(plane):
    """
    Converts a Rhino plane to a RoboDK pose (4x4 matrix).
    Parameters:
        - plane: Rhino.Geometry.Plane object to be converted.
    Returns:
        - pose: RoboDK 4x4 transformation matrix (rdk.Mat).
    """
    pose = rdk.Mat([
        [plane.XAxis.X, plane.YAxis.X, plane.ZAxis.X, plane.OriginX],
        [plane.XAxis.Y, plane.YAxis.Y, plane.ZAxis.Y, plane.OriginY],
        [plane.XAxis.Z, plane.YAxis.Z, plane.ZAxis.Z, plane.OriginZ],
        [0, 0, 0, 1]
    ])
    return pose

def format_tcp_orientation(plane, milestone):
    """
    Formats the TCP orientation (X, Y, Z axes) for a given milestone.
    Parameters:
        - plane: Rhino.Geometry.Plane object representing the TCP.
        - milestone: String representing the step or target being reported.
    Returns:
        - String formatted with the X, Y, and Z axes of the TCP.
    """
    x_axis = plane.XAxis
    y_axis = plane.YAxis
    z_axis = plane.ZAxis
    return f"TCP orientation at {milestone}:\n  X-axis: {x_axis}\n  Y-axis: {y_axis}\n  Z-axis: {z_axis}"

def format_target_coordinates(plane, milestone):
    """
    Formats the target point coordinates as a string for a given milestone.
    Parameters:
        - plane: Rhino.Geometry.Plane object representing the target.
        - milestone: String representing the step or target being reported.
    Returns:
        - String formatted with the target point's coordinates.
    """
    return f"{plane.OriginX}, {plane.OriginY}, {plane.OriginZ}"

# Main Logic for Processing Inputs and Creating the RoboDK Program
if not RobotName:
    SuccessMessage = "Error: RobotName is required."
elif not PlanesList or not isinstance(PlanesList, list):
    SuccessMessage = "Error: PlanesList must be a list of Rhino Plane objects."
elif any(not isinstance(plane, rg.Plane) for plane in PlanesList):
    SuccessMessage = "Error: All elements in PlanesList must be Rhino Plane objects."
elif UpdateRoboDK is None:
    SuccessMessage = "Error: UpdateRoboDK toggle is required."
elif not UpdateRoboDK:
    SuccessMessage = "UpdateRoboDK is set to False. No action taken."
else:
    # Connect to RoboDK only if UpdateRoboDK is True
    RDK = rl.Robolink()  # Initialize RoboDK connection
    SuccessMessage = "Connected to RoboDK."
    
    # Retrieve the robot by name
    robot = RDK.Item(RobotName, rl.ITEM_TYPE_ROBOT)
    
    if not robot.Valid():
        SuccessMessage += f"\nError: Could not find {RobotName} in RoboDK."
    else:
        SuccessMessage += f"\nFound robot '{robot.Name()}' in RoboDK."
        
        # Get the robot's base and tool
        robot_base = robot.Parent()
        tool = robot.getLink(rl.ITEM_TYPE_TOOL)
        
        if not robot_base.Valid() or not tool.Valid():
            SuccessMessage += "\nCannot proceed without valid base and tool."
        else:
            # Set the robot to its home position before starting
            robot.MoveJ(robot.JointsHome())

            # Create a new program in RoboDK
            program_name = "MoveThroughPlanesProgram"
            program = RDK.AddProgram(program_name, robot)
            program.setFrame(robot_base)
            program.setTool(tool)

            # Set default values for speed, acceleration, and blending
            speed_ms = 0.3
            accel_mss = 3.0
            blend_radius_m = 0.001

            # Variables to track current index in planes and SetSpeedOrder
            speed_index = 0
            stop_index = 0

            # Process each plane in PlanesList independently
            for idx, plane in enumerate(PlanesList):
                # Check if we need to insert a stop command here
                if StopOrder and stop_index < len(StopOrder) and idx == StopOrder[stop_index]:
                    # Display a message to the user and wait for confirmation to proceed
                    user_choice = dialogs.ShowMessageOkCancel("Process stopped. Click OK to continue.", "User Confirmation")
                    
                    if not user_choice:  # If user clicks "Cancel"
                        SuccessMessage += f"\nProcess cancelled by the user at Plane {idx + 1}."
                        break  # Stop execution or exit the loop

                    # Insert a pause instruction into the RoboDK program
                    program.RunInstruction("Pause", True)
                    SuccessMessage += f"\nProcess continued at Plane {idx + 1}."
                    stop_index += 1

                # Check if we need to insert a speed change command here
                if SetSpeedOrder and speed_index < len(SetSpeedOrder) and idx == SetSpeedOrder[speed_index]:
                    # Change the speed of the robot in the program
                    speed_value = SetSpeedValue[speed_index]
                    program.setSpeed(speed_value)
                    SuccessMessage += f"\nSpeed set to {speed_value} at Plane {idx + 1}."
                    speed_index += 1

                # Convert the Rhino plane to a RoboDK pose
                target_pose_robot = plane_to_pose(plane)
                base_pose_inv = robot_base.Pose().inv()  # Inverse of the base pose matrix
                target_pose_relative = base_pose_inv * target_pose_robot  # Calculate the relative target pose

                try:
                    # Attempt a linear movement to the target pose
                    robot.MoveL(target_pose_relative, False)
                    TargetPoses.append(target_pose_relative)
                    
                    # Create a target in RoboDK for this pose
                    target_name = f"Plane_{idx + 1}"
                    target = RDK.AddTarget(target_name, robot_base, robot)
                    target.setAsCartesianTarget()
                    target.setPose(target_pose_relative)
                    program.MoveL(target)
                    SuccessMessage += f"\nPlane {idx + 1} reachable with linear movement."

                except Exception as e:
                    try:
                        # If linear movement fails, attempt a joint movement to the target pose
                        robot.MoveJ(target_pose_relative, False)
                        TargetPoses.append(target_pose_relative)
                        
                        # Create a target in RoboDK for this pose
                        target_name = f"Plane_{idx + 1}"
                        target = RDK.AddTarget(target_name, robot_base, robot)
                        target.setAsCartesianTarget()
                        target.setPose(target_pose_relative)
                        program.MoveJ(target)
                        SuccessMessage += f"\nPlane {idx + 1} reachable with joint movement."

                    except Exception as e_joint:
                        # If joint movement also fails, mark the plane as unreachable
                        SuccessMessage += f"\nPlane {idx + 1} unreachable. Error: {str(e_joint)}"
                        PlaneExtra.append(plane)
                        continue

                # Record the TCP orientation and coordinates for reporting purposes
                TCPOrientations.append(format_tcp_orientation(plane, f"Plane {idx + 1}"))
                TargetPointCoordinates.append(format_target_coordinates(plane, f"Plane {idx + 1}"))

            # Finalize the program name for the outputs
            Program = program_name
            SuccessMessage += f"\nProgram '{program_name}' created successfully."




















