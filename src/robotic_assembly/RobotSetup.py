# Import necessary modules for RoboDK communication and robot programming
import robolink as rl        # RoboLink module to interact with RoboDK software
import robodk as rdk         # RoboDK module for robot simulations and manipulations
from math import radians     # Function to convert degrees to radians

# Set this to True to enable debug outputs for troubleshooting
debug = True

# Check if the script is running inside Rhino/Grasshopper environment
try:
    # Try to import Rhino.Geometry module
    import Rhino.Geometry as rg
    inside_rhino = True
    if debug:
        print("Running inside Rhino/Grasshopper environment.")
except ImportError:
    # If import fails, assume script is running outside Rhino and use rhino3dm
    import rhino3dm as rg
    inside_rhino = False
    if debug:
        print("Running outside Rhino; using rhino3dm.")

# Initialize the RoboDK API (RoboLink) to communicate with RoboDK software
RDK = rl.Robolink()

# Define a custom exception class for robot-related errors
class RobotError(Exception):
    pass

def connect_to_robot(robot_name):
    """
    Connects to a robot in the RoboDK station by its name.

    Parameters:
    robot_name (str): The name of the robot to connect to.

    Returns:
    robot (Robolink.Item): The robot item in RoboDK.

    Raises:
    RobotError: If the robot is not found in the RoboDK station.
    """
    if debug:
        print(f"Attempting to connect to robot '{robot_name}'.")
    # Search for the robot in the RoboDK station
    robot = RDK.Item(robot_name, rl.ITEM_TYPE_ROBOT)
    if not robot.Valid():
        # If robot is not found, raise an error
        raise RobotError(f"Could not find robot '{robot_name}' in RoboDK.")
    if debug:
        print(f"Successfully connected to robot '{robot_name}'.")
    return robot

def set_robot_base(robot, rotation_angles, robot_name):
    """
    Sets the robot's base frame orientation to the specified rotation angles while keeping the current position.

    Parameters:
    robot (Robolink.Item): The robot item in RoboDK.
    rotation_angles (tuple): A tuple containing rotation angles (rx_deg, ry_deg, rz_deg) in degrees.
    robot_name (str): The name of the robot.

    Returns:
    str: A success message indicating the new base orientation.

    Raises:
    RobotError: If the robot base cannot be found or accessed.
    """
    if debug:
        print(f"Setting base orientation for robot '{robot_name}'.")
        print(f"Rotation angles (degrees): RX={rotation_angles[0]}, RY={rotation_angles[1]}, RZ={rotation_angles[2]}")

    # Get the robot's parent item, which should be its base frame
    robot_base = robot.Parent()
    if robot_base.Valid():
        # Retrieve the current pose (position and orientation) of the robot base
        current_base_pose = robot_base.Pose()
        # Extract the position component of the pose
        current_position = current_base_pose.Pos()

        # Unpack rotation angles
        rx_deg, ry_deg, rz_deg = rotation_angles
        # Create a rotation matrix from the specified rotation angles
        rotation_matrix = (
            rdk.rotx(radians(rx_deg)) *
            rdk.roty(radians(ry_deg)) *
            rdk.rotz(radians(rz_deg))
        )

        # Combine the current position with the new rotation to create the new base pose
        new_base_pose = rdk.transl(*current_position) * rotation_matrix

        # Update the robot's base frame with the new pose
        robot_base.setPose(new_base_pose)

        if debug:
            print("Robot base orientation has been set successfully.")
    else:
        # If the robot base is not valid, raise an error
        raise RobotError("Could not find or access the robot base.")

    return f"Robot '{robot_name}' base orientation set to RX={rx_deg}, RY={ry_deg}, RZ={rz_deg} degrees."

def get_current_base_position(robot):
    """
    Retrieves the current base position and the first joint rotation of the robot.

    Parameters:
    robot (Robolink.Item): The robot item in RoboDK.

    Returns:
    str: A message containing the current base position and the first joint rotation.

    Raises:
    RobotError: If the robot base cannot be retrieved.
    """
    if debug:
        print("Retrieving current base position of the robot.")
    # Get the robot's parent item, which should be its base frame
    robot_base = robot.Parent()
    if robot_base.Valid():
        # Retrieve the current pose (position and orientation) of the robot base
        current_base_pose = robot_base.Pose()
        # Extract the position component of the pose
        current_position = current_base_pose.Pos()
        # Get the robot's current joint angles
        joints = robot.Joints().list()
        # Get the rotation angle of the first joint (in degrees)
        first_joint_deg = joints[0]
        if debug:
            print(f"Current base position: X={current_position[0]}, Y={current_position[1]}, Z={current_position[2]}, First joint rotation: {first_joint_deg} degrees.")
        return f"Current base position: X={current_position[0]}, Y={current_position[1]}, Z={current_position[2]}, First joint rotation: {first_joint_deg} degrees"
    else:
        # If the robot base is not valid, raise an error
        raise RobotError("Could not retrieve base position.")

def validate_point_index(point_index, points_list):
    """
    Validates that the given point index is within the bounds of the points list.

    Parameters:
    point_index (int): The index of the point to validate.
    points_list (list): The list of points.

    Returns:
    bool: True if the index is valid, False otherwise.
    """
    if debug:
        print(f"Validating point index {point_index} (Total points: {len(points_list)}).")
    # Check if point_index is within the range of points_list
    return 0 <= point_index < len(points_list)

def rhino_to_robodk_coordinates(point):
    """
    Converts a point from Rhino coordinate system to RoboDK coordinate system.

    Parameters:
    point (rg.Point3d): The point to convert.

    Returns:
    tuple: A tuple containing the X, Y, Z coordinates.
    """
    if debug:
        print(f"Converting point ({point.X}, {point.Y}, {point.Z}) to RoboDK coordinates.")
    # Return the point's coordinates as a tuple
    return point.X, point.Y, point.Z

# Main script logic starts here
try:
    # If the script is running outside Rhino, set default input parameters
    if not inside_rhino:
        # Define the robot's name as it appears in RoboDK
        RobotName = 'YourRobotName'
        # Define a list of table points (positions) using Rhino geometry points
        TablePoints = [
            rg.Point3d(100, 200, 300),
            rg.Point3d(400, 500, 600),
        ]
        # Select the index of the point to use from TablePoints
        PointIndex = 0
        # Boolean flag to determine if the robot's base should be set
        SetBase = True
        # Define the rotation angles for setting the robot base orientation
        RotationAngles = (0, 0, 180)  # Rotation angles as a tuple (RX, RY, RZ) in degrees

    # Debug output of input parameters
    if debug:
        print("All input parameters have been validated.")
        print(f"RobotName: {RobotName}")
        print(f"PointIndex: {PointIndex}")
        print(f"SetBase: {SetBase}")
        if SetBase:
            print(f"RotationAngles: RX={RotationAngles[0]}, RY={RotationAngles[1]}, RZ={RotationAngles[2]}")

    # Connect to the robot specified by RobotName
    robot = connect_to_robot(RobotName)

    # Validate that the selected point index is within the range of TablePoints
    if not validate_point_index(PointIndex, TablePoints):
        raise IndexError(f"Point index {PointIndex} is out of bounds.")

    # Retrieve the selected point from TablePoints
    point = TablePoints[PointIndex]
    # Convert the Rhino point to RoboDK coordinates
    x, y, z = rhino_to_robodk_coordinates(point)

    # Debug output of the point coordinates
    if debug:
        print(f"Using point coordinates: X={x}, Y={y}, Z={z}")

    if SetBase:
        # Set the robot's base frame orientation to the specified rotation angles
        SuccessMessage = set_robot_base(robot, RotationAngles, RobotName)
        # Prepare a message with the new base orientation
        BasePosition = f"Base orientation set to: RX={RotationAngles[0]}, RY={RotationAngles[1]}, RZ={RotationAngles[2]} degrees"

        if debug:
            # Debug output of success messages
            print(SuccessMessage)
            print(BasePosition)
    else:
        # Retrieve the current base position and first joint rotation
        BasePosition = get_current_base_position(robot)
        SuccessMessage = f"Connected to robot '{RobotName}' and retrieved current base position and first joint rotation."

        if debug:
            # Debug output of success messages
            print(SuccessMessage)
            print(BasePosition)

except (RobotError, IndexError, NameError) as e:
    # Catch any custom RobotError, IndexError, or NameError exceptions
    SuccessMessage = f"Error: {e}"
    BasePosition = ""

    if debug:
        # Debug output of the error message
        print(SuccessMessage)

# If the script is running inside Rhino, assign outputs to variables
if inside_rhino:
    # Assign output variables for Rhino/Grasshopper components
    A = SuccessMessage
    B = BasePosition
else:
    # If running outside Rhino, print the outputs to the console
    print(SuccessMessage)
    print(BasePosition)
