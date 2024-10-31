import robolink as rl        # RoboDK API for communication 
import robodk as rdk         # RoboDK robotic simulation environment
import Rhino.Geometry as rg  # Rhino geometry for working with 3D points
import math                  # Math module for calculations

# Initialize RoboDK link
RDK = rl.Robolink()
# Ensure that RobotName is defined correctly from Grasshopper
if 'RobotName' not in globals() or not RobotName:
    raise NameError("RobotName is not defined or empty.")

# Fetch robot item by name
robot = RDK.Item(RobotName, rl.ITEM_TYPE_ROBOT)

# Ensure the robot is found in RoboDK
if not robot.Valid():
    raise NameError(f"Error: Could not find robot '{RobotName}' in RoboDK.")

# Define functions
def connect_to_robot(robot_name) -> str:
    """
    Connect to the robot using its name and return a success or error message.

    :param robot_name: The name of the robot in RoboDK.
    :return: A message indicating success or error.
    """
    robot = RDK.Item(robot_name, rl.ITEM_TYPE_ROBOT)
    if not robot.Valid():
        return f"Error: Could not find robot '{robot_name}' in RoboDK."
    else:
        return f"Connected to robot '{robot_name}' in RoboDK."


def set_robot_base(robot, x, y, z, rz_deg) -> str:
    """
    Set the robot's base at a specific point (x, y, z) with a rotation along the Z-axis.

    :param robot: The robot item in RoboDK.
    :param x: X coordinate for the base position.
    :param y: Y coordinate for the base position.
    :param z: Z coordinate for the base position.
    :param rz_deg: Rotation angle around the Z-axis in degrees.
    :return: A success or error message regarding the base setting.
    """
    rz_rad = math.radians(rz_deg)  # Convert degrees to radians
    rotation_matrix = rdk.rotz(rz_rad)  # Create a rotation matrix
    translation_matrix = rdk.transl(x, y, z)  # Create a translation matrix
    base_pose = translation_matrix * rotation_matrix  # Combine translation and rotation

    robot_base = robot.Parent()  # Get the robot's base (parent in RoboDK)

    if robot_base.Valid():
        robot_base.setPose(base_pose)  # Set the base pose
        return f"Robot '{RobotName}' base set at Point {PointIndex} with RotationZ={rz_deg} degrees successfully."
    else:
        return "Error: Could not find or access the robot base."


def get_current_base_position(robot) -> str:
    """
    Retrieve the current base position and rotation of the robot.

    :param robot: The robot item in RoboDK.
    :return: A string containing the current base position and rotation details.
    """
    robot_base = robot.Parent()  # Get the robot's base

    if robot_base.Valid():
        current_base_pose = robot_base.Pose()  # Get the current pose of the base
        current_position = current_base_pose.Pos()  # Extract position (X, Y, Z)
        rx_rad, ry_rad, rz_rad = rdk.pose_2_xyzrpy(current_base_pose)[3:]  # Get rotation angles
        rz_deg = math.degrees(rz_rad)  # Convert Z-axis rotation to degrees
        return f"Current base position: X={current_position[0]}, Y={current_position[1]}, Z={current_position[2]}, RotationZ={rz_deg} degrees"
    else:
        return "Error: Could not retrieve base position."


def validate_point_index(point_index, points_list) -> bool:
    """
    Validate whether the point index is within the bounds of the points list.

    :param point_index: The index of the point to check.
    :param points_list: The list of points.
    :return: True if the index is valid, False otherwise.
    """
    return 0 <= point_index < len(points_list)


# Function to transform Rhino Z-axis values for RoboDK
def rhino_to_robodk_coordinates(point):
    """
    Transform a Rhino point to RoboDK-compatible coordinates, ensuring correct height (Z) adjustment.
    
    :param point: A Rhino point (with X, Y, Z values).
    :return: Transformed X, Y, Z coordinates.
    """
    # Assuming that Rhino Z needs to map directly to RoboDK Z and no additional transformation is needed.
    # If a transformation is needed (for example, flipping the Z axis), apply it here.
    return point.X, point.Y, point.Z

# Main script logic
SuccessMessage = connect_to_robot(RobotName)  # Connect to the robot
if "Error" not in SuccessMessage:
    # Validate the point index
    if not validate_point_index(PointIndex, TablePoints):
        SuccessMessage = f"Error: Point index {PointIndex} is out of bounds."
    else:
        point = TablePoints[PointIndex]  # Get the point at the specified index
        x, y, z = rhino_to_robodk_coordinates(point)  # Transform Rhino coordinates to RoboDK coordinates

        # Set the base if SetBase is true, otherwise retrieve current base position
        if SetBase:
            rz_deg = RotationZ  # Get the rotation value for the base
            SuccessMessage = set_robot_base(robot, x, y, z, rz_deg)
            BasePosition = f"Base set at: X={x}, Y={y}, Z={z}, RotationZ={rz_deg} degrees"
        else:
            BasePosition = get_current_base_position(robot)
else:
    BasePosition = ""

# Output the messages
print(SuccessMessage)
print(BasePosition)


