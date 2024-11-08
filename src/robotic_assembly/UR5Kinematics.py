# Import necessary modules
import robolink as rl          # RoboDK API for communication with RoboDK software
import robodk as rdk           # RoboDK modeling and simulation library
import Rhino.Geometry as rg    # Rhino.Geometry for handling geometric entities in Rhino
import math                    # Math module for mathematical functions
import numpy as np             # NumPy for numerical computations

# === Inputs ===
# RobotName: Name of the robot in RoboDK (e.g., "UR5")

# Initialize output variables
RobotPosition = None   # Will store the position of the robot's Tool Center Point (TCP)
X_Axis = None          # Will store a line representing the robot's TCP X-axis
Y_Axis = None          # Will store a line representing the robot's TCP Y-axis
Z_Axis = None          # Will store a line representing the robot's TCP Z-axis
JointPoints = []       # List to store the positions of each joint
JointLines = []        # List to store lines connecting the joints
TCPPlane = None        # Plane representing the TCP's orientation
SuccessMessage = ""    # Message indicating success or error

# Initialize a connection to RoboDK
RDK = rl.Robolink()

# Retrieve the robot item from RoboDK by its name
robot = RDK.Item(RobotName, rl.ITEM_TYPE_ROBOT)

# Check if the robot item is valid
if not robot.Valid():
    # If the robot is not found, set an error message
    SuccessMessage = f"Error: Could not find {RobotName} in RoboDK."
else:
    # If the robot is found, set a success message
    SuccessMessage = f"Connected to {RobotName} in RoboDK."

    # Retrieve the robot's base item (parent item)
    robot_base = robot.Parent()
    if robot_base.Valid():
        # If the base is valid, get its pose (transformation matrix)
        base_pose = robot_base.Pose()
    else:
        # If not, use an identity matrix as the base pose
        base_pose = rdk.Mat()

    # Get the robot's current joint angles in degrees
    joints_deg = robot.Joints().list()
    # Convert joint angles from degrees to radians for computation
    joints_rad = [math.radians(angle) for angle in joints_deg]

    # Define the UR5 robot's standard Denavit-Hartenberg (DH) parameters in millimeters
    d1 = 89.159     # Offset along previous Z to the common normal
    a2 = -425.00    # Length of the common normal (negative due to robot configuration)
    a3 = -392.25    # Length of the common normal (negative due to robot configuration)
    d4 = 109.15     # Offset along previous Z to the common normal
    d5 = 94.65      # Offset along previous Z to the common normal
    d6 = 82.3       # Offset along previous Z to the end effector

    # DH parameters: alpha (twist angles), a (link lengths), d (link offsets), theta (joint angles)
    alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]   # Twist angles between links
    a = [0, a2, a3, 0, 0, 0]                              # Link lengths
    d = [d1, 0, 0, d4, d5, d6]                            # Link offsets
    theta = joints_rad                                     # Joint angles from the robot

    # Convert the base pose to a NumPy array for matrix operations
    T_base = np.array(base_pose.rows)
    # Ensure that the base transformation matrix is 4x4
    if T_base.shape != (4, 4):
        # Append an error message if the matrix is not 4x4
        SuccessMessage += " Error: Base pose matrix is not 4x4."
    else:
        # Initialize the overall transformation matrix with the base pose
        T = T_base

        # Initialize a list to store the positions of each joint
        JointPoints = []

        # Initialize a list to store the transformation matrix of each joint
        T_matrices = []

        # Compute the transformation for each of the six joints using the DH parameters
        for i in range(6):
            # Extract DH parameters for the current joint
            ai = a[i]
            alphai = alpha[i]
            di = d[i]
            thetai = theta[i]

            # Compute trigonometric functions for efficiency
            cos_theta = math.cos(thetai)
            sin_theta = math.sin(thetai)
            cos_alpha = math.cos(alphai)
            sin_alpha = math.sin(alphai)

            # Create the transformation matrix from the previous joint to the current joint
            T_joint = np.array([
                [cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, ai * cos_theta],
                [sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, ai * sin_theta],
                [0,          sin_alpha,              cos_alpha,             di],
                [0,          0,                      0,                     1]
            ])

            # Update the overall transformation matrix by multiplying with the current joint's matrix
            T = np.dot(T, T_joint)

            # Save the transformation matrix for this joint
            T_matrices.append(T.copy())

            # Extract the position of the current joint from the transformation matrix
            position = T[:3, 3]  # Get the x, y, z coordinates

            # Create a Point3d object for the joint position
            joint_point = rg.Point3d(position[0], position[1], position[2])
            # Add the joint position to the list
            JointPoints.append(joint_point)

        # Create lines connecting the joints for visualization
        JointLines = []
        for i in range(1, len(JointPoints)):
            # Create a line from the previous joint to the current joint
            line = rg.Line(JointPoints[i - 1], JointPoints[i])
            # Add the line to the list
            JointLines.append(line)

        # Optionally, connect the robot base to the first joint
        if JointPoints:
            # Get the base position from the base transformation matrix
            base_position = T_base[:3, 3]
            base_point = rg.Point3d(base_position[0], base_position[1], base_position[2])
            # Create a line from the base to the first joint
            line = rg.Line(base_point, JointPoints[0])
            # Insert the line at the beginning of the JointLines list
            JointLines.insert(0, line)

        # Retrieve the robot's pose relative to its base (end-effector pose)
        robot_pose = robot.Pose()

        # Compute the absolute pose of the TCP (Tool Center Point) in the world coordinate system
        absolute_pose = base_pose * robot_pose

        # Extract the TCP position from the absolute pose
        position = absolute_pose.Pos()  # Get the x, y, z coordinates

        # Create a Point3d object for the TCP position
        RobotPosition = rg.Point3d(position[0], position[1], position[2])

        # Extract orientation vectors (axes) from the rotation matrix of the absolute pose
        x_vector = rg.Vector3d(absolute_pose[0, 0], absolute_pose[1, 0], absolute_pose[2, 0])  # X-axis
        y_vector = rg.Vector3d(absolute_pose[0, 1], absolute_pose[1, 1], absolute_pose[2, 1])  # Y-axis
        z_vector = rg.Vector3d(absolute_pose[0, 2], absolute_pose[1, 2], absolute_pose[2, 2])  # Z-axis

        # Create a plane at the TCP position with the extracted orientation vectors
        TCPPlane = rg.Plane(RobotPosition, x_vector, y_vector)

        # Scale the orientation vectors for visualization purposes
        scale_factor = 200  # Adjust this value as needed for your visualization
        x_vector *= scale_factor
        y_vector *= scale_factor
        z_vector *= scale_factor

        # Create lines representing the robot's TCP axes
        X_Axis = rg.Line(RobotPosition, rg.Point3d(RobotPosition + x_vector))
        Y_Axis = rg.Line(RobotPosition, rg.Point3d(RobotPosition + y_vector))
        Z_Axis = rg.Line(RobotPosition, rg.Point3d(RobotPosition + z_vector))

        # Optionally, connect the last joint to the TCP position
        if JointPoints and RobotPosition:
            # Create a line from the last joint to the TCP
            line = rg.Line(JointPoints[-1], RobotPosition)
            # Add the line to the list of joint lines
            JointLines.append(line)
