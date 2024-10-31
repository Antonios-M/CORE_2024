import robolink as rl  # RoboDK API 
import robodk as rdk
import Rhino.Geometry as rg
import math
import numpy as np

# === Inputs ===
# RobotName: Name of the robot in RoboDK (e.g., "UR5")

# Initialize outputs
RobotPosition = None
X_Axis = None
Y_Axis = None
Z_Axis = None
JointPoints = []
JointLines = []
TCPPlane = None
SuccessMessage = ""

# Initialize RoboDK connection
RDK = rl.Robolink()

# Get the robot by name
robot = RDK.Item(RobotName, rl.ITEM_TYPE_ROBOT)

if not robot.Valid():
    SuccessMessage = f"Error: Could not find {RobotName} in RoboDK."
else:
    SuccessMessage = f"Connected to {RobotName} in RoboDK."

    # Get the robot's base pose
    robot_base = robot.Parent()
    if robot_base.Valid():
        base_pose = robot_base.Pose()
    else:
        base_pose = rdk.Mat()

    # Get the robot's current joint angles (degrees)
    joints_deg = robot.Joints().list()
    # Convert joint angles to radians
    joints_rad = [math.radians(angle) for angle in joints_deg]

    # UR5 standard DH parameters (in millimeters)
    d1 = 89.159
    a2 = -425.00
    a3 = -392.25
    d4 = 109.15
    d5 = 94.65
    d6 = 82.3

    alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]
    a = [0, a2, a3, 0, 0, 0]
    d = [d1, 0, 0, d4, d5, d6]
    theta = [
        joints_rad[0],
        joints_rad[1],
        joints_rad[2],
        joints_rad[3],
        joints_rad[4],
        joints_rad[5],
    ]

    # Initialize the base transformation matrix
    T_base = np.array(base_pose.rows)
    # Ensure T_base is a 4x4 matrix
    if T_base.shape != (4, 4):
        SuccessMessage += " Error: Base pose matrix is not 4x4."
    else:
        # Initialize the transformation matrix
        T = T_base

        # Initialize joint positions list
        JointPoints = []

        # Store transformation matrices for each joint
        T_matrices = []

        for i in range(6):  # Six joints
            # DH parameters for current joint
            ai = a[i]
            alphai = alpha[i]
            di = d[i]
            thetai = theta[i]

            # Compute transformation matrix using DH parameters
            cos_theta = math.cos(thetai)
            sin_theta = math.sin(thetai)
            cos_alpha = math.cos(alphai)
            sin_alpha = math.sin(alphai)

            T_joint = np.array([
                [cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, ai * cos_theta],
                [sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, ai * sin_theta],
                [0,          sin_alpha,              cos_alpha,             di],
                [0,          0,                      0,                     1]
            ])

            # Update the overall transformation matrix
            T = np.dot(T, T_joint)

            # Save the transformation matrix for this joint
            T_matrices.append(T.copy())

            # Extract the position of the current joint
            position = T[:3, 3]

            # No conversion needed since we're working in millimeters
            joint_point = rg.Point3d(position[0], position[1], position[2])
            JointPoints.append(joint_point)

        # Create lines connecting the joints
        JointLines = []
        for i in range(1, len(JointPoints)):
            line = rg.Line(JointPoints[i - 1], JointPoints[i])
            JointLines.append(line)

        # Optionally, connect the base to the first joint
        if JointPoints:
            base_position = T_base[:3, 3]
            base_point = rg.Point3d(base_position[0], base_position[1], base_position[2])
            line = rg.Line(base_point, JointPoints[0])
            JointLines.insert(0, line)

        # Fetch the robot's pose relative to its base
        robot_pose = robot.Pose()

        # Compute the absolute pose of the TCP in the world coordinate system
        absolute_pose = base_pose * robot_pose

        # Extract position from the absolute pose
        position = absolute_pose.Pos()  # [x, y, z]

        # Create Rhino point for the robot's TCP position
        RobotPosition = rg.Point3d(position[0], position[1], position[2])

        # Extract orientation vectors from the rotation matrix of the absolute pose
        x_vector = rg.Vector3d(absolute_pose[0,0], absolute_pose[1,0], absolute_pose[2,0])
        y_vector = rg.Vector3d(absolute_pose[0,1], absolute_pose[1,1], absolute_pose[2,1])
        z_vector = rg.Vector3d(absolute_pose[0,2], absolute_pose[1,2], absolute_pose[2,2])

        # Create a plane for the TCP based on position and orientation vectors
        TCPPlane = rg.Plane(RobotPosition, x_vector, y_vector)

        # Scale vectors for visualization purposes
        scale_factor = 200  # Adjust as needed
        x_vector *= scale_factor
        y_vector *= scale_factor
        z_vector *= scale_factor

        # Create lines representing the robot's TCP axes
        X_Axis = rg.Line(RobotPosition, rg.Point3d(RobotPosition + x_vector))
        Y_Axis = rg.Line(RobotPosition, rg.Point3d(RobotPosition + y_vector))
        Z_Axis = rg.Line(RobotPosition, rg.Point3d(RobotPosition + z_vector))

        # Optionally, connect the last joint to the TCP
        if JointPoints and RobotPosition:
            line = rg.Line(JointPoints[-1], RobotPosition)
            JointLines.append(line)


























