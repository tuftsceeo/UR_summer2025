# UR_library_2025.py
# Modification and addition to MyUR3e.py (https://github.com/tuftsceeo/Universal-Robots-ROS2-CEEO/tree/main) 
# Written by Leia Hannes and Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025

# External Libraries:
import math
import json
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

# Internal Libraries:
from myur.ik_solver.ur_kinematics import URKinematics
from myur.trajectory_planner import TrajectoryPlanner

# ROS2:
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

# ROS2 Msg Types:
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int32MultiArray


class MyUR3e(rclpy.node.Node):
    """
    A class to control the UR3e robot arm using ROS2.

    Public Attributes:
        ik_solver (URKinematics): Inverse kinematics solver for the UR3e robot.
        joint_states (JointStates): Instance for subscribing to joint states.
        tool_wrench (ToolWrench): Instance for subscribing to tool wrench data.
        gripper (Gripper): Instance for subscribing to and controlling the gripper.
    """

    def __init__(
        self, response_callback=None, result_callback=None, timer_callback=None
    ):
        """
        Initialize the MyUR3e node.
        """
        rclpy.init()
        super().__init__("remote_control_client")
        self.get_logger().debug("Initializing MyUR3e...")

        # Action Client Setup
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        controller_name = (
            self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        )
        self.joints = self.get_parameter("joints").value
        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self.get_logger().debug(f"Waiting for action server on {controller_name}")
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        if not self._action_client.wait_for_server(timeout_sec=10):
            self.__del__()
            raise RuntimeError(
                "Action server not available after waiting for 10 seconds. Check ROS UR Driver."
            )

        # Private Attributes
        self._timer = self.create_timer(1.0, self.get_timer_callback)
        self._send_goal_future = None
        self._get_result_future = None
        self._id = 0
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self.trajectory_file = "trajectory_file.json"
        self._trajectories = {}
        try:
            with open(self.trajectory_file, "r") as file:
                self._trajectories = json.load(file)
        except (IOError, json.JSONDecodeError):  # except file not found
            with open(
                self.trajectory_file, "w"
            ) as file:  # BUG could this ever wipe the file?
                json.dump(self._trajectories, file, indent=4)

        # Public Attributes
        self.vis = TrajectoryPlanner()
        self.ik_solver = URKinematics("ur3e")
        self.joint_states = JointStates()
        self.tool_wrench = ToolWrench()
        self.gripper = Gripper()
        self.done = True

        # Set Functions
        self.response_callback = response_callback
        self.result_callback = result_callback
        self.timer_callback = timer_callback

    def __del__(self):
        # De-init rclpy
        rclpy.shutdown()

    ###########################################################################
    ############################# PUBLIC METHODS ##############################
    ###########################################################################

    ########################### CLASS ACCESS METHODS ##########################

    def get_trajectory(self, name):
        """
        Loads all trajectories from the JSON file and retrieves a specific one by name.

        Args:
            name (string): name corresponding to trajectory
        Returns:
            list: coordinates
        """
        if os.path.exists(self.trajectory_file):
            try:
                with open(self.trajectory_file, "r") as file:
                    self._trajectories = json.load(file)
            except (IOError, json.JSONDecodeError) as e:
                raise IOError(f"An error occurred while loading the file: {e}")

        trajectory = self._trajectories.get(name)

        if trajectory is None:
            raise ValueError(f"No trajectory found with the name: {name}")

        return trajectory["trajectory"]

    ############################# SERVICE METHODS #############################

    def read_gripper(self):
        """
        Get the current state of the gripper.

        Returns:
            list: [POS (int), SPE (int), FOR (int)]
        """
        [position, speed, force] = self.gripper.get()
        return [
            int(100 * position / 255),
            int(100 * speed / 255),
            int(100 * force / 255),
        ]

    def read_joints_pos(self, degrees=True):
        """
        Get the angle of each joint in radians.

        Args:
            degrees (bool): True for degrees, False for radians
        Returns:
            list: [Pan, Lift, Elbow, Wrist 1, Wrist 2, Wrist 3]
        """
        if degrees:
            return self.convert_angles(
                self.joint_states.get_joints()["position"], to_degrees=True
            )
        return self.joint_states.get_joints()["position"]

    def read_joints_eff(self):
        """
        Get the effort of each joint.

        Returns:
            list: [Pan, Lift, Elbow, Wrist 1, Wrist 2, Wrist 3]
        """
        return self.joint_states.get_joints()["effort"]

    def read_global_pos(self,degrees=True):
        """
        Get the global position of end effector in meters.

        Returns:
            list: [x,y,z,rx,ry,rz]
        """
        return self.solve_fk(
            self.read_joints_pos(degrees=degrees), degrees=degrees, euler=True
        )

    def read_force(self):
        """
        Get the force exerted on the end effector in Newtons (relative to the end effector).

        Returns:
            list: [x,y,z]
        """
        return self.tool_wrench.get()["force"]

    def read_torque(self):
        """
        Get the torque exerted on the end effector in Newton Meters (relative to the end effector).

        Returns:
            list: [x,y,z]
        """
        return self.tool_wrench.get()["torque"]

    ############################# MOVEMENT METHODS ############################

    def solve_ik(self, cords, degrees=True, q_guess=None, out_degrees=True):
        """
        Solve inverse kinematics for given coordinates.

        Args:
            cords (list): A list of coordinates, either [x, y, z, rx, ry, rz] or [x, y, z, qw, qx, qy, qz].
            degrees (bool): True if degrees, False if radians. Only applies to rx, ry, rz.
            q_guess (list, optional): A list of joint angles used to find the closest IK solution in radians.
        Returns:
            list: Joint positions that achieve the given coordinates. [see self.joints]
        """
        if q_guess is None:  # Use current robot pose as q_guess
            q_guess = self.read_joints_pos(degrees=False)

        # If coordinates in euler format convert to quaternions
        if len(cords) == 6:
            for i, angle in enumerate(
                cords[3:7]
            ):  # Deviate zeros to prevent unsolved ik
                if angle == 0:
                    cords[i + 3] = 0.1
            r = R.from_euler("zyx", cords[3:6], degrees=degrees)
            quat = r.as_quat(scalar_first=True).tolist()
            cords = cords[0:3] + quat

        if out_degrees:
            return self.convert_angles(
                self.ik_solver.inverse(cords, False, q_guess=q_guess), to_degrees=True
            )
        return self.ik_solver.inverse(
            cords, False, q_guess=q_guess
        )  # output is radians

    def solve_fk(self, angles, degrees=True, euler=True):
        """
        Solve forward kinematics for given joint positions.

        Args:
            angles (list): A list of joint angles.
            degrees (bool): True if degrees, False if radians.
            euler (bool): True if euler rotation desired, False for quaternion.
        Returns:
            list: End effector coordinates resulting from joint angles.
        """
        if degrees:
            angles = self.convert_angles(angles, to_degrees=False)

        cords_q = self.ik_solver.forward(angles)
        if euler:
            r = R.from_quat(cords_q[3:7], scalar_first=True)
            euler = r.as_euler("zyx", degrees=degrees).tolist()
            cords = cords_q[0:3].tolist() + euler
        else:
            cords = cords_q

        return cords


    def move_gripper(self, position, speed=50, force=50, wait=True):
        """
        Move the gripper to the specified position with given speed and force.

        Args:
            position (int): Position for the gripper.
            speed (int): Speed for the gripper.
            force (int): Force for the gripper.
        """
        self.gripper.control(
            int(255 * position / 100),
            int(255 * speed / 100),
            int(255 * force / 100),
            wait,
        )

    def move_global(
        self, coordinates, time=5, degrees=True, vis_only=False, wait=True, interp=None
    ):
        """
        Move the robot to specified global coordinates.

        Args:
            coordinates (list): List of coordinates to move to.
                either [x, y, z, rx, ry, rz] or [x, y, z, qx, qy, qz, qw].
            time (float/tuple, optional): If float, time step between each coordinate. If
                tuple, first float represents time to first pos, second float is all following steps.
            degrees (bool): True for degrees, False for radians. Ignore if using quaternions.
            vis_only (bool, optional): True if no motion is desired, False if motion is desired.
            wait (bool, optional): True if blocking is desired, False if non blocking is desired.
            interp (string, optional): Options are None, linear, arc, spline.
        """
        if type(coordinates) == str:  # Retrieve trajectory from json by name
            coordinates = self.get_trajectory(coordinates)
        elif type(coordinates[0]) != list:  # Format single point
            coordinates = [coordinates]

        joint_positions = []
        for i, cord in enumerate(coordinates):
            if i == 0:
                joint_positions.append(
                    self.solve_ik(cord, degrees=degrees, out_degrees=False)
                )
            else:
                joint_positions.append(
                    self.solve_ik(
                        cord,
                        degrees=degrees,
                        out_degrees=False,
                        q_guess=joint_positions[i - 1],
                    )
                )

        self.vis.add_trajectory(coordinates, joint_positions)

        if None in joint_positions:
            raise RuntimeError(
                f"IK solution not found for {joint_positions.count(None)}/{len(joint_positions)} points"
            )
        elif vis_only == False:
            self.move_joints(
                joint_positions,
                time=time,
                degrees=False,
                vis_only=vis_only,
                wait=wait,
                interp=None,
            )

    def move_joints_r(
        self, joint_deltas, time=5, degrees=True, vis_only=False, wait=True, interp=None
    ):
        """
        Move the robot relative to where it was using joint angles.

        Args:
            joint_deltas (list): List of relative movements. [j1,j2,j3,j4,j5,j6].
            time (float/tuple, optional): If float, time step between each coordinate. If
                tuple, first float represents time to first pos, second float is all following steps.
            vis_only (bool, optional): True if no motion is desired, False if motion is desired.
            wait (bool, optional): True if blocking is desired, False if non blocking is desired.
        """
        if type(time) == tuple:
            raise ValueError(
                "Time cannot be a tuple: relative movements do not need time to arrive at first point."
            )

        if type(joint_deltas) == str:  # Retrieve trajectory from json by name
            joint_deltas = self.get_trajectory(joint_deltas)
        elif type(joint_deltas[0]) != list:  # Format single point
            joint_deltas = [joint_deltas]

        sequence = []
        for i, delta in enumerate(joint_deltas):
            if i == 0:
                curr = self.read_joints_pos(degrees=degrees)
                sequence.append([sum(x) for x in zip(curr, delta)])
            else:
                sequence.append([sum(x) for x in zip(sequence[i - 1], delta)])
        self.move_joints(
            sequence,
            time=(time / len(joint_deltas), time - time / len(joint_deltas)),
            degrees=degrees,
            vis_only=vis_only,
            wait=wait,
            interp=interp,
        )

    def move_joints(self, joint_positions, time=5, degrees=True, vis_only=False, wait=True, interp=None):
        """
        Move the robot joints to the specified angular positions.

        Args:
            joint_positions (list): List of joint positions. [j1,j2,j3,j4,j5,j6].
            time (float/tuple, optional): If float, time step between each coordinate. If
                tuple, first float represents time to first pos, second float is all following steps.
            degrees (bool): True for degrees, False for radians.
            vis_only (bool, optional): True if no motion is desired, False if motion is desired.
            wait (bool, optional): True if blocking is desired, False if non blocking is desired.
            interp (string, optional): Options are None, linear, arc, spline.
        """
        
        if type(joint_positions) == str:  # Retrieve trajectory from json by name
            joint_positions = self.get_trajectory(joint_positions)
        elif type(joint_positions[0]) != list:  # Format single point
            joint_positions = [joint_positions]

        if interp != None:  # interpolate angles
            joint_positions = self.interpolate(joint_positions, interp)

        if not vis_only:
            trajectory = self.make_trajectory(
                joint_positions, degrees=degrees, time=time
            )
            self.execute_trajectory(trajectory)
            if wait:
                self.wait(self)
            else:
                spinthread = threading.Thread(target=lambda: self.spin_async(self._id))
                spinthread.start()

    def stop(self):
        """
        Stop the robot at its current pose. For use in non blocking scenarios.
        """
        stop_trajectory = self.make_trajectory(None, stop=True)
        self.execute_trajectory(stop_trajectory, stop=True)
        self.wait(self)
        self.get_logger().info(f"Goal #{self._id}: Stopped")

    ###########################################################################
    ############################# PRIVATE METHODS #############################
    ###########################################################################

    ############################ ROS CLIENT METHODS ###########################

    def spin_async(self, curr_id):
        """
        Spin the ROS Node asynchronously using threads.

        Args:
            curr_id (int): ID number representing unique goals.
        """

        def check_id():
            if curr_id is not self._id:
                self.get_logger().info(
                    f"Goal #{curr_id}: Replaced with Goal #{self._id}"
                )
                return True
            return False

        self._executor.spin_once()
        if check_id():
            return
        while self._send_goal_future is None:  # screen None
            self._executor.spin_once()
            if check_id():
                return
        while not self._send_goal_future.done():  # check done
            self._executor.spin_once()
            if check_id():
                return
        if self._send_goal_future.result().accepted:
            while self._get_result_future is None:  # screen None
                self._executor.spin_once()
                if check_id():
                    return
            while not self._get_result_future.done():  # check done
                self._executor.spin_once()
                if check_id():
                    return
            self._executor.spin_once()  # must spin once more for final result callback
            error_code = self.error_code_to_str(
                self._get_result_future.result().result.error_code
            )
        else:
            self.get_logger().info(
                f"Goal #{self._id}: rejected :( (Check driver logs for more info)"
            )

        self.get_logger().debug(f"Cleanly exiting async thread: {error_code}")

        self._send_goal_future = None
        self._get_result_future = None
        self.done = True

    def make_trajectory(self, joint_positions, degrees=True, time=5, stop=False):
        """
        Create a trajectory for the robot to follow.

        Args:
            joint_positions (list): List of joint positions.
            degrees (bool): True for degrees, False for radians.
            time (int, optional): Time step between each position.
            stop (bool, optional): if True creates a stop trajectory.

        Returns:
            JointTrajectory: The created trajectory.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints

        if not stop:
            self._id += 1

            last_arrival = 0
            for i, position in enumerate(joint_positions):
                point = JointTrajectoryPoint()

                if not degrees:
                    point.positions = position
                else:
                    point.positions = self.convert_angles(position, to_degrees=False)

                if (type(time) != tuple):  # robot spends 5 seconds getting to starting pose and time seconds executing trajectory
                    if i == 0:
                        arrival = 5
                    else:
                        arrival = 5 + i * (time / (len(joint_positions) - 1))
                elif (type(time) == tuple and time[0] == "cv"):  # move end effector at constant velocity
                    if i == 0: # time to start of trajectory
                        dist = np.linalg.norm(
                            np.array(self.solve_fk(joint_positions[i], degrees=False)[0:3])
                            - np.array(self.solve_fk(self.read_joints_pos(degrees=False), degrees=False)[0:3])
                        )
                        arrival = last_arrival + dist / time[1]
                    else: # time during trajectory
                        dist = np.linalg.norm(
                            np.array(self.solve_fk(joint_positions[i], degrees=False)[0:3])
                            - np.array(self.solve_fk(joint_positions[i - 1], degrees=False)[0:3])
                        )
                        arrival = last_arrival + dist / time[2]
                elif (
                    type(time) == tuple and time[0] == 0
                ):  # robot already in starting pose
                    arrival = (i + 1) * (time[1] / (len(joint_positions) - 1))
                elif type(time) == tuple:  # robot needs time to get to starting pose
                    if i == 0:
                        arrival = time[0]
                    else:
                        arrival = time[0] + i * (time[1] / (len(joint_positions) - 1))

                if last_arrival == arrival: arrival += 0.01
                last_arrival = arrival
                sec = int(arrival - (arrival % 1))
                nanosec = int(arrival % 1 * 1000000000)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                trajectory.points.append(point)
        else:
            positions = self.read_joints_pos()
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=0, nanosec=100000000)
            trajectory.points.append(point)
        return trajectory

    def execute_trajectory(self, trajectory, stop=False):
        """
        Execute the given trajectory.

        Args:
            trajectory (JointTrajectory): The trajectory to execute.
            stop (bool, optional): if True creates a stop trajectory.
        """
        if not stop:
            self.get_logger().info(f"Goal #{self._id}: Executing")
        self.done = False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal)
        this_future = self._send_goal_future
        self.get_logger().debug(f"Sent trajectory #{self._id}")
        this_future.add_done_callback(
            lambda this_future: self.goal_response_callback(this_future, self._id)
        )

    def wait(self, client):
        """
        Wait for the action to complete. Blocking.

        Args:
            client (ActionClient): The action client.
        """
        self._executor.spin_once()
        while not client.done:
            self._executor.spin_once()

    ################################ CALLBACKS ################################

    def get_timer_callback(self, *args, **kwargs):
        """
        Callback initiated at a set interval.
        """
        if self.timer_callback:
            self.timer_callback(*args, **kwargs)
        else:
            pass

    def goal_response_callback(self, future, curr_id, *args, **kwargs):
        """
        Callback for when a goal response is received.

        Args:
            future (Future): The future object containing the goal response.
        """
        goal_handle = future.result()

        if curr_id is self._id:

            if not goal_handle.accepted:
                self.get_logger().info(
                    f"Goal #{self._id} rejected :( (Check driver logs for more info)"
                )
                return

            # user defined callback
            if self.response_callback:
                self.response_callback(*args, **kwargs)

            self.get_logger().debug(f"Goal #{self._id} accepted :)")
            self._get_result_future = goal_handle.get_result_async()

            this_future = self._get_result_future
            this_future.add_done_callback(
                lambda this_future: self.get_result_callback(this_future, curr_id)
            )

    def get_result_callback(self, future, curr_id, *args, **kwargs):
        """
        Callback for when a result is received.

        Args:
            future (Future): The future object containing the result.
        """
        result = future.result().result

        if curr_id is self._id:
            self.get_logger().debug(
                f"Done with result from #{self._id}: {self.error_code_to_str(result.error_code)}"
            )
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                # user defined callback
                if self.result_callback:
                    self.result_callback(*args, **kwargs)
                self.done = True
                self.get_logger().info(f"Goal #{self._id}: Completed")

    ########################################################
    #################### STATIC METHODS ####################
    ########################################################

    @staticmethod
    def convert_angles(point, to_degrees):
        """
        Convert a point from degrees to radians.

        Args:
            point (list): List of points in degrees.

        Returns:
            list: List of points in radians.
        """
        if to_degrees:
            return [math.degrees(p) for p in point]
        return [math.radians(p) for p in point]

    @staticmethod
    def error_code_to_str(error_code):
        """
        Convert an error code to a string.

        Args:
            error_code (int): The error code.

        Returns:
            str: The error code as a string.
        """
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"
        elif type(error_code) == list:
            if error_code[0] == 1:
                safety_mode = "NORMAL"
            elif error_code[0] == 2:
                safety_mode = "REDUCED"
            elif error_code[0] == 3:
                safety_mode = "PROTECTIVE_STOP"
            elif error_code[0] == 4:
                safety_mode = "RECOVERY"
            elif error_code[0] == 5:
                safety_mode = "SAFEGUARD_STOP"
            elif error_code[0] == 6:
                safety_mode = "SYSTEM_EMERGENCY_STOP"
            elif error_code[0] == 7:
                safety_mode = "ROBOT_EMERGENCY_STOP"
            elif error_code[0] == 8:
                safety_mode = "VIOLATION"
            elif error_code[0] == 9:
                safety_mode = "FAULT"
            elif error_code[0] == 10:
                safety_mode = "VALIDATE_JOINT_ID"
            elif error_code[0] == 11:
                safety_mode = "UNDEFINED_SAFETY_MODE"
            elif error_code[0] == 12:
                safety_mode = "AUTOMATIC_MODE_SAFEGUARD_STOP"
            elif error_code[0] == 13:
                safety_mode = "SYSTEM_THREE_POSITION_ENABLING_STOP"
            if error_code[1] == -1:
                robot_mode = "NO_CONTROLLER"
            elif error_code[1] == 0:
                robot_mode = "DISCONNECTED"
            elif error_code[1] == 1:
                robot_mode = "CONFIRM_SAFETY"
            elif error_code[1] == 2:
                robot_mode = "BOOTING"
            elif error_code[1] == 3:
                robot_mode = "POWER_OFF"
            elif error_code[1] == 4:
                robot_mode = "POWER_ON"
            elif error_code[1] == 5:
                robot_mode = "IDLE"
            elif error_code[1] == 6:
                robot_mode = "BACKDRIVE"
            elif error_code[1] == 7:
                robot_mode = "RUNNING"
            elif error_code[1] == 8:
                robot_mode = "UPDATING_FIRMWARE"
            return [safety_mode, robot_mode]

class JointStates(rclpy.node.Node):
    """
    Subscribe to the joint_states topic.
    """

    def __init__(self):
        """
        Initialize the JointStates node.
        """
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )
        self.ik_solver = URKinematics("ur3e")
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when joint states are received.

        Args:
            msg (JointState): The joint state message.
        """
        data = {
            "name": [msg.name[5]] + msg.name[0:5],
            "position": [msg.position[5]] + msg.position[0:5].tolist(),
            "velocity": [msg.velocity[5]] + msg.velocity[0:5].tolist(),
            "effort": [msg.effort[5]] + msg.effort[0:5].tolist(),
        }

        self.states = data
        self.done = True

    def get_joints(self):
        """
        Get the current joint states.

        Returns:
            dict: The current joint states.
        """
        self.wait(self)
        self.done = False
        return self.states

    def wait(self, client):
        """
        Wait for the joint states to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for joint_states_client")

class ToolWrench(rclpy.node.Node):
    """
    Subscribe to the /force_torque_sensor_broadcaster/wrench topic.
    Data =  {
            header = [sec, nanosec],
            force = [x,y,z],
            torque = [x,y,z]
            }
    """

    def __init__(self):
        """
        Initialize the ToolWrench node.
        """
        super().__init__("wrench_subscriber")
        self.subscription = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.listener_callback,
            10,
        )
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when wrench data is received.

        Args:
            msg (WrenchStamped): The wrench stamped message.
        """
        data = {
            "header": [msg.header.stamp.sec, msg.header.stamp.nanosec],
            "force": [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            "torque": [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
        }

        self.states = data
        self.done = True

    def get(self):
        """
        Get the current wrench data.

        Returns:
            dict: The current wrench data.
        """
        self.wait(self)
        self.done = False
        return self.states

    def wait(self, client):
        """
        Wait for the wrench data to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for wrench client")

class Gripper(rclpy.node.Node):
    """
    Subscribe and publish to Gripper topics.
    """

    def __init__(self):
        """
        Initialize the Gripper node.
        """
        super().__init__("gripper_client")
        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/gripper/state",
            self.listener_callback,
            10,
        )

        self.publisher_ = self.create_publisher(Int32MultiArray, "/gripper/control", 10)

        self.states = None
        self.done = False
        self.active = False

    def listener_callback(self, msg):
        """
        Callback for when gripper state data is received.

        Args:
            msg (Int32MultiArray): The gripper state message.
        """
        self.states = msg.data
        self.done = True

    def get(self):
        """
        Get the current state of the gripper.

        Returns:
            list: The current state of the gripper.
        """
        self.wait(self)
        self.done = False
        return list(self.states[0:3])

    def control(self, POS, SPE, FOR, BLOCK):
        """
        Control the gripper with the given position, speed, and force.

        Args:
            POS (int): Position for the gripper.
            SPE (int): Speed for the gripper.
            FOR (int): Force for the gripper.
        """
        msg = Int32MultiArray()
        msg.data = [POS, SPE, FOR]
        self.publisher_.publish(msg)

        if BLOCK:
            # wait until the gripper acknowledges that it will try to go to the requested position
            time.sleep(0.05)
            self.get()
            while self.states[4] != self.states[3]:
                self.get()
                time.sleep(0.05)

            # wait until not moving
            self.get()
            while self.states[5] == 0:
                self.get()
                time.sleep(0.05)

    def wait(self, client):  # class gripper
        """
        Wait for the gripper state to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for gripper client")

class Trajectory(): 
    def __init__(self):
        self.trajectory = []
    def relative_to_global(self, relative_trajectory, global_start, degrees=True):
        '''
        Create a list of global movements from a list of relative movements.  

        Args:
            trajectory (list): list of points (either start and end, or longer linear line)
            global_start (list): global start point for the trajectory 
            degrees (bool): true returns the angle in degrees, false returns the angle in radians

        Return:
            Returns the new global trajectory. 
        '''
        self.trajectory = [global_start]

        for rel in relative_trajectory:
            new_pose = np.add(self.trajectory[-1], rel).tolist()
            self.trajectory.append(new_pose)
        return self.trajectory

    def set_z(self, z_value, coordinates_list=None):
        '''
        sets z value for list of coordinates. 

        Args:
            z-value (float): height in meters to set the z coordinate
            trajectory (list): optional list of points (either start and end, or longer linear line). Default uses self. 

        Return:
            Returns the adjusted coordinates.
        '''
        if coordinates_list is not None: 
            self.trajectory = coordinates_list
        for point in self.trajectory:
            if len(point) > 2:
                point[2] = z_value
        return self.trajectory
    
    def align2d(self, coordinates_list=None, degrees=True):
        '''
        Calculate the global z-angles to turn the gripper perpendicular to the direction of a given trajectory. 

        Args:
            trajectory (list): optional list of points (either start and end, or longer linear line). Default uses self. 
            degrees (bool): true returns the angle in degrees, false returns the angle in radians

        Return:
            Returns the new global trajectory with the correct z-angles.
        '''

        # verify that the input trajectory is correct
        #if len(traj) == 6 and not isinstance(traj[0], list):
        #    print("ERROR: Expected multiple points but only 1 was given.")
        #    return

        # find the first angle to turn to
        if coordinates_list is not None:
            self.trajectory = coordinates_list
        for i in range(len(self.trajectory) - 1):
            x = self.trajectory[i+1][0] - self.trajectory[i][0]
            y = self.trajectory[i+1][1] - self.trajectory[i][1]
            if x == 0:
                if y > 0:
                    angle = 90
                else: 
                    angle = 270
            elif x < 0:
                angle = 180 + math.atan(y/x)*180/math.pi
            elif y < 0:
                angle = 360 + math.atan(y/x)*180/math.pi
            else:
                angle = math.atan(y/x)*180/math.pi


            if i != 0:
                if angle - self.trajectory[i-1][5] > 180:
                    angle = angle - 360
                if self.trajectory[i-1][5] > 180 and (angle <= 180 or self.trajectory[i-1][5] > 360):
                    if int(self.trajectory[i-1][5]/360) > 0:
                        angle += 360 * int(self.trajectory[i-1][5]/360)
                    else:
                        angle += 360

            self.trajectory[i][5] = angle

        for i in range(len(self.trajectory)):
            self.trajectory[i][5] += 90

        return self.trajectory

    def linear_interp(self, coordinates_list=None, step_size = .01, debugging=False, aligned=False):
        '''
        Linearly interpolate a trajectory. For trajectories that only change the angle, nothing will happen

        Args:
            coordinates_list (list): optional list of points (there must be at least 2). default uses self. 
            step_size (float): step_size in meters. smaller steps result in a finer interpolation. Default 1 cm
            debugging: True will turn on print statements, including the results of the interpolation. False only prints errors. 

        Return:
            Returns interpolated trajectory. In the case of an error, the original list is returned. 
        '''
        if coordinates_list is not None: 
            self.trajectory = coordinates_list
        if debugging:
            print("\n--- Original Coordinates ---")
            for coord in self.trajectory:
                print(coord)
            
        all_interpolated_coords = []

        if len(self.trajectory) < 2:
            print(f"Error: coordinate list invalid. Length is {len(self.trajectory)}. Ensure at least 2 points to interpolate")
            return self.trajectory
        if step_size <= 0: 
            print("Error: step size must be greater than 0.")
            return self.trajectory
        all_interpolated_coords.append(self.trajectory[0]) # start w/ first value

        for i in range(len(self.trajectory) - 1):
            # find total dist between pts
            position_diffs_sq = [(self.trajectory[i+1][j] - self.trajectory[i][j])**2 for j in range(3)]
            segment_total_dist = math.sqrt(sum(position_diffs_sq))

            # Calculate differences for angle components (roll, pitch, yaw)
            angle_diffs = [self.trajectory[i+1][j+3] - self.trajectory[i][j+3] for j in range(3)]
            num_steps_for_segment = int(segment_total_dist / step_size)

            if num_steps_for_segment == 0:
                num_steps_for_segment = 1 # take at least 1 step btwn

            # Generate interpolated points for this segment
            for step in range(1, num_steps_for_segment + 1):
                t = (step/num_steps_for_segment) # Calculate interpolation factor (t)
                interpolated_point = []
                # Interpolate position components
                for j in range(3):
                    interpolated_point.append(self.trajectory[i][j] + (self.trajectory[i+1][j] - self.trajectory[i][j]) * t)
                # Interpolate angle components
                for j in range(3):
                    interpolated_point.append(self.trajectory[i][j+3] + angle_diffs[j] * t)

                all_interpolated_coords.append(interpolated_point)
        self.trajectory = all_interpolated_coords
        if aligned: 
            self.trajectory = self.align2d()
        
        if debugging:
            print("\n--- Interpolated Coordinates ---")
            for i, coord in enumerate(self.trajectory):
                print(f"Point {i}: {coord}")
            print(f"Total interpolated points: {len(self.trajectory)}")
        return self.trajectory

    def add_corners(self, coordinates_list=None):
        '''
        Create a new trajectory with extra points at the corners so that the end effector doesn't turn until
        it reaches the next linear section of the trajectory.

        Args:
            trajectory (list): optional list of points the robot is moving through. Default uses self.trajectory

        Return:
            Returns a new trajectory with corner points inserted.
        '''

        if coordinates_list is not None:
            self.trajectory = coordinates_list

        new_trajectory = []
        
        for point in range(len(self.trajectory) - 1):
            # if the z-angles change, add a turning point between them
            new_trajectory.append(self.trajectory[point])
            
            if self.trajectory[point][5] != self.trajectory[point + 1][5]:
                turning_point = list(self.trajectory[point + 1])
                turning_point[5] = self.trajectory[point][5]
                new_trajectory.append(turning_point)
        self.trajectory = new_trajectory
        return self.trajectory
    
    def draw_circle(self, center, radius, num_points=36, degrees=True, debugging=False, aligned=False, plane_rotation=[0, 0, 0]):
        """
        Generates a circular 6-DOF trajectory.

        Parameters:
            center (list): [x, y, z, roll, pitch, yaw]
            radius (float): Radius of the circle.
            num_points (int): Number of points around the circle.
            degrees (bool): True if orientation angles are in degrees.
            debugging (bool): If True, prints debug info.
            aligned (bool): If True, aligns end-effector orientation to circle tangent (not implemented yet).
            plane_rotation (list): [rx, ry, rz] rotation (Euler angles) of the circle plane.

        Returns:
            list of poses: Each pose is [x, y, z, roll, pitch, yaw]
        """
        x0, y0, z0, roll, pitch, yaw = center
        rot_matrix = self.euler_rotation_matrix(*plane_rotation, degrees=degrees)

        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        self.trajectory = []

        for theta in angles:
            # Circle in XY plane
            local = np.array([
                radius * np.cos(theta),
                radius * np.sin(theta),
                0
            ])
            # Apply rotation
            rotated = rot_matrix @ local
            position = np.array([x0, y0, z0]) + rotated
            pose = list(position) + [roll, pitch, yaw]
            self.trajectory.append(pose)
        if aligned:
            self.trajectory = self.align2d()
        if debugging:
            print("\n--- Coordinates ---")
            for i, coord in enumerate(self.trajectory):
                print(f"Point {i}: {coord}")
        return self.trajectory

    @staticmethod
    def euler_rotation_matrix(rx, ry, rz, degrees=True):
        """Create a rotation matrix from Euler angles (XYZ convention)."""
        if degrees:
            rx, ry, rz = np.radians([rx, ry, rz])

        # Rotation around X-axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])

        # Rotation around Y-axis
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])

        # Rotation around Z-axis
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])

        # Combined rotation (XYZ order: R = Rz @ Ry @ Rx)
        return Rz @ Ry @ Rx