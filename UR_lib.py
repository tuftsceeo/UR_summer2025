# Written by Leia Hannes and Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025

import math, copy
import numpy as np

def relative_to_global(trajectory, global_start, degrees=True):
    '''
    Create a list of global movements from a list of relative movements.  

    Args:
        trajectory (list): list of points (either start and end, or longer linear line)
        global_start (list): global start point for the trajectory 
        degrees (bool): true returns the angle in degrees, false returns the angle in radians

    Return:
        Returns the new global trajectory. 
    '''
    global_trajectory = [global_start]

    for rel in trajectory:
        new_pose = np.add(global_trajectory[-1], rel).tolist()
        global_trajectory.append(new_pose)
    return global_trajectory
    

def align2d(trajectory, degrees=True):
    '''
    Calculate the global z-angles to turn the gripper perpendicular to the direction of a given trajectory. 

    Args:
        trajectory (list): list of points (either start and end, or longer linear line)
        degrees (bool): true returns the angle in degrees, false returns the angle in radians

    Return:
        Returns the new global trajectory with the correct z-angles.
    '''

    # verify that the input trajectory is correct
    if len(trajectory) == 6 and not isinstance(trajectory[0], list):
        print("ERROR: Expected multiple points but only 1 was given.")
        return
    
    #######################################################################################################################
    # possible fix for orignial coordinates chaning
    # trajectory = copy.deepcopy(trajectory) make a copy of the trajectory to avoid editing the original points
    #######################################################################################################################

    # loop through each pair of points and calculate the angle
    for i in range(len(trajectory) - 1):
        
        # determine the direction that the arm is moving
        direction_vector = [trajectory[i+1][0] - trajectory[i][0], trajectory[i+1][1] - trajectory[i][1]] 
    
        # calculate the global angle from the origin
        if direction_vector[0] == 0.0:
            angle = math.pi/2
        elif direction_vector[1] == 0.0:
            angle = 0
        else:
            angle = math.atan(direction_vector[1]/direction_vector[0])

        if direction_vector[0] < 0 or direction_vector[1] < 0:
            angle += math.pi

        
        # convert to degrees if needed
        if degrees:
            angle = angle*180/math.pi + 90

        # add the new angle
        trajectory[i][5] = angle

    # return the new trajectory
    return trajectory

def linear_interp(coordinates_list, step_size, debugging=False, aligned=False):
    '''
    Linearly interpolate a trajectory. For trajectories that only change the angle, nothing will happen

    Args:
        coordinates_list (list): list of points (there must be at least 2)
        step_size (float): step_size in meters. smaller steps result in a finer interpolation. 
        debugging: True will turn on print statements, including the results of the interpolation. False only prints errors. 

    Return:
        Returns interpolated trajectory. In the case of an error, the original list is returned. 
    '''

    if debugging:
        print("\n--- Original Coordinates ---")
        for coord in coordinates_list:
            print(coord)
        
    all_interpolated_coords = []

    if len(coordinates_list) < 2:
        print(f"Error: coordinate list invalid. Length is {len(coordinates_list)}. Ensure at least 2 points to interpolate")
        return coordinates_list
    if step_size <= 0: 
        print("Error: step size must be greater than 0.")
        return coordinates_list
    all_interpolated_coords.append(coordinates_list[0]) # start w/ first value

    for i in range(len(coordinates_list) - 1):
        # find total dist between pts
        position_diffs_sq = [(coordinates_list[i+1][j] - coordinates_list[i][j])**2 for j in range(3)]
        segment_total_dist = math.sqrt(sum(position_diffs_sq))

        # Calculate differences for angle components (roll, pitch, yaw)
        angle_diffs = [coordinates_list[i+1][j+3] - coordinates_list[i][j+3] for j in range(3)]
        num_steps_for_segment = int(segment_total_dist / step_size)

        if num_steps_for_segment == 0:
            num_steps_for_segment = 1 # take at least 1 step btwn

        # Generate interpolated points for this segment
        for step in range(1, num_steps_for_segment + 1):
            t = (step/num_steps_for_segment) # Calculate interpolation factor (t)
            interpolated_point = []
            # Interpolate position components
            for j in range(3):
                interpolated_point.append(coordinates_list[i][j] + (coordinates_list[i+1][j] - coordinates_list[i][j]) * t)
            # Interpolate angle components
            for j in range(3):
                interpolated_point.append(coordinates_list[i][j+3] + angle_diffs[j] * t)

            all_interpolated_coords.append(interpolated_point)
    if aligned: 
        all_interpolated_coords = align2d(all_interpolated_coords)
    
    if debugging:
        print("\n--- Interpolated Coordinates ---")
        for i, coord in enumerate(all_interpolated_coords):
            print(f"Point {i}: {coord}")
        print(f"Total interpolated points: {len(all_interpolated_coords)}")
    return all_interpolated_coords


def add_corners(trajectory):
    '''
    Create a new trajectory with extra points at the corners so that the end effector doesn't turn until
    it reaches the next linear section of the trajectory.

    Args:
        trajectory (list): list of points the robot is moving through

    Return:
        Returns a new trajectory with corner points inserted.
    '''
    new_trajectory = []
    
    for point in range(len(trajectory) - 1):
        # if the z-angles change, add a turning point between them
        new_trajectory.append(trajectory[point])
        
        if trajectory[point][5] != trajectory[point + 1][5]:
            turning_point = list(trajectory[point + 1])
            turning_point[5] = trajectory[point][5]
            new_trajectory.append(turning_point)
        
    return new_trajectory


def draw_circle(center, radius, num_points=36, degrees=True, debugging=False, aligned=False):
    """
    Generates a circular trajectory for a 6-DOF robot arm.

    Parameters:
        center (list or tuple): [x, y, z, roll, pitch, yaw] - center point of the circle.
        radius (float): Radius of the circle.
        num_points (int): Number of points in the trajectory.
        use_degrees (bool): If True, angles are in degrees. If False, in radians.

    Returns:
        list of lists: Each entry is [x, y, z, roll, pitch, yaw].
    """

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

def draw_circle(center, radius, num_points=36, degrees=True, debugging=False, aligned=False, plane_rotation=[0, 0, 0]):
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
    rot_matrix = euler_rotation_matrix(*plane_rotation, degrees=degrees)

    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    trajectory = []

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
        trajectory.append(pose)
    if aligned:
        trajectory = align2d(trajectory)
    if debugging:
        print(f"Theta {np.degrees(theta):.1f}° -> {pose}")
    return trajectory
