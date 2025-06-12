# Written by Leia Hannes and Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025
# exmaple

import math
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

    # loop through each pair of points and calculate the angle
    for i in range(len(trajectory) - 1):
        
        # determine the direction that the arm is moving
        direction_vector = [trajectory[i+1][0] - trajectory[i][0], trajectory[i+1][1] - trajectory[i][1]] 
        print(direction_vector)
    
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
        print("\n--- Original Coordinates ---")
        for coord in coordinates_list:
            print(coord)

        print("\n--- Interpolated Coordinates ---")
        for i, coord in enumerate(all_interpolated_coords):
            print(f"Point {i}: {coord}")
        print(f"Total interpolated points: {len(all_interpolated_coords)}")
    return all_interpolated_coords