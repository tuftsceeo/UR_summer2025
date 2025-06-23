# Written by Leia Hannes
# Center for Engineering Education and Outreach
# Summer of 2025

import matplotlib.pyplot as plt
import math

def impact(corners, points, flip_rows=True):
    '''
    Finds equally spaced points on a grid and creates trajectory for impact testing

    Args:
        corners (list): global coordinates of the 4 corners of the plate (bottom left, bottom right, top right, top left)
                        can be a square or rectangle, does not have to be aligned with axes
        points (int list): number of points (x,y) to put in a grid
        flip_rows (bool): if False returns trajectory with each row of points starting from the left
                          if True returns trajectory zig-zagging through points for smoother arm motion

    Returns: 
        Returns a trajectory with points at all vertices of the given grid
    '''

    # check for input errors
    if len(corners) > 4:
        print("ERROR: Expected 4 points in corners but more were given.")
        return corners
    if len(corners) < 4:
        print("ERROR: Expected 4 points in corners but fewer were given.")
        return corners
    # check that diagonals are equal
    if (corners[2][0] - corners[0][0])**2 + (corners[2][1] - corners[0][1])**2 != (corners[3][0] - corners[1][0])**2 + (corners[3][1] - corners[1][1])**2:
        print("ERROR: invalid input. Given coordinates do not define a square or rectangle")
        return corners
    
    # calculate the spacing between the points between bottom corners
    x_len_x = corners[1][0] - corners[0][0]
    y_len_x = corners[1][1] - corners[0][1]

    x_spacing_bottom = x_len_x / (points[0] - 1)
    y_spacing_bottom = y_len_x / (points[0] - 1)

    # calculate the spacing between the points between side corners
    x_len_y = corners[3][0] - corners[0][0]
    y_len_y = corners[3][1] - corners[0][1]

    x_spacing_side = x_len_y / (points[1] - 1)
    y_spacing_side = y_len_y / (points[1] - 1)

    # calculate each row of points
    traj = []
    for j in range(points[1]):
        row = []
        for i in range(points[0]):
            point = [corners[0][0] + x_spacing_bottom * i + x_spacing_side * j, 
                     corners[0][1] + y_spacing_bottom * i + y_spacing_side * j]
            row.append(point)
        traj.append(row)
    
    # inverts every other row so the arm motion is smoother
    if flip_rows:
        for j in range(points[1]):
                if j % 2 == 1:
                    traj[j] = traj[j][::-1]

    # reformat the points into one list
    final_trajectory = []
    for j in range(points[1]):
        for i in range(points[0]):
            final_trajectory.append(traj[j][i])

    return final_trajectory
