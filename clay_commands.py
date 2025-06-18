# Written by Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025

# contains clay specific functions. UR_lib must also be imported. 

import UR_lib

def roll_sequence(center, length=.1, width=.01, cycles=3, linear_interp=True, debug=False):
    """
    Generates a coil rolling trajectory with linear interpretation.

    4 cycle example: 

       |-----------trajectory-----------|
           /\      /\       /\      /\     |  
          /  \    /  \     /  \    /  \    |
         /    \  /    \   /    \  /    \   |  length
        /      \/      \./      \/      \  |
        ----            ^center
        ^width

    Parameters:
        center (list): [x, y] center point of trajectory
        length (float): length of roll motion [m]
        width (float): Horizontal distance travelled between rolls [m]
        cycles (int): True if orientation angles are in degrees.
        linear_interp (bool): Performs linear interpolation of trajectory if true (recommended)

    Returns:
        coil rolling trajectory 
    """

    num_points = (2*cycles) + 1
    cx, cy = center
    trajectory = []
    for i in range(num_points):
        x = cx + (i - cycles) * width  # horizontal offset
        y = cy + (length / 2) * (-1) ** i  # alternate up and down
        trajectory.append([x, y, 0, 0, 0, 0])

    reverse = trajectory[-2::-1]  # exclude last point and reverse
    trajectory = trajectory + reverse
    if debug:
        print("\n--- Initial Coordinates ---")
        for i, point in enumerate(trajectory):
            print(f"Point {i}: {point}")    
    if linear_interp:
        trajectory = UR_lib.linear_interp(trajectory)
    return trajectory

