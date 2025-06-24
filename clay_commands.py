# Written by Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025

# contains clay specific functions. UR_lib must also be imported. 

import force
from myur import MyUR3e
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
        x = cx + (length / 2) * (-1) ** i  # alternate up and down
        y = cy + (i - cycles) * width  # horizontal offset
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

def roll_coil(diameter):
    """
    rolls a coil! 

    Parameters:
        diameter (float): desired coil diameter [cm]

    Returns:
        NA
    """
    robot = MyUR3e()    
    force_detected = 0
    # write something to go down until force is felt
    while force_detected < 5: # force in Newtons
        print("do something")
        force_detected = force.get_force()
    print(f"stop! detected {force} N force.")

    # read position
    center = robot.read_global_pos()
    z = .2 # or center[2] # not sure which is better here [m]
    center = [center[0], center[1]]
    trajectory = roll_sequence(center)

    while z > diameter:
        print(f"z is {z}. move again. ")
        UR_lib.set_z(trajectory, z)
        robot.move_global(trajectory, 15)
        z -= .03    # move arm
    print("done moving.")
