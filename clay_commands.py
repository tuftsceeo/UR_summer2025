# Written by Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025
from UR_library_2025 import MyUR3e, Trajectory
import time
import numpy as np

class ClaySculpt(MyUR3e):
    def __init__(self):
        super().__init__()
        self.traj = ClayTrajectory()
        self.zero_force = 0
    def calibrate_z(self):
        list = []
        for i in range(10):
            list.append(self.tool_wrench.get()["force"][2])
            time.sleep(.1)
        self.zero_force = np.average(list)
    def roll_coil(self, diameter):
        """
        rolls a coil! 

        Parameters:
            diameter (float): desired coil diameter [cm]

        Returns:
            NA
        """
        self.calibrate_z()
        force_detected = [0, 0, self.zero_force]

        starting_pos = self.read_global_pos()
        # go down until force is felt
        while force_detected[2] > (self.zero_force - 3): # force in Newtons
            force_detected = self.tool_wrench.get()["force"]
            self.traj.relative_to_global([[0, 0, -.002, 0, 0, 0]], starting_pos)
            target = self.traj.trajectory[1]
            self.move_global(target)
            print(f"detected {force_detected[2]} N force.")
            starting_pos = target
        print(f"stop! detected {force_detected[2]} N force.")

        # read position, then move up
        center = self.read_global_pos()
        z = center[2] # or center[2] # not sure which is better here [m]
        self.traj.relative_to_global([[0, 0, .05, 0, 0, 0]], center)
        self.move_global(self.traj.trajectory)

        center = [(center[0]-.01), (center[1]-.02)]
        self.traj.roll_sequence(center)

        while z > diameter:
            print(f"z is {z}. move again. ")
            self.traj.set_z(z)
            self.move_global(self.traj.trajectory, 15)
            z -= .002    # move arm
        self.traj.relative_to_global([[0, 0, .05, 0, 0, 0]], self.traj.trajectory)
        self.move_global(self.traj.trajectory)
        print("done moving.")

class ClayTrajectory(Trajectory):
    def __init__(self):
        super().__init__()
    def roll_sequence(self, center, length=.1, width=.01, cycles=3, linear_interp=True, debug=False):
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
            cycles (int): periods. 
            linear_interp (bool): Performs linear interpolation of trajectory if true (recommended)

        Returns:
            coil rolling trajectory 
        """

        num_points = (2*cycles) + 1
        cx, cy = center
        self.trajectory = []
        for i in range(num_points):
            x = cx + (length / 2) * (-1) ** i  # alternate up and down
            y = cy + (i - cycles) * width  # horizontal offset
            self.trajectory.append([x, y, 0, 0, 0, 0])

        reverse = self.trajectory[-2::-1]  # exclude last point and reverse
        self.trajectory = self.trajectory + reverse
        if debug:
            print("\n--- Initial Coordinates ---")
            for i, point in enumerate(self.trajectory):
                print(f"Point {i}: {point}")    
        if linear_interp:
            self.linear_interp()
        return self.trajectory