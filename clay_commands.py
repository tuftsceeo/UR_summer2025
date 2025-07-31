# Written by Hannah Loly
# Center for Engineering Education and Outreach
# Summer of 2025
from UR_summer2025.UR_library_2025 import MyUR3e, Trajectory
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
def roll_coil(self, diameter, step=0.002, sweep=False, force_based=False, force_threshold=2.0):
    """
    Rolls a coil using the UR arm.

    Parameters:
        diameter (float): Final z-height to stop rolling [m].
        step (float): Step size for z descent [m].
        sweep (bool): If True, uses sweeping motion during roll.
        force_based (bool): If True, descend using force threshold instead of fixed step.
        force_threshold (float): Threshold force in Newtons to stop descent if force_based is True.

    Returns:
        None
    """
    self.calibrate_z()
    force_detected = [0, 0, self.zero_force]

    starting_pos = self.read_global_pos()
    # go down until 2N force is felt
    while force_detected[2] > (self.zero_force - force_threshold): # force in Newtons
        force_detected = self.tool_wrench.get()["force"]
        self.traj.relative_to_global([[0, 0, -step, 0, 0, 0]], starting_pos)
        target = self.traj.trajectory[1]
        self.move_global(target)
        print(f"detected {force_detected[2]} N force.")
        starting_pos = target
    print(f"stop! detected {force_detected[2]} N force.")

    center = self.read_global_pos()
    z = center[2]

    # move up for clearance
    self.traj.relative_to_global([[0, 0, 0.05, 0, 0, 0]], center)
    self.move_global(self.traj.trajectory)

    center = [(center[0] - 0.03), center[1]] # use only x and y. add offset in x-direction to help center
    # generate roll sequence centered on clay
    if sweep==False:
        self.traj.roll_sequence(center, length=0.11)
    else:
        self.traj.roll_sequence(center, sweep=True)
    self.traj.set_z(z)

    print("Begin roll sequence.")
    while z > diameter:
        print(f"z is {z}. move again. ")

        if force_based:
            contact = False
            while not contact:
                current_force = self.tool_wrench.get()["force"][2]
                if current_force < (self.zero_force - force_threshold):
                    contact = True
                    print(f"Force threshold met: {current_force:.2f} N")
                else:
                    self.traj.relative_to_global([[0, 0, -.0005, 0, 0, 0]], self.traj.trajectory[0])
                    next_pos = self.traj.trajectory[1]
                    self.move_global(next_pos)
                    z = next_pos[2]
                    if z <= diameter:
                        print("Desired diameter reached.")
                        break
        else:
            self.traj.set_z(z)

        self.move_global(self.traj.trajectory, 15)

        if not force_based:
            z -= step

    # Move up and finish
    self.traj.relative_to_global([[0, 0, 0.1, 0, 0, 0]], self.traj.trajectory[-1])
    self.move_global(self.traj.trajectory)
    print("done moving.")


    def blend(self, height, offset_1=-.12, offset_2=.05, pass_width=.0025, length=.2, increment=.05, alternate=True):
        """
        blends coils together. Blends at 30 degree angle

        Parameters:
            height (float): sets height for blending. [m]
            offset_1 (float): y offset to account for rx = 30 degrees. 
                              if the tool length is changed, this will need to be adjusted 
            offset_2 (float): y offset to account for rx = -30 degrees 
            pass_width (float): distance between passes [m]
            length (float): length that smoothing should be done [m]
            increment (float):  increments that smoothing should be done in. [m]
            alternate (bool): smooths both directions if True
        Returns:
            NA
        """
        calc_cycles = int(2+(increment/pass_width)) # add 2 for seam overlap
        time = calc_cycles*2.25
        for i in range(int(length/increment)):
            x = .2 + (increment*i)
            self.traj.blend_traj([x, offset_1], cycles=calc_cycles, angle = -30, width=pass_width, diameter=height)
            self.move_global(self.traj.trajectory, time)
            if alternate:
                self.traj.blend_traj([x, offset_2], cycles=calc_cycles, angle = 30, width=pass_width, diameter=height)
                self.move_global(self.traj.trajectory, time)

class ClayTrajectory(Trajectory):
    def __init__(self):
        super().__init__()
    def roll_sequence(self, center, length=.1, width=.01, cycles=3, linear_interp=True, sweep=False, debug=False):
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
            sweep (bool): True rolls in sweeping motion, False rolls with constant angle
            debug (bool): Turns on print statements if true 

        Returns:
            coil rolling trajectory 
        """

        num_points = (2*cycles) + 1
        cx, cy = center
        self.trajectory = []
        for i in range(num_points):
            x = cx + (length / 2) * (-1) ** i  # alternate up and down
            y = cy + (i - cycles) * width  # horizontal offset
            if sweep==True: 
                if (i % 2):
                    self.trajectory.append([x, y, 0, 0, -2, 0])
                else:  
                    self.trajectory.append([x, y, 0, 0, 6, 0])
            else:
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

    def blend_traj(self, center, diameter, angle=30, length=.1, width=.01, cycles=8, linear_interp=True, debug=False):
        """
        Generates a blending trajectory with linear interpretation.

        8 cycle example: 

        |-----------trajectory-----------|     _
         |   |   |   |   |   |   |   |   |     |
         |   |   |   |   |   |   |   |   |  length
         |   |   |   |   |   |   |   |   |     |
         |   |   |   |   |   |   |   |   |     _
          ----           ^center
            ^width

        Parameters:
            center (list): [x, y] center point of trajectory
            diameter(float): sets height for the coils [m]
            angle(float): sets angle that the tool is held at
            length (float): length of roll motion [m]
            width (float): Horizontal distance travelled between rolls [m]
            cycles (int): periods. 
            linear_interp (bool): Performs linear interpolation of trajectory if true (recommended)
            debug (bool): turns on print statements if true 

        Returns:
            blend trajectory 
        """

        center = [center[0], center[1], .32, angle, 0, -90] # .32 ensures the tool lifts up to change angle
        relative_trajectory = []
        relative_trajectory.append([0, 0, -(.32 - diameter), 0, 0, 0])
        for i in range(cycles):
            x = width 
            y = length 
            if (angle > 0):
                relative_trajectory.append([0, y, 0, 0, 0, 0])
                relative_trajectory.append([0, 0, .02, 0, 0, 0])
                relative_trajectory.append([x, -y, 0, 0, 0, 0])
                relative_trajectory.append([0, 0, -.02, 0, 0, 0])
            else: 
                relative_trajectory.append([x, 0, .02, 0, 0, 0])
                relative_trajectory.append([0, y, 0, 0, 0, 0])
                relative_trajectory.append([0, 0, -.02, 0, 0, 0])
                relative_trajectory.append([0, -y, 0, 0, 0, 0])
        self.relative_to_global(relative_trajectory, center)
        if debug:
            print("\n--- Initial Coordinates ---")
            for i, point in enumerate(self.trajectory):
                print(f"Point {i}: {point}")    
        if linear_interp:
            self.linear_interp()
        return self.trajectory