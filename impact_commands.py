# Written by Leia Hannes
# Center for Engineering Education and Outreach
# Summer of 2025

from UR_library_2025 import MyUR3e, Trajectory
from websocket import *
import time
import numpy as np
import math

class ImpactTesting(MyUR3e):
    def __init__(self):
        super().__init__()
        self.traj_function = ImpactTrajectory()
        self.channel = Channel_Communication()

    def flat_plate_impact_test(self):
        '''
        Calls all relevant functions to run an impact test on a flat plate
        '''
        # get corners
        corner1, corner3, points = self.set_position()

        # calculate trajectory
        corners = self.traj_function.get_corners_diagonal(corner1, corner3)
        traj = self.traj_function.impact_path(corners, points)
        print(traj)
        # probe for heights
        new_traj = self.callibrate_heights(traj)

        # run the test
        self.run_test(traj)

    def set_position(self):
        '''
        Reads from channels from the Robot Control Panel to get the starting and ending points for the test
        
        Returns:
            the diagonal corners
        '''
        self.channel.clear_queue()
        corner1 = None
        corner3 = None
        points = []

        done = False
        while not done:
            data = self.channel.read_data()
            if data != None:
                data = json.loads(data)
                if data['topic'] == '/modal/URarm/control':
                    data = json.loads(data['value'])
                    cmd = data['command']
                    delta = float(data['delta'])/1000
        
                    if delta > 0.04:
                        self.channel.send_data('/modal/URarm/control/response', "ERROR: MAXIMUM MOVEMENT DELTA OF 4CM EXCEEDED")

                    else:
                        if cmd == "+x":
                            print("moving", delta, "in x")
                            self.move_global_r([delta, 0, 0, 0, 0, 0])
                        elif cmd == "-x":
                            print("moving", -1*delta, "in x")
                            self.move_global_r([-1*delta, 0, 0, 0, 0, 0])
                        elif cmd == "+y":
                            print("moving", delta, "in y")
                            self.move_global_r([0, delta, 0, 0, 0, 0])
                        elif cmd == "-y":
                            print("moving", -1*delta, "in y")
                            self.move_global_r([0, -1*delta, 0, 0, 0, 0])
                        elif cmd == "+z":
                            print("moving", delta, "in z")
                            self.move_global_r([0, 0, delta, 0, 0, 0])
                        elif cmd == "-z":
                            print("moving", -1*delta, "in z")
                            self.move_global_r([0, 0, -1*delta, 0, 0, 0])
                        elif cmd == "g_open":
                            print("opening gripper")
                            self.move_gripper(58)
                        elif cmd == "g_close":
                            print("closing gripper")
                            self.move_gripper(60)
                        elif cmd == "c1_save":
                            corner1 = self.read_global_pos()
                        elif cmd == "c3_save":
                            corner3 = self.read_global_pos()
                        else:
                            print("invalid command")
                if corner1 and corner3:
                    done = True

        done = False
        while not done:
            data = self.channel.read_data()
            if data != None:
                data = json.loads(data)
                if data['topic'] == '/modal/URarm/control':
                    data = json.loads(data['value'])
                    cmd = data['command']
                    delta = int(data['delta'])
                    
                    if cmd == "ready":
                        points.append(delta)
                        points.append(delta)
                        done = True
        return corner1, corner3, points

    def callibrate_heights(self, traj):
        '''
        Probe the plate twice at each point in the trajectory and record the new heights
    
        Args:
            traj (list): the list of points to probe at
        
        Returns:
            the new trajectory with the new heights and the offset to center the hammer over the impact point
        '''
        new_z = []
        self.channel.clear_queue()
        
        for point in traj:
            
            # move to each point in the trajectory
            self.move_global(point)
            self.move_global_r([0, 0, 0.001, 0, 0, 0])
        
            # extend the probe
            self.channel.send_data('/modal/tool/request', 'probe')
            time.sleep(1)
            self.move_global_r([0, 0, -0.001, 0, 0, 0])

            # wait for response from probe
            done = False
            while not done:
                data = self.channel.read_data()
                if data != None:
                    data = json.loads(data)
                    if data['topic'] == '/modal/URarm/tool/response' and data['value']['sensor_status'] == 'sensing':
                        done = True
        
            # lower arm at 1mm increments until probe responds
            done = False
            while not done:
                self.move_global_r([0, 0, -0.001, 0, 0, 0])
                for i in range(10):
                    data = self.channel.read_data()
                    if data != None:
                        data = json.loads(data)
                        if data['topic'] == '/modal/URarm/tool/response' and data['value']['sensor_status'] == 'sensed':
                            done = True
        
            # extend the probe again
            self.move_global_r([0, 0, 0.003, 0, 0, 0])
            self.channel.send_data('/modal/tool/request', 'probe')
            time.sleep(2)
            self.move_global_r([0, 0, -0.0015, 0, 0, 0])
            
            # wait for response from probe
            done = False
            while not done:
                data = self.channel.read_data()
                if data != None:
                    data = json.loads(data)
                    if data['topic'] == '/modal/URarm/tool/response' and data['value']['sensor_status'] == 'sensing':
                        done = True
        
            # lower arm at 0.2mm increments until probe responds
            done = False
            while not done:
                self.move_global_r([0, 0, -0.0002, 0, 0, 0])
                for i in range(10):
                    data = self.channel.read_data()
                    if data != None:
                        data = json.loads(data)
                        if data['topic'] == '/modal/URarm/tool/response' and data['value']['sensor_status'] == 'sensed':
                            done = True
        
            # record new height
            print("initial height:", point[2])
            print("new height:", self.read_global_pos()[2])
            new_z.append(self.read_global_pos()[2] + 0.0005)
        
        new_traj = []
        for i in range(len(traj)):
            new_traj.append([traj[i][0]  - 0.015, traj[i][1] + 0.012, new_z[i], traj[i][3], traj[i][4], traj[i][5]])
    
        return new_traj

    def run_test(self, traj):
        '''
        Communicate with the Obie apps to run a full impact test
        
        Args:
            traj (list): the points to impact at
        '''
        
        finished_test = False
        while not finished_test:
    
            # wait for a request
            done = False
            while not done:
                data = self.channel.read_data()
                if data != None:
                    data = json.loads(data)
                    if data['topic'] == '/modal/URarm/request':
                        request = data['value']
                        done = True
            print(request)
            
            # move to the requested point
            point_index = int(request['point']) - 1
            point = traj[point_index]
            self.move_global(point)
            
            # enter hits loop
            for i in range(int(request['hits'])):
                # wait for ready from obie
                done = False
                start = time.monotonic()
                while not done:
                    data = self.channel.read_data()
                    if data != None:
                        data = json.loads(data)
                        if data['topic'] == '/modal/URarm/trigger':
                            done = True
                    # timeout after 15 seconds
                    if time.monotonic() > start + 15:
                        for i in range(10):
                            data = self.channel.read_data()
                        self.channel.send_data('/modal/tool/request', 'hit')
                        start = time.monotonic()
                
                # do a hit
                for i in range(10):
                    data = self.channel.read_data()
                self.channel.send_data('/modal/tool/request', 'hit')
                
            if point_index == len(traj) - 1:
                finished_test = True
        print("done")

        
class ImpactTrajectory(Trajectory):
    def __init__(self):
        super().__init__()

    def get_corners_diagonal(self, first_point, last_point):
        '''
        Calculate the position of all 4 corners from the start point and the end point
        
        Args:
            first_point (list): global coordinates of first impact point
            last_point (list): global coordinates of last impact point
    
        Returns:
            list of all 4 corners
        '''
        dx = (last_point[0] - first_point[0])
        dy = (last_point[1] - first_point[1])
    
        d = math.sqrt(dx**2 + dy**2)
    
        center = [first_point[0] + dx/2, first_point[1] + dy/2]
    
        vector = [dx/2, dy/2]
    
        v1 = [-1*vector[1], vector[0]]
        v2 = [vector[1], -1*vector[0]]
    
        p1 = [center[0] + v1[0], center[1] + v1[1], first_point[2], 0, 0, 0]
        p2 = [center[0] + v2[0], center[1] + v2[1], first_point[2], 0, 0, 0]
    
        corners = [first_point, p1, last_point, p2]
    
        return corners

    def impact_path(self, corners, points, flip_rows=False):
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
        #if (corners[2][0] - corners[0][0])**2 + (corners[2][1] - corners[0][1])**2 != (corners[3][0] - corners[1][0])**2 + (corners[3][1] - corners[1][1])**2:
        #    print("ERROR: invalid input. Given coordinates do not define a square or rectangle")
        #    return corners
        
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
                         corners[0][1] + y_spacing_bottom * i + y_spacing_side * j,
                         corners[0][2], 0.0, 0.0, 0.0]
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
