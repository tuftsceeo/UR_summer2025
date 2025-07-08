# Written by Leia Hannes
# Center for Engineering Education and Outreach
# Summer of 2025

from UR_summer2025.UR_library_2025 import MyUR3e, Trajectory
import matplotlib.pyplot as plt
import math, time, socket

class ImpactTesting(MyUR3e):
    def __init__(self):
        super().__init__

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

    def hit_hammer(self):
        s = socket.socket()
        s.connect(("10.247.137.49", 1234))  # Pico's IP and port
        s.send(b"run_impact")

        data = s.recv(1024).decode().strip()
        s.close()
        return data
    
    def run_test(self, corners, points, flip_rows=False, hits=1, automatic=True, vis_only=False, graph=False):
        '''
        Runs a full impact test on a flat plate.
        
        Args:
            corners (list): global coordinates of the 4 corners of the plate (bottom left, bottom right, top right, top left)
                can be a square or rectangle, does not have to be aligned with axes
            points (int list): number of points (x,y) to put in a grid
            flip_rows (bool): if False returns trajectory with each row of points starting from the left
                if True returns trajectory zig-zagging through points for smoother arm motion
            hits (int): number of times to hit the plate at each location
            automatic (bool): if False asks for user confirmation before each hit and movement. hits variable not used,
                will continue to ask for hits until told to procede
        '''
        traj = self.impact_path(corners, points, flip_rows=flip_rows)

        if hits == 0 and automatic:
            for point in traj:
                self.move_global(point, vis_only=vis_only)
                time.sleep(0.1)
            self.move_global(traj[0], vis_only=vis_only)
            print("Test Complete") 
        elif hits == 0:
            for point in traj:
                self.move_global(point, vis_only=vis_only)
                choice = input("Next position?: ")
                if choice == "n":
                    break
            if input("Go back to start?:  ") != "n":
                self.move_global(traj[0], vis_only=vis_only)
            print("Test Complete")
        elif automatic:
            for point in traj:
                self.move_global(point, vis_only=vis_only)
                for i in range(hits):
                    self.hit_hammer()
            self.move_global(traj[0], vis_only=vis_only)
            print("Test Complete")
        else:
            for point in traj:
                self.move_global(point, vis_only=vis_only)

                done_hitting = False
                if input("Hit?: ") == "n":
                    done_hitting = True

                while not done_hitting:
                    self.hit_hammer()
                
                    choice = input("Hit again?: ")
                    if choice == "n":
                        done_hitting = True
                choice = input("Next position?: ")
                if choice == "n":
                    break

            if input("Go back to start?:  ") != "n":
                self.move_global(traj[0], vis_only=vis_only)
                
            print("Test Complete")

        if graph:
            x = []
            y = []
            x_corners = [corners[1][0], corners[2][0], corners[3][0]]
            y_corners = [corners[1][1], corners[2][1], corners[3][1]]
            
            for point in traj:
                x.append(point[0])
                y.append(point[1])

            plt.plot(x,y, c='b')
            plt.scatter(x,y, c='b')
            plt.scatter(x_corners, y_corners, c='g')
            plt.scatter(corners[0][0], corners[0][1], c='orange')
            plt.legend(["Trajectory", "Impact Points", "Corners", "Starting Point"])
            plt.show()
