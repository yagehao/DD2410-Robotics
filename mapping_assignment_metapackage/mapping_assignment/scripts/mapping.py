#!/usr/bin/env python3

"""
    # {Yage Hao}
    # {980707 9547}
    # {yage@kth.se}
"""

# Python standard library
import math 
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """
        # init para
        x_map = []
        y_map = []
        x_index = []
        y_index = []
        x_index_robot = []
        y_index_robot = []
        x_pos = pose.pose.position.x 
        y_pos = pose.pose.position.y 

        k = 0
        for i in range(len(scan.ranges)):

            # break the bound, continue to next range
            if scan.ranges[i] <= scan.range_min or scan.ranges[i] >= scan.range_max:
                continue 

            else:
                #convert goal to map frame 
                angle_laser = scan.angle_min + i*scan.angle_increment 
                x_map.append(x_pos + scan.ranges[i] * cos(robot_yaw + angle_laser))
                y_map.append(y_pos + scan.ranges[i] * sin(robot_yaw + angle_laser))

                # convert goal coord to grid-map index 
                x_index.append(int((x_map[k] - origin.position.x) / resolution))
                y_index.append(int((y_map[k] - origin.position.y) / resolution))

                # convert robot current pos to grid-map index 
                x_index_robot.append(int((x_pos - origin.position.x) / resolution))
                y_index_robot.append(int((y_pos - origin.position.y) / resolution))

                # fill in free space
                start = (x_index_robot[k], y_index_robot[k])
                end = (x_index[k], y_index[k])
                free_space = self.raytrace(start, end)
                for j in free_space:
                    self.add_to_map(grid_map, j[0], j[1], self.free_space)
                
                k += 1
        
        # fill in occupied space 
        for i in range(len(x_index)):
            self.add_to_map(grid_map, x_index[i], y_index[i], self.occupied_space)



        """
        For C only!
        Fill in the update correctly below.
        """
        
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = np.amin(x_index)
        # The minimum y index in 'grid_map' that has been updated
        update.y = np.amin(y_index)
        # Maximum x index - minimum x index + 1
        update.width = np.amax(x_index) - np.amin(x_index) + 1
        # Maximum y index - minimum y index + 1
        update.height = np.amax(y_index) - np.amin(y_index) + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        
        for dy in range(np.amin(y_index), np.amax(y_index) + 1):
            for dx in range(np.amin(x_index), np.amax(x_index) + 1):
                dxy = grid_map[dx, dy]
                update.data.append(dxy) 
        

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update


    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        
        width = grid_map.get_width()
        height = grid_map.get_height()
        for x in range(width):
            for y in range(height):
                if grid_map[x, y] == self.occupied_space:
                    x1 = x - self.radius
                    y1 = y - self.radius 
                    
                    for dx in range(2*self.radius + 1):
                        for dy in range(2 * self.radius + 1):
                            x2 = x1 + dx 
                            y2 = y1 + dy 
                            #fill in the C space cells whose distance to occupied cells <= robot radius and not filled
                            if  math.hypot(x2-x, y2-y) <= self.radius and grid_map[x2, y2] != self.occupied_space and self.is_in_bounds(grid_map, x2, y2):
                                self.add_to_map(grid_map, x2, y2, self.c_space)
        
        
        # Return the inflated map
        return grid_map
