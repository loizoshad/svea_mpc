#! /usr/env/bin python3

import numpy as np
import rospy
import math
import casadi

from park.msg import ParkingInformation
from park.ellispes_info import EllipseInfo


class WaypointGenerator(object):
    '''
    Inputs: 
        - param park_info   : Variable containing all the information about the parking spot as provided by the CC team
        - type park_info    : park.msg.ParkingInformation

        - param state       : Current state of the vehicle
        - type state        : numpy.ndarray([x, y, yaw]) [m, m, rad]

    - Description:
        - This class is used to handle the information provided by the CC team about the parking spot
        - Given the current state of the vehicle and the parking spot information, this class will generate 
        the ellipses that represent the obstacles around the parking spot


    - Outputs:
        - param ellipses    : Ellipses that represent the obstacles around the parking spot
        - type ellipses     : list of park.ellispes_info.EllipseInfo
        - param waypoints   : Waypoints that the vehicle should follow to park
        - type waypoints    : list of numpy.ndarray([x, y, yaw]) [m, m, rad]

    '''

    def __init__(self) -> None:
        # Vehicle parameters
        self.vehicle_length = 0.50 # [m]
        self.vehicle_width = 0.249 # [m]
        self.dist_to_mid = 0.13    # [m] Distance from the center of the vehicle to the middle of the rear axle
        self.same_lane = True

    def generate_waypoints(self, state = None, park_info = None):
        self.define_ellipses(state = state, park_info = park_info)
        self.compute_waypoints()

        return self.get_waypoints()

    def get_waypoints(self):
        return self.waypoints

    def get_ellipses(self):
        return self.ellipses


    def define_ellipses(self, state = None, park_info = None):
        self.extract_park_info(state = state, park_info = park_info)

        self.ellipses = [
            EllipseInfo(    c1 = self.park_corners[0, :],
                            c2 = self.park_corners[1, :],
                            inflation = 0.2 if park_info.park_type == 1 else 0.01),
            EllipseInfo(    c1 = self.park_corners[1, :],
                            c2 = self.park_corners[2, :],
                            inflation = 0.1 if park_info.park_type == 1 else 0.01),
            EllipseInfo(    c1 = self.park_corners[2, :],
                            c2 = self.park_corners[3, :],
                            inflation = 0.2 if park_info.park_type == 1 else 0.01)
        ]

        return self.get_ellipses()

    def extract_park_info(self, state = None, park_info = None):
        '''
        Extract the parking information provided by the CC team
        
        Input:
            The corners of the parking spot are always provided in an
            anti-clockwise manner. The first point, however, is arbitrary.

        Output: 
            - park_corners
                The corners of the parking spot are always provided in the following order:
                With respect to the direction the vehicle is currently facing:
                    - Rear left
                    - Rear right
                    - Front right
                    - Front left
            - park_type
        '''

        self.state = state
        self.park_info = park_info

        # Corners of the parking spot
        self.park_corners = np.array([
            [park_info.vertices_parking_slot.points[0].x, park_info.vertices_parking_slot.points[0].y],  
            [park_info.vertices_parking_slot.points[1].x, park_info.vertices_parking_slot.points[1].y],   
            [park_info.vertices_parking_slot.points[2].x, park_info.vertices_parking_slot.points[2].y],   
            [park_info.vertices_parking_slot.points[3].x, park_info.vertices_parking_slot.points[3].y]    
        ])

        # Calculate distance from the current state to each of the corners
        distances = np.linalg.norm(self.park_corners - self.state[:2], axis=1)
        
        # Find the two closest corners
        closest_corners = np.argsort(distances) 

        park_spot_len_1 = np.linalg.norm(self.park_corners[0]-self.park_corners[1])
        park_spot_len_2 = np.linalg.norm(self.park_corners[0]-self.park_corners[3])
        width_thres = (park_spot_len_1 + park_spot_len_2)/2


        if self.is_same_lane(self.state,np.array([self.park_info.midpoint.x,self.park_info.midpoint.y])): # Same lane
            longest_dist = closest_corners[3]
            if np.linalg.norm(self.park_corners[longest_dist]-self.park_corners[(longest_dist+1)%4]) < width_thres: # Short distance to next corner to the right
                if self.park_info.park_type == 1:  # Pararell parking: case 0
                    rear_left, rear_right, front_right, front_left = self.set_corners(self.park_corners, case = 0, start = longest_dist)
                else: # Perpendicular parking: case 1
                    rear_left, rear_right, front_right, front_left = self.set_corners(self.park_corners, case = 1, start = longest_dist)
            else: # Long distance to next corner
                if self.park_info.park_type == 1:  # Pararell parking: case 1
                    rear_left, rear_right, front_right, front_left = self.set_corners(self.park_corners, case = 1, start = longest_dist)
                else: # Perpendicular parking: case 0
                    rear_left, rear_right, front_right, front_left = self.set_corners(self.park_corners, case = 0, start = longest_dist)
        else: # Opposite lane, only perpendicular parking
            self.same_lane = False
            closet_dist = closest_corners[0]
            second_closet_dist = closest_corners[1]
            if (closet_dist+1)%4 != second_closet_dist: # case 0
                rear_left, rear_right, front_right, front_left = self.set_corners(self.park_corners, case = 0, start = closet_dist)
            else: # case 1
                rear_left, rear_right, front_right, front_left = self.set_corners(self.park_corners, case = 1, start = closet_dist)
     
        self.park_corners = np.array([rear_left, rear_right, front_right, front_left])

        self.park_type = self.park_info.park_type   # Type of the parking spot

    @staticmethod
    def is_same_lane(state, midpoint):
        if np.linalg.norm(state[:2]-midpoint[:2]) < 10 or True: #4: #TODO: If we are going to use oppisite lane parking; tune threshold
            return True
        else:
            return False

    @staticmethod
    def set_corners(park_corners, case, start):
        if case == 0: # Start is front right and move counterclockwise
            front_right = park_corners[start]
            front_left  = park_corners[(start + 1)%4]
            rear_left   = park_corners[(start + 2)%4]
            rear_right  = park_corners[(start + 3)%4]
        else: # Start is rear right and move counterclockwise
            rear_right  = park_corners[start]
            front_right = park_corners[(start + 1)%4]
            front_left  = park_corners[(start + 2)%4]
            rear_left   = park_corners[(start + 3)%4]
        return rear_left, rear_right, front_right, front_left

    def compute_waypoints(self):
        '''
        Compute the waypoints that the vehicle should follow to park according to if its a parallel or perpendicular parking spot
        '''
        self.parallel_park() if self.park_type == 1 else self.perpendicular_park()

    def perpendicular_park(self):

        self.waypoints = np.zeros((3, 3)) # Two waypoints of the form [x, y, theta]
        
        r_min = 0.4#0.25    # Minimum turning radius
        alpha = np.arctan2(self.park_corners[2, 1] - self.park_corners[1, 1], self.park_corners[2, 0] - self.park_corners[1, 0]) + np.pi/2

        # NOTE: We first compute waypoint 2 as waypoing 1 depends on it

        # Waypoint 2
        self.waypoints[1, 0] = (self.park_corners[3, 0] + self.park_corners[0, 0]) / 2 # Mid point between the two corners # TODO: Check if you need offset
        self.waypoints[1, 1] = (self.park_corners[3, 1] + self.park_corners[0, 1]) / 2 # Mid point between the two corners # TODO: Check if you need offset
        self.waypoints[1, 2] = alpha # TODO: Check if you need offset

        # Waypoint 1
        self.waypoints[0, 0] = self.waypoints[1, 0] + r_min * np.cos(alpha - np.pi/4) * math.sqrt(2)
        self.waypoints[0, 1] = self.waypoints[1, 1] + r_min * np.sin(alpha - np.pi/4) * math.sqrt(2)
        self.waypoints[0, 2] = (alpha - np.pi/2) + np.pi/4

        # Waypoint 3
        self.waypoints[2, 0] = (self.park_corners[1, 0] + self.park_corners[3, 0]) / 2 - self.dist_to_mid * np.cos(alpha)
        self.waypoints[2, 1] = (self.park_corners[1, 1] + self.park_corners[3, 1]) / 2 - self.dist_to_mid * np.sin(alpha)
        self.waypoints[2, 2] = alpha


    def parallel_park(self):
        """
        This method calculates waypoints dependend on some maths :D
        """

        r_min = 0.25            # Minimum turning radius [0.25m]
        width_car = 0.2485      # Width of the car
        length_car = 0.586      # Length of the car
        dist_to_middle = 0.133  # Distance from the middle of the car to the point considered as the state of the car

        # length_axes = 0.324; overshoot_back = 0.16; overshoot_front = 0.102 # Geometry of the car # TODO: Is this needed?

        parking_spot_width = np.linalg.norm(self.park_corners[1,:] - self.park_corners[0,:])    # Width of the parking spot
        parking_spot_length = np.linalg.norm(self.park_corners[2,:] - self.park_corners[1,:])   # Length of the parking spot
        alpha = np.arctan2(self.park_corners[2,1] - self.park_corners[1,1], self.park_corners[2,0] - self.park_corners[1,0])    # Angle parallel to the length of the parking spot
        beta = alpha + np.pi / 2    # Angle parallel to the width of the parking spot

        # safety
        safety_distance = 0.1
        safety_distance_y = 0.07
        safety_distance_end_point = 0.15

        # start parameter
        x_start = length_car / 2
        y_start = width_car / 2 + safety_distance_y
        borders_road = True
        park_inside = 0.25 # 0.15

        # initialize start / end
        start_point = np.zeros((1,3))
        end_point = np.zeros((1,3))
        o_1 = np.zeros((1,2))
        o_2 = np.zeros((1,2))

        # determine point at the front left of the parking spot
        start_point[0, 0] = self.park_corners[3, 0] + np.cos(beta) * y_start + np.cos(alpha) * x_start
        start_point[0, 1] = self.park_corners[3, 1] + np.sin(beta) * y_start + np.sin(alpha) * x_start
        start_point[0, 2] = alpha 

        # determine point at the back of the parking spot
        end_point[0, 0] = (self.park_corners[0, 0] + self.park_corners[1, 0]) / 2 + np.cos(alpha) * (length_car/2 - dist_to_middle + safety_distance_end_point)
        end_point[0, 1] = (self.park_corners[0, 1] + self.park_corners[1, 1]) / 2 + np.sin(alpha) * (length_car/2 - dist_to_middle + safety_distance_end_point)
        end_point[0, 2] = alpha    # TODO: Check if this is sufficient or if we need to calculate based on the parking spot
        if borders_road:
            end_point[0,0] = end_point[0,0] + np.cos(beta) * (parking_spot_width - width_car - park_inside) / 2
            end_point[0,1] = end_point[0,1] + np.sin(beta) * (parking_spot_width - width_car - park_inside) / 2

        o_1[0,0] = start_point[0,0] - np.cos(beta) * r_min
        o_1[0,1] = start_point[0,1] - np.sin(beta) * r_min

        o_2[0,0] = end_point[0,0] + np.cos(beta) * r_min
        o_2[0,1] = end_point[0,1] + np.sin(beta) * r_min

        # s1 = np.linalg.norm(end_point[0, :2] - self.park_corners[2,:]/2 + self.park_corners[3,:]/2)
        s1 = np.linalg.norm((self.park_corners[3,:]/2 + self.park_corners[2,:]/2) - end_point[0, :2] )

        if borders_road:
            p_temp = np.zeros((1,2))
            p_temp[0,0] = self.park_corners[3,0]/2 + self.park_corners[2,0]/2 + np.cos(beta) * (parking_spot_width - width_car - park_inside) / 2
            p_temp[0,1] = self.park_corners[2,1]/2 + self.park_corners[3,1]/2 + np.sin(beta) * (parking_spot_width - width_car - park_inside) / 2
            s1 = np.linalg.norm(end_point[0, :2] - p_temp)
        dist_o = np.linalg.norm(o_2 - o_1)

        # don't ask whats going on here pls
        theta = np.abs(np.arcsin( min(1.0, (x_start + s1)/dist_o) ) - np.arccos(2*r_min/dist_o))

        # initializes two intermediate waypoints
        temp_1 = np.zeros((1,3))
        temp_2 = np.zeros((1,3))

        angle_1 = alpha + theta + np.pi / 2
        temp_1[0,0] = o_1[0,0] + np.cos(angle_1) * r_min
        temp_1[0,1] = o_1[0,1] + np.sin(angle_1) * r_min
        temp_1[0,2] = alpha + theta


        angle_2 = alpha + theta - np.pi / 2
        temp_2[0,0] = o_2[0,0] + np.cos(angle_2) * r_min
        temp_2[0,1] = o_2[0,1] + np.sin(angle_2) * r_min
        temp_2[0,2] = alpha + theta

        goal = np.zeros((1,3))

        goal[0, 0] = (self.park_corners[0, 0] + self.park_corners[2, 0]) / 2 - np.cos(alpha) * (self.dist_to_mid*0.0)
        goal[0, 1] = (self.park_corners[0, 1] + self.park_corners[2, 1]) / 2 - np.sin(alpha) * (self.dist_to_mid*0.0)
        goal[0, 2] = alpha

        if borders_road:
            goal[0,0] = goal[0,0] + np.cos(beta) * (parking_spot_width - width_car - park_inside) / 2
            goal[0,1] = goal[0,1] + np.sin(beta) * (parking_spot_width - width_car - park_inside) / 2


        # put array together
        points = np.zeros((5,3))
        points[0,:] = start_point
        points[1,:] = temp_1
        points[2,:] = temp_2
        points[3,:] = end_point
        points[4,:] = goal

        self.waypoints = points



