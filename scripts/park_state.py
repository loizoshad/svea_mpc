#! /usr/bin/env python3

import numpy as np
import rospy

import math

from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from park.msg import ParkingInformation
from park.path_smoothing import smooth_path
from park.park_waypoint_generator import WaypointGenerator
        
class ParkState(object):
    def __init__(self, controller_manager = None, trajectory = [[0, 0]]) -> None:
        self.controller_manager = controller_manager
        self.trajectory = trajectory
        self.park_inf_received = False
        self.park_spot_corners = None
        self.park_spot_type = None

        self.waypoint_generator = WaypointGenerator()


        rospy.Subscriber('/parking_information', ParkingInformation, self.parking_information_callback)


    def parking_information_callback(self, parking_information: ParkingInformation) -> None:
            rospy.loginfo('Parking Information')
            # Unwrap the points from the message and convert them to a numpy array
            self.park_spt_corners = np.array([[point.x, point.y] for point in parking_information.vertices_parking_slot.points]) # TODO: remove when you adapt the controller to the changes
            self.park_spt_type = parking_information.park_type      # TODO: remove when you adapt the controller to the changes
            self.park_info = parking_information
            self.park_inf_received = True

    def run(self, vehicle_state: object)-> object:
        if self.park_inf_received:   # If we have received parking information
            self.waypoint_generator.generate_waypoints(state = vehicle_state.pose, park_info = self.park_info)
            ellipses_data = self.waypoint_generator.get_ellipses() 
            self.controller_manager.controller.setup_parking_data(ellipses_data)    # Pass the ellipses information to the controller
            self.update_traj(self.waypoint_generator.waypoints)

        # Publish the ellipses for visualization purposes
        for i in range(3):
            self.waypoint_generator.ellipses[i].publish_ellipse()

        self.controller_manager.control_step()

        if self.controller_manager.path_is_finished: 
            rospy.loginfo("Parking finished")
            vehicle_state.is_parked = True

        return vehicle_state




    def update_traj(self, path: list) -> None:
        # self.trajectory =  np.array(smooth_path(path = path, smoothing = False)) # Generates trajectory and then smoothens it
        self.trajectory = path
        # print(f'path = {self.trajectory}')
        self.park_inf_received = False
        assert len(self.trajectory[0]) == len(self.trajectory[1])
        self.controller_manager.trajectory = self.trajectory
        
        self.controller_manager.controller.traj_x = np.asarray(self.trajectory)[:, 0]
        self.controller_manager.controller.traj_y = np.asarray(self.trajectory)[:, 1]
        self.controller_manager.controller.traj_yaw = np.asarray(self.trajectory)[:, 2]
        self.controller_manager.data_handler.update_traj(self.controller_manager.controller.traj_x,
                                                         self.controller_manager.controller.traj_y)

        rospy.loginfo("Trajectory for parallel parking generated")




                                    

            