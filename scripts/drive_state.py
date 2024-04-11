#! /usr/bin/env python3

import numpy as np
import rospy
from park.path_smoothing import smooth_path


class DriveState(object):
    def __init__(self, controller_manager = None, trajectory = [[0, 0]]) -> None:
        self.controller_manager = controller_manager
        self.trajectory = trajectory


    def run(self, vehicle_state: object)-> object:
        
        if self.trajectory[0][0] != vehicle_state.path[0][0]: # If the path has changed
            self.update_traj(vehicle_state.path)
           
        self.controller_manager.control_step() 

        if self.controller_manager.path_is_finished: 
            rospy.loginfo("Path finished")
            vehicle_state.path_finished = True

        return vehicle_state


    def update_traj(self, path: list) -> None:
        self.trajectory =  np.array(smooth_path(path)) # Generates trajectory and then smoothens it
        assert len(self.trajectory[0]) == len(self.trajectory[1])
        self.controller_manager.trajectory = self.trajectory
        
        self.controller_manager.controller.traj_x = np.asarray(self.trajectory)[:, 0]
        self.controller_manager.controller.traj_y = np.asarray(self.trajectory)[:, 1]
        self.controller_manager.controller.traj_yaw = np.asarray(self.trajectory)[:, 2]
        self.controller_manager.data_handler.update_traj(self.controller_manager.controller.traj_x,
                                                         self.controller_manager.controller.traj_y)

        rospy.loginfo("Trajectory generated")