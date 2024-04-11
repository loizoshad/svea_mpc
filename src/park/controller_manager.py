#! /usr/bin/env/python3

'''
This code is a modified version of the code found in 'path_following_sveas.py in the svea_core package.
'''


'''
For a control class to be supported it needs to fullfill the following requirements:

1. The class must have a method called 'compute_control' that takes a state as input and returns a control as output.
    def compute_control(self, state=None):
        # Input : state (from svea.states import VehicleState)
        # Output: steering (float), velocity (float)
        return steering, velocity
2. The class must have a variable called 'target' that is a tuple of the form (x,y) and is updated by the compute_control method.
3. The class must have a variable called 'target_velocity' that is a float and is updated by the compute_control method.
4. The class must have a variable called 'yaw_target'.
5. The class must have a variable called 'traj_x' that is a list of floats and is updated by the compute_control method.
6. The class must have a variable called 'traj_y' that is a list of floats and is updated by the compute_control method.
7. The class must have a variable called 'path_is_finished' that is a boolean and is updated by the compute_control method.
8. The class must have a method called 'initialize_state'
''' #TODO: Update this comment

import rospy
import math
import numpy as np

from svea.svea_managers.svea_archetypes import SVEAManager
from svea.data import TrajDataHandler


#TODO: Merge with drive_state.py

class ControllerManager(SVEAManager):

    def __init__(self, localizer, controller, traj_x = None, traj_y = None,
                 data_handler = TrajDataHandler, vehicle_name='', init_state = None, vehicle_state = None, 
                 goal_threshold_pos=0.1, goal_threshold_yaw=6*math.pi/180):

        SVEAManager.__init__(self, localizer, controller,
                             data_handler = data_handler,
                             vehicle_name = vehicle_name)

        self.controller.initialize_state(init_state)
        self.vehicle_state = vehicle_state
        self.trajectory = [[]]
        self.goal_threshold_pos = goal_threshold_pos
        self.goal_threshold_yaw = goal_threshold_yaw
        self.new_traj = True
        self.path_finished = False

        rospy.loginfo("Control type: " + self.controller.__class__.__name__)


    def control_step(self):

        # Compute control
        steering, velocity = self.controller.compute_control(self.state)
        velocity = velocity * self.vehicle_state.speed_guard.multiplier
        self.send_control(steering, velocity)

        self.data_handler.update_target(self.controller.target) # Update target plot in rviz   

    @property
    def path_is_finished(self):
        end_point = self.trajectory[-1]
        pos_err = math.sqrt((self.state.x - end_point[0])**2 + (self.state.y - end_point[1])**2)
        yaw_err = abs(self.state.yaw - end_point[2])
        return pos_err < self.goal_threshold_pos and yaw_err < self.goal_threshold_yaw
