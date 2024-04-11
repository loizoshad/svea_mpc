"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math

class PurePursuitController(object):

    k = 0.6*0.1  # look forward gain
    Lfc = 0.4 # 0.2*0.075  # look-ahead distance
    K_p = 1.0  # speed control propotional gain
    K_i = 0.2  # speed control integral gain
    L = 0.324  # [m] wheel base of vehicle

    # Use this flags to show how each change improves the performance
    enable_speed_control = False

    def __init__(self, vehicle_name='', dt=0.01, target_radius=0.2):
        self.vehicle_name = vehicle_name
        self.dt = dt
        self.state = None

        self.traj_x = []
        self.traj_y = []
        self.traj_yaw = []  # Needed for the interface with the DriveState class
        self.target = None
        self.target_radius = target_radius
        self.target_max_dist = 1.0 # Maximum distance to target before it is considered far away

        # initialize with 0 velocity
        self.target_velocity = 0.8
        self.error_sum = 0.0

        self.slow_nodes = 1 # how many nodes before end to slow down
        self.last_nodes = 0 # If we should slow down
        self.last_nodes_vel = 0.5 # [m]


    def compute_control(self, state, target=None):
        self.state = state
        if target:
            self.target = target
        elif self.target is None or self.target_reached or self.target_far_away:
            self.target = self.get_next_target()

        steering = self.compute_steering(state)
        velocity = self.compute_velocity(state)
        return steering, velocity


    def compute_steering(self, state):
        tx, ty = self.target
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        if state.v < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta


    def compute_velocity(self, state):
        
        if self.last_nodes and self.enable_speed_control:
            target_vel = self.last_nodes_vel
        else:
            target_vel = self.target_velocity

        error = target_vel - state.v    
        self.error_sum += error * self.dt
        P = error * self.K_p
        I = self.error_sum * self.K_i
        correction = P + I
        return target_vel + correction


    def get_next_target(self): # Find the next target point
        self.target_idx = self._calc_target_index()
        tx = self.traj_x[self.target_idx]
        ty = self.traj_y[self.target_idx]
        return (tx, ty)


    def _calc_target_index(self): 
        k = 0.1 #0.2# 0.6*0.1  # look forward gain
        Lfc = 0.2 #0.4 # 0.2*0.075  # look-ahead distance
        Lf = k * self.state.v + Lfc
        yaw_thresh = 62.5*math.pi/180 
                
        # Find index of nearest point 
        dx = [self.state.x - icx for icx in self.traj_x]
        dy = [self.state.y - icy for icy in self.traj_y]
        # err_yaw = [abs(self.state.yaw - icyaw) for icyaw in self.traj_yaw]

        distances = [abs(math.sqrt(idx ** 2 + idy ** 2) - k * self.state.v) for (idx, idy) in zip(dx, dy)]
        dist = min(distances) # Distance to closest point
        idx = distances.index(dist)

        # Make sure the distance to the closest point greater than Lf 
        while dist < Lf and (idx + 1) < len(self.traj_x): # TODO: Add check for yaw error
            dx = self.traj_x[idx + 1] - self.traj_x[idx]
            dy = self.traj_y[idx + 1] - self.traj_y[idx]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            idx += 1

        return idx


    @property
    def target_reached(self):
        dx = self.target[0] - self.state.x
        dy = self.target[1] - self.state.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        bearing = math.atan2(dy, dx)
        target_is_behind = abs(bearing - self.state.yaw) > math.pi / 2
        close_to_target = distance < self.target_radius
        return  target_is_behind or close_to_target

    @property
    def target_far_away(self): # If localization init is wrong, get new target when it's fixed
        dx = self.target[0] - self.state.x
        dy = self.target[1] - self.state.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance > self.target_max_dist

    # Not sure if this is needed
    def initialize_state(self, state):
        # self.state = state
        pass