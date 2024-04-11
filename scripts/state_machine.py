#! /usr/bin/env python3

import rospy

from rospy import Publisher, Rate
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
from svea.data import TrajDataHandler, RVIZPathHandler
from svea.states import VehicleState as vhs
from geometry_msgs.msg import PoseWithCovarianceStamped

from svea_msgs.msg import VehicleState as VehicleStateMsg
from tf.transformations import quaternion_from_euler

from standby_state import StandbyState
from drive_state import DriveState
from park_state import ParkState
# from unpark_state import UnparkState

from park.dynamic_model import DynamicModel
from park.speed_guard import SpeedGuard
from park.controller_manager import ControllerManager
from park.pure_pursuit_custom import PurePursuitController
from park.nonlinear_mpc import NonlinearMPC
from park.nonlinear_mpc_parking import NonlinearParkingMPC
from svea.svea_managers.path_following_sveas import SVEAPurePursuit # TODO: Check if necessary
from park.nonlinear_mpc_parking import NonlinearParkingMPC

def publish_initialpose(state, n=40):

    p = PoseWithCovarianceStamped()
    p.header.frame_id = 'map'
    p.pose.pose.position.x = state.x
    p.pose.pose.position.y = state.y

    q = quaternion_from_euler(0, 0, state.yaw)
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]

    pub = Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = Rate(10)

    for _ in range(n):
        pub.publish(p)
        rate.sleep()

class VehicleState: # keep track of global information about the vehicle

    def __init__(self, current_state = "standby"):
        '''
        - param self.path: list of points to follow
        - type self.path: list of 2-element lists
        '''
        self.pose = [0, 0, 0, 0] # x, y, yaw, speed
        rospy.Subscriber("/state", VehicleStateMsg, self.vehicle_pose_callback)

        self.speed_guard = SpeedGuard()         # Keep track of speed and stop if too high
        self.speed_guard.start()                # Start the speed guard thread

        self.current_state = current_state
        self.last_state = None
        
        # Flags for state transitions
        self.is_parked = False
        self.is_on_road = True # TODO: Might want to make this depend on the localization node
        self.has_path = False
        self.path_finished = False
        self.path_request_sent = False
        self.obstacle = False

        self.path = None

    def vehicle_pose_callback(self, msg):
        self.pose = [msg.x, msg.y, msg.yaw, msg.v]


class StateMachine:

    def __init__(self):

        ## Initialize node

        rospy.init_node('state_machine')

        # The ROS node needs to be initialized before using rospy.Rate() 
        self.rate = rospy.Rate(10)

        # Auxiliary objects
        self.vehicle_state = VehicleState()    # Keep track of global information about the vehicle
        

        self.IS_SIM = load_param('~is_sim', False)
        self.RUN_LIDAR = load_param('~run_lidar', True)
        self.USE_RVIZ = load_param('~use_rviz', True)
        self.CTRL_TYPE = load_param('~ctrl_type')

        # Extract initial state
        self.STATE = load_param('~state')
        # initial state
        state = vhs(*self.STATE)
        publish_initialpose(state)


        # Start SVEA manager
        self.ctrl_manager = ControllerManager(  LocalizationInterface, 
                                                NonlinearMPC if self.CTRL_TYPE == 'NLMPC' else PurePursuitController,
                                                data_handler = RVIZPathHandler if self.USE_RVIZ else TrajDataHandler, init_state = None,
                                                vehicle_state = self.vehicle_state)

        self.park_manager = ControllerManager(  LocalizationInterface, 
                                                NonlinearParkingMPC,
                                                data_handler = RVIZPathHandler if self.USE_RVIZ else TrajDataHandler, init_state = None,
                                                vehicle_state = self.vehicle_state)

        # Initialize objects of state machine states
        self.standby_state = StandbyState(obstacle_timeout=10)
        self.drive_state = DriveState(controller_manager = self.ctrl_manager)
        self.park_state = ParkState(controller_manager = self.park_manager)
        # self.unpark_state = UnparkState()
        
        # Initiate Simulation
        if self.IS_SIM:
            
            # Dynamic model for simulation purposes
            self.sim_model = DynamicModel(state)

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=0.01,
                                     run_lidar=self.RUN_LIDAR,
                                     start_paused=False).start()


        # Initiate svea manager control interface
        self.ctrl_manager.start(wait=True)
        self.park_manager.start(wait=True)


    def run(self):
        while not rospy.is_shutdown():
            self.spin()


    def spin(self):

        # Check obstacles
        self.vehicle_state.obstacle = self.vehicle_state.speed_guard.check_obstacles()
        if self.vehicle_state.obstacle and not self.vehicle_state.current_state == "standby":
            self.transition("standby")


        # Run current state
        if self.vehicle_state.current_state == "standby":
            self.vehicle_state = self.standby_state.run(self.vehicle_state)
            if self.vehicle_state.is_parked:
                self.transition("unpark")
            elif self.vehicle_state.has_path and not self.vehicle_state.obstacle:
                self.transition("drive")
            self.ctrl_manager.visualize_data()

        elif self.vehicle_state.current_state == "drive":
            self.vehicle_state = self.drive_state.run(self.vehicle_state) 
            if self.vehicle_state.path_finished:
                self.transition("park")
            self.ctrl_manager.visualize_data()

        elif self.vehicle_state.current_state == "park":
            self.vehicle_state = self.park_state.run(self.vehicle_state) # Should set is_parked to true when done
            if self.vehicle_state.is_parked:
                self.transition("standby")
                self.vehicle_state.has_path = False
                self.vehicle_state.path_finished = False
                rospy.loginfo("Parking successful")
            self.park_manager.visualize_data()

        elif self.vehicle_state.current_state == "unpark":
            self.vehicle_state = self.unpark_state.run(self.vehicle_state) # Should set is_on_road to true when done
            if self.vehicle_state.is_on_road:
                self.transition("standby")
        else:
            rospy.logerr("Invalid state: " + self.vehicle_state.current_state)
            self.vehicle_state.current_state = "standby"
            self.ctrl_manager.visualize_data()

        # self.ctrl_manager.visualize_data()

        self.rate.sleep()


    def transition(self, new_state):
        self.vehicle_state.last_state = self.vehicle_state.current_state
        self.vehicle_state.current_state = new_state
        rospy.loginfo("Transitioning to state: " + self.vehicle_state.current_state)



def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


if __name__ == '__main__':

    StateMachine().run()
