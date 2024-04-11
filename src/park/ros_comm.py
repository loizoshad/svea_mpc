#! /usr/bin/env python3

'''
This class can be used to to subscribe and/or publish to ROS topics without dealing with the ROS API directly.

In order to subscribe from a topic call the respective 'get' function:
    Example: To subscribe to the topic '/city/speed_limit' call get_speed_limit()
    NameOfROSCommObject.get_speed_limit()

In order to publish to a topic call the respective function that contains the publisher of the desired topic:
    Example: To publish to the topic '/request_path' call request_path()
        NameOfROSCommObject.publish_request_path()
'''

import rospy
import numpy as np

from std_msgs.msg import Float32 # Used for Speed limit
from geometry_msgs.msg import PoseWithCovariance # Used for Request new path from current pose
from geometry_msgs.msg import PoseStamped
from svea_msgs.msg import VehicleState # Used to publish current vehicle state
from nav_msgs.msg import Path # Used to subscribe to path
from geometry_msgs.msg import PolygonStamped # Used to visualize the car in rviz
from geometry_msgs.msg import Point32 # Used to visualize the car in rviz

# from city import Obstacle # Used to publish detected obstacles
# from city import ObstacleList

class ROSComm(object):

    def __init__(self) -> None:
        # Define topic names
        self.path_topic = '/city/path'
        self.vehicle_state_topic = '/state'
        self.smooth_path_topic = '/smooth_path'

        # Initiate subscribers
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_callback)
        self.veh_state_sub = rospy.Subscriber(self.vehicle_state_topic, VehicleState, self.vehicle_state_callback)

        # Initiate publishers
        self.smooth_path_pub = rospy.Publisher(self.smooth_path_topic, Path, queue_size = 10)

        # Initialize variables
        self.path = Path()
        self.vehicle_state = VehicleState()
        

    def path_callback(self, msg):
        self.path = msg
        self.have_path = True

    def vehicle_state_callback(self, msg):
        self.vehicle_state = msg


    def get_path(self):
        return self.path

    def get_vehicle_state(self):
        return self.vehicle_state

    def pub_smooth_path(self, data):
        self.smooth_path_pub.publish(data)        
