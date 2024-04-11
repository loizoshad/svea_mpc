#! /usr/bin/env python3

"""Class to be run as a node to simulate getting obstacles from infrastructure
"""

from threading import Thread
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PointStamped
from el2425_standards.msg import ConvexShapeList, ConvexShape


class InfraSim():
    """
    Used to send simulated obstacles detected by camera
    """
    UPDATE_TIME = 0.1  # s

    def __init__(self):
        self.topic = "/obstacles_infra"
        self.point_list = []

        # Initialize node
        rospy.init_node('infra_sim')

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Infra Obstacle Simulation Node")
        self._start_publish()
        self._start_listener()
        rospy.loginfo("Simulated Infra successfully initialized")
        self._start_simulation()
        rospy.spin()

    def _start_listener(self):
        # Start subscriber for the state data
        rospy.Subscriber('clicked_point', PointStamped, self._update_point,
                         tcp_nodelay=True)

    def _start_publish(self):
        self._infra_pub = rospy.Publisher(self.topic, ConvexShapeList,
                                          queue_size=1, tcp_nodelay=True)

    def publish_obstacle(self, obstacle_list):
        self._infra_pub.publish(obstacle_list)

    def _start_simulation(self):
        rate = rospy.Rate(1/self.UPDATE_TIME)
        while not rospy.is_shutdown():
            self._sim_obstacle()
            rate.sleep()

    def _update_point(self, point_msg):
        self.point_list = [point_msg.point]
        print(point_msg.point)

    def _sim_obstacle(self):
        obst_list = []
        for point in self.point_list:

            obst = ConvexShape()
            obst.points = [point]
            obst_list.append(obst)

        convex_shape_list = ConvexShapeList()
        convex_shape_list.locations = obst_list

        self.publish_obstacle(convex_shape_list)


if __name__ == '__main__':
    # Start node
    InfraSim().start()
