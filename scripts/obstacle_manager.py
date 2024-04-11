#! /usr/bin/env python3

"""
Handling lidar data and obstacle detection. Runs as a publisher
in the background.
"""
import rospy
import numpy as np
from math import cos, sin, sqrt, radians, atan2, degrees
from threading import Thread
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from visualization_msgs.msg import Marker
from svea_msgs.msg import VehicleState
from std_msgs.msg import Float32
from el2425_standards.msg import ConvexShapeList, ConvexShape

from svea.simulators.viz_utils import publish_lidar_points


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class ObstacleManager(object):
    """Manager that runs in the background and keeps track of obstacles mainly
    though the lidar interface. When needed it slows down the vehichle though
    a speed multiplyer published as a topic that hte SVEA subscribes to.

    :param vehicle_name: Name of vehicle lidar is attached to
    :type vehicle_name: str, optional
    """

    LIDAR_MAX = 5  # m
    UPDATE_TIME = 0.1 
    SCAN_TIME = 0.025  # time between scans/between publication [seconds]
    # dist between SVEA rear axle and lidar mount point [m]
    LIDAR_OFFSET = 0.30
    ZONE_DANGER_RAD = radians(14)  # danger zone +- 23%
    ZONE_CAUTION_RAD = radians(44)  # caution zone +- 45%
    OBST_RAD = 0.5  # m
    OBST_MAX = 10  # m
    ROUND = 5
    SWELL = radians(3)

    DZ = {
        "slow": 3,              # Time to target [seconds]
        "slow_width": 2,        # Determines width of bubble
        "stop": 1,              # Time to target [seconds]
        "stop_width": 0.6,      # Determines width of bubble
        "stop_dist": 0.5,       # Distance to target [meters]
        "stop_dist_width": 0.4, # Determines width of bubble
        "stop_dist_infra": 2,   # Distance to target [meters]
        "stop_dist_infra_width": 2, # Determines width of bubble
        "stop_speed": 0.2       # Minimum non-zero speed [m/s]
    }

    def __init__(self, vehicle_name='') -> None:

        # Initialize node
        rospy.init_node('obstacle_manager')

        # Parameters
        self.USE_PLOT = load_param('~use_lidar_plot', False)

        # publisher for lidar evaluated pointclouds
        sub_ns = vehicle_name + '/' if vehicle_name else ''
        self._zone_slow_topic = sub_ns + 'viz_zone_slow'
        self._zone_stop_topic = sub_ns + 'viz_zone_stop'
        self._zone_stop_dist_topic = sub_ns + 'viz_zone_stop_dist'
        self._zone_stop_dist_infra_topic = sub_ns + 'viz_zone_stop_infra_dist'

        self._infra_dist_topic = sub_ns + 'viz_infra_dist'

        if vehicle_name:
            self.vehicle_name = vehicle_name
        else:
            namespace = rospy.get_namespace()
            self.vehicle_name = namespace.split('/')[-2]

        self._lidar_position = None  # [x, y, yaw] -> [m, m, rad]
        # self._lidar_position = [-7.4, -15.3, 0.9] # for debugging
        self._obstacles = []
        self.latest_scan_msg = None
        self._obstacles_infra = []
        self.scan = None
        self.setup_done = False
        self.obst_distance = None
        self.infra_obstacles = []

        self._speed_multiplier = 1.0

    ##############################
    #           Setup            #
    ##############################

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: SimLidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Obstacle Management Node")
        # to publish the obstacle data
        self._start_publish()
        # to get the state of the car
        self._start_listener()
        rospy.loginfo("Obstacle manager successfully initialized")
        self._run_obstacle_manager()
        rospy.spin()


    def _start_listener(self):
        # Start subscriber for the state data
        rospy.Subscriber('state', VehicleState, self._update_lidar_position,
                         tcp_nodelay=True)
        # Start the subscriber for the lidar scan data
        rospy.Subscriber('scan', LaserScan, self._new_scan,
                         tcp_nodelay=True)
        # Start the subscriber for the city obstacles
        rospy.Subscriber('obstacles_infra', ConvexShapeList,
                         self._new_infra_obstacles, tcp_nodelay=True)

    def _start_publish(self):
        # publish speed multiplier 
        self._speed_multiplier_pub = rospy.Publisher('/speed_multiplier',
                                                     Float32, queue_size=1,
                                                     tcp_nodelay=True)
        # publish zones 
        self._zone_slow_pub = rospy.Publisher(
            self._zone_slow_topic, PolygonStamped, 
            queue_size=1, tcp_nodelay=True)
        
        self._zone_stop_pub = rospy.Publisher(
            self._zone_stop_topic, PolygonStamped, 
            queue_size=1, tcp_nodelay=True)

        self._zone_stop_dist_pub = rospy.Publisher(
            self._zone_stop_dist_topic, PolygonStamped, 
            queue_size=1, tcp_nodelay=True)

        self._zone_stop_dist_infra_pub = rospy.Publisher(
            self._zone_stop_dist_infra_topic, PolygonStamped, 
            queue_size=1, tcp_nodelay=True)

        # publish distance 
        self._infra_dist_pub = rospy.Publisher(self._infra_dist_topic, 
            PointCloud, queue_size=1, tcp_nodelay=True)
        

    def _publish_speed_multiplier(self, speed_multiplier):
        self._speed_multiplier_pub.publish(speed_multiplier)

    def _publish_viz_points(self):
        publish_lidar_points(self._viz_points_safe_pub,
                             self.lidar_obstacles[self.safe_index])
        publish_lidar_points(self._viz_points_caution_pub,
                             self.lidar_obstacles[self.caution_index])
        publish_lidar_points(self._viz_points_danger_pub,
                             self.lidar_obstacles[self.danger_index])

    def _run_obstacle_manager(self):
        rate = rospy.Rate(1/self.UPDATE_TIME)
        while not rospy.is_shutdown():
            if self._lidar_position is None or self.setup_done is False:
                rate.sleep()
                continue
            # update the lidar bstacles 
            self._update_lidar_obstacle()
            # update the infra obstacles 
            self._update_infra_obstacle()
            # update speed multiplier
            self._check_caution_level()
            # pulbish visualization
            self._rviz_publisher()
            rate.sleep()

    ##############################
    #    callback functions      #
    ##############################

    def _update_lidar_position(self, state_msg):
        """Updates the lidar position using the vehicle state and the
        known offset between the SVEA rear axle and lidar mount

        :param vehicle_state: State of vehicle lidar is attached to
        :type vehicle_state: VehicleState
        """
        # get x and y values from latest state msg
        self.state_v = state_msg.v
        self.vehicle_xy = np.array([state_msg.x, state_msg.y])
        yaw = state_msg.yaw
        # get the lidar offset and calculate rotation
        offset = [self.LIDAR_OFFSET, 0.0]
        rot = np.matrix([[np.cos(-yaw), np.sin(-yaw)],
                         [-np.sin(-yaw), np.cos(-yaw)]])
        rot_offset = np.dot(rot, offset).tolist()[0]
        lidar_xy = self.vehicle_xy + np.array(rot_offset)
        # add position and yaw together
        self._lidar_position = np.append(lidar_xy, yaw)

    def _new_scan(self, scan_msg):
        ''' Callback function that runs every time there is a new
        LIDAR scan available.
        '''
        # udpate scan varaibles
        self._read_scan(scan_msg)
        # Check that we have a position
        if self._lidar_position is None:
            return
        # perform lidar setup
        if self.setup_done is False:
            self._first_scan_setup()

    def _new_infra_obstacles(self, infra_msg):
        ''' Callback function that runs every time we have a new
        obstacle from infrastructure team.
        '''
        self.infra_obstacles = infra_msg.locations  

    def _read_scan(self, scan_msg):
        # store latest scan data
        self.latest_scan_msg = scan_msg
        self.lidar_distance = scan_msg.ranges
        # store angle data
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        # store time data
        self.time_increment = scan_msg.time_increment
        self.last_scan_time = scan_msg.scan_time

    def _first_scan_setup(self):
        rospy.loginfo("First LIDAR scan recived!")
        rospy.loginfo("=> Setting up obstacle mangager")
        rospy.loginfo("=> angle_min: {:.5} angle_max: {:.5} ".format(
            self.angle_min, self.angle_max))
        rospy.loginfo("====== calculating angle index ======")

        # generate lidar angle array
        self.rounded_lidar_increment = round(self.angle_increment, self.ROUND)

        pos_angles = np.arange(0, self.angle_max, self.rounded_lidar_increment)
        neg_angles = np.flip(
            np.arange(-self.rounded_lidar_increment, self.angle_min,
                      -self.rounded_lidar_increment))
        self.lidar_angles = np.concatenate((neg_angles, pos_angles))
        self.lidar_angles = np.round_(self.lidar_angles, self.ROUND)

        self.stop_dist = np.clip(self._zone_quadratic(
            self.lidar_angles,
            self.DZ['stop_dist'],
            self.DZ['stop_dist_width']), 0, self.DZ['stop_dist'])

        self.stop_dist_infra = np.clip(self._zone_quadratic(
            self.lidar_angles,
            self.DZ['stop_dist_infra'],
            self.DZ['stop_dist_infra_width']), 0, self.DZ['stop_dist_infra'])

        self.setup_done = True
        rospy.loginfo("========= LIDAR setup done =========")
        return

    ##############################
    #      update functions      #
    ##############################

    def _update_infra_obstacle(self):
        """Read the nw obstacles gotte from the infrastructure team"""
        def point_to_ang_dist(point):
            delta_x = point[0] - self._lidar_position[0]
            delta_y = point[1] - self._lidar_position[1]
            dist = sqrt(delta_x**2 + delta_y**2)

            delta_theta = atan2(delta_y, delta_x)
            theta = delta_theta - self._lidar_position[2]
            return dist, theta

        def round_to_multiple(number, multiple):
            return multiple * round(number / multiple)

        def gen_circle_points(point, rad):
            obst_x = point.x
            obst_y = point.y
            angles = np.arange(0, 2*np.pi, 2*np.pi/12)

            points = []
            for theta in angles:
                obst_point = (obst_x + cos(theta) * rad,
                              obst_y + sin(theta) * rad)
                points.append(obst_point)

            return points

        def store_point(dist, theta):
            # get adjusted theta
            theta = round_to_multiple(theta, self.rounded_lidar_increment)
            theta = round(theta, self.ROUND)
            # check if in lidar range
            if theta > self.angle_max or \
                    theta < self.angle_min:
                return
            
            # get angle index
            ind = np.where(self.lidar_angles == theta)[0]
            for i in ind:
                for j in range(-swell,swell):
                    k = i + j 
                    if k >= 0 and k < self.obst_distance.shape[0]:
                        if self.obst_distance[k] > dist:
                            self.obst_distance[k] = dist


        # create empty obstacle array and get integer degree angles
        self.obst_distance = np.ones(shape=self.lidar_angles.shape)*self.OBST_MAX
        swell = int(self.SWELL/self.rounded_lidar_increment)
        
        for obtacle in self.infra_obstacles:
            obstacle_points = obtacle.points

            for obtacle_point in obstacle_points:
                locat_dist, locat_theta = point_to_ang_dist(
                    (obtacle_point.x, obtacle_point.y))
                if locat_dist >= self.OBST_MAX:
                    store_point(locat_dist, locat_theta)
                    continue

                point_cloud = gen_circle_points(obtacle_point, self.OBST_RAD)
                for c_point in point_cloud:

                    c_dist, c_theta = point_to_ang_dist(c_point)
                    store_point(c_dist, c_theta)

    def _update_lidar_obstacle(self):
        """Reads the latest LiDAR scan and calculates the coordinates for
        each angle.
        """
        # generate obstacels
        self.lidar_obstacles = np.array([self.calc_point(angle, dist)
                                         for angle, dist in zip(self.lidar_angles,
                                                     self.lidar_distance)])

        self.relative_v = np.clip(
            self.state_v * np.cos(self.lidar_angles), 1e-3, self.LIDAR_MAX)

        self.slow_zone = np.clip(self._zone_quadratic(
            self.lidar_angles,
            self.relative_v*self.DZ['slow'],
            self.DZ['slow_width']), 0, self.DZ['slow'])

        self.stop_zone = np.clip(self._zone_quadratic(
            self.lidar_angles,
            self.relative_v*self.DZ['stop'],
            self.DZ['stop_width']), 0, self.DZ['stop'])

        self.time_to_target = self.lidar_distance / self.relative_v

    def calc_point(self, angle, dist, ros=False):
        lidar_x = self._lidar_position[0]
        lidar_y = self._lidar_position[1]
        lidar_heading = self._lidar_position[2]

        if ros:
            point = Point32()
            point.x = lidar_x + cos(lidar_heading + angle) * dist
            point.y = lidar_y + sin(lidar_heading + angle) * dist
            point.z = self.LIDAR_OFFSET
        else:
            point = (lidar_x + cos(lidar_heading + angle) * dist,
                     lidar_y + sin(lidar_heading + angle) * dist)
        return point

    def _check_caution_level(self):
        """ Calculate the distance to obtacles in different zones and
        determin what precautions that should be taken.
        """
        def calc_speed_multi(time_to_target, slow, stop, stop_speed):
            # m*x + c
            m = ((1 - stop_speed) / (slow - stop))
            x = (time_to_target - stop)
            c = stop_speed
            return m*x + c

        if self.obst_distance is not None:
            rel_stop_dist_infra = self.obst_distance - self.stop_dist_infra
            ind_std_infra = np.nanargmin(rel_stop_dist_infra)

            if rel_stop_dist_infra[ind_std_infra] <= 0.0:
                self._speed_multiplier = 0.0
                self._publish_speed_multiplier(self._speed_multiplier)
                return

        np_dist = np.array(self.lidar_distance)
        np_ttt = np.abs(self.time_to_target)

        rel_stop_dist = np_dist - self.stop_dist
        rel_stop_zone = np_ttt - self.stop_zone
        rel_slow_zone = np_ttt - self.slow_zone
        ind_std = np.nanargmin(rel_stop_dist)
        ind_stz = np.nanargmin(rel_stop_zone)
        ind_slz = np.nanargmin(rel_slow_zone)

        if rel_stop_dist[ind_std] <= 0.0:
            self._speed_multiplier = 0.0
        elif rel_stop_zone[ind_stz] <= 0.0:
            self._speed_multiplier = 0.01
        elif rel_slow_zone[ind_slz] <= 0.0:
            self._speed_multiplier = calc_speed_multi(
                time_to_target=np_ttt[ind_slz], slow=self.DZ['slow'],
                stop=self.DZ['stop'], stop_speed=self.DZ['stop_speed'])
        else:
            self._speed_multiplier = 1.0
        self._publish_speed_multiplier(self._speed_multiplier)

    def _dist(self, pt1, pt2):
        return sqrt((pt1[0] - pt2[0]) ** 2
                    + (pt1[1] - pt2[1]) ** 2)

    def _zone_quadratic(self, thetas, cond, mult):
        rel_stop_dist = -mult*np.power(thetas, 2) + cond
        return rel_stop_dist


    ##############################
    #       visualization        #
    ##############################

    def _rviz_publisher(self):

        # Publish slow zone to rviz
        self._rviz_publish_zone(
            self.slow_zone,
            self._zone_slow_pub
        )
        
        # Publish stop zone to rviz
        self._rviz_publish_zone(
            self.stop_zone,
            self._zone_stop_pub
        )

        # Publish stop dist zone to rviz
        self._rviz_publish_zone(
            self.stop_dist,
            self._zone_stop_dist_pub
        )
        # Publish stop dist infra zone to rviz
        self._rviz_publish_zone(
            self.stop_dist_infra,
            self._zone_stop_dist_infra_pub
        )

        # Publish infra obstacle cloud  
        self._rviz_publish_cloud(
            self.obst_distance, 
            self._infra_dist_pub
        )        
    
    def _rviz_publish_zone(self, distance, publisher):
        poly_msg = Polygon()
        poly_msg.points = [self.calc_point(angle, dist, ros=True) for angle, dist in 
                            zip(self.lidar_angles, distance)]
        
        polystmped_msg = PolygonStamped()
        polystmped_msg.header.frame_id = "map"
        polystmped_msg.polygon = poly_msg
        publisher.publish(polystmped_msg)
        return

    def _rviz_publish_cloud(self, distance, publisher):
        points = np.array([self.calc_point(angle, dist, ros=True) for angle, dist in 
                            zip(self.lidar_angles, distance)])
        ind = np.where(distance < self.OBST_MAX)[0]
        cloud = PointCloud()
        cloud.header.frame_id = "map"
        cloud.points = points[ind]

        publisher.publish(cloud)
        return


if __name__ == '__main__':
    # Start Node
    ObstacleManager().start()
