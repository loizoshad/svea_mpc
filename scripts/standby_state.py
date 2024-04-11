import rospy 
from park.msg import RequestPath
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


class StandbyState():
        
    def __init__(self, path_request_timeout = 5, obstacle_timeout = 5):
        self.path_request_time = None
        self.path_request_timeout = path_request_timeout
        self.path_tmp = None
        self.path_received = False
        self.path_request_publisher = rospy.Publisher('/req_new_path', RequestPath, queue_size=1)
        self.point_cloud_msg = None
        rospy.Subscriber('/path', Path, self.path_received_callback)
        rospy.Subscriber('/viz_lidar_points', PointCloud, self.point_cloud_callback, queue_size=1)
        self.obstacle_time = None
        self.obstacle_timeout = obstacle_timeout
        

    def run(self, vehicle_state: object) -> object:

        if not (vehicle_state.is_on_road or vehicle_state.is_parked):
            rospy.logwarn("Vehicle is not on road or parked")
            return vehicle_state

        if not vehicle_state.has_path: #TODO: Handle if new path is received without requests
            if not vehicle_state.path_request_sent:
                self.request_new_path(vehicle_state.pose)
                vehicle_state.path_request_sent = True
                self.path_request_time = rospy.get_time()
            elif rospy.get_time() - self.path_request_time > self.path_request_timeout:
                rospy.logwarn("Path request timed out")
                vehicle_state.path_request_sent = False
            elif self.path_received: # Flag set by path subscriber callback
                if self.path_tmp != vehicle_state.path:
                    vehicle_state.path = self.path_tmp
                    vehicle_state.has_path = True
                else:
                    rospy.logwarn("Received path is the same as current path")
                self.path_received = False
                vehicle_state.path_request_sent = False
        elif vehicle_state.obstacle:
            if not self.obstacle_time: # First time obstacle is detected
                self.obstacle_time = rospy.get_time()
            elif rospy.get_time() - self.obstacle_time > self.obstacle_timeout:
                rospy.logwarn("Obstacle timed out") 
                vehicle_state.obstacle = False
                self.obstacle_time = None
                vehicle_state.has_path = False
                vehicle_state.path_request_sent = False
        
        elif self.obstacle_time:
            self.obstacle_time = None
                
        return vehicle_state


    def request_new_path(self, vehicle_pose: list) -> None:

        rospy.loginfo("Requesting new path from vehicle pose: " + str([round(num,3) for num in vehicle_pose]))
        path_request = RequestPath()

        path_request.pose.pose.position.x = vehicle_pose[0]
        path_request.pose.pose.position.y = vehicle_pose[1]
        q = quaternion_from_euler(0, 0, vehicle_pose[3])
        path_request.pose.pose.orientation.z = q[2]
        path_request.pose.pose.orientation.w = q[3]

        if self.point_cloud_msg:
            path_request.pointcloud = self.point_cloud_msg

        self.path_request_publisher.publish(path_request)


    def path_received_callback(self, path: Path) -> None:  
        rospy.loginfo("Path received")
        positions = [pose_stamped.pose.position for pose_stamped in path.poses]
        self.path_tmp = [[position.x, position.y] for position in positions]
        self.path_received= True


    def point_cloud_callback(self, point_cloud: PointCloud) -> None:
        self.point_cloud_msg = point_cloud


    