#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from park.msg import RequestPath

def pub_path_cb(data):
    rospy.loginfo("CC_emulator received request for new path")

    # TODO: adjust waypoints for specific parking testing cases
    # waypoints = [
    #     [-7.4, -15.3],
    #     [-2.6, -7.39],
    #     [10.3, 11.7],
    #     [5.72, 14.6],
    #     [-5.2, -1.29],
    #     [-3.1, -2.88]
    # ]
    # waypoints = [[-5.43, -1.71], [-3.249, -3.246]] # for parallel testing
    waypoints = [[3.66, 4.78], [4.74, 4.05], [3.02, 1.47]]

    pub = rospy.Publisher("/path", Path, queue_size=10)

    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    msg.poses = [PoseStamped() for i in range(len(waypoints))]

    for i in range(len(waypoints)):
        msg.poses[i].pose.position.x = waypoints[i][0]
        msg.poses[i].pose.position.y = waypoints[i][1]

    pub.publish(msg)
    
    rospy.loginfo("CC_emulator published new path")
    rospy.sleep(3)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/req_new_path", RequestPath, pub_path_cb)
    rospy.loginfo("CC_emulator is running")
    rospy.spin()

if __name__ == '__main__':
    listener()
