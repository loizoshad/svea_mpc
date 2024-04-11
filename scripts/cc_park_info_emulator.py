#!/usr/bin/env python3

import rospy
from park.msg import ParkingInformation
from park.msg import RequestPath
from park.msg import ConvexShape
from park.msg import ConvexShapeList
from geometry_msgs.msg import Point
import numpy as np

def pub_path_cb(data):
    rospy.loginfo("Park_Information received request for new path")
    pub = rospy.Publisher("/parking_information", ParkingInformation, queue_size=10)

    park_corners_msg = ConvexShape() # Create empty ConvexShape message
    park_corners = park_corners_msg.points # Create empty list of points
    # Add points to list
    big_parking_spot = True

    # if big_parking_spot:
    #     park_corners.append(Point(-3.3, -4.3, 0))
    #     park_corners.append(Point(-2.907, -3.721, 0))
    #     park_corners.append(Point(-3.816, -3.103, 0))
    #     park_corners.append(Point(-4.21, -3.682, 0))
    # else:
    #     park_corners.append(Point(-3.15, -4.1, 0))
    #     park_corners.append(Point(-2.936, -3.786, 0))
    #     park_corners.append(Point(-3.515, -3.392, 0))
    #     park_corners.append(Point(-3.729, -3.707, 0))
    array = np.array([[2.61, 1.45], [2.89, 1.86], [2.31, 2.26], [2.03, 1.84]])
    park_corners = [Point(array[i,0], array[i,1], 0) for i in range(4)]

    park_corners_msg.points = park_corners # Add list of points to ConvexShape message


    msg = ParkingInformation()
    msg.header.frame_id = "parking_information"
    msg.header.stamp = rospy.Time.now()

    msg.vertices_parking_slot = park_corners_msg
    msg.park_type = 0   # 1 if parallel, 0 if perpendicular

    pub.publish(msg)

    rospy.loginfo("Park_emulator published new ParkingInformation")
    rospy.sleep(3)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/req_new_path", RequestPath, pub_path_cb)
    rospy.loginfo("CC_emulator is running")
    rospy.spin()


if __name__ == '__main__':
    listener()