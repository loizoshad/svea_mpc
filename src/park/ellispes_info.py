#! /usr/env/bin python3

import numpy as np
import math
import rospy
import time

from geometry_msgs.msg import PolygonStamped, Point32

'''
TODO: Add docstring
'''

class EllipseInfo(object):

    # Index of Ellipse created
    index = 0

    def __init__(   self,
                    f1 = None,
                    f2 = None,
                    inflation = 1.0,
                    vis_points = None,
                    resolution = 100,
                    c1 = None,
                    c2 = None,) -> None:
        '''
        - param f1:         focal point 1
        - type f1:          np.array([x, y]) [m, m]
        - param f2:         focal point 2
        - type f2:          np.array([x, y]) [m, m]
        - param inflation:  Inflation of ellipse in each direction 
        - type inflation:   float [0.0, 1.0]
            - max 1.0 -> Circle, 
            - min 0.0 -> Line        
        - param vis_points: Points for visualization
        - type vis_points:  geometry_msgs/PolygonStamped
        - param resolution: Number of points to generate for the visualization polygon
        - type resolution:  int
        - param c1:         First corner of the parking spot obstacle
        - type c1:          np.array([x, y]) [m, m]
        - param c2:         Second corner of the parking spot obstacle
        - type c2:          np.array([x, y]) [m, m]
        '''
        self.center = np.array([0.0, 0.0])
        self.set_focal_points(f1, f2)
        self.set_inflation(inflation)
        self.set_vis_points(vis_points)
        self.set_resolution(resolution)
        self.set_corners(c1, c2)

        EllipseInfo.index += 1  # Increment class index

        # Create publisher for visualization
        print(f'Ellipse {EllipseInfo.index} created')
        self.vis_pub = rospy.Publisher('/parking_ellipse' + str(EllipseInfo.index), PolygonStamped, queue_size=10)

        self.visualize_ellipse()


    def publish_ellipse(self):
        '''
        Publish ellipse for visualization
        '''
        self.vis_pub.publish(self.vis_points)

    def set_focal_points(self, f1 = None, f2 = None):
        '''
        Set focal points of ellipse
        '''
        self.f1 = f1 if f1 is not None else np.array([0.0, 0.0])
        self.f2 = f2 if f2 is not None else np.array([0.0, 0.0])

    def set_inflation(self, inflation):
        '''
        Set inflation of ellipse
        '''
        self.inflation = inflation * 45 * math.pi / 180     # Map inflation from 0-1 to 0-45 degrees

    def set_vis_points(self, vis_points):
        '''
        Set points for visualization of the ellipse
        '''
        self.vis_points = vis_points if vis_points is not None else PolygonStamped()

    def set_resolution(self, resolution):
        '''
        Set resolution of visualization points
        '''
        self.resolution = resolution

    def set_corners(self, c1, c2):
        '''
        Set corners of parking spot obstacle
        '''
        self.c1 = c1 if c1 is not None else np.array([0.0, 0.0])
        self.c2 = c2 if c2 is not None else np.array([0.0, 0.0])

    def visualize_ellipse(self):
        self.compute_focal_points()             # Compute focal points of ellipse
        self.generate_vis_points()              # Generate points for visualization
        self.publish_ellipse()                  # Publish ellipse for visualization

        time.sleep(0.1)                       # Sleep for 0.1 seconds so that it has time to publish the data

    def compute_focal_points(self):
        '''
        Compute focal points of ellipse
        '''
        c = np.linalg.norm(self.c1 - self.c2)   # Distance between corners
        theta = np.arctan2(self.c2[1] - self.c1[1], self.c2[0] - self.c1[0]) # Angle between corners
        b = self.inflation * (c / 2)            # Semi-minor axis
        f = 2*math.sqrt((c/2)**2 - b**2)
        d = (c - f) / 2                         # Distance between corners and focal points
        self.r = c                              # Radius of ellipse
        self.f1[0] = self.c1[0] + math.cos(theta) * d
        self.f1[1] = self.c1[1] + math.sin(theta) * d
        self.f2[0] = self.c2[0] - math.cos(theta) * d
        self.f2[1] = self.c2[1] - math.sin(theta) * d


    def generate_vis_points(self, resolution = 10):
        '''
        Given the focal points, compute the points for visualization of the ellipse
        '''

        c = np.linalg.norm(self.c1 - self.c2)   # Distance between corners
        b = self.inflation * (c / 2)            # Semi-minor axis
        a = c / 2                               # Semi-major axis
        center = (self.f1 + self.f2) / 2        # Center of ellipse
        theta = np.arctan2(self.c2[1] - self.c1[1], self.c2[0] - self.c1[0]) # Angle between corners

        # Parameterize ellipse equation to get x and y coordinates
        t = np.linspace(0, 2*math.pi, resolution)
        x = a * np.cos(t)
        y = b * np.sin(t)

        # Rotate ellipse
        R = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        points = np.array([x, y])
        points = np.matmul(R, points)

        # Translate ellipse
        points[0, :] = points[0, :] + center[0]
        points[1, :] = points[1, :] + center[1]

        x = points[0, :]
        y = points[1, :]


        # Generate polygon for visualization
        self.vis_points.polygon.points = []
        self.vis_points.header.frame_id = "map"

        self.vis_points.polygon.points = [Point32(x[i], y[i], 0.0) for i in range(len(x))]


