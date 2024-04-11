#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math

def get_functions_car_corners(x):
    """x: [x,y,theta], dist:[length, width, dist_to_middle]"""
    """First function is for the front right point, second function derives mid right point, third back right point"""
    # x = np.MX.sym('x', 3, 1)
    # dist = np.MX.sym('dist', 3, 1)

    dist = np.array([0.5, 0.249, 0.13])    # Length, width, distance to middle

    fn_x0 = x[0] - (-dist[2] + dist[0] / 2) * np.cos(x[2]) - dist[1] / 2 * np.cos(x[2] - np.pi/2)
    fn_x1 = x[0] - (-dist[2] + dist[0] / 2) * np.cos(x[2]) + dist[1] / 2 * np.cos(x[2] - np.pi/2)
    fn_x2 = x[0] + (dist[2] + dist[0] / 2) * np.cos(x[2])  + dist[1] / 2 * np.cos(x[2] - np.pi/2)
    fn_x3 = x[0] + (dist[2] + dist[0] / 2) * np.cos(x[2])  - dist[1] / 2 * np.cos(x[2] - np.pi/2)

    fn_y0 = x[1] - (-dist[2] + dist[0] / 2) * np.sin(x[2]) - dist[1] / 2 * np.sin(x[2] - np.pi/2)
    fn_y1 = x[1] - (-dist[2] + dist[0] / 2) * np.sin(x[2]) + dist[1] / 2 * np.sin(x[2] - np.pi/2)
    fn_y2 = x[1] + (dist[2] + dist[0] / 2) * np.sin(x[2])  + dist[1] / 2 * np.sin(x[2] - np.pi/2)
    fn_y3 = x[1] + (dist[2] + dist[0] / 2) * np.sin(x[2])  - dist[1] / 2 * np.sin(x[2] - np.pi/2)

    bl = np.array([fn_x0, fn_y0])
    br = np.array([fn_x1, fn_y1])
    tr = np.array([fn_x2, fn_y2])
    tl = np.array([fn_x3, fn_y3])

    print(f'bl: {bl}')
    print(f'br: {br}')
    print(f'tr: {tr}')
    print(f'tl: {tl}')



    def plot_ellipsis(c1, c2, b):
        '''
        Plot ellipsis using its two center points and the semi-minor axis
        '''
        # Focal points
        f1 = c1
        f2 = c2

        # Distance between focal points
        d = np.linalg.norm(f1 - f2)

        r = d / math.cos(b)

        # Center of ellipse
        c = (f1 + f2) / 2

        # Semi-major axis
        a = r/2

        # Semi-minor axis
        # b = np.sqrt(a**2 - (d / 2)**2)
        b = 0.9 *r / 2

        # Rotation angle
        theta = np.arctan2(f2[1] - f1[1], f2[0] - f1[0])

        # Ellipse parameters
        A = a * np.cos(theta)
        B = b * np.sin(theta)
        C = a * np.sin(theta)
        D = b * np.cos(theta)
        E = (f1[0] + f2[0]) / 2
        F = (f1[1] + f2[1]) / 2

        # Plot ellipse
        t = np.linspace(0, 2 * np.pi)
        x = A * np.cos(t) + B * np.sin(t) + E
        y = C * np.cos(t) + D * np.sin(t) + F




# def plot_ellipsis(f1 = np.array([0.0, 0.0]), f2 = np.array([0.0, 0.0]), inflation = 1.0):
#     '''
#     inflation: 1.0 = Max inflation (45 degrees)
#     ...
#     inflation: 0.0 = No inflation (0 degrees)
#     '''
    
#     # Map inflation from 0-1 to 0-45 degrees
#     inflation = inflation * 45 * math.pi / 180

#     # inflation = 45 * math.pi / 180

#     print(f'inflation = {inflation} radians')
    
#     # Focal points
#     f1 = np.array([-0.2, 0])
#     f2 = np.array([0.2, 0])

#     # Distance between focal points
#     d = np.linalg.norm(f1 - f2)

#     r = d / math.cos(inflation)

#     # Center of ellipse
#     c = (f1 + f2) / 2

#     # Semi-major axis
#     a = r/2

#     # Semi-minor axis
#     b = np.sqrt(a**2 - (d / 2)**2)

#     # Rotation angle
#     theta = np.arctan2(f2[1] - f1[1], f2[0] - f1[0])

#     # Ellipse parameters
#     A = a * np.cos(theta)
#     B = b * np.sin(theta)
#     C = a * np.sin(theta)
#     D = b * np.cos(theta)
#     E = (f1[0] + f2[0]) / 2
#     F = (f1[1] + f2[1]) / 2

#     # Plot ellipse
#     t = np.linspace(0, 2 * np.pi, 1000)
#     x = A * np.cos(t) + B * np.sin(t) + E
#     y = C * np.cos(t) + D * np.sin(t) + F
#     plt.plot(x, y, 'k')

#     plt.xlim(-1, 1)
#     plt.ylim(-1, 1)
#     plt.grid()
#     plt.show()




def main():
    center_of_car = np.array([0, 0, 0])
    # get_functions_car_corners(x = center_of_car)
    plot_ellipsis()

if __name__ == "__main__":
    main()