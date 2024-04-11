#!/usr/bin/env python3

# This Temp version is when cost map i defined as y number of columns, y number of rows

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Rectangle
#from ros_comms import ROSComm

#import rospy


class CostMap:


    class Node:

        def __init__(self,x,y,yaw):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.parent = None

    class Vertice:

        def __init__(self,x1,y1,x2,y2,grid_size):
            if np.abs(x1) <= np.abs(x2):
                self.x1 = x1/grid_size
                self.x2 = x2/grid_size
            else:
                self.x1 = x2/grid_size
                self.x2 = x1/grid_size
            self.y1 = y1/grid_size
            self.y2 = y2/grid_size
            self.k = (self.y2-self.y1)/(self.x2-self.x1)
            self.xstep = np.abs(x2-x1)/grid_size
            self.ystep = np.abs(y2-y1)/grid_size

    class AreaBounds:
    
        def __init__(self,area):
            self.area = area
            self.xmin_origin = self.area[0]
            self.ymin_origin = self.area[1]
            self.xmin = 0
            self.ymin = 0
            dx = self.area[2]-self.area[0]
            dy = self.area[3]-self.area[1]
            self.xmax = np.hypot(dx, dy)

            self.R, self.alpha = self.rotation_matrix(dx,dy)
            
            self.area = self.rotate(self.R,area)
            

            dx = area[6]-area[0]
            dy = area[7]-area[1]
            self.ymax = np.hypot(dx, dy)
            print(self.xmax,self.ymax)
        @staticmethod
        def rotation_matrix(dx,dy):
            alpha = math.atan2(dy,dx)
            R = np.array([[np.cos(alpha),-np.sin(alpha)],[np.sin(alpha), np.cos(alpha)]])
            return R, alpha
        
        @staticmethod
        def rotate(R,area):
            rotated_area = [np.dot(R,[area[0],area[1]])[0],np.dot(R,[area[0],area[1]])[1], 
            np.dot(R,[area[2],area[3]])[0],np.dot(R,[area[2],area[3]])[1],
            np.dot(R,[area[4],area[5]])[0],np.dot(R,[area[4],area[5]])[1],
            np.dot(R,[area[6],area[7]])[0],np.dot(R,[area[6],area[7]])[1]]
            return rotated_area



    def __init__(self,vehicle_init,vehicle_goal,parking_area,grid_size,obstacle_list,parking_spot_area):

        if parking_area is not None:
            self.original_parking_area = parking_area
            self.parking_area = self.AreaBounds(parking_area)
        self.vehicle_init = self.Node(vehicle_init[0], vehicle_init[1], vehicle_init[2])
        self.vehicle_goal = self.Node(vehicle_goal[0], vehicle_goal[1], vehicle_goal[2])
        # self.vehicle_init = self.rotate_vehicle(self.parking_area.R,self.parking_area.alpha,self.vehicle_init)
        # print(self.vehicle_init.x)
        # self.vehicle_init = self.scale_vehicle(self.vehicle_init,self.parking_area,grid_size)
        # print(self.vehicle_init.x)
        # self.car = self.get_car_vertices(vehicle_init)
        # self.car = self.rotate_rectangle(self.car,self.parking_area.R)
        # self.car = self.target_rectangle(self.car,self.parking_area)
        self.original_vehicle_init = vehicle_init
        self.grid_size = grid_size
        self.eca_value = math.ceil(3/self.grid_size)
        self.ca_value = math.ceil(6/self.grid_size)
        self.original_parking_spot = parking_spot_area
        self.parking_spot = parking_spot_area
        #self.obstacle_list = obstacle_list
        #self.start = self.rotate_init(self.start)
        #self.goal = self.rotate_init(self.goal)
        self.original_obstacle_list = obstacle_list
        self.obstacle_list = obstacle_list
        self.cost_map = np.zeros((int((self.parking_area.xmax)/grid_size), int((self.parking_area.ymax)/grid_size)))
        self.cost_map_array = []
        self.obstacle_list_vertices = []


    @staticmethod
    def rotate_vehicle(R,alpha,vehicle):
        vehicle.x = np.dot(R,[vehicle.x,vehicle.y])[0]
        vehicle.y = np.dot(R,[vehicle.x,vehicle.y])[1]
        vehicle.yaw += alpha
        return vehicle

    
    @staticmethod
    def scale_vehicle(vehicle,parking_area,grid_size):
        vehicle.x = (vehicle.x - parking_area.xmin_origin)/grid_size
        vehicle.y = (vehicle.y - parking_area.ymin_origin)/grid_size
        return vehicle

    @staticmethod
    def rotate_rectangle(rectangle,R):
        #print(type(alpha))
        #R = np.array([[np.cos(alpha),-np.sin(alpha)],[np.sin(alpha), np.cos(alpha)]])
        rotated_rectangle = [np.dot(R,[rectangle[0],rectangle[1]])[0],np.dot(R,[rectangle[0],rectangle[1]])[1], 
        np.dot(R,[rectangle[2],rectangle[3]])[0],np.dot(R,[rectangle[2],rectangle[3]])[1],
        np.dot(R,[rectangle[4],rectangle[5]])[0],np.dot(R,[rectangle[4],rectangle[5]])[1],
        np.dot(R,[rectangle[6],rectangle[7]])[0],np.dot(R,[rectangle[6],rectangle[7]])[1]]
        return rotated_rectangle

    # @staticmethod
    # def rotate_parking_spot(parking_spot,node):
    #     R = np.array([[np.cos(node.yaw),-np.sin(node.yaw)],[np.sin(node.yaw), np.cos(node.yaw)]])
    #     rotated_parking_spot = [np.dot(R,[parking_spot[0],parking_spot[1]])[0],np.dot(R,[parking_spot[0],parking_spot[1]])[1],
    #     np.dot(R,[parking_spot[2],parking_spot[3]])[0],np.dot(R,[parking_spot[0],parking_spot[1]])[1],]
    #     pass


        
    def define_obstacle(self):

        new_obstacle_list = []
        for obstacle in self.obstacle_list:
            rotated_obstacle = self.rotate_rectangle(obstacle, self.parking_area.R)
            target_obstacle = self.target_rectangle(rotated_obstacle, self.parking_area)
            #print(target_obstacle)
            widht_hight_obstacle = self.find_width_and_hight_of_obstacles(target_obstacle)
            scaled_obstacle = self.scale_rectangle(widht_hight_obstacle, self.grid_size)
            new_obstacle_list.append(scaled_obstacle)
        self.obstacle_list = new_obstacle_list

    def define_parking_spot(self):
        rotated_parking_spot = self.rotate_rectangle(self.parking_spot, self.parking_area.R)
        target_parking_spot = self.target_rectangle(rotated_parking_spot, self.parking_area)
        #scaled_parking_spot = self.scale_parking_spot(target_parking_spot,self.grid_size)
        #print(target_parking_spot)
        self.parking_spot = target_parking_spot

    def define_vehicle(self):
        rotated_vehicle = self.rotate_vehicle(self.parking_area.R,self.parking_area.alpha,self.vehicle_init)
        scaled_vehicle = self.scale_vehicle(rotated_vehicle,self.parking_area,self.grid_size)
        self.vehicle_init = scaled_vehicle


    def find_vertices_of_obstacles(self):

        for (ox1,oy1,ox2,oy2,ox3,oy3,ox4,oy4) in self.obstacle_list:
            vert12 = self.Vertice(ox1,oy1,ox2,oy2,self.grid_size)
            vert23 = self.Vertice(ox2,oy2,ox3,oy3,self.grid_size)
            vert34 = self.Vertice(ox3,oy3,ox4,oy4,self.grid_size)
            vert41 = self.Vertice(ox4,oy4,ox1,oy1,self.grid_size)
            self.obstacle_list_vertices.append((vert12,vert23,vert34,vert41))

    
    def update_cost_map(self):

        # for (vert12,vert23,vert34,vert41) in self.obstacle_list_vertices:
        #     # self.draw_vertice(self.cost_map,vert12)
        #     # self.draw_vertice(self.cost_map,vert23)
        #     i = 0
        #     for x in range(int(vert12.x1),int(vert12.x2)):
        #         self.obstacle_area(self.cost_map,int(vert12.y1 + vert12.k*i),int(x))
        #         print(int(vert12.y1 + vert12.k*i),int(x))
        #         i += self.grid_size
        #     i =0
        #     for x in range(int(vert23.x1),int(vert23.x1+vert23.xstep)):
        #         self.obstacle_area(self.cost_map,int(vert23.y1 + vert23.k*i),int(x))
        #         i += self.grid_size
        #     i =0
        #     for x in range(int(vert34.x1),int(vert34.x1+vert34.xstep)):
        #         self.obstacle_area(self.cost_map,int(vert34.y1 + vert34.k*i),int(x))
        #         i += self.grid_size            
        #     i =0
        #     for x in range(int(vert41.x1),int(vert41.x1+vert41.xstep)):
        #         self.obstacle_area(self.cost_map,int(vert41.y1 + vert41.k*i),int(x))
        #         i += self.grid_size
            # if vert12.k*self.grid_size+vert12.y1 
            # y12 = vert12.k*self.grid_size+vert12.y1
            # self.obstacle_area(self.cost_map,y,x)

        for (ox,oy,width,hight) in self.obstacle_list:
            self.round_corners(self.cost_map,oy,ox,self.ca_value,1)
            self.round_corners(self.cost_map,oy,ox,self.eca_value,2)
            self.round_corners(self.cost_map,oy+hight-1,ox,self.ca_value,1)      
            self.round_corners(self.cost_map,oy+hight-1,ox,self.eca_value,2)
            self.round_corners(self.cost_map,oy,ox+width-1,self.ca_value,1)
            self.round_corners(self.cost_map,oy,ox+width-1,self.eca_value,2)
            self.round_corners(self.cost_map,oy+hight-1,ox+width-1,self.ca_value,1)
            self.round_corners(self.cost_map,oy+hight-1,ox+width-1,self.eca_value,2)
            small_hight = False
            small_width = False
            if hight < self.eca_value:
                small_hight_eca = hight
                small_hight_ca = hight 
            else:
                small_hight_eca = self.eca_value
                small_hight_ca = self.ca_value
            if width < self.eca_value:
                small_width_eca = width
                small_width_ca = width
            else:
                small_width_eca = self.eca_value
                small_width_ca = self.ca_value
            # if small_hight == False and small_width == False:
            #     for x in range(ox, ox+width):
            #         for y in range(oy, oy+hight):
            #             self.obstacle_area(self.cost_map,y,x)
            #             self.extreme_caution_area(self.cost_map,y+self.eca_value,x)
            #             self.caution_area(self.cost_map,y+self.ca_value,x)
            #             self.extreme_caution_area(self.cost_map,y,x+self.eca_value)
            #             self.caution_area(self.cost_map,y,x+self.ca_value)
            #             if y < self.eca_value:
            #                 pass
            #             else:
            #                 self.extreme_caution_area(self.cost_map,y-self.eca_value,x)
            #             if y < self.ca_value:
            #                 pass
            #             else:
            #                 self.caution_area(self.cost_map,y-self.ca_value,x)                      
            #             if x < self.eca_value:
            #                 pass
            #             else:
            #                 self.extreme_caution_area(self.cost_map,y,x-self.eca_value)
            #             if x < self.ca_value:
            #                 pass
            #             else:
            #                 self.caution_area(self.cost_map,y,x-self.ca_value)
            # if small_hight == True and small_width == False:
            for x in range(ox, ox+width):
                for y in range(oy, oy+hight):
                    self.obstacle_area(self.cost_map,y,x)
                    for k in range(small_hight_ca,self.ca_value+1):
                        self.caution_area(self.cost_map,y+k,x)
                        if y < self.ca_value:
                            pass
                        else:
                            self.caution_area(self.cost_map,y-k,x)      
                    for k in range(small_hight_eca,self.eca_value+1):
                        self.extreme_caution_area(self.cost_map,y+k,x)
                        if y < self.eca_value:
                            pass
                        else:
                            self.extreme_caution_area(self.cost_map,y-k,x)
                    for k in range(small_width_ca,self.ca_value+1):
                        self.caution_area(self.cost_map,y,x+k)
                        if x < self.ca_value:
                            pass
                        else:
                            self.caution_area(self.cost_map,y,x-k)
                    for k in range(small_width_eca,self.eca_value+1):
                        self.extreme_caution_area(self.cost_map,y,x+k)
                        if x < self.eca_value:
                            pass
                        else:
                            self.extreme_caution_area(self.cost_map,y,x-k)

                                    
                    
                    

            #self.cost_map[int(self.start.y),int(self.start.x)] = 5               
    
    def turn_cost_map_to_array(self):
        
        for k in self.cost_map.flatten():
            self.cost_map_array.append(int(k))

    def plot_parking_space(self,states = None,parking_spot = None):

        plt.figure("Grid Map")
        plt.imshow(self.cost_map, extent=[self.parking_area.ymin, self.parking_area.ymax, self.parking_area.xmin, self.parking_area.xmax], origin="lower")
        ax = plt.gca()
        ax.set_xticks(np.linspace(self.parking_area.ymin, self.parking_area.ymax, (np.abs(self.parking_area.ymax-self.parking_area.ymin))/self.grid_size+1))
        ax.set_yticks(np.linspace(self.parking_area.xmin, self.parking_area.xmax, (np.abs(self.parking_area.xmax-self.parking_area.xmin))/self.grid_size+1))
        ax.grid(color='b', linewidth=0.5)
        self.draw_parking_spot(self.parking_spot,ax)
        # plt.plot(np.array([self.car[0],self.car[2],self.car[4],self.car[6],self.car[0]]),
        #     np.array([self.car[1],self.car[3],self.car[5],self.car[7],self.car[1]]),'--r')
        self.draw_car([self.vehicle_init.x,self.vehicle_init.y,self.vehicle_init.yaw],ax)
        if states != None:
            for state in states:
                self.draw_car(state,ax,self.parking_area)
        plt.show()
    
    def plot_original_parking_space(self):
        
        plt.figure("Original Map")
        ax = plt.gca()
        plt.plot(np.array([self.original_parking_area[0],self.original_parking_area[2],self.original_parking_area[4],self.original_parking_area[6],
        self.original_parking_area[0]]),np.array([self.original_parking_area[1],self.original_parking_area[3],self.original_parking_area[5]
        ,self.original_parking_area[7],self.original_parking_area[1]]),'-k')
        for obstacle in self.original_obstacle_list:
            plt.plot(np.array([obstacle[0],obstacle[2],obstacle[4],obstacle[6],obstacle[0]]),
            np.array([obstacle[1],obstacle[3],obstacle[5],obstacle[7],obstacle[1]]),'--y')
        plt.plot(np.array([self.original_parking_spot[0],self.original_parking_spot[2],self.original_parking_spot[4],self.original_parking_spot[6]
        ,self.original_parking_spot[0]]),np.array([self.original_parking_spot[1],self.original_parking_spot[3],self.original_parking_spot[5],
        self.original_parking_spot[7],self.original_parking_spot[1]]),'-g')
        self.draw_car(self.original_vehicle_init,ax) 

        # plt.figure("rotated Map")
        # ax = plt.gca()
        # plt.plot(np.array([self.parking_area.area[0],self.parking_area.area[2],self.parking_area.area[4],self.parking_area.area[6],
        # self.parking_area.area[0]]),np.array([self.parking_area.area[1],self.parking_area.area[3],self.parking_area.area[5]
        # ,self.parking_area.area[7],self.parking_area.area[1]]),'-k')

        # plt.plot(np.array([self.rotated_obstacle[0],self.rotated_obstacle[2],self.rotated_obstacle[4],self.rotated_obstacle[6],self.rotated_obstacle[0]]),
        # np.array([self.rotated_obstacle[1],self.rotated_obstacle[3],self.rotated_obstacle[5],self.rotated_obstacle[7],self.rotated_obstacle[1]]),'--y')
        # plt.plot(np.array([self.rotated_parking_spot[0],self.rotated_parking_spot[2],self.rotated_parking_spot[4],self.rotated_parking_spot[6]
        # ,self.rotated_parking_spot[0]]),np.array([self.rotated_parking_spot[1],self.rotated_parking_spot[3],self.rotated_parking_spot[5],
        # self.rotated_parking_spot[7],self.rotated_parking_spot[1]]),'-g')
        # self.draw_car(self.rotated_vehicle_init,ax) 


    @staticmethod

    def draw_parking_spot(parking_spot,ax):
        ax.plot(np.array([parking_spot[0],parking_spot[2],parking_spot[4],parking_spot[6],parking_spot[0]]),
        np.array([parking_spot[1],parking_spot[3],parking_spot[5],parking_spot[7],parking_spot[1]]),'-g')


    @staticmethod

    def draw_car(state,ax,parking_area=None):

        #  O--------O   
        #  |        |   249mm 
        #  O--------O   
        #     324mm

        # 34.7097 = np.sqrt(np.square(24.9/2)+np.square(32.4))
        # 0.36686 = arctan((24.9/2)/(32.4))
        # parking_area should not be needed here. The states we receive will already be adjusted to the cost map
        if parking_area == None:
            ox = 0
            oy = 0
        else:
            ox = parking_area.xmin_orgiin
            oy = parking_area.ymin_origin
            print()
        x1 = (state[0] + 24.9/2*np.cos(-np.pi/2+state[2]))/1
        x2 = (state[0] + 34.7097*np.cos(-0.36686+state[2]))/1
        x3 = (state[0] + 34.7097*np.cos(0.36686+state[2]))/1 
        x4 = (state[0] + 24.9/2*np.cos(np.pi/2+state[2]))/1

        y1 = (state[1] + 24.9/2*np.sin(-np.pi/2+state[2]))/1
        y2 = (state[1] + 34.7097*np.sin(-0.36686+state[2]))/1
        y3 = (state[1] + 34.7097*np.sin(0.36686+state[2]))/1
        y4 = (state[1] + 24.9/2*np.sin(np.pi/2+state[2]))/1 

        ax.plot(np.array([x1,x2,x3,x4,x1]),np.array([y1,y2,y3,y4,y1]),'-r')
        ax.plot(state[0],state[1],'*r')
    
    @staticmethod
    def get_car_vertices(state):
        x1 = (state[0] + 24.9/2*np.cos(-np.pi/2+state[2]))/1
        x2 = (state[0] + 34.7097*np.cos(-0.36686+state[2]))/1
        x3 = (state[0] + 34.7097*np.cos(0.36686+state[2]))/1 
        x4 = (state[0] + 24.9/2*np.cos(np.pi/2+state[2]))/1

        y1 = (state[1] + 24.9/2*np.sin(-np.pi/2+state[2]))/1
        y2 = (state[1] + 34.7097*np.sin(-0.36686+state[2]))/1
        y3 = (state[1] + 34.7097*np.sin(0.36686+state[2]))/1
        y4 = (state[1] + 24.9/2*np.sin(np.pi/2+state[2]))/1
        return [x1,y1,x2,y2,x3,y3,x4,y4] 

    @staticmethod
    def target_rectangle(rectangle,parking_area):
        ex = parking_area.xmin_origin 
        ey = parking_area.ymin_origin
        print(ex,ey)
        scaled_rectangle = [rectangle[0]-ex,rectangle[1]-ey,rectangle[2]-ex,rectangle[3]-ey,rectangle[4]-ex,rectangle[5]-ey,rectangle[6]-ex,rectangle[7]-ey]
        return scaled_rectangle

    @staticmethod
    def find_width_and_hight_of_obstacles(obstacle):
        ox = min(obstacle[0], obstacle[2], obstacle[4], obstacle[6])
        # index = [obstacle[0], obstacle[2], obstacle[4], obstacle[6]].index(ox)
        # oy = obstacle[2*index+1]
        oy = min(obstacle[1], obstacle[3], obstacle[5], obstacle[7])
        width_list = np.array([np.abs(obstacle[0]-obstacle[2]), np.abs(obstacle[0]-obstacle[4]), np.abs(obstacle[0]-obstacle[6]), np.abs(obstacle[2]-obstacle[4]), np.abs(obstacle[2]-obstacle[6]), np.abs(obstacle[4]-obstacle[6])])
        hight_list = np.array([np.abs(obstacle[1]-obstacle[3]), np.abs(obstacle[1]-obstacle[5]), np.abs(obstacle[1]-obstacle[7]), np.abs(obstacle[3]-obstacle[5]), np.abs(obstacle[3]-obstacle[7]), np.abs(obstacle[5]-obstacle[7])])
        width = np.amax(width_list)
        hight = np.amax(hight_list)
        width_hight_obstacle = [ox,oy,width,hight]
        return width_hight_obstacle

    @staticmethod
    def scale_rectangle(obstacle,grid_size):
        rescaled_obstacle = [math.floor(obstacle[0]/grid_size),math.floor(obstacle[1]/grid_size),math.ceil(obstacle[2]/grid_size),math.ceil(obstacle[3]/grid_size)]
        #print("ox, oy, width and hight ")
        #print(rescaled_obstacle[0],rescaled_obstacle[1],rescaled_obstacle[2],rescaled_obstacle[3])
        return rescaled_obstacle

    @staticmethod
    def scale_parking_spot(parking_spot,grid_size):
        scaled_parking_spot = [math.ceil(parking_spot[0]/grid_size),math.ceil(parking_spot[1]/grid_size),
        math.ceil(parking_spot[2]/grid_size),math.ceil(parking_spot[3]/grid_size),
        math.ceil(parking_spot[4]/grid_size),math.ceil(parking_spot[5]/grid_size),
        math.ceil(parking_spot[6]/grid_size),math.ceil(parking_spot[7]/grid_size)]
        return scaled_parking_spot



    @staticmethod
    def obstacle_area(cost_map,y,x):
        if y >= 0 and x >= 0:
            try:
                cost_map[y,x] = 3
            except IndexError:
                pass        

    @staticmethod
    def extreme_caution_area(cost_map,y,x):
        if y >= 0 and x >= 0:
            try:
                if cost_map[y,x] == 0 or cost_map[y,x] == 1:
                    cost_map[y,x] = 2
            except IndexError:
                pass
        
    @staticmethod
    def caution_area(cost_map,y,x):
        if y >= 0 and x >= 0:
            try:
                if cost_map[y,x] == 0:
                    cost_map[y,x] = 1
            except IndexError:
                pass

    @staticmethod
    def round_corners(cost_map,cy,cx,r,rad):
        top = cy + r
        bottom = cy - r
        left = cx - r
        right = cx + r
        for y in range(bottom,top+1):
            for x in range(left,right+1):
                if CostMap.inside_circle(cx,cy,x,y,r):
                    if rad == 1:
                        CostMap.caution_area(cost_map,y,x)
                    elif rad == 2:
                        CostMap.extreme_caution_area(cost_map,y,x)


    @staticmethod
    def inside_circle(cx,cy,x,y,r):
        dx = cx - x
        dy = cy - y
        c = dx*dx + dy*dy - r*r       
        if c <= 0:
            return True
        else:
            return False


def main():

    print("start "+__file__)
    # rospy.init_node("cost_map_node")
    # rospy.loginfo("cost_map_node started")
    #sub = ROSComm() # Subscribe to get parking_area, grid_size, obstacles_list, vehicle_init, vehicle_goal
    grid_size = 1
    parking_area = [0,0,25,0,25,20,0,20]#[0,-20,40,20,0,60,-40,20]
    vehicle_init = [0,0,0]#[2,12,np.pi/4]

    vehicle_goal = [5,5,0.5]
    vehicle_state = [(0.5,0.7,0.0),(1.2,1.0,0.5)]
    obstacle_list =  [(5,5,10,5,10,10,5,10)]#[(0,-20,20,0,0,20,-20,0),(10,10,20,0,40,20,30,30)]#(10,5,15,10,12.5,12.5,7.5,7.5),(5,0,10,5,5,10,0,5),(15,10,20,15,15,20,10,15)]
    parking_spot_area = [10,5,15,5,15,20,10,20]#[2,20,9,13,26,30,19,37] 
    cost_map = CostMap(vehicle_init,vehicle_goal,parking_area,grid_size,obstacle_list,parking_spot_area)
    #cost_map.find_vertices_of_obstacles()
    cost_map.plot_original_parking_space()
    cost_map.define_obstacle()
    cost_map.define_parking_spot()    
    cost_map.define_vehicle()
    cost_map.update_cost_map()
    cost_map.turn_cost_map_to_array()
    cost_map.plot_parking_space() # Uncomment to plot cost_map
    # pub = ROSComm()
    # rate = rospy.Rate(1)
    # print(type(cost_map.start.x))
    # while not rospy.is_shutdown():
    #     rospy.loginfo("Hello")
    #     pub.cost_map_publish([cost_map.parking_area.xmax,cost_map.parking_area.ymax],cost_map.grid_size,cost_map.cost_map_array,vehicle_init,vehicle_goal)
    #     rate.sleep()

if __name__ == '__main__':
    main()
 