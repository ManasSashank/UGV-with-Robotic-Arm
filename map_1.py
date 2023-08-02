#!/usr/bin/env python3

# ros includes
import rospy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse
import cv2
import numpy as np
import math


class mapData:
    """docstring for mapData"""
    def __init__(self):
        self.width = 0
        self.height = 0
        self.resolution = 0
        self.eta = 0.5
        self.maps = list()


        self.map_topic_name = "/map"
        self.map_sub_handle = rospy.Subscriber(self.map_topic_name, OccupancyGrid, self.map_data_callback)

        rospy.loginfo("Waiting for data on /map topic..")
        rospy.wait_for_message(self.map_topic_name, OccupancyGrid, timeout=10)




    def map_data_callback(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        self.map_image = np.zeros((self.height, self.width, 1), dtype="int8")
        for i in range(0, self.height):
            for j in range(0, self.width):
                if msg.data[i * self.width + j] > 0 :
                    self.map_image[i][j] = 1
                else:
                    self.map_image[i][j] = int(msg.data[i * self.width + j])

        self.maps = list(self.map_image)
        #print(self.maps)

    def get_map(self):
        return self.map_image,self.maps

    def get_map_dimensions(self):
        return self.width, self.height, self.resolution
    
    def obstacle_symbol(self):
        self.symbol_resolution = math.pow(self.eta,2)/math.pow(self.resolution,2)
        print(self.symbol_resolution)
        

if __name__ == '__main__':

    rospy.init_node("target_estimation")

    mapdata = mapData()


    _w, _h, resolution = mapdata.get_map_dimensions()
    z = mapdata.obstacle_symbol()
    


    rate = rospy.Rate(1)

    try:
        while not rospy.is_shutdown():
            maps2,maps = mapdata.get_map()
            #print(maps2,np.shape(maps2),_w,_h,resolution)
            print(z)
            width, height, resolution = mapdata.get_map_dimensions()
            rate.sleep()
    except KeyboardInterrupt:
        pass
