#!/usr/bin/env python3

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus, CollisionData
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np

class PathPlanner:
    def __init__ (self):
        rospy.init_node("path_planner", anonymous=True)
        
        rospy.Subscriber("/lane_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("CollisionData", CollisionData, self.col_callback)
        
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        
        self.is_path = False
        self.is_status = False
        self.is_col = False

        self.min_col_safety_dist = 3 # meter
        
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_status and self.is_col:
                if self.check_object(self.lane_path, self.col_msg):
                    # local_path = self.local_planner(self.lane_path, self.status)
                    # local_path_index = self.collision_check(self.col_msg, local_path)
                    # self.path_pub.publish(local_path[local_path_index])

                    self.path_pub.publish(self.lane_path)
                else:
                    self.path_pub.publish(self.lane_path)
            rate.sleep()
                
        
    # 11 lane_path info (nav_msgs/Path)
    def path_callback(self, msg):
        self.is_path = True
        self.lane_path = msg
        
    def status_callback(self, msg):
        self.is_status = True
        self.status = msg
        
    def col_callback(self, msg):
        self.is_col = True
        self.col_msg = msg
        
    def check_object(self, ref_path, col_data):
        is_crash = False
        for obstacle in col_data.collision_object:
            for path in ref_path.poses:
                dist = sqrt(pow(path.pose.position.x - obstacle.position.x, 2) + pow(path.pose.position.y - obstacle.position.y, 2))
                if dist < self.min_col_safety_dist:
                    is_crash = True
                    break

        return is_crash
    
    def collision_check(self, object_data, out_path):
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path 
        
        for obstacle in object_data.collision_object:                        
            for path_num in range(len(out_path)) :                    
                for path in out_path[path_num].poses :                                
                    dist = sqrt(pow(obstacle.position.x - path.pose.position.x, 2) + pow(obstacle.position.y - path.pose.position.y, 2))
                    if dist < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))     

        return selected_lane


    # make local path planning algorithm
    # def local_planner(self, ref_path, vehicle_status):

    #     return out_path

if __name__ == '__main__':
    try:
        PathPlanner()
    except rospy.ROSInterruptException:
        pass