#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
from std_msgs.msg import Float64, Float32
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class pure_pursuit :
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path_lane_1", Path, self.global_path_lane_1_callback)
        rospy.Subscriber("/global_path_lane_2", Path, self.global_path_lane_2_callback)
        rospy.Subscriber("/local_path_lane_1", Path, self.local_path_lane_1_callback)
        rospy.Subscriber("/local_path_lane_2", Path, self.local_path_lane_2_callback)
        
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        
        rospy.Subscriber("/dist_forward", Float32, self.scan_dist_callback)
        rospy.Subscriber("/clusters", PoseArray, self.scan_pose_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_local_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path_lane_1 = False
        self.is_global_path_lane_2 = False
        self.is_scan_dist = False
        self.is_scan_pose = False
        self.changing_lane = False
        self.prev_changing_lane = False
        self.is_lc_complete = False

        self.lane_flag = 1

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()
        
        self.scan_dist_buffer = []

        self.vehicle_length = 4.635
        self.lfd = 25
        self.min_lfd = 10
        self.max_lfd = 30
        self.lfd_gain = 0.8 #0.78
        ########################## tuning ##########################
        self.target_velocity = 60
        self.obstacle_safety_speed = 30.0 # 장애물 회피 안전속도 15 kph
        self.safety_dist = 25.0 # 장애물 안전거리
        self.buffer_size = 10
        ############################################################

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.5) #0.15
        while True:
            if self.is_global_path_lane_1 == True:
                self.velocity_list_lane_1 = self.vel_planning.curvedBaseVelocity(self.global_path_lane_1, 50)
                break
            else:
                print('Waiting global path data')
                
        while True:
            if self.is_global_path_lane_2 == True:
                self.velocity_list_lane_2 = self.vel_planning.curvedBaseVelocity(self.global_path_lane_2, 50)
                break
            else:
                print('Waiting global path data')

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_local_path == True and self.is_odom == True and self.is_status == True and self.is_scan_dist == True and self.is_scan_pose == True:
                print(f"self.is_path (/local_path) : {self.is_local_path}")
                print(f"self.is_status (/Ego_topic)  : {self.is_status}")
                print(f"self.is_odom (/odom)         : {self.is_odom}")                
                
                self.detect_obstacle()
                if self.changing_lane == True:
                    if self.status_msg.velocity.x * 3.6 > self.obstacle_safety_speed:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1.0
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    else:
                        # LC
                        if self.lane_flag == 1:
                            self.changing_lane = True
                            self.pure_pursuit_drive(self.global_path_lane_2)
                            if self.is_lc_complete == self.lc_complete():
                                self.changing_lane = False
                                self.lane_flag = 2
                                self.scan_dist_buffer = []
                                # rospy.loginfo("LC complete!")

                        else:
                            self.changing_lane = True
                            self.pure_pursuit_drive(self.global_path_lane_1)
                            if self.is_lc_complete == self.lc_complete():
                                self.changing_lane = False
                                self.lane_flag = 1
                                self.scan_dist_buffer = []
                                # rospy.loginfo("LC complete!")

                else:
                    # LK
                    if self.lane_flag == 1:
                        self.pure_pursuit_drive(self.global_path_lane_1)
                    else:
                        self.pure_pursuit_drive(self.global_path_lane_2)
                        
                        
                # ========================== Monitoring ==========================
                rospy.loginfo(f"current lane: {self.lane_flag}")
                rospy.loginfo(f"changing_lane: {self.changing_lane}")
                # ================================================================
                
            else:
                print(f"self.is_path (/local_path) : {self.is_local_path}")
                print(f"self.is_status (/Ego_topic)  : {self.is_status}")
                print(f"self.is_odom (/odom)         : {self.is_odom}")

            rate.sleep()

    # ===================== callback function =======================
    def global_path_lane_1_callback(self,msg):
        self.global_path_lane_1 = msg
        self.is_global_path_lane_1 = True

    def global_path_lane_2_callback(self,msg):
        self.global_path_lane_2 = msg
        self.is_global_path_lane_2 = True

    def local_path_lane_1_callback(self,msg):
        self.local_path_lane_1 = msg
        self.is_local_path = True

    def local_path_lane_2_callback(self,msg):
        self.local_path_lane_2 = msg
        self.is_local_path = True

    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg= msg
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def scan_dist_callback(self, msg):
        self.is_scan_dist = True
        self.scan_dist = msg

    def scan_pose_callback(self, msg):
        self.is_scan_pose = True
        self.scan_pose = msg
    # ===============================================================

    def detect_obstacle(self):
        if not self.changing_lane:
            if self.is_scan_dist:
                if float(self.scan_dist.data) < self.safety_dist:
                    self.scan_dist_buffer.append(float(self.scan_dist.data))
                    if len(self.scan_dist_buffer) == self.buffer_size:
                        self.changing_lane = True
                        self.scan_dist_buffer = []
                else:
                    self.scan_dist_buffer = []
            else:
                self.scan_dist_buffer = []
        
    def lc_complete(self):
        min_dist = float('inf') # 자차 현재 위치와 global path 사이의 최단 거리
        
        current_position = (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y)
        if self.lane_flag == 1:  
            path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.global_path_lane_1.poses]
        else:
            path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.global_path_lane_2.poses]
            
        for point in path_points:
            dx = current_position[0] - point[0]
            dy = current_position[1] - point[1]
            dist = sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                
        if min_dist < 0.001: # LC 판단 기준값
            self.is_lc_complete = True
            return True
        else:
            self.is_lc_complete = False
            return False
            
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.pose.pose.position.x - pose.pose.position.x
            dy = ego_status.pose.pose.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self, path):

        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain
        
        if self.lfd < self.min_lfd : 
            self.lfd=self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd=self.max_lfd
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                [0                    ,0                    ,1            ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(path.poses) :
            path_point=i.pose.position

            global_path_point = [path_point.x,path_point.y,1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0]>0 :
                dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        theta = atan2(local_path_point[1],local_path_point[0])
        self.Max_steering = 1
        steering = atan2((self.vehicle_length*2*sin(theta)), self.lfd)
        #steering = max(-self.Max_steering, min(self.Max_steering, steering))        
        
        if steering is None:
            print("[ERROR] you need to change pure_pursuit at line 179 : calcu_steering !")
            exit()

        return steering
    
    def pure_pursuit_drive(self, global_path_lane_):
        self.current_waypoint = self.get_current_waypoint(self.odom_msg, global_path_lane_)

        if self.lane_flag == 1:
            velocity_list = self.velocity_list_lane_1
            local_path = self.local_path_lane_1
        else:
            velocity_list = self.velocity_list_lane_2
            local_path = self.local_path_lane_2

        self.target_velocity = velocity_list[self.current_waypoint]*3.6
        steering = self.calc_pure_pursuit(local_path)

        if self.is_look_forward_point :
            self.ctrl_cmd_msg.steering = steering
        else : 
            print("no found forward point")
            self.ctrl_cmd_msg.steering = 0.0

        output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

        if output > 0.0:
            self.ctrl_cmd_msg.accel = output
            self.ctrl_cmd_msg.brake = 0.0
        else:
            self.ctrl_cmd_msg.accel = 0.0
            self.ctrl_cmd_msg.brake = -output

        print(output)
        print(steering)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

class pidControl: 
    def __init__(self):
        self.p_gain = 0.3 #0.05
        self.i_gain = 0.00053 #1/100
        self.d_gain = 0.05 # 1/2
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.03 #0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            v_max = sqrt(r*9.8*self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass