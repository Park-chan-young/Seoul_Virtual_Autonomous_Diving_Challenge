#!/usr/bin/env python3

import time
import rospy
import numpy as np

from tf.transformations import euler_from_quaternion
from math import sin, sqrt, atan2, pi
from sklearn.cluster import DBSCAN

from morai_msgs.msg  import EgoVehicleStatus, CollisionData, CtrlCmd
from geometry_msgs.msg import Point, PoseArray,Pose
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PurePursuit:
    def __init__(self):

        rospy.init_node('pure_pursuit', anonymous=True)

        # rospy.Subscriber("/path", Path, self.path_callback)
        rospy.Subscriber("/custom_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/dist_forward", Float32, self.scan_dist_callback)
        rospy.Subscriber("/clusters", PoseArray, self.scan_pose_callback)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.col_callback)

        self.target_velocity_pub = rospy.Publisher('target_velocity', Float64, queue_size=1)
        self.control_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_status = False
        self.is_col = False
        self.is_odom = False
        self.is_scan_dist = False
        self.is_scan_pose = False

        self.is_obstacle_front = False
        self.is_obstacle_back = False
        self.is_obstacle_right = False
        self.is_obstacle_left = False
        
        # PID
        # ==========================
        self.kp = 0.56
        self.ki = 0.0007
        self.kd = 0.2
        self.target_velocity = 30
        self.integral_limit = 10.0 # Clamping (anti_windup)
        # ==========================
        
        self.max_velocity = 50
        self.lookahead_distance = 0.0
        self.road_friciton = 0.15
        self.obstacle_distance = 0.0    # meter
        self.safety_distance = 20.0     # meter
        
        self.pid = PID(self.kp, self.ki, self.kd, self.integral_limit)
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, self.road_friciton)
        # self.movav = MovingAverage(20)
        
        
        # TODO // IONIQ5 specifications
        # ===================================================
        self.vehicle_length = 4.635
        self.vehicle_width = 1.890
        self.wheel_length = 3.0
        # ===================================================

        while True:
            if self.is_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.lane_path, 20)
                break
            else:
                rospy.loginfo('waiting "land_path" data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path == True and self.is_status == True:
                
                self.obstacle_distance = sqrt((self.status_msg.position.x - self.scan_pose.poses[0].position.x)**2 + (self.status_msg.position.y - self.scan_pose.poses[0].position.y)**2)
                self.safe_speed = self.calulate_safe_speed(self.obstacle_distance)

                if self.obstacle_distance < self.safety_distance:
                    self.obstacle_detector()
                    if self.status_msg.velocity.x > self.safe_speed:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1.0

                    if self.is_obstacle_right == True:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 0.0
                        self.ctrl_cmd_msg.steering = 0.5    # 좌회전
                    elif self.is_obstacle_left == True:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 0.0
                        self.ctrl_cmd_msg.steering = -0.5   # 우회전
                    
                else:
                    # self.target_velocity 결정 (곡률 기반 속도 결정)
                    self.target_velocity = self.velocity_list[0] * 3.6
                    self.target_velocity_pub.publish(self.target_velocity)

                    # 종방향 속도제어 먼저         
                    output_velocity = self.pid.compute(self.target_velocity, self.status_msg.velocity.x * 3.6) # [kph]
                    
                    if output_velocity > 0.0:
                        self.ctrl_cmd_msg.accel = 1.0
                        self.ctrl_cmd_msg.brake = 0.0
                        
                    elif -5.0 < output_velocity <= 0.0:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 0.0
                        
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1.0

                    # 속도 기반 lookahead_distance 만들어서 횡방향 조향 제어
                    desired_path = self.path_to_array(self.lane_path)
                    current_vel_magnitude = np.linalg.norm(np.array([self.status_msg.velocity.x, self.status_msg.velocity.y, self.status_msg.velocity.z]))
                    self.lookahead_distance =  current_vel_magnitude / self.max_velocity * sqrt(desired_path[-1][0]**2 + desired_path[-1][1]**2)
                    self.lookahead_point = min(desired_path, key=lambda c:abs(sqrt(c[0]**2 + c[1]**2)-self.lookahead_distance))


                    if self.lookahead_point[0] != 0:
                        alpha = self.lookahead_point[1] / self.lookahead_point[0]
                        delta = atan2(2 * self.vehicle_length * sin(alpha), self.lookahead_distance)
                    else:
                        delta = 0 # go straight

                    self.ctrl_cmd_msg.steering = delta
                
                self.control_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()


    def path_callback(self, msg):
        self.is_path = True
        self.lane_path = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def col_callback(self, msg):
        self.is_col = True
        self.col_msg = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.odom_msg = msg

    def scan_dist_callback(self, msg):
        self.is_scan_dist = True
        self.scan_dist = msg

    def scan_pose_callback(self, msg):
        self.is_scan_pose = True
        self.scan_pose = msg

    def path_to_array(self, lane_path):
        if lane_path is None or len(lane_path.poses) != 60:
            return None

        desired_path = []

        for pose in lane_path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            desired_path.append((x, y))

        desired_path = np.array(desired_path)

        return desired_path
    
    def calulate_safe_speed(self, obstacle_distance):
        if obstacle_distance < 3.0: # 장애물과의 거리 3[m] 이내
            safe_speed = 0.0
        elif 3.0 <= obstacle_distance < 6.0:
            safe_speed = 7.5
        elif 6.0 <= obstacle_distance < 10.0:
            safe_speed = 15.0
        elif 10.0 <= obstacle_distance < self.safety_distance:
            safe_speed = 20.0
        else:
            safe_speed = self.target_velocity
        return safe_speed
    
    def obstacle_detector(self):
        diff_x = self.scan_pose.poses[0].position.x - self.status_msg.position.x
        diff_y = self.scan_pose.poses[0].position.y - self.status_msg.position.y

        obstacle_angle = atan2(diff_y, diff_x)

        if -pi/2 < obstacle_angle  < pi/2:
            self.is_obstacle_front = True
            self.is_obstacle_back = False
        else:
            self.is_obstacle_front = False
            self.is_obstacle_back = True

        if self.is_obstacle_front == True:

            if -self.vehicle_width / 2 < diff_y < 0:
                self.is_obstacle_left = True
            elif 0 <= diff_y < self.vehicle_width / 2:
                self.is_obstacle_right = True

        else:
            self.is_obstacle_right = False
            self.is_obstacle_left = False
    
# class MovingAverage:
#     def __init__(self, n):
#         self.samples = n
#         self.data = []
#         self.weights = [i for i in range(-1, n+1)]

#     def add_sample(self, new_sample):
#         if len(self.data) < self.samples:
#             self.data.append(new_sample)
#         else:
#             self.data.pop(0)
#             self.data.append(new_sample)

#     def get_mm(self):
#         return sum(self.data) / len(self.data)

#     def get_wmm(self):
#         s = 0
#         for i in range(-1, len(self.data)):
#             s += self.data[i] * self.weights[i]
#         return s / sum(self.weights[:len(self.data)])

class velocityPlanning:
    def __init__(self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i + box].pose.position.x
                y = gloabl_path.poses[i + box].pose.position.y
                x_list.append([-2 * x, -2 * y, 1])
                y_list.append((-x * x) - (y * y))

            # TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a * a + b * b - c)

            # TODO: (7) 곡률 기반 속도 계획
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

class PID:
    def __init__(self, kp, ki, kd, integral_limit):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = integral_limit

    def compute(self, target_value, current_value):
        
        self.target_value = target_value
        error = self.target_value - current_value
        
        self.integral += error
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
            
        derivative = error - self.prev_error
        control_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        return control_output
    
if __name__ == '__main__':
    try:
        test_track = PurePursuit()

    except rospy.ROSInterruptException:
        pass
