#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from morai_msgs.msg  import EgoVehicleStatus
import cv2
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

class CustomPath:
    def __init__ (self):
        rospy.init_node('custom_path_pub')
        
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        
        self.path_publisher = rospy.Publisher('custom_path', Path, queue_size=1)
    
        self.is_status = False
        self.exponential_base = 1.0 # (1.0 ~ 1.25)
        
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            custom_path = self.create_lane_path()
            self.path_publisher.publish(custom_path)
            # self.cal_curvature()
            # self.show_custom_path()
            rate.sleep()
                
    
    def status_callback(self, msg):
        self.is_status = True
        self.status = msg

    def create_lane_path(self):
        lane_path = Path()
        lane_path.header.frame_id = '/map'

        # Define the desired path with 60 points
        for i in range(60):
            pose = PoseStamped()
            pose.header.frame_id = '/map'

            # Calculate x and y positions based on the canvas size (30x40)
            x_pub = i * 30.0 / 60.0
            # y_pub = 1 - self.exponential_base ** x_pub # (right turn)
            y_pub = self.exponential_base ** x_pub - 1 # (left turn)

            # Set the position of the point for publishing
            pose.pose.position = Point(x_pub, y_pub, 0.0)

            # Set the orientation (for a straight path, you can use a unit quaternion)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            lane_path.poses.append(pose)

        return lane_path

    # def show_custom_path(self):
    #     # Create a blank canvas (480x640)
    #     canvas = np.zeros((480, 640, 3), dtype=np.uint8)
        
    #     for i in range(60):
    #         # Calculate x and y positions for visualization (480x640)
    #         x_vis = 320.0
    #         y_vis = 480 - (480 - i * (480.0 / 30.0))
            
    #         # Draw a point on the canvas for visualization
    #         cv2.circle(canvas, (int(x_vis), int(y_vis)), 3, (0, 0, 255), -1)
            
    #     # Display the canvas (480x640)
    #     cv2.imshow('Lane Path', canvas)
    #     cv2.waitKey(1)
        
    # def cal_curvature(self):
    #     x = sp.symbols('x')
    #     y = self.exponential_base ** (x)
    #     dy_dx = sp.diff(y,x)
    #     d2y_dx2 = sp.diff(dy_dx, x)
    #     k = (1 + dy_dx**2)**(3/2) / sp.Abs(d2y_dx2)
    #     curvature = sp.lambdify(x, k, 'numpy')
    #     x_values = range(10)
    #     curvature_values = [curvature(x_val) for x_val in x_values]
    #     # Plot the curvature
    #     plt.plot(x_values, curvature_values)
    #     plt.xlabel('x')
    #     plt.ylabel('Curvature')
    #     plt.title('Path Curvature')
    #     plt.grid(True)
    #     plt.show()


if __name__ == '__main__':
    try:
        CustomPath()
    except rospy.ROSInterruptException:
        pass