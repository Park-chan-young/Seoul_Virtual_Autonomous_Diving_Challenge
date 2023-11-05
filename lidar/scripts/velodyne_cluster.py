#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import tf

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN
from nav_msgs.msg import Odometry
# ...

class SCANCluster:
    def __init__(self):
        # ...
        self.odom_msg = None
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10)


        self.pc_np = None
        self.dbscan = DBSCAN(eps=0.5, min_samples=5)


    def callback(self, msg):
        if self.odom_msg is not None:
            orientation_x = self.odom_msg.pose.pose.orientation.x
            orientation_y = self.odom_msg.pose.pose.orientation.y
            orientation_z = self.odom_msg.pose.pose.orientation.z
            orientation_w = 1
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            cluster_msg = PoseArray()
        else:
            pc_xy = self.pc_np[:, :2]
            db = self.dbscan.fit_predict(pc_xy)
            n_cluster = np.max(db) + 1
            cluster_msg = PoseArray()
            cluster_list = []


            for c in range(n_cluster):
                # ...

                c_tmp = np.mean(pc_xy[db==c, :], axis=0)

                # 방해물의 상대적인 위치를 글로벌 좌표로 변환합니다.
                local_obstacle_x =  c_tmp.tolist()[0]
                local_obstacle_y = c_tmp.tolist()[1]
                local_obstacle_z = 0.68
                local_point = (local_obstacle_x, local_obstacle_y, local_obstacle_z)
                rotation_quaternion = (orientation_x, orientation_y, orientation_z, orientation_w)
                p = tf.transformations.quaternion_matrix(rotation_quaternion)[:3, :3]
                transformed_point = tuple(p.dot(local_point) + (0, 0, 0))                

                tmp_pose = Pose()
                tmp_pose.position.x = transformed_point[0]
                tmp_pose.position.y = transformed_point[1]
                
                print( tmp_pose.position.x)
                print(' ')
                print(tmp_pose.position.y)                
                
                cluster_msg.poses.append(tmp_pose)
                
        self.cluster_pub.publish(cluster_msg)
        
        
    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            
            # LiDAR의 PointCloud Data로부터 Distance와 Angle 값을 계산하는 영역입니다.
            # 각 Point의 XYZ 값을 활용하여 Distance와 Yaw Angle을 계산합니다.
            # Input : point (X, Y, Z, Intensity)            

            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])

            if point[0] > 0 and 1.50 > point[2] > 0 and dist < 8 and angle > -30/180*np.pi and angle < 30/180*np.pi:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np

    def odom_callback(self, msg):
        self.is_status=True
        self.odom_msg=msg


    # ...

if __name__ == '__main__':
    rospy.init_node('velodyne_clustering', anonymous=True)
    scan_cluster = SCANCluster()
    rospy.spin()