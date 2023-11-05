#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose, PointStamped
from sklearn.cluster import DBSCAN
import math


# lidar_velodyne_cluster 는 LiDAR의 Point들을 물체 단위로 구분하는 Clustering 예제입니다.
# PointCloud Data를 입력받아 DBSCAN Algorithm을 활용하여 Clustering을 수행합니다.
# 교육생분들은 DBSCAN의 Parameter를 조절하여 적절한 Clustering 결과를 얻어내야 합니다. 

# 노드 실행 순서
# 1. DBSCAN Parameter 입력
# 2. 각 Cluster를 대표하는 위치 값 계산
# 3. PointCloud Data로부터 Distance, Angle 값 계산

class Laser_to_Map:
    def __init__(self):

        rospy.init_node('local_to_global_transformer')

        self.listener = tf.TransformListener()

        self.cluster_sub = rospy.Subscriber("clusters", PoseArray, self.callback)

        self.LCM_pub = rospy.Publisher("Laser_to_Map", PoseArray, queue_size=10)

        self.is_status = False
        self.status_msg = None

    def callback(self, msg):
        LCM_msg = PoseArray()
        for MAP_Point in msg.poses:
            local_point = PointStamped()
            local_point.header.frame_id = "Ego"
            local_point.point.x = MAP_Point.position.x
            local_point.point.y = MAP_Point.position.y
            local_point.point.z = 0

        # 글로벌 좌표계로의 변환
            self.listener.waitForTransform("map", "Ego", rospy.Time(), rospy.Duration(4.0))
            global_point = self.listener.transformPoint("map", local_point)
            tmp_pose=Pose()
            tmp_pose.position.x =round(global_point.point.x,2)
            tmp_pose.position.y = round(global_point.point.y,2)
            print(tmp_pose.position.x)
            print(' ')
            print(tmp_pose.position.y)            
            LCM_msg.poses.append(tmp_pose)

        self.LCM_pub.publish(LCM_msg)




if __name__ == '__main__':
    try:
        LCM = Laser_to_Map()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass