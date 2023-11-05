#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from nav_msgs.msg import Odometry

class pathMaker :
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        # /Ego_topic 토픽 구독
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.is_status=False
        # 패키지 경로 로드 & 파일 쓰기 모드
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/'+path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True :
                # turtle 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        x=self.odom_msg.pose.pose.position.x
        y=self.odom_msg.pose.pose.position.y
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        # 이전 waypoint와의 거리가 2 meter 이상이어야 기록
        if distance > 2.0:
            data='{0}\t{1}\n'.format(x,y)
            self.f.write(data)
            self.f.flush()
            self.prev_x=x
            self.prev_y=y
            print("write : ", x,y)
    
    def odom_callback(self,msg):
        self.is_status=True
        self.odom_msg=msg

if __name__ == '__main__' :
    try:
        p_m=pathMaker("path_planner", "path/global_path")
        rospy.spin()
    except rospy.ROSInternalException:
        pass