#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class ObjectDetector:
    def __init__(self):

        rospy.init_node("object_detector")
        self.bridge = CvBridge()

        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.obstacle_detect_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=1)

        # ================== HSV color threshold ==================
        
        # red => red berral / traffice barrel
        self.lower_red = np.array([0, 30, 10])
        self.upper_red = np.array([20, 100, 40])

        # # brown => cargo box / woodbox
        # self.lower_brown = np.array([0, 100, 100])
        # self.upper_brown = np.array([10, 255, 255])

        # # black => hyndai_grandeur
        # self.lower_black = np.array([0, 0, 0])
        # self.upper_black = np.array([180, 255, 30])

        self.colors = {
            'red': (self.lower_red, self.upper_red),
            # 'brown': (self.lower_brown, self.upper_brown),
            # 'black': (self.lower_black, self.upper_black)
        }

        self.pixel_threshold = 100

        # OpenCV 화면 출력 관련 변수
        self.window_name = "Object Detection"
        cv2.namedWindow(self.window_name)

    def detect_obstacle(self, image, color_name, lower, upper):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(image, image, mask=mask)
        return result

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        
        roi = cv_image[120:360, 120:520].copy()
        obstacle_detected = False

        for color_name, (lower, upper) in self.colors.items():
            result = self.detect_obstacle(roi, color_name, lower, upper)
            mask = cv2.inRange(result, lower, upper)  # Calculate mask from result

            if cv2.countNonZero(mask) > self.pixel_threshold:
                obstacle_detected = True
                rospy.loginfo(f"{color_name} obstacle detected")

                # Resize the mask to match the size of roi
                mask = cv2.resize(mask, (roi.shape[1], roi.shape[0]), interpolation=cv2.INTER_LINEAR)
                num_red_pixels = cv2.countNonZero(mask)

                # Create a colored result image with original colors
                result_color = cv2.bitwise_and(roi, roi, mask=mask)

                cv2.imshow(self.window_name, result_color)
                rospy.loginfo(f"number of red pixel: {num_red_pixels}")

        cv2.imshow("Raw", cv_image)
        cv2.imshow("ROI", roi)
        

        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_detected
        self.obstacle_detect_pub.publish(obstacle_msg)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        object_detector = ObjectDetector()
        object_detector.run()
    except rospy.ROSInterruptException:
        pass

