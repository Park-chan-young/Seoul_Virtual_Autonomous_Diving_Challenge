#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

class ObjectDetector:
    def __init__(self):

        rospy.init_node('object_detector')

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.obstacle_detect_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=1)

        # Load Yolo
        self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        self.classes = []
        with open("coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def image_callback(self, image_msg):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(image_msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Resize the image to the input size of Yolo
            height, width, channels = cv_image.shape
            blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)

            class_ids = []
            confidences = []
            boxes = []

            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(self.classes[class_ids[i]])
                    confidence = confidences[i]
                    color = (0, 255, 0)  # BGR
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(cv_image, label + " " + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Check if any objects are detected
            object_detected = len(indexes) > 0

            # Publish the object detection result
            self.obstacle_detect_pub.publish(object_detected)

            # Show the image with bounding boxes
            cv2.imshow("Object Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    object_detector = ObjectDetector()
    object_detector.run()
