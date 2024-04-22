#!/usr/bin/env python3
# coding=utf-8
import rospy
from cv_bridge import CvBridge
import cv2
from second_coursework.srv import MoveToRoom, MoveToRoomResponse
from sensor_msgs.msg import Image
from second_coursework.msg  import YOLODetection
from second_coursework.msg import CheckRoomAction, CheckRoomFeedback, CheckRoomGoal, CheckRoomResult
import numpy as np
from yolov4 import Detector
import random
from std_msgs.msg import String


class YOLOv4ROSITR:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}

        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)

        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/k22035173/ros_ws/src/second_coursework/cfg/coco.data')

        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.yolo_service = rospy.Service('yolo_detect', MoveToRoom, self.yolo_service)



    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")

    def yolo_service(self, request):
        res = MoveToRoomResponse()


        if self.cv_image is None:
            self.tts_pub.publish('nothing detected ')

            rospy.logwarn("No image received for detection.")
            res.success = False
            res.violation_detected = False
            res.violation_type = ""
            res.detections = []
            return res

        else:
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            cv_height, cv_width, _ = self.cv_image.shape
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            rospy.loginfo("Received image for detection")
            people_in_kitchen = False
            cat_detected = False
            dog_detected = False
            rospy.loginfo("ready image for detection")

            for detection in detections:
                rospy.loginfo("detections !!!!!")
                class_name = detection.class_name
                confidence = detection.class_confidence
                box = detection.left_x, detection.top_y, detection.width, detection.height
                rospy.loginfo(f'{class_name.ljust(10)} ')


                if class_name == "person" and request.room_name == 'D':
                    people_in_kitchen = True
                    rospy.loginfo(f"Violation rule 2: Person in kitchen")
                    self.tts_pub.publish('please leave the kitchen ')

                if class_name == "cat" :
                    cat_detected = True
                if class_name == "dog":
                    dog_detected = True
                if cat_detected:
                    if dog_detected:
                        rospy.loginfo(f"Violation rule 1 detected: Cat and dog together")
                        self.tts_pub.publish('please leave  dog and cat  ')

                res.success = not (people_in_kitchen or (cat_detected and dog_detected))
                res.violation_detected = people_in_kitchen or ( cat_detected and  dog_detected)
                res.violation_type = "people in kitchen" if people_in_kitchen else "cat and dog together" if (cat_detected and   dog_detected) else ""

                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                res.detections.append(d)


        return res

if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()