#!/usr/bin/env python3
import rospy
from second_coursework.srv import MoveToRoom


rospy.init_node('setYolo')

rospy.wait_for_service('yolo_detect')
yolo_detect = rospy.ServiceProxy('yolo_detect', MoveToRoom)
yolo_detect('D')
print(yolo_detect)
