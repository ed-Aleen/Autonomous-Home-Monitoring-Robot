#!/usr/bin/env python3

import rospy
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from second_coursework.srv import MoveToRoom, MoveToRoomResponse
from second_coursework.msg import YOLODetection

from second_coursework.srv import MoveToRoom, MoveToRoomResponse
from second_coursework.msg import YOLODetection

def move_to_room(req):
    room_ranges = {
        'A': {'x': (1.5, 3), 'y': (6.6, 10.0)},
        'B': {'x': (4.2, 6.9), 'y': (6.6, 10.0)},
        'D': {'x': (1.5, 3), 'y': (1, 5.3)}
    }

    if req.room_name not in room_ranges:
        return MoveToRoomResponse(False, [], False, "")
    x_range = room_ranges[req.room_name]['x']
    y_range = room_ranges[req.room_name]['y']
    x = random.uniform(x_range[0], x_range[1])
    y = random.uniform(y_range[0], y_range[1])

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait or client.get_state() != actionlib.GoalStatus.SUCCEEDED:
        return MoveToRoomResponse(False, [], False, "")

    try:
        rospy.wait_for_service('yolo_detect')
        yolo_client = rospy.ServiceProxy('yolo_detect', MoveToRoom)
        yolo_response = yolo_client(req.room_name)
        return MoveToRoomResponse(
            success=True,
            detections=[],
            violation_detected=yolo_response.violation_detected,
            violation_type=yolo_response.violation_type
        )
    except rospy.ServiceException as e:
        rospy.logerr("Service call to YOLO detection failed: %s" % e)
        return MoveToRoomResponse(False, [], False, "")

def main():
    rospy.init_node('room_service')
    service = rospy.Service('move_robot_to_room', MoveToRoom, move_to_room)
    rospy.loginfo("Ready to move robot to room.")
    rospy.spin()

if __name__ == "__main__":
    main()
