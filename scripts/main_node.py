#!/usr/bin/env python3

import rospy
import actionlib
from second_coursework.srv import MoveToRoom, MoveToRoomRequest
from second_coursework.msg import CheckRoomAction,CheckRoomGoal

def feedback_callback(feedback):
    rospy.loginfo(f"Feedback received: Robot Position - x={feedback.robot_position.x}, y={feedback.robot_position.y}, Rule Broken - {feedback.rule_broken}")


def move_robot_to_room(room_name):
    try:
        rospy.wait_for_service('move_robot_to_room')
        move_robot = rospy.ServiceProxy('move_robot_to_room', MoveToRoom)
        resp = move_robot(room_name)
        if resp.success:
            print(f"Robot successfully moved to room {room_name}")
        else:
            print(f"Failed to move robot to room {room_name}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

rospy.init_node('main_node')


rooms = ['A', 'B', 'D']
for room in rooms:
    print(f"Requesting to move to room {room}")
    move_robot_to_room(room)

print("Process completed")

client = actionlib.SimpleActionClient('room_visit', CheckRoomAction)
client.wait_for_server()

goal = CheckRoomGoal()
goal.num_checks_per_room =rospy .get_param('checks',3)

client.send_goal(goal, feedback_cb=feedback_callback)
client.wait_for_result()

result = client.get_result()

if result:
    rospy.loginfo(f"Result: {result.rule_violations}")
else:
    rospy.logwarn("No result received from action server.")
