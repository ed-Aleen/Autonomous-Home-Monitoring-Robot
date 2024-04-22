#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import CheckRoomAction, CheckRoomFeedback, CheckRoomResult,CheckRoomGoal

def feedback_callback(feedback):
    rospy.loginfo(f"Feedback received: Robot Position - x={feedback.robot_position.x}, y={feedback.robot_position.y}, Rule Broken - {feedback.rule_broken}")
def client():
    rospy.init_node('room_visit_client')

    client = actionlib.SimpleActionClient('room_visit', CheckRoomAction)
    client.wait_for_server()

    goal = CheckRoomGoal()
    goal.num_checks_per_room = 2

    client.send_goal(goal, feedback_cb=feedback_callback)
    client.wait_for_result()

    result = client.get_result()

    if result:
        rospy.loginfo(f"Result: {result.rule_violations}")
    else:
        rospy.logwarn("No result received from action server.")

if __name__ == '__main__':
    try:
        client()
    except rospy.ROSInterruptException:
        pass
