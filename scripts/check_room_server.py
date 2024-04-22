#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import CheckRoomFeedback, CheckRoomResult, CheckRoomAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from second_coursework.srv import MoveToRoom, MoveToRoomRequest
import random
import threading
from stages import  RoomAlternatorState,RoomAlternatorState,StopState,NavigateRoomState,CheckRuleViolatedState,move_robot_cb
import rospy
import smach
from second_coursework.srv import MoveToRoom, MoveToRoomRequest
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from smach import Concurrence
from smach import CBState
from geometry_msgs.msg import PoseWithCovarianceStamped


class RoomVisitActionServer:
    def __init__(self):
        self._action_server = actionlib.SimpleActionServer(
            'room_visit',
            CheckRoomAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.current_pose = Point()
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)

        self.rule_violations = [0, 0]
        self._action_server.start()



    def update_pose(self, msg):
        self.current_pose = msg.pose.pose.position

    def update_rule_violation_count(self, rule_number):
        index = rule_number - 1
        if index < len(self.rule_violations):
            self.rule_violations[index] += 1
        else:
            rospy.logerr(f"Invalid rule number: {rule_number}")
    def publish_feedback(self, feedback):
        if self._action_server.is_active():
            self._action_server.publish_feedback(feedback)

    def execute_cb(self, goal):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        sm.userdata.counter = goal.num_checks_per_room*3
        sm.userdata.room_name = 'A'
        self.rule_violations = [0, 0]

        with sm:
            smach.StateMachine.add('ROOM_ALTERNATOR', RoomAlternatorState(),
                                   transitions={'A': 'MOVE_TO_A', 'B': 'MOVE_TO_B', 'D': 'MOVE_TO_D','finished': 'succeeded'},
                                   remapping={'counter': 'counter', 'room_name': 'room_name'})


            smach.StateMachine.add('MOVE_TO_A', CBState(move_robot_cb),
                                   transitions={'succeeded': 'CONCURRENT_NAVIGATION', 'failed': 'aborted'},
                                   remapping={'room_name': 'room_name'})

            smach.StateMachine.add('MOVE_TO_B', CBState(move_robot_cb),
                                   transitions={'succeeded': 'CONCURRENT_NAVIGATION', 'failed': 'aborted'},
                                   remapping={'room_name': 'room_name'})

            smach.StateMachine.add('MOVE_TO_D',  CBState(move_robot_cb),
                                   transitions={'succeeded': 'CONCURRENT_NAVIGATION', 'failed': 'aborted'},
                                   remapping={'room_name': 'room_name'})


            sm_concurrence = smach.Concurrence(
                outcomes=['violation_detected', 'navigation_completed','no_violation_detected'],
                default_outcome='navigation_completed',
                input_keys=['counter', 'room_name'],
                output_keys=['counter', 'room_name'],
                outcome_map={
                    'violation_detected': {'CHECK_RULE_VIOLATED': 'violation_detected'},
                    'no_violation_detected': {'CHECK_RULE_VIOLATED': 'no_violation'},

                    'navigation_completed': {'NAVIGATE_ROOM': 'completed'}
                }
            )

            with sm_concurrence:
                smach.Concurrence.add('NAVIGATE_ROOM', NavigateRoomState(),
                                      remapping={'room_name': 'room_name'})
                smach.Concurrence.add('CHECK_RULE_VIOLATED', CheckRuleViolatedState(action_server=self, check_interval=1.0),
                                      remapping={'room_name': 'room_name'})


            smach.StateMachine.add('CONCURRENT_NAVIGATION', sm_concurrence,
                                   transitions={
                                       'violation_detected': 'ROOM_ALTERNATOR',
                                       'no_violation_detected':'ROOM_ALTERNATOR',
                                       'navigation_completed': 'ROOM_ALTERNATOR'
                                   },
                                   remapping={'room_name': 'room_name', 'counter': 'counter'})



            smach.StateMachine.add('NAVIGATE_IN_ROOM', NavigateRoomState(),
                                               transitions={'completed': 'MOVE_TO_D'},
                                               remapping={'room_name': 'room_name'})



            smach.StateMachine.add('STOP', StopState(),
                                   transitions={'stopped': 'succeeded'})

        outcome = sm.execute()
        print(f"Total violations for Rule 1: {self.rule_violations[0]}")
        print(f"Total violations for Rule 2: {self.rule_violations[1]}")

        result = CheckRoomResult()
        result.rule_violations = self.rule_violations
        feedback=CheckRoomFeedback()

        if outcome == 'succeeded':
            self._action_server.set_succeeded(result)
        else:
            self._action_server.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node('room_visit_action_server')

    server = RoomVisitActionServer()
    rospy.spin()