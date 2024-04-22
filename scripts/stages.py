#!/usr/bin/env python3

import rospy
import smach
from second_coursework.srv import MoveToRoom, MoveToRoomRequest
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from smach import Concurrence
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from smach import CBState
from second_coursework.msg import CheckRoomAction, CheckRoomFeedback, CheckRoomResult,CheckRoomGoal
from rospy import Time


@smach.cb_interface(input_keys=['room_name'], outcomes=['succeeded', 'failed'])
def move_robot_cb(userdata):
    rospy.loginfo(f'Moving to room: {userdata.room_name}')
    try:
        move_to_room = rospy.ServiceProxy('move_robot_to_room', MoveToRoom)
        response = move_to_room(userdata.room_name)
        return 'succeeded' if response.success else 'failed'
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return 'failed'

class RoomAlternatorState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['A', 'B', 'D', 'finished'],
                             input_keys=['counter', 'room_name'],
                             output_keys=['counter', 'room_name'])

    def execute(self, userdata):
        if userdata.counter <= 0:
            return 'finished'
        elif userdata.counter % 3 == 0:
            userdata.room_name = 'A'
        elif userdata.counter % 2 == 0:
            userdata.room_name = 'D'
        else:
            userdata.room_name = 'B'
        userdata.counter -= 1
        return userdata.room_name



class NavigateRoomState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed'], input_keys=['room_name'], output_keys=['room_name'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self. room_ranges = {
            'A': {'x': (1.5, 3), 'y': (6.6, 10.0)},
            'B': {'x': (4.2, 8.9), 'y': (6.6, 10.0)},
            'D': {'x': (1.5, 3), 'y': (1, 5.3)}

        }

    def execute(self, userdata):
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < 60:
            x_range = self.room_ranges[userdata.room_name]['x']
            y_range = self.room_ranges[userdata.room_name]['y']
            x = random.uniform(x_range[0], x_range[1])
            y = random.uniform(y_range[0], y_range[1])

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(5))  # Wait for 5 seconds

        return 'completed'

class NavigateState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['completed'])
        self.duration = duration

    def execute(self, userdata):
        rospy.sleep(self.duration)
        return 'completed'

class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopped'])

    def execute(self, userdata):

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        stop_goal = MoveBaseGoal()
        stop_goal.target_pose.header.stamp = rospy.Time.now()
        stop_goal.target_pose.header.frame_id = "map"
        stop_goal.target_pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

        client.send_goal(stop_goal)
        client.wait_for_result(rospy.Duration.from_sec(0.5))

        return 'stopped'


class CheckRuleViolatedState(smach.State):
    def __init__(self, action_server, check_interval=1.0, timeout=60.0):
        super(CheckRuleViolatedState, self).__init__(
            outcomes=['violation_detected', 'no_violation', 'failed'],
            input_keys=['room_name'],output_keys=['room_name'])

        self.action_server = action_server
        self.check_interval = rospy.Duration(check_interval)
        self.timeout = timeout
        self.current_pose = Point()
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)

    def update_pose(self, data):
        self.current_pose = data.pose.pose.position

    def execute(self, userdata):
        rospy.loginfo('Executing state CheckRuleViolatedState')
        start_time = rospy.get_time()  # Record the start time
        rospy.wait_for_service('yolo_detect')
        while not rospy.is_shutdown():
            if rospy.get_time() - start_time > self.timeout:
                rospy.loginfo("No violation detected within timeout.")
                return 'no_violation'

            try:
                yolo_service = rospy.ServiceProxy('yolo_detect', MoveToRoom)
                response = yolo_service(userdata.room_name)
                if response.violation_detected:
                    rospy.loginfo(f'Violation Detected: {response.violation_type}')
                    print(f"Rule violation detected: {response.violation_type} at position {self.current_pose}")
                    rule_broken_number = 2 if response.violation_type == "people in kitchen" else 1
                    self.action_server.update_rule_violation_count(rule_broken_number)
                    print(f"Rule {rule_broken_number} broken. Total violations: {self.action_server.rule_violations}")
                    feedback = CheckRoomFeedback()
                    feedback.robot_position = self.current_pose
                    feedback.rule_broken = rule_broken_number
                    self.action_server.publish_feedback(feedback)

                    return 'violation_detected'
                rospy.sleep(self.check_interval)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return 'failed'
