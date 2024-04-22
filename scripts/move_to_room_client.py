#!/usr/bin/env python3

import rospy
from second_coursework.srv import MoveToRoom, MoveToRoomRequest


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


if __name__ == "__main__":
    rospy.init_node('room_enter', anonymous=True)

    # List of rooms to move the robot to
    rooms = ['A', 'B', 'D']
    for room in rooms:
        print(f"Requesting to move to room {room}")
        move_robot_to_room(room)

    print("Process completed")
