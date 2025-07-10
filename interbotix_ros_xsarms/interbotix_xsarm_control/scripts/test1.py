#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ContactsState
from perception import SimPerception
from utils_move import move_to, open_gripper, close_gripper, attach_object_when_grasped, detach_object_when_released, _tip_cb, move_to_home_pos


def main():
    rospy.init_node("wx250s_cartesian_pose_and_gripper_client")
    perception = SimPerception()
    # Subscribe to the fingertip contact sensors
    rospy.Subscriber("/wx250s/left_tip_contact", ContactsState, _tip_cb, callback_args='left')
    rospy.Subscriber("/wx250s/right_tip_contact", ContactsState, _tip_cb, callback_args='right')

    # Example usage:
    move_to_home_pos()
    open_gripper()

    object_pose = list(perception.get("spatula"))
    x, y, z = object_pose
    move_to(x, y, z + 0.17)  # Move above the spatula
    move_to(x, y, z)
    close_gripper()
    model, link = attach_object_when_grasped(timeout=3.0)
    move_to(x, y, z + 0.17)
    
    object_pose = list(perception.get("napkin"))
    x, y, z = object_pose
    move_to(x, y, z + 0.15)  # Move above the napkin
    open_gripper()
    if model:
        detach_object_when_released(model, link)

    move_to_home_pos()


if __name__ == "__main__":
    main()