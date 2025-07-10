#!/usr/bin/env python3
import os
import sys
import math
import rospy
import actionlib
import tf.transformations as tft

from moveit_commander import roscpp_initialize, MoveGroupCommander
from moveit_msgs.msg import (
    MoveGroupAction, MoveGroupGoal, Constraints,
    PositionConstraint, OrientationConstraint, JointConstraint,
    BoundingVolume, MoveItErrorCodes
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.msg import ContactsState

from moveit_commander import PlanningSceneInterface
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
scene = PlanningSceneInterface(ns="/wx250s")

def attach_box_to_gripper(box_name: str, box_size: tuple, box_pose: PoseStamped, timeout: float = 2.0):
    """
    Add a box to the planning scene and attach it to the gripper for collision-aware planning.
    box_name: unique name for the object
    box_size: (x, y, z) size in meters
    box_pose: PoseStamped of the object (should be in the world or base frame)
    """
    scene.add_box(box_name, box_pose, size=box_size)
    rospy.sleep(timeout)
    eef_link = "wx250s/ee_gripper_link"
    scene.attach_box(eef_link, box_name)
    rospy.loginfo(f"[MoveIt] Attached {box_name} to {eef_link} in planning scene.")


left_contact  = None
right_contact = None

def build_goal(group_name: str,
                target_link: str,
                pose: PoseStamped,
                pos_tolerance: float = 0.001,
                orient_tolerance: float = 0.01,
                velocity_scale: float = 0.2,
                accel_scale: float = 0.2,
                planning_time: float = 5.0,
                attempts: int = 30) -> MoveGroupGoal:
    goal = MoveGroupGoal()
    goal.request.group_name                    = group_name
    goal.request.num_planning_attempts         = attempts
    goal.request.allowed_planning_time         = planning_time
    goal.request.max_velocity_scaling_factor   = velocity_scale
    goal.request.max_acceleration_scaling_factor = accel_scale

    pc = PositionConstraint()
    pc.header     = pose.header
    pc.link_name  = target_link

    box           = SolidPrimitive()
    box.type      = SolidPrimitive.BOX
    box.dimensions = [pos_tolerance] * 3

    region = BoundingVolume()
    region.primitives.append(box)
    region.primitive_poses.append(pose.pose)
    pc.constraint_region = region
    pc.weight            = 1.0

    oc = OrientationConstraint()
    oc.header  = pose.header
    oc.link_name = target_link
    oc.orientation = pose.pose.orientation
    oc.absolute_x_axis_tolerance = orient_tolerance
    oc.absolute_y_axis_tolerance = orient_tolerance
    oc.absolute_z_axis_tolerance = orient_tolerance
    oc.weight                    = 1.0

    goal.request.goal_constraints.append(
        Constraints(position_constraints=[pc],
                    orientation_constraints=[oc])
    )
    return goal


def move_to(x: float, y: float, z: float) -> None:
    target = PoseStamped()
    target.header.frame_id = "wx250s/base_link"
    target.header.stamp    = rospy.Time.now()
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z - 1.015

    roll, pitch, yaw = 0.0, math.pi / 2, 0.0
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    target.pose.orientation.x, target.pose.orientation.y, \
        target.pose.orientation.z, target.pose.orientation.w = q

    goal   = build_goal("interbotix_arm", "wx250s/ee_gripper_link", target)
    client = actionlib.SimpleActionClient(f"/wx250s/move_group",
                                            MoveGroupAction)
    rospy.loginfo("Waiting for /wx250s/move_group action server…")
    client.wait_for_server()
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(20.0)):
        client.cancel_goal()
        rospy.logfatal("Arm motion timed out.")
        sys.exit(1)

    if client.get_result().error_code.val == MoveItErrorCodes.SUCCESS:
        rospy.loginfo("✔  Arm reached Cartesian pose.")
    else:
        rospy.logerr("Arm MoveIt error: %d",
                        client.get_result().error_code.val)

roscpp_initialize(sys.argv)
_gripper = MoveGroupCommander("interbotix_gripper",
                                robot_description=f"/wx250s/robot_description",
                                ns="/wx250s")

def move_to_home_pos(timeout: float = 15.0) -> bool:
    arm = MoveGroupCommander("interbotix_arm",
                             robot_description=f"/wx250s/robot_description",
                             ns="/wx250s")
    arm.set_named_target("Home")
    rospy.loginfo("Moving arm to 'Home' position...")
    success = arm.go(wait=True)
    arm.stop()
    return success

def _joint_constraints_for(state_name: str,
                            tol: float = 1e-3) -> Constraints:
    named = _gripper.get_named_target_values(state_name)
    if not named:
        raise ValueError(f"Unknown gripper named state '{state_name}'")

    cons = Constraints()
    for j, pos in named.items():
        jc = JointConstraint()
        jc.joint_name      = j
        jc.position        = pos
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight          = 1.0
        cons.joint_constraints.append(jc)
    return cons

def _send_gripper_goal(state: str, timeout: float = 15.0) -> bool:
    client = actionlib.SimpleActionClient(f"/wx250s/move_group",
                                            MoveGroupAction)
    client.wait_for_server()
    goal = MoveGroupGoal()
    goal.request.group_name = "interbotix_gripper"
    goal.request.goal_constraints.append(_joint_constraints_for(state))
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(timeout)):
        client.cancel_goal()
        rospy.logwarn("Gripper goal timed out.")
        return False
    ok = client.get_result().error_code.val == MoveItErrorCodes.SUCCESS
    rospy.loginfo("Gripper → '%s'  %s", state,
                    "SUCCESS" if ok else "FAIL")
    return ok

def open_gripper()  -> bool: return _send_gripper_goal("Open")
def close_gripper() -> bool: return _send_gripper_goal("Closed")

def _tip_cb(msg: ContactsState, which: str) -> None:
    global left_contact, right_contact
    if which == 'left':
        left_contact = msg
    else:
        right_contact = msg

def _common_collision_name() -> (bool, str, str):
    if left_contact is None or right_contact is None:
        return False, '', ''

    def coll_set(cs):
        result = set()
        for s in cs.states:
            if hasattr(s, 'collision2_name') and s.collision2_name:
                if 'wx250s' in s.collision2_name:
                    if hasattr(s, 'collision1_name') and s.collision1_name:
                        result.add(s.collision1_name)
                else:
                    result.add(s.collision2_name)
        return result

    common = coll_set(left_contact).intersection(coll_set(right_contact))
    if not common:
        return False, '', ''
    full = next(iter(common))
    model, link, *_ = full.split('::')
    return True, model, link

def attach_object_when_grasped(timeout: float = 3.0):
    srv  = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    srv.wait_for_service()
    start = rospy.Time.now()
    rate  = rospy.Rate(50)

    while (rospy.Time.now() - start).to_sec() < timeout \
            and not rospy.is_shutdown():

        ready, model, link = _common_collision_name()
        if ready:
            for tip_link in ("wx250s/left_finger_link", "wx250s/right_finger_link"):
                req = AttachRequest()
                req.model_name_1 = "wx250s"
                req.link_name_1  = tip_link
                req.model_name_2 = model
                req.link_name_2  = link

                resp = srv(req)
                if resp.ok:
                    rospy.loginfo("Attached [%s] ↔ [%s::%s]",
                                    tip_link, model, link)
                else:
                    rospy.logerr("Attach failed for %s", tip_link)
                    return False

                rospy.sleep(0.02)
            return model, link
        rate.sleep()

    rospy.logwarn("No dual-tip contact within %.1f s – not attaching.", timeout)
    return None, None

def detach_object_when_released(model: str,
                                link: str,
                                timeout: float = 2.0) -> bool:
    srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    try:
        srv.wait_for_service(timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("✗ /link_attacher_node/detach not available")
        return False

    success = True
    for tip_link in ("wx250s/left_finger_link", "wx250s/right_finger_link"):
        req = AttachRequest()
        req.model_name_1 = "wx250s"
        req.link_name_1  = tip_link
        req.model_name_2 = model
        req.link_name_2  = link
        ok = srv(req).ok
        rospy.loginfo("%s detach %s",
                        "✔" if ok else "✗", tip_link)
        success &= ok
        rospy.sleep(0.02)
    return success

__all__ = [
    "move_to",
    "open_gripper",
    "close_gripper",
    "attach_object_when_grasped",
    "detach_object_when_released",
    "_tip_cb"
]