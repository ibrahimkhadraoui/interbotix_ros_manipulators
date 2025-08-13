#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from builtin_interfaces.msg import Duration
from math import pi
from tf_transformations import quaternion_from_euler


def main(args=None):
    rclpy.init(args=args)
    log = get_logger("moveit_py.demo")
    helper = rclpy.create_node("moveit_py_helper")

    robot = MoveItPy(node_name="moveit_py")
    model = robot.get_robot_model()

    try:
        group_names = list(model.joint_model_group_names)
    except AttributeError:
        group_names = [g.get_name() for g in model.joint_model_groups]
    log.info(f"Available joint model groups: {group_names}")

    GROUP = "interbotix_arm"
    if GROUP not in group_names:
        log.error(f"Selected group '{GROUP}' not found. Available: {group_names}")
        helper.destroy_node(); rclpy.shutdown(); sys.exit(1)

    jmg = model.get_joint_model_group(GROUP)
    log.debug(f"Joint model group: {jmg}")
    tip_link = jmg.link_model_names[-1]   # last link is typically the tip
    log.info(f"Using tip link: {tip_link}")

    arm = robot.get_planning_component(GROUP)

    # Wait a fixed amount of time for controllers/joints to be ready
    log.info("Sleeping 8 seconds to allow controllers and joint states to start...")
    time.sleep(8)

    # Named target
    # arm.set_start_state_to_current_state()
    # arm.set_goal_state(configuration_name="Sleep")
    # log.info("Planning to named target: Sleep")
    # plan_named = arm.plan()
    # if plan_named and getattr(plan_named, "trajectory", None):
    #     log.info("Executing Sleep...")
    #     robot.execute(GROUP, plan_named.trajectory, True)
    # else:
    #     log.warning("Planning to 'Sleep' returned no trajectory; continuing anyway.")

    # Pose goal via /compute_ik
    pose = PoseStamped()
    pose.header.frame_id = "wx200/base_link"     # verify planning frame below if needed
    pose.pose.position.x = 0.05                  # reachable forward
    pose.pose.position.y = 0.00
    pose.pose.position.z = 0.67

    if quaternion_from_euler:
        qx, qy, qz, qw = quaternion_from_euler(-0.038, -1.57, 0.038)  # yaw/pitch/roll
        log.info(f"[my_moveit_py] Using quaternion from euler: {qx}, {qy}, {qz}, {qw}")
    else:
        # identity quaternion if tf_transformations not present
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        log.info("[my_moveit_py] tf_transformations not available; using identity quaternion.")

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    # arm.set_start_state_to_current_state()
    # Optional: quick IK pre-check against move_group's /compute_ik service
    ik_ok = True
    try:
        cli = helper.create_client(GetPositionIK, '/compute_ik')
        if not cli.wait_for_service(timeout_sec=5.0):
            log.warn("IK service /compute_ik not available; skipping pre-check.")
        else:
            req = GetPositionIK.Request()
            req.ik_request = PositionIKRequest()
            req.ik_request.group_name = GROUP
            req.ik_request.ik_link_name = tip_link
            req.ik_request.pose_stamped = pose
            req.ik_request.timeout = Duration(sec=1, nanosec=0)

            resp = cli.call(req)
            ik_ok = (resp.error_code.val == 1)  # SUCCESS
            log.info(f"IK pre-check: {'SUCCESS' if ik_ok else f'ERROR code {resp.error_code.val}'}")
    except Exception as e:
        log.warn(f"IK pre-check failed with exception: {e}")

    # Plan & execute only if IK looks doable (or if you want, proceed anyway)
    arm.set_goal_state(pose_stamped_msg=pose, pose_link=tip_link)
    plan_result = arm.plan()
    traj = getattr(plan_result, "trajectory", None)
    if traj:
        log.info("Executing pose goal...")
        robot.execute(GROUP, traj, True)
    else:
        log.warning("Planning to pose goal returned no trajectory.")

    log.info("Demo finished.")
    helper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
