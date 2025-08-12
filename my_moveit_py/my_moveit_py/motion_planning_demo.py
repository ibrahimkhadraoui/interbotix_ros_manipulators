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

def main(args=None):
    rclpy.init(args=args)
    log = get_logger("moveit_py.demo")
    helper = rclpy.create_node("moveit_py_helper")

    robot = MoveItPy(node_name="moveit_py_demo")
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

    tip_link = "wx200/ee_gripper_link"
    arm = robot.get_planning_component(GROUP)

    # Wait a fixed amount of time for controllers/joints to be ready
    log.info("Sleeping 20 seconds to allow controllers and joint states to start...")
    time.sleep(10)

    # Named target
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="Upright")
    log.info("Planning to named target: Upright")
    plan_named = arm.plan()
    if plan_named and getattr(plan_named, "trajectory", None):
        log.info("Executing Upright...")
        robot.execute(GROUP, plan_named.trajectory, True)
    else:
        log.warning("Planning to 'Upright' returned no trajectory; continuing anyway.")

    # # Pose goal via /compute_ik
    # pose = PoseStamped()
    # pose.header.frame_id = "wx200/base_link"
    # pose.pose.position.x = 0.30
    # pose.pose.position.y = 0.10
    # pose.pose.position.z = 0.20
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0

    # ik_cli = helper.create_client(GetPositionIK, "/compute_ik")
    # if not ik_cli.wait_for_service(timeout_sec=5.0):
    #     log.error("Service /compute_ik not available.")
    # else:
    #     req = GetPositionIK.Request()
    #     req.ik_request = PositionIKRequest(
    #         group_name=GROUP,
    #         pose_stamped=pose,
    #         timeout=Duration(sec=1, nanosec=0),
    #         ik_link_name=tip_link
    #     )
    #     fut = ik_cli.call_async(req)
    #     rclpy.spin_until_future_complete(helper, fut, timeout_sec=5.0)
    #     if not fut.result():
    #         log.error("IK call failed (no response).")
    #     else:
    #         res = fut.result()
    #         if res.error_code.val == 1:
    #             js = res.solution.joint_state
    #             jmg = model.get_joint_model_group(GROUP)
    #             arm_joint_names = list(jmg.active_joint_model_names)
    #             targets = {n: p for n, p in zip(js.name, js.position) if n in arm_joint_names}

    #             if targets:
    #                 arm.set_start_state_to_current_state()
    #                 arm.set_goal_state(joint_space_goal=targets)
    #                 log.info("Planning to IK joint solution...")
    #                 plan_pose = arm.plan()
    #                 if plan_pose and getattr(plan_pose, "trajectory", None):
    #                     log.info("Executing pose IK joint goal...")
    #                     robot.execute(GROUP, plan_pose.trajectory, True)
    #                 else:
    #                     log.error("Planning to IK joint solution failed.")
    #             else:
    #                 log.error("IK returned no matching arm joints.")
    #         else:
    #             log.error(f"IK failed with error_code={res.error_code.val}")

    log.info("Demo finished.")
    helper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
