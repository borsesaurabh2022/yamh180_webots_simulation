#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from mh180_interface.action import ManiposeReq


class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__("mh180_movegroup_actioncli_node")
        # declre parameters for tf2 ros function
        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            JointState,
            "/mh180/joint_states",
            self.JointStates_cb,
            1,
            callback_group=cb_group,
        )
        self.mplan_action_client = ActionClient(self, MoveGroup, "/mh180/move_action", callback_group=cb_group)
        self.manipose_action_srv = ActionServer(
            self,
            ManiposeReq,
            "/mh180/manipose_req",
            self.ActionSrv_cb,
            callback_group=cb_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state = None
        self.sample_count = 20
        self.get_result_future_state = False

    def ActionSrv_cb(self, goal_handle):
        self.get_logger().info("[ActionSrv_cb]:Received and EXecuting motion plan goal.")
        self.current_goal_handle = goal_handle
        self.motionplan_req = True

        if self.current_goal_handle.request.data:
            self.pose = Point()
            self.quat = Quaternion()
            self.pose.x = self.current_goal_handle.request.tcp_pose[0]
            self.pose.y = self.current_goal_handle.request.tcp_pose[1]
            self.pose.z = self.current_goal_handle.request.tcp_pose[2]
            self.quat.x = self.current_goal_handle.request.tcp_pose[3]
            self.quat.y = self.current_goal_handle.request.tcp_pose[4]
            self.quat.z = self.current_goal_handle.request.tcp_pose[5]
            self.quat.w = self.current_goal_handle.request.tcp_pose[6]

            result = ManiposeReq.Result()

            mp_req = self.MoveplanRequestmessage(self.pose, self.quat)
            self.send_goal(mp_req)

            while True:
                if self.get_result_future_state:
                    if self.get_result_future.result().status == 4:
                        goal_handle.succeed()
                        self.current_goal_handle = None
                        result.response = True
                        result.message = "Motion plan goal succeeded"
                        self.motionplan_req = False
                        self.get_result_future_state = False
                        return result

                    elif self.get_result_future.result().status == 6:
                        self.current_goal_handle = None
                        goal_handle.abort()
                        result.response = False
                        result.message = "Motion plan goal unsuccessful"
                        self.motionplan_req = False
                        self.get_result_future_state = False
                        return result

    def JointStates_cb(self, joint_state):
        self.joint_state = joint_state

    def MoveplanRequestmessage(self, pose_, quat_):
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = "mh180/base_link"
        self.motion_plan_request.workspace_parameters.min_corner.x = -4.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -4.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -0.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 4.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 4.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 4.0
        self.motion_plan_request.start_state.is_diff = False

        self.motion_plan_request.start_state.joint_state = self.joint_state

        pc = PositionConstraint()
        pc.header.stamp = self.get_clock().now().to_msg()
        pc.header.frame_id = "mh180/base_link"
        pc.link_name = "mh180/tool0"
        pc.weight = 10.0
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = 2
        sp.dimensions = [0.001]
        bv.primitives.append(sp)

        ps = Pose()
        ps.position.x = pose_.x
        ps.position.y = pose_.y
        ps.position.z = pose_.z
        ps.orientation.x = quat_.x
        ps.orientation.y = quat_.y
        ps.orientation.z = quat_.z
        ps.orientation.w = quat_.w
        bv.primitive_poses.append(ps)
        pc.constraint_region = bv

        oc = OrientationConstraint()
        oc.header.stamp = self.get_clock().now().to_msg()
        oc.header.frame_id = "mh180/base_link"
        oc.link_name = "mh180/tool0"
        oc.weight = 10.0
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.orientation.x = quat_.x
        oc.orientation.y = quat_.y
        oc.orientation.z = quat_.z
        oc.orientation.w = quat_.w

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        self.motion_plan_request.goal_constraints.append(constraints)
        self.motion_plan_request.pipeline_id = "move_group"
        self.motion_plan_request.planner_id = "PTP"
        self.motion_plan_request.group_name = "motoman_mh180"
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        return self.motion_plan_request

    def PlanningOptions(self):
        self.planning_option = PlanningOptions()
        self.planning_option.plan_only = False
        self.planning_option.look_around = False
        self.planning_option.look_around_attempts = 0
        self.planning_option.max_safe_execution_cost = 0.0
        self.planning_option.replan = True
        self.planning_option.replan_attempts = 10
        self.planning_option.replan_delay = 0.1

    def send_goal(self, motion_seq_):
        try:
            self.PlanningOptions()
            goal_msg = MoveGroup.Goal()
            goal_msg.request = motion_seq_
            goal_msg.planning_options = self.planning_option
            self.mplan_action_client.wait_for_server()
            self._send_goal_future = self.mplan_action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goalresponse_cb)
        except Exception as e:
            print("[send_goal]: Exception in sending the goal to motion plan request", e)

    def goalresponse_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("[goalresponse_cb]: Motion-plan goal rejected by server")
            return
        self.get_logger().info("[goalresponse_cb]: Motion-plan goal accepted by server")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.getresult_cb)

    def getresult_cb(self, future):
        status = future.result().status
        self.get_result_future_state = True
        if status == 4:
            self.get_logger().info("[getresult_cb]: Motion-plan goal succeeded ")
        elif status == 6:
            self.get_logger().info("[getresult_cb]: Motion plan goal unsuccessful")


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveGroupActionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)
    try:
        action_client.get_logger().info("Beginning client, shut down with CTRL-C")
        executor.spin()
    except KeyboardInterrupt:
        action_client.get_logger().info("Keyboard interrupt, shutting down.\n")
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
