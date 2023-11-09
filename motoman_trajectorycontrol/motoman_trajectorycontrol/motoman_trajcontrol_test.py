
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from copy import deepcopy
import numpy as np
from motoros2_interfaces.srv import ResetError, StartTrajMode
from control_msgs.action import FollowJointTrajectory
from industrial_msgs.msg import RobotStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from motoman_interface.srv import JointangleReq
from geometry_msgs.msg import Point, Quaternion

global_joint_states = None
initial_jointangle = True
userdefined_jointangle = False
motionplan_request_ready = True

class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('motoman_follow_joint_trajectory')

        # declre parameters for tf2 ros function
        self.create_subscription(JointState, '/joint_states', self.JointStates_cb,1)
        self.FollowJointTraj_actionclient = ActionClient(self, FollowJointTrajectory, '/motoman_controller/follow_joint_trajectory') 
        self.TcpPose_srv = self.create_service(JointangleReq, '/tcppose_req', self.TcpPoseSrv_cb) 
        self.send_goal()
    
    def TcpPoseSrv_cb(self, request, response):
        global userdefined_jointangle
        if request.data:
            self.get_logger().info('Incoming request\na: %d b: %d' % (request.request, request.tcp_pose))
            userdefined_jointangle = True
            self.pose = Point()
            self.quat = Quaternion()
            self.pose.x = request.tcp_pose(0)
            self.pose.y = request.tcp_pose(1)
            self.pose.z = request.tcp_pose(2)
            self.quat.x = request.tcp_pose(3)
            self.quat.y = request.tcp_pose(4)
            self.quat.z = request.tcp_pose(5)
            self.quat.w = request.tcp_pose(6)
            response.response = True
            response.message = "Request Initiated"
        return response
    
    def GetJointangles(self,pose_,quat_):
        global global_joint_states
        try:
            get_joint_angle_request = PositionIKRequest()
            get_joint_angle_request.group_name = 'motoman_mh180'
            get_joint_angle_request.avoid_collisions = True

            robot_state = RobotState()
            robot_state.joint_state = global_joint_states
            robot_state.is_diff = True
            get_joint_angle_request.robot_state = robot_state
         
            posestamped = PoseStamped()
            posestamped.header.stamp = self.get_clock().now().to_msg()
            posestamped.header.frame_id = 'base_link'
            posestamped.pose.position.x = pose_.x
            posestamped.pose.position.y = pose_.y
            posestamped.pose.position.z = pose_.z
            posestamped.pose.orientation.x = quat_.x
            posestamped.pose.orientation.y = quat_.y
            posestamped.pose.orientation.z = quat_.z
            posestamped.pose.orientation.w = quat_.w
            get_joint_angle_request.pose_stamped = posestamped
            
           
            get_joint_angle_request.timeout.nanosec = 100

            get_poistionik = GetPositionIK.Request()
            get_poistionik.ik_request = get_joint_angle_request
            self.joint_angle_future = self.getjointangles_cli.call_async(get_poistionik)
            self.get_logger().info('Joint angle calculation request initiated')
            self.joint_angle_future.add_done_callback(self.joint_angle_calculation_cb)
        except Exception as e:
            print("Exception in Position IK claculation request:", e)
            
    def joint_angle_calculation_cb(self, joint_angle_future):
        global motionplan_request_ready
        try:
            result = joint_angle_future.result()
            error_code = joint_angle_future.result().error_code.val
            self.joint_angle_results = result.solution.joint_state
            self.get_logger().info('Joint angle error code', error_code)
            self.get_logger().info('Joint angle result', result)
            self.get_logger().info('Joint angle result_reused variable', self.joint_angle_results)
            if (error_code == 1) and  motionplan_request_ready == True:
                self.send_goal()
            else:
                self.GetJointangles(self.pose,self.quat)
        except Exception as e:
            print("There is exception in joint angle calculation:",e)

    def JointStates_cb(self, msg):
        global global_joint_states
        global_joint_states = msg
        self.joint_states = msg
        
    def JointTrajMsg(self):
        self.joint_traj = JointTrajectory()
        self.joint_traj.header.stamp = self.get_clock().now().to_msg()
        self.joint_traj.header.frame_id = 'base_link'
        self.joint_traj.joint_names = [ "group_1/joint_1", 
                                        "group_1/joint_2", 
                                        "group_1/joint_3", 
                                        "group_1/joint_4", 
                                        "group_1/joint_5",
                                        "group_1/joint_6"]
        self.start_state = JointTrajectoryPoint()
        self.start_state.positions = self.joint_states.position
        self.start_state.velocities = self.joint_states.velocity
        self.start_state.time_from_start.sec = 1
        self.joint_traj.points.append(self.start_state)
        
        self.end_state = JointTrajectoryPoint()
        if initial_jointangle:
            self.end_state.positions = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.end_state.velocities = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            self.end_state.time_from_start.sec = 5
            self.joint_traj.points.append(self.end_state)
            
        elif userdefined_jointangle:
            self.end_state.positions = self.joint_angle_results.position
            self.end_state.velocities = self.joint_angle_results.velocity
            self.end_state.time_from_start.sec = 5 
            self.joint_traj.points.append(self.end_state)
            
    def send_goal(self):
        global motionplan_request_ready
        try:
            print("Initiating the goal pose")
            motionplan_request_ready = False
            self.goal_msg = FollowJointTrajectory.Goal()
            self.JointTrajMsg()
            self.goal_msg.trajectory = self.joint_traj
            self.goal_msg.goal_time_tolerance.sec = 1
            self.FollowJointTraj_actionclient.wait_for_server()
            self.send_goal_future = self.FollowJointTraj_actionclient.send_goal_async(self.goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            print("Exception in sending the goal to motion plan request", e)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
            
    def get_result_callback(self, future):
        global motionplan_request_ready, initial_jointangle, userdefined_jointangle
        status = future.result().status
        self.get_logger().info('Goal_status', status)
        if status == 4:
            motionplan_request_ready = True
            self.get_logger().info('Motion plan goal succeeded')
            if initial_jointangle:  
                self.get_logger().info('Initial_jointangle pose reached')
                self.get_logger().info('waiting for userdefined_jointangle goal pose')
                initial_jointangle = False
            elif userdefined_jointangle:
                self.get_logger().info('userdefined_jointangle pose reached')
                self.get_logger().info('waiting for the next userdefined_jointangle goal pose')
                userdefined_jointangle = False
        elif status == 6:
            self.get_logger().info('Motion plan goal unsuccessful')
            motionplan_request_ready = True

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveGroupActionClient()
    while global_joint_states is None:
        rclpy.spin_once(action_client)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()


# """Trajectory follower client for the UR5 robot used for multi-robot demonstration."""

# import rclpy
#from webots_ros2_universal_robot.follow_joint_trajectory_client import FollowJointTrajectoryClient


# GOAL = {
#     'joint_names': [
#         'group_1/joint_1',
#         'group_1/joint_2',
#         'group_1/joint_3',
#         'group_1/joint_4',
#         'group_1/joint_5',
#         'group_1/joint_6',
#     ],
#     'points': [
#         {
#             'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495],
#             'time_from_start': {'sec': 0, 'nanosec': 0}
#         },
#         {
#             'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495],
#             'time_from_start': {'sec': 3, 'nanosec': 0}
#         },
#         {
#             'positions': [0.0, 0.0, 0.0, 0.0, 0.85, 0.85],
#             'time_from_start': {'sec': 4, 'nanosec': 0}
#         },
#         {
#             'positions': [0.63, -2.26, -1.88, -2.14, 0.85, 0.85],
#             'time_from_start': {'sec': 5, 'nanosec': 0}
#         },
#         {
#             'positions': [0.63, -2.26, -1.88, -2.14, 0.0495, 0.0495],
#             'time_from_start': {'sec': 6, 'nanosec': 0}
#         },
#         {
#             'positions': [0.63, -2.0, -1.88, -2.14, 0.0495, 0.0495],
#             'time_from_start': {'sec': 7, 'nanosec': 0}
#         },
#         {
#             'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495],
#             'time_from_start': {'sec': 8, 'nanosec': 0}
#         },
#         {
#             'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495],
#             'time_from_start': {'sec': 9, 'nanosec': 0}
#         }
#     ]
# }


# def main(args=None):
#     rclpy.init(args=args)
#     controller = FollowJointTrajectoryClient('motoman_controller', 'motoman_controller')

#     controller.send_goal(GOAL, 10)
#     rclpy.spin(controller)


# if __name__ == '__main__':
#     main()