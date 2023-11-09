
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
trajmode_request_initiated = False
errorreset_request_initiated = False

class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('motoman_follow_joint_trajectory')

        # declre parameters for tf2 ros function
        self.create_subscription(JointState, 'joint_states', self.JointStates_cb,1)
        self.create_subscription(RobotStatus, 'robot_status', self.RobotStatus_cb,1)
        self.FollowJointTraj_actionclient = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')
        self.ErrorReset_client = self.create_client(ResetError, 'reset_error')
        self.StartTrajMode_client = self.create_client(StartTrajMode, 'start_traj_mode')   
        self.TcpPose_srv = self.create_service(JointangleReq, '/tcppose_req', self.TcpPoseSrv_cb)  
    
    def TcpPoseSrv_cb(self, request, response):
        global userdefined_jointangle
        if request.data == 'True':
            userdefined_jointangle = True
            self.pose = Point()
            self.quat = Quaternion()
            self.pose.x = request.tcp_pose[0]
            self.pose.y = request.tcp_pose[1]
            self.pose.z = request.tcp_pose[2]
            self.quat.x = request.tcp_pose[3]
            self.quat.y = request.tcp_pose[4]
            self.quat.z = request.tcp_pose[5]
            self.quat.w = request.tcp_pose[6]
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
        global jointangles_request_ready, motionplan_request_ready
        try:
            result = joint_angle_future.result()
            error_code = joint_angle_future.result().error_code.val
            self.joint_angle_results = result.solution.joint_state
            self.get_logger().info(f'Joint angle error code: {error_code}')
            self.get_logger().info(f'Joint angle result: {result}')
            self.get_logger().info(f'Joint angle result_reused variable: {self.joint_angle_results}')
            if (error_code == 1) and  motionplan_request_ready == True:
                self.send_goal()
            else:
                self.GetJointangles(self.pose,self.quat)
        except Exception as e:
            print("There is exception in joint angle calculation:",e)

    def RobotStatus_cb(self, msg):
        global trajmode_request_initiated, errorreset_request_initiated, motionplan_request_ready
        if motionplan_request_ready:
            if not trajmode_request_initiated:
                if not errorreset_request_initiated:
                    self.get_logger().info('Received RobotStatus message:')
                    self.get_logger().info(f'E-Stopped: {msg.e_stopped}')
                    self.get_logger().info(f'Drives Powered: {msg.drives_powered}')
                    self.get_logger().info(f'Motion Possible: {msg.motion_possible}')
                    self.get_logger().info(f'In Motion: {msg.in_motion}')
                    self.get_logger().info(f'Error state: {msg.in_error}')
                    if msg.in_error:
                        self.get_logger().info('Robot in error state')
                        self.get_logger().info(f'Error code: {msg.error_codes}')
                        self.SendErrorResetReq()
                        errorreset_request_initiated = True
                    if msg.mode == "AUTO":
                        if not msg.e_stopped:
                            if not msg.in_error:
                                if not msg.in_motion:
                                    trajmode_request_initiated = True
                                    self.SendStartTrajModeReq()
        
    def SendErrorResetReq(self):
        while not self.ErrorReset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Error reset service not available, trying again...')
        self.ErrorReset_request = ResetError.Request()
        self.ErrorReset_request.reset = True
        self.get_logger().info('Initiated ErrorReset request')
        self.ErrorReset_future = self.ErrorReset_client.call_async(self.ErrorReset_request)
        self.ErrorReset_future.add_done_callback(self.ErrorResetSrv_cb)
        
    def ErrorResetSrv_cb(self):
        global errorreset_request_initiated
        self.ErrorReset_result = self.ErrorReset_future.result().success
        if self.ErrorReset_result:
            self.get_logger().info('Reset error success')
            errorreset_request_initiated = False
        else:
            self.get_logger().info('Reset error failed')
            errorreset_request_initiated = False          
            
    def SendStartTrajModeReq(self):
        while not self.StartTrajMode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Trjectory mode set service not available, trying again...')
        self.StartTrajMode_request = StartTrajMode.Request()
        self.StartTrajMode_request.mode = StartTrajMode.Request.MODE_TRAJECTORY
        self.get_logger().info('Initiated Trajectory mode')
        self.StartTrajMode_future = self.StartTrajMode_client.call_async(self.StartTrajMode_request)
        self.StartTrajMode_future.add_done_callback(self.ErrorResetSrv_cb)

    def TrajModeSrv_cb(self):
        global  trajmode_request_initiated, initial_jointangle, userdefined_jointangle
        self.StartTrajMode_result = self.StartTrajMode_future.result().success
        if self.StartTrajMode_result:
            self.get_logger().info('Trejectory mode initiation successfull')
            if initial_jointangle == True and userdefined_jointangle == False:
                self.send_goal()
                trajmode_request_initiated = False 
            elif userdefined_jointangle == True and initial_jointangle == False:
                trajmode_request_initiated = False 
                self.GetJointangles(self.pose,self.quat)
            elif initial_jointangle == False and userdefined_jointangle == False:
                self.get_logger().info('goal pose already reached')
                self.get_logger().info('waiting for the next goal pose')
        else:
            self.get_logger().info('Trejectory mode initiation unsuccessfull')
            trajmode_request_initiated = False

    def JointStates_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state
        
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
        self.start_state.positions = self.joint_state.position
        self.start_state.velocities = self.joint_state.velocity
        self.start_state.time_from_start = rclpy.time.Duration(seconds=1.0)
        self.joint_traj.points.append(self.start_state)
        
        self.end_state = JointTrajectoryPoint()
        if initial_jointangle:
            self.end_state.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.end_state.velocities = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            #self.end_state.time_from_start = rclpy.time.Duration(seconds=1.0)
            self.end_state.time_from_start.sec = 5
            self.joint_traj.points.append(self.end_state)
            
        elif userdefined_jointangle:
            self.end_state.positions = self.joint_angle_results.position
            self.end_state.velocities = self.joint_angle_results.velocity
            #self.end_state.time_from_start = rclpy.time.Duration(seconds=1.0)
            self.end_state.time_from_start.sec = 5
            self.joint_traj.points.append(self.end_state)
            
    def send_goal(self):
        global motionplan_request_ready
        try:
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
        self.get_logger().info(f'Goal_status: {status}')
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
    