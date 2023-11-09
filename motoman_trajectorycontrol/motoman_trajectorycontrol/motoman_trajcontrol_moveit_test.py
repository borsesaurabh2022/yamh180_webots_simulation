import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest, JointConstraint, Constraints, PlanningOptions, RobotState
from moveit_msgs.action import MoveGroup
from copy import deepcopy
from industrial_msgs.msg import RobotStatus
from motoros2_interfaces.srv import ResetError, StartTrajMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from motoman_interface.srv import JointangleReq
from geometry_msgs.msg import Point, Quaternion

global_joint_states = None
initial_jointangle = False
userdefined_jointangle = False
motionplan_request_ready = True
trajmode_request_initiated = False
errorreset_request_initiated = False

class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('motoman_follow_joint_trajectory')

        # declre parameters for tf2 ros function
        self.create_subscription(JointState, '/joint_states', self.JointStates_cb,1)
        #self.create_subscription(RobotStatus, 'robot_status', self.RobotStatus_cb,1)
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        #self.ErrorReset_client = self.create_client(ResetError, 'reset_error')
        #self.StartTrajMode_client = self.create_client(StartTrajMode, 'start_traj_mode')
        self.getjointangles_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.TcpPose_srv = self.create_service(JointangleReq, '/tcppose_req', self.TcpPoseSrv_cb)

        
    def TcpPoseSrv_cb(self, request, response):
        global userdefined_jointangle
        if request.data == 'True':
            #self.get_logger().info('Incoming request\na: %d b: %d' % (request.data, request.tcp_pose))
            userdefined_jointangle = True
            pose = Point()
            quat = Quaternion()
            pose.x = request.tcp_pose[0]
            pose.y = request.tcp_pose[1]
            pose.z = request.tcp_pose[2]
            quat.x = request.tcp_pose[3]
            quat.y = request.tcp_pose[4]
            quat.z = request.tcp_pose[5]
            quat.w = request.tcp_pose[6]
            self.GetJointangles(pose, quat)
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
            print("Exception in PositionIK claculation request:", e)
            
    def joint_angle_calculation_cb(self, joint_angle_future):
        global motionplan_request_ready
        try:
            result = joint_angle_future.result()
            error_code = joint_angle_future.result().error_code.val
            self.joint_angle_results = result.solution.joint_state.position
            #self.get_logger().info('Joint angle error code', error_code)
            #self.get_logger().info('Joint angle result', result)
            #self.get_logger().info('Joint angle result_reused variable', self.joint_angle_results)
            if (error_code == 1) and  motionplan_request_ready == True:
                self.send_goal()
           # else:
                #self.GetJointangles(self.pose,self.quat)
        except Exception as e:
            print("Exception in jointangle calculation:",e)
                                       
    def JointStates_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state
        
    def MoveplanRequestmessage(self):
        global initial_jointangle, userdefined_jointangle
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True
        self.motion_plan_request.start_state.joint_state = self.joint_state

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        if initial_jointangle:
            joints = {}
            joints['group_1/joint_1'] = 0.5
            joints['group_1/joint_2'] = 0.0
            joints['group_1/joint_3'] = 0.5
            joints['group_1/joint_4'] = 0.0
            joints['group_1/joint_5'] = 0.0
            joints['group_1/joint_6'] = 0.0

        elif userdefined_jointangle:
            joints = {}
            joints['group_1/joint_1'] = self.joint_angle_results[0]
            joints['group_1/joint_2'] = self.joint_angle_results[1]
            joints['group_1/joint_3'] = self.joint_angle_results[2]
            joints['group_1/joint_4'] = self.joint_angle_results[3]
            joints['group_1/joint_5'] = self.joint_angle_results[4]
            joints['group_1/joint_6'] = self.joint_angle_results[5]

        constraints = Constraints()
        for (joint, angle) in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)
        self.motion_plan_request.pipeline_id = 'ompl'
        self.motion_plan_request.group_name = 'motoman_mh180'
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0
    
    def PlanningOptions(self):
        self.planning_option = PlanningOptions()
        self.planning_option.plan_only = False
        self.planning_option.look_around = False
        self.planning_option.look_around_attempts = 0
        self.planning_option.max_safe_execution_cost = 0.
        self.planning_option.replan = True
        self.planning_option.replan_attempts = 10
        self.planning_option.replan_delay = 0.1
    
    def send_goal(self):
        global motionplan_request_ready
        try:
            motionplan_request_ready = False
            self.MoveplanRequestmessage()
            self.PlanningOptions()
            goal_msg = MoveGroup.Goal()
            goal_msg.request = self.motion_plan_request
            goal_msg.planning_options = self.planning_option
            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
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
        if status == 4:
            motionplan_request_ready = True
            self.get_logger().info('Motion plan goal succeeded')
            if initial_jointangle:  
                self.get_logger().info('Initial_jointangle pose reached')
                self.get_logger().info('waiting for userdefined goal pose')
                initial_jointangle = False
            elif userdefined_jointangle:
                self.get_logger().info('userdefined pose reached')
                self.get_logger().info('waiting for the next userdefined goal pose')
                userdefined_jointangle = False
        elif status == 6:
            self.get_logger().info('Motion plan goal unsuccessful')
            motionplan_request_ready = True

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveGroupActionClient()
    while global_joint_states is None:
        rclpy.spin_once(action_client)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()