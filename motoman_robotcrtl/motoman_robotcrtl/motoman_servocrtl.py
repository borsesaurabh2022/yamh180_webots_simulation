import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from moveit_msgs.msg import ServoStatus
from std_srvs.srv import SetBool
import numpy as np
import time

servopause_initiated = False
jointjog_initiated = False 
twiststamp_initiated = False

class MoveitServoControl(Node):

    def __init__(self):
        super().__init__('motoman_servocrtl')
        self.create_subscription(Joy,'/joy',self.JoyMsgCallback, 1)
        #self.create_subscription(ServoStatus, '/servo_node/status',self.ServoStatusCb, 1)
        self.jointjog_publisher = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 1)
        self.twiststamp_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)
        self.posestamp_publisher = self.create_publisher(PoseStamped, '/servo_node/pose_target_cmds',1)
        self.servocommand_cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.servopause_cli = self.create_client(SetBool, '/servo_node/pause_servo')
        self.currentstateof_button0 = 0
        self.prevstateof_button0 = 0
        self.prevstateof_button3 = 0
        self.currentstateof_button3 = 0
        self.prevstateof_button8 = 0
        self.currentstateof_button8 = 0
        
    def JoyMsgCallback(self, msg): 
        global jointjog_initiated, twiststamp_initiated, servopause_initiated
        self.scaling_factor = 0.15
        self.scaling_factor_amp = self.scaling_factor*10
        self.currentstateof_button0 = msg.buttons[0]
        self.currentstateof_button3 = msg.buttons[3]
        self.currentstateof_button8 = msg.buttons[8]
        if not jointjog_initiated:
            if self.prevstateof_button0 == 0 and self.currentstateof_button0 == 1:
                cmd = 0
                self.SendServocmdTyp_Req(cmd)
        elif jointjog_initiated:
            j1 = msg.axes[0]*self.scaling_factor
            j2 = msg.axes[1]*self.scaling_factor
            j3 = msg.axes[3]*self.scaling_factor
            j4 = msg.axes[4]*self.scaling_factor
            if msg.axes[2] < 0:
                j5 = (msg.axes[2]+1)*self.scaling_factor_amp
            elif msg.axes[2] >= 0:
                j5 = (msg.axes[2]-1)*self.scaling_factor_amp
            if msg.axes[5] < 0:
                j6 = (msg.axes[5]+1)*self.scaling_factor_amp
            elif msg.axes[5] >= 0:
                j6 = (msg.axes[5]-1)*self.scaling_factor_amp          
            self.JointJog_Pub(j1, j2, j3, j4, j5, j6)
        if not twiststamp_initiated:
            if self.prevstateof_button3 == 0 and self.currentstateof_button3 == 1:
                cmd = 1
                self.SendServocmdTyp_Req(cmd)
        elif twiststamp_initiated:
            if msg.axes[0] == 0:
                xl = 0.0
            elif msg.axes[0] != 0:
                xl = np.sign(msg.axes[0])*0.4
            if msg.axes[1] == 0:
                yl = 0.0
            elif msg.axes[1] != 0:
                yl = np.sign(msg.axes[1])*0.4
            if msg.axes[2]-1 == 0:
                zl = 0.0
            elif -1 < (msg.axes[2]-1) < 0:
                zl = 0.4
            elif -2 <= (msg.axes[2]-1) <= -1:
                zl = -0.4
            if msg.axes[3] == 0:
                xa = 0.0
            elif msg.axes[3] != 0:
                xa = np.sign(msg.axes[3])*0.4
            if msg.axes[4] == 0:
                ya = 0.0
            elif msg.axes[4] != 0:
                ya = np.sign(msg.axes[3])*0.4
            if msg.axes[5]-1 == 0:
                za = 0.0
            elif -1 < (msg.axes[5]-1) < 0:
                za = 0.4
            elif -2 <= (msg.axes[5]-1) <= -1:
                za = -0.4
            self.TwistStamp_Pub(xl,yl,zl,xa,ya,za)
            
        if not servopause_initiated:
            if self.prevstateof_button8 == 0 and self.currentstateof_button8 == 1:
                cmd = True
                self.ServoPause_Req(cmd)
        elif servopause_initiated:
            if self.prevstateof_button8 == 1 and self.currentstateof_button8 == 1:
                cmd = False
                self.ServoPause_Req(cmd)
                
    def JointJog_Pub(self, j_1, j_2, j_3, j_4, j_5, j_6):
        joint_jog = JointJog()
        joint_jog.header.stamp = self.get_clock().now().to_msg()
        joint_jog.joint_names = ['group_1/joint_1',
                                 'group_1/joint_2',
                                 'group_1/joint_3',
                                 'group_1/joint_4',
                                 'group_1/joint_5',
                                 'group_1/joint_6']
        joint_jog.velocities = [j_1, j_2, j_3, j_4, j_5, j_6]
        self.jointjog_publisher.publish(joint_jog)
        
    def TwistStamp_Pub(self,x_l,y_l,z_l,x_a, y_a, z_a):
        twist_stamped = TwistStamped()
        twist_stamped.header.frame_id = "link_t"
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist.linear.x = x_l
        twist_stamped.twist.linear.y = y_l
        twist_stamped.twist.linear.z = z_l
        twist_stamped.twist.angular.x = x_a
        twist_stamped.twist.angular.y = y_a
        twist_stamped.twist.angular.z = z_a  
        self.twiststamp_publisher.publish(twist_stamped)
            
    def SendServocmdTyp_Req(self, cmd_val):
        while not self.servocommand_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Error reset service not available, trying again...')
        self.ServoCmdtyp_request = ServoCommandType.Request()
        self.ServoCmdtyp_request.command_type = cmd_val
        #self.get_logger().info('Initiated servo command type request')
        self.ServoCmdtyp_future = self.servocommand_cli.call_async(self.ServoCmdtyp_request)
        self.ServoCmdtyp_future.add_done_callback(self.ServoCmdtypSrv_cb)

    def ServoCmdtypSrv_cb(self, ServoCmdtyp_future):
        global jointjog_initiated, twiststamp_initiated
        self.ServoCmdtyp_result = ServoCmdtyp_future.result().success
        if self.ServoCmdtyp_result:
            if self.prevstateof_button0 == 0 and self.currentstateof_button0 == 1:
                jointjog_initiated = True
                twiststamp_initiated = False
                self.get_logger().info('Requested servo mode: Joint_jog intiated')
                self.prevstateof_button0 = 0
                self.currentstateof_button0 = 0
            elif self.prevstateof_button3 == 0 and self.currentstateof_button3 == 1:
                twiststamp_initiated = True
                jointjog_initiated = False
                self.get_logger().info('Requested servo mode: twist_stamped intiated')
                self.prevstateof_button3 = 0
                self.currentstateof_button3 = 0      
        else:
            self.get_logger().info('Failure in initializing the requested servo mode')
            
    def ServoPause_Req(self, cmd_val):
        while not self.servopause_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Error reset service not available, trying again...')
        self.ServoPause_request = SetBool.Request()
        self.ServoPause_request.data = cmd_val
        #self.get_logger().info('Initiated servo pause request')
        self.ServoPause_future = self.servopause_cli.call_async(self.ServoPause_request)
        self.ServoPause_future.add_done_callback(self.ServoPauseSrv_cb)
    
    def ServoPauseSrv_cb(self, ServoPause_future):
        global servopause_initiated
        self.ServoPause_result = ServoPause_future.result().success
        if self.ServoPause_result:
            if ServoPause_future.result().message == 'Servoing disabled':
                time.sleep(1)
                self.prevstateof_button8 = 1
                self.currentstateof_button8 = 0
                self.get_logger().info('Requested servo pause intiated')
                servopause_initiated = True
                self.get_logger().info(f'Response message: {ServoPause_future.result().message}')
            elif ServoPause_future.result().message == 'Servoing enabled':
                time.sleep(1)
                self.prevstateof_button8 = 0
                self.currentstateof_button8 = 0
                self.get_logger().info('Servo_pause deactivate request initiated')
                servopause_initiated = False
                self.get_logger().info(f'Response message: {ServoPause_future.result().message}')
        else:
            self.get_logger().info('Failure in initializing the requested servo pause')
    
def main(args=None):
    rclpy.init(args=args)
    motoman_servocrtl = MoveitServoControl()
    rclpy.spin(motoman_servocrtl)
    motoman_servocrtl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()