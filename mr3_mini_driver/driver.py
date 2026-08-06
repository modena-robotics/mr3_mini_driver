import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from modena_common.enums import JogType
from modena_interfaces.msg import JogCmd
from modena_interfaces.msg import RobotState
from mr3_mini_driver.comm import MR3MiniComm, Message, MoveResult
import math


class MR3MiniDriver(Node):
    def __init__(self):
        super().__init__("mr3_mini_driver")

        self.joint_pos = None
        self.tool_pose = None
        self.is_jogging = False
                
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]

        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)        
        self.robot_state_pub = self.create_publisher(RobotState, "robot_state", 10)
        self.jog_cmd_sub = self.create_subscription(JogCmd, "jog_cmd", self.jog_callback, 10)        
        
        self.get_logger().info("MR-3 Mini driver started")

        self.comm = MR3MiniComm()
        self.comm.init_uart()
        self.get_logger().info("Connection successfully established")

        self.uart_timer = self.create_timer(0.005, self.read_uart)        

    def jog_callback(self, msg):
        if msg.type != JogType.JOG_NONE:
            if not self.is_jogging:
                self.is_jogging = True
                self.comm.jog_start(msg)

        else:
            if self.is_jogging:
                self.is_jogging = False
                self.comm.jog_stop()

    def read_uart(self):
        # Read and process all complete UART packets currently available
        while True:
            uart_msg = self.comm.read_and_parse()

            if uart_msg is None:
                break

            cmd = uart_msg[0]
            data = uart_msg[1]

            if cmd == Message.MSG_ROBOT_STATE:
                self.joint_pos = data[0]
                self.tool_pose = data[1]
                
                state = RobotState()
                state.joint_pos = self.joint_pos
                state.tool_pose = self.tool_pose
                self.robot_state_pub.publish(state)
                
                msg = JointState()
                msg.name = self.joint_names
                msg.position = [math.radians(self.joint_pos[0]), math.radians(self.joint_pos[1]), math.radians(self.joint_pos[2]), math.radians(self.joint_pos[3])]
                msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_pub.publish(msg)
            
            elif cmd == Message.MSG_MOVE_RESULT:
                match data:             
                    # Jogging cannot be started because a physical limit would be exceeded
                    case MoveResult.MOVE_J1_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Joint 1 limit is too close")
                    case MoveResult.MOVE_J2_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Joint 2 limit is too close")
                    case MoveResult.MOVE_J3_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Joint 3 limit is too close")
                    case MoveResult.MOVE_J4_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Joint 4 limit is too close")
                    case MoveResult.MOVE_MOTOR_3_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Motor 3 joint limit is too close")
                    case MoveResult.MOVE_PASSIVE_JOINT_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Passive joint limit is too close")
                    case MoveResult.MOVE_WORKSPACE_JOG_LIMIT_PROXIMITY:
                        self.get_logger().warning("Cannot start jogging: Workspace limit is too close")
                    
                    # Requested jog speed, velocity, or acceleration exceeds the limit
                    case MoveResult.MOVE_JOG_SPEED_OUT_OF_RANGE:
                        self.get_logger().warning("Cannot start jogging: requested jog speed is outside the valid range (1-100%)")
                    case MoveResult.MOVE_JOINT_JOG_VEL_LIMIT_EXCEEDED:
                        self.get_logger().warning("Cannot start jogging: requested joint jog velocity exceeds the limit")
                    case MoveResult.MOVE_JOINT_JOG_ACC_LIMIT_EXCEEDED:
                        self.get_logger().warning("Cannot start jogging: requested joint jog acceleration exceeds the limit")
                    case MoveResult.MOVE_LINEAR_JOG_VEL_LIMIT_EXCEEDED:
                        self.get_logger().warning("Cannot start jogging: requested linear jog velocity exceeds the limit")
                    case MoveResult.MOVE_LINEAR_JOG_ACC_LIMIT_EXCEEDED:
                        self.get_logger().warning("Cannot start jogging: requested linear jog acceleration exceeds the limit")  
            
            elif cmd == Message.MSG_UNKNOWN:
                self.get_logger().warning(f"Unknown message code received: {cmd}")                  
                                                             
    def destroy_node(self):
        self.uart_timer.cancel()
        
        try:
            self.comm.end_uart()
        finally:
            super().destroy_node()       


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = MR3MiniDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:        
        if node is not None:
            node.get_logger().error(f"Unexpected error: {e}")
        else:
            print(f"Unexpected error: {e}")
    finally:     
        if node is not None:
            node.destroy_node()
        
        if rclpy.ok():              
            rclpy.shutdown()
        

if __name__ == "__main__":
    main()
