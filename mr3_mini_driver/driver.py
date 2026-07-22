import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from modena_common.enums import JogType
from modena_interfaces.msg import JogCmd
from modena_interfaces.msg import RobotState
from mr3_mini_driver.comm import MR3MiniComm, Message
import math


class MR3MiniDriver(Node):
    def __init__(self):
        super().__init__("mr3_mini_driver")

        # Current joint positions
        self.joint_pos = None

        # Current tool pose
        self.tool_pose = None
        
        # Joint names
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]

        # Publishes joint states received from the robot controller
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        
        # Publishes robot state in user-friendly units
        self.robot_state_pub = self.create_publisher(RobotState, "robot_state", 10)

        # Subscribes to receive jog commands
        self.jog_cmd_sub = self.create_subscription(JogCmd, "jog_cmd", self.jog_callback, 10)        

        # Indicates if the robot is currently being jogged
        self.is_jogging = False
        
        # Announces node creation
        self.get_logger().info("MR-3 Mini driver started")

        # Communication object
        self.comm = MR3MiniComm()

        # Initializes UART communication
        self.comm.init_uart()
        self.get_logger().info("Connection successfully established")

        # Timer for UART messages
        self.uart_timer = self.create_timer(0.005, self.read_uart)        

    def jog_callback(self, msg):
        if msg.type != JogType.JOG_NONE:  # if a valid button was pressed
            if not self.is_jogging:       # calls jog_start only once
                self.is_jogging = True
                self.comm.jog_start(msg)

        else:                             # if an invalid or no button was pressed
            if self.is_jogging:           # calls jog_stop only once
                self.is_jogging = False
                self.comm.jog_stop()

    def read_uart(self):
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
        pass  # Ctrl + C received: shuts down cleanly
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
