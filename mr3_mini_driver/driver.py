import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from modena_common.enums import JogType
from modena_interfaces.msg import JogCmd, JogStatus, RobotState
from modena_interfaces.action import MoveHome, MoveRest, MoveAbsJ, MoveJ, MoveL, WaitTime
from mr3_mini_driver.comm import MR3MiniComm, Message, MoveResult, Fault, Warning
import math
import threading
import time


UART_THREAD_PERIOD = 0.005
HEARTBEAT_PERIOD = 0.1
HEARTBEAT_PERIOD_TICKS = round(HEARTBEAT_PERIOD/UART_THREAD_PERIOD)


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
        self.jog_status_pub = self.create_publisher(JogStatus, "jog_status", 10)
        
        self.move_home_srv = ActionServer(self, MoveHome, "move_home", self.move_home_callback)
        self.move_rest_srv = ActionServer(self, MoveRest, "move_rest", self.move_rest_callback)
        self.move_abs_j_srv = ActionServer(self, MoveAbsJ, "move_abs_j", self.move_abs_j_callback)
        self.move_j_srv = ActionServer(self, MoveJ, "move_j", self.move_j_callback)
        self.move_l_srv = ActionServer(self, MoveL, "move_l", self.move_l_callback)
        self.wait_time_srv = ActionServer(self, WaitTime, "wait_time", self.wait_time_callback)
        
        self.get_logger().info("MR-3 Mini driver started")

        self.comm = MR3MiniComm()
        self.comm.init_uart()
        self.get_logger().info("Connection successfully established")
        
        self.heartbeat_enabled = False
        self.heartbeat_tick_counter = 0    
        
        self.inst_finished = threading.Event()
        self.inst_result = None
        self.result_code = None

        self.uart_thread_running = threading.Event()
        self.uart_thread_running.set()
        
        self.uart_thread = threading.Thread(target=self.uart_thread_loop, daemon=True)
        self.uart_thread.start()

    def jog_callback(self, msg):
        if msg.type != JogType.JOG_NONE:
            if not self.is_jogging:
                self.is_jogging = True
                self.jog_status_pub.publish(JogStatus(code=0))
                self.comm.jog_start(msg)

        else:
            if self.is_jogging:
                self.is_jogging = False
                self.comm.jog_stop()
                
    def execute_instruction(self, goal_handle, comm_fcn, result_type, use_goal=True):
        goal = goal_handle.request
        
        self.inst_finished.clear()
        self.inst_result = None
        self.result_code = None
        
        if use_goal:
            comm_fcn(goal)
        else:
            comm_fcn()
        
        self.inst_finished.wait()        
        goal_handle.succeed()
        
        result = result_type()
        result.inst_result = self.inst_result
        result.result_code = self.result_code
        
        return result

    def move_home_callback(self, goal_handle):
       return self.execute_instruction(goal_handle, self.comm.move_home, MoveHome.Result)
    
    def move_rest_callback(self, goal_handle):
        return self.execute_instruction(goal_handle, self.comm.move_rest, MoveRest.Result, False)
    
    def move_abs_j_callback(self, goal_handle):
        return self.execute_instruction(goal_handle, self.comm.move_abs_j, MoveAbsJ.Result)
    
    def move_j_callback(self, goal_handle):
        return self.execute_instruction(goal_handle, self.comm.move_j, MoveJ.Result)
    
    def move_l_callback(self, goal_handle):
        return self.execute_instruction(goal_handle, self.comm.move_l, MoveL.Result)
    
    def wait_time_callback(self, goal_handle):
        return self.execute_instruction(goal_handle, self.comm.wait_time, WaitTime.Result)
                
    def pulse_beat(self):
        self.comm.send_heartbeat()
        
    def uart_thread_loop(self):
        while self.uart_thread_running.is_set():
            self.read_uart()            
            
            if self.heartbeat_enabled:
                self.heartbeat_tick_counter += 1
                
                if self.heartbeat_tick_counter >= HEARTBEAT_PERIOD_TICKS:
                    self.pulse_beat()
                    self.heartbeat_tick_counter = 0
            
            time.sleep(UART_THREAD_PERIOD)

    def read_uart(self):
        # Read and process all complete UART packets currently available
        while True:
            uart_msg = self.comm.read_and_parse()

            if uart_msg is None:
                break

            cmd = uart_msg[0]
            data = uart_msg[1]
            
            match cmd:
                case Message.MSG_ROBOT_STATE:
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
                
                case Message.MSG_SYSTEM_INIT_OK:
                    self.get_logger().info("System initialized")
                    self.heartbeat_tick_counter = 0
                    self.heartbeat_enabled = True

                case Message.MSG_MOVE_RESULT:
                    if data in (
                        MoveResult.MOVE_JOG_FINISHED,
                        MoveResult.MOVE_J1_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_J2_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_J3_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_J4_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_MOTOR_3_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_PASSIVE_JOINT_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_WORKSPACE_JOG_LIMIT_PROXIMITY,
                        MoveResult.MOVE_JOG_SPEED_OUT_OF_RANGE,
                        MoveResult.MOVE_JOINT_JOG_VEL_LIMIT_EXCEEDED,
                        MoveResult.MOVE_JOINT_JOG_ACC_LIMIT_EXCEEDED,
                        MoveResult.MOVE_LINEAR_JOG_VEL_LIMIT_EXCEEDED,
                        MoveResult.MOVE_LINEAR_JOG_ACC_LIMIT_EXCEEDED,
                    ):
                        self.jog_status_pub.publish(JogStatus(code=data))
                        
                    match data:             
                        case MoveResult.MOVE_HOMING_FINISHED:
                            self.get_logger().info("Homing finished")
                        case MoveResult.MOVE_FINISHED:
                            self.inst_result = data
                            self.result_code = 0
                            self.inst_finished.set()
                        case _:
                            self.inst_result = data
                            self.inst_finished.set()
                            
                case Message.MSG_INST_RESULT:                    
                    self.inst_result = data[0]
                    self.result_code = data[1]
                    self.inst_finished.set()
                        
                case Message.MSG_FAULT_RAISE:                
                    match data:
                        # Hardware timer
                        case Fault.FAULT_HW_TIMER_INIT:
                            self.get_logger().error("Timer initialization failed")
                        case Fault.FAULT_HW_TIMER_COUNTER:
                            self.get_logger().error("Failed to set timer counter value")
                        case Fault.FAULT_HW_TIMER_ALARM:
                            self.get_logger().error("Failed to set timer alarm value")
                        case Fault.FAULT_HW_TIMER_ENABLE_INTR:
                            self.get_logger().error("Failed to enable timer interrupt")
                        case Fault.FAULT_HW_TIMER_ISR_ADD:
                            self.get_logger().error("Failed to add timer ISR callback")
                        case Fault.FAULT_HW_TIMER_START:
                            self.get_logger().error("Failed to start timer")
                        
                        # Control loop
                        case Fault.FAULT_CONTROL_LOOP_QUEUE_INIT:
                            self.get_logger().error("Control queue initialization failed")
                        case Fault.FAULT_CONTROL_LOOP_TASK_INIT:
                            self.get_logger().error("Control task initialization failed")
                        
                        # Program executor
                        case Fault.FAULT_PROG_EXEC_QUEUE_INIT:
                            self.get_logger().error("Program queue initialization failed")
                        case Fault.FAULT_PROG_EXEC_TASK_INIT:
                            self.get_logger().error("Program task initialization failed")
                            
                        # Communication monitor
                        case Fault.FAULT_COMM_MONITOR_TASK_INIT:
                            self.get_logger().error("Communication monitor initialization failed")
                        case Fault.FAULT_COMM_MONITOR_TIMEOUT:
                            self.get_logger().error("Heartbeat timed out")
                            
                        # Unknown
                        case _:
                            self.get_logger().error(f"Unknown fault code received: {data}")
                                    
                case Message.MSG_WARNING_RAISE:
                    match data:
                        case Warning.WARNING_STATE_FEEDBACK_TASK_INIT:
                            self.get_logger().warning("State feedback initialization failed")
                        case Warning.WARNING_CONTROL_LOOP_QUEUE_FULL:
                            self.get_logger().warning("Control queue is full: the last message has been discarded")
                        case Warning.WARNING_PROG_EXEC_QUEUE_FULL:
                            self.get_logger().warning("Program queue is full: the last instruction has been discarded")
                        case _:
                            self.get_logger().error(f"Unknown warning code received: {data}")

                case Message.MSG_UNKNOWN:
                    self.get_logger().warning(f"Unknown message code received: {cmd}")                  
                                                             
    def destroy_node(self):
        self.heartbeat_enabled = False
        self.uart_thread_running.clear()
        self.uart_thread.join()
        
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
