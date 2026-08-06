import serial
from serial.tools import list_ports
import struct
from enum import IntEnum
import time


HEADER_1 = 0xAA
HEADER_2 = 0x55
HEADERS = bytes([HEADER_1, HEADER_2])


class CommState(IntEnum):
    COMM_ESTABLISHED = 0x00
    COMM_DISCONNECTED = 0x01


class Message(IntEnum):
    # Communication
    MSG_ESTABLISH_COMM = 0x00
    MSG_COMM_ESTABLISHED = 0x01
    MSG_DISCONNECT = 0x02
    MSG_DISCONNECTED = 0x03
    
    # Data
    MSG_ROBOT_STATE = 0x0A

    # Move result
    MSG_MOVE_RESULT = 0x1E

    # Jog
    MSG_JOG_START = 0x28
    MSG_JOG_STOP = 0x29   
    
    # Unknown code
    MSG_UNKNOWN = 0xFF


class MoveResult(IntEnum):
    # General results
    MOVE_FINISHED = 0x00
    MOVE_FEASIBLE = 0x01
    MOVE_INFEASIBLE = 0x02           # move is infeasible: reduce speed or increase move distance    
    MOVE_POSE_REACHABLE = 0x03
    MOVE_OUTSIDE_OF_WORKSPACE = 0x04
    MOVE_UNREACHABLE_INT_PT = 0x05   # an intermediate waypoint in a linear move is not reachable
    MOVE_JOG_FINISHED = 0x06
    
    # Move parameter limit violations
    MOVE_SPEED_OUT_OF_RANGE = 0x64
    MOVE_JOINT_VEL_LIMIT_EXCEEDED = 0x65
    MOVE_LINEAR_VEL_LIMIT_EXCEEDED = 0x66
    MOVE_JOINT_ACC_LIMIT_EXCEEDED = 0x67
    MOVE_LINEAR_ACC_LIMIT_EXCEEDED = 0x68
    
    # Jog parameter limit violations
    MOVE_JOG_SPEED_OUT_OF_RANGE = 0x69
    MOVE_JOINT_JOG_VEL_LIMIT_EXCEEDED = 0x6A
    MOVE_JOINT_JOG_ACC_LIMIT_EXCEEDED = 0x6B
    MOVE_LINEAR_JOG_VEL_LIMIT_EXCEEDED = 0x6C
    MOVE_LINEAR_JOG_ACC_LIMIT_EXCEEDED = 0x6D

    # Physical joint limit violations
    MOVE_J1_LIMIT_VIOLATION = 0x96
    MOVE_J2_LIMIT_VIOLATION = 0x97
    MOVE_J3_LIMIT_VIOLATION = 0x98
    MOVE_J4_LIMIT_VIOLATION = 0x99
    MOVE_MOTOR_3_LIMIT_VIOLATION = 0x9A
    MOVE_PASSIVE_JOINT_LIMIT_VIOLATION = 0x9B
    
    # Jogging cannot be started because a physical limit would be exceeded
    MOVE_J1_JOG_LIMIT_PROXIMITY = 0x9C,
    MOVE_J2_JOG_LIMIT_PROXIMITY = 0x9D,
    MOVE_J3_JOG_LIMIT_PROXIMITY = 0x9E,
    MOVE_J4_JOG_LIMIT_PROXIMITY = 0x9F,
    MOVE_MOTOR_3_JOG_LIMIT_PROXIMITY = 0xA0,
    MOVE_PASSIVE_JOINT_JOG_LIMIT_PROXIMITY = 0xA1
    MOVE_WORKSPACE_JOG_LIMIT_PROXIMITY = 0xA2


class MR3MiniComm:
    def __init__(self):
        self.uart = None
        self.comm_state = CommState.COMM_DISCONNECTED

        self.packet = bytearray(64)
        self.index = 0
    
    def init_uart(self, retries=3, timeout=0.2):
        # Try every available serial port
        for port in list_ports.comports():
            try:
                self.uart = serial.Serial(port.device, 115200, timeout=0)
            except serial.SerialException:
                continue
            
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            
            # Try to establish communication
            for _ in range(retries):
                self.init_comm()
                start_time = time.time()
                
                while time.time() - start_time < timeout:
                    while True:
                        uart_msg = self.read_and_parse()
                        
                        if uart_msg is None:
                            break
                        
                        cmd = uart_msg[0]
                        
                        if cmd == Message.MSG_COMM_ESTABLISHED:
                            self.comm_state = CommState.COMM_ESTABLISHED
                            return
                
                time.sleep(0.005)
            
            # Incorrect controller: close port and try the next one
            self.uart.close()
            self.uart = None
        
        raise RuntimeError("No compatible Modena controller found")    

    def init_comm(self):
        code = Message.MSG_ESTABLISH_COMM
        data = bytes([0])

        packet = self.build_packet(code, data)

        self.uart.write(packet)

    def end_uart(self):
        self.disconnect()
        self.close_uart()
    
    def disconnect(self, retries=3, timeout=0.2):
        for _ in range(retries):
            self.end_comm()
            start_time = time.time()

            while time.time() - start_time < timeout:
                while True:
                    uart_msg = self.read_and_parse()

                    if uart_msg is None:
                        break

                    cmd = uart_msg[0]
                    
                    if cmd == Message.MSG_DISCONNECTED:
                        self.comm_state = CommState.COMM_DISCONNECTED
                        return True

                time.sleep(0.005)

        raise RuntimeError("Failed to disconnect")

    def end_comm(self):
        code = Message.MSG_DISCONNECT
        data = bytes([0])

        packet = self.build_packet(code, data)

        self.uart.write(packet)    

    def close_uart(self):
        if self.uart.is_open and self.uart is not None:
            self.uart.close()

    def check_comm(self):
        if self.comm_state != CommState.COMM_ESTABLISHED:
            raise RuntimeError("Connection not established")
    
    def jog_start(self, msg):
        self.check_comm()

        code = Message.MSG_JOG_START
        data = struct.pack("<BBBb", msg.type, msg.index, msg.speed, msg.dir)
        packet = self.build_packet(code, data)
        self.uart.write(packet)

    def jog_stop(self):
        self.check_comm()

        code = Message.MSG_JOG_STOP
        data = bytes([0])
        packet = self.build_packet(code, data)
        self.uart.write(packet)

    def build_packet(self, code, data): # packet structure = [HEADER_1][HEADER_2][LENGTH][CODE][DATA][CHECKSUM]
        code = bytes([code])
        length = bytes([len(code) + len(data)])
        checksum = sum(HEADERS + length + code + data)
        checksum = bytes([checksum & 0xFF])
        packet = HEADERS + length + code + data + checksum

        return packet    

    def read_and_parse(self):
        if self.uart.in_waiting > 0:
            data = self.uart.read(self.uart.in_waiting)
            n = len(data)
            
            # Receive buffer overflow: discard buffered data and resync
            if self.index + n > len(self.packet):            
                self.index = 0

            # Append new data to buffer            
            self.packet[self.index : self.index + n] = data
            self.index += n

        while True:
            if self.index < 3:
                return None

            # If headers check fails: shift left by 1 byte (resync)
            if self.packet[0] != HEADER_1 or self.packet[1] != HEADER_2:                
                self.packet[: self.index - 1] = self.packet[1 : self.index]
                self.index -= 1
                continue

            total_length = (self.packet[2] + 4)

            # Wait for more data
            if self.index < total_length:
                return None  

            checksum = sum(self.packet[0 : total_length - 1]) & 0xFF

            # If checksum fails: remove invalid packet from the buffer
            if checksum != self.packet[total_length - 1]:               
                remaining = self.index - total_length

                if remaining > 0:
                    self.packet[:remaining] = self.packet[total_length:self.index]

                self.index = remaining

                return None

            cmd = Message(self.packet[3])

            if cmd == Message.MSG_ROBOT_STATE:
                joint_pos = struct.unpack("<4f", self.packet[4:20])
                pose = struct.unpack("<4f", self.packet[20:36])
                state = (joint_pos, pose)
                result = (cmd, state)

            elif cmd == Message.MSG_MOVE_RESULT:
                moveResult = MoveResult(self.packet[4])
                result = (cmd, moveResult)

            elif cmd == Message.MSG_COMM_ESTABLISHED:
                result = (cmd, None)

            elif cmd == Message.MSG_DISCONNECTED:
                result = (cmd, None)
                
            else:
                cmd == Message.MSG_UNKNOWN
                result = (cmd, None)

            # Remove processed packet
            remaining = self.index - total_length

            if remaining > 0:
                self.packet[:remaining] = self.packet[total_length : self.index]
                self.index = remaining
            else:
                self.index = 0

            return result   
