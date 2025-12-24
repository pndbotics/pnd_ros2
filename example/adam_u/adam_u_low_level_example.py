import math
import rclpy
import os
import time
import threading
from rclpy.node import Node
from pnd_adam.msg import LowCmd, LowState, MotorCmd, MotorState, HandCmd, HandState


import numpy as np

ADAM_NUM_MOTOR = 19


Kp = [
    60.0,  # waistRoll (0)
    60.0,  # waistPitch (1)
    60.0,  # waistYaw (2)
    9.0,   # neckYaw (3)
    9.0,   # neckPitch (4)
    18.0,  # shoulderPitch_Left (5)
    9.0,   # shoulderRoll_Left (6)
    9.0,   # shoulderYaw_Left (7)
    9.0,   # elbow_Left (8)
    9.0,   # wristYaw_Left (9)
    9.0,   # wristPitch_Left (10)
    9.0,   # wristRoll_Left (11)
    18.0,  # shoulderPitch_Right (12)
    9.0,   # shoulderRoll_Right (13)
    9.0,   # shoulderYaw_Right (14)
    9.0,   # elbow_Right (15)
    9.0,   # wristYaw_Right (16)
    9.0,   # wristPitch_Right (17)
    9.0    # wristRoll_Right (18)
]

# Kd 配置数组（对应19个关节）
Kd = [
    1.0,   # waistRoll (0)
    1.0,   # waistPitch (1)
    1.0,   # waistYaw (2)
    0.9,   # neckYaw (3)
    0.9,   # neckPitch (4)
    0.9,   # shoulderPitch_Left (5)
    0.9,   # shoulderRoll_Left (6)
    0.9,   # shoulderYaw_Left (7)
    0.9,   # elbow_Left (8)
    0.9,   # wristYaw_Left (9)
    0.9,   # wristPitch_Left (10)
    0.9,   # wristRoll_Left (11)
    0.9,   # shoulderPitch_Right (12)
    0.9,   # shoulderRoll_Right (13)
    0.9,   # shoulderYaw_Right (14)
    0.9,   # elbow_Right (15)
    0.9,   # wristYaw_Right (16)
    0.9,   # wristPitch_Right (17)
    0.9    # wristRoll_Right (18)
]

class DemonController(Node):
    def __init__(self):
        super().__init__('demon_controller')
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]

        self.lowcmd_pub_ = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.handcmd_pub_ = self.create_publisher(HandCmd, 'handcmd', 10)
        self.close_hand = np.array([500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500], dtype=int)
        self.getstate_flag = False
        
        self.timer = self.create_timer(self.control_dt_, self.LowCmdWrite)
        self.mutex = threading.Lock()
        
        self.low_state = LowState()
        self.low_state.motor_state = [MotorState() for _ in range(ADAM_NUM_MOTOR)]
        self.low_cmd = LowCmd()
        self.low_cmd.motor_cmd = [MotorCmd() for _ in range(ADAM_NUM_MOTOR)]  

        self.lowstate_sub_ = self.create_subscription(
            LowState,
            "lowstate",
            self.getLowState,
            10)
        

    def getLowState(self, msg):
        # 获取实际接收到的电机数量
        received_len = len(msg.motor_state)
        
        if received_len < ADAM_NUM_MOTOR:
            # 如果收到的数据不足，先复制已有的数据
            self.low_state = msg
            # 计算缺失的数量
            missing_count = ADAM_NUM_MOTOR - received_len
            # 用默认的 MotorState 对象补齐列表
            # 这样 self.low_state.motor_state[i].q 就会默认为 0.0
            self.low_state.motor_state.extend([MotorState() for _ in range(missing_count)])
            
            # 可选：打印警告信息（仅打印一次或限制频率）
            # print(f"Warning: Received only {received_len} motor states. Padding with {missing_count} default states.")
        else:
            # 如果数量足够或更多，直接赋值
            self.low_state = msg
            
        self.getstate_flag = True
        # print("len(low_state.motor_state)=", len(self.low_state.motor_state))
 
    def LowCmdWrite(self):
        if(self.getstate_flag):
            self.time_ += self.control_dt_          
            handcmd = HandCmd()

            if self.time_ < self.duration_ :
                # set robot to zero posture
                for i in range(ADAM_NUM_MOTOR):
                    ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                    self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                    self.low_cmd.motor_cmd[i].tau = 0. 
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q 
                    self.low_cmd.motor_cmd[i].dq = 0. 
                    self.low_cmd.motor_cmd[i].kp = Kp[i]
                    self.low_cmd.motor_cmd[i].kd = Kd[i]            
                    
                for i in range(12):
                    handcmd.position[i] = self.close_hand[i]
                self.lowcmd_pub_.publish(self.low_cmd)
                self.handcmd_pub_.publish(handcmd)
        # else:
        #     print("Waiting for LowState...")

       
def main(args=None):
    os.environ["ROS_DOMAIN_ID"] = "2"
    rclpy.init(args=args)
    demon_controller = DemonController()
    rclpy.spin(demon_controller)
    demon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    input("Demo adam Pro robot movement by ROS2. Press enter to start")
    main()
