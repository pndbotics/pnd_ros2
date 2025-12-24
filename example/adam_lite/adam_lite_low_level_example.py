import math
import rclpy
import os
import time
import threading
from rclpy.node import Node
# from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from pnd_adam.msg import LowCmd, LowState, MotorCmd, MotorState


import numpy as np

ADAM_NUM_MOTOR = 25


Kp = [
    305.0, 700.0, 405.0, # hip left
    305.0, 20.0, 0.0, #knee and ankle
    
    305.0, 700.0, 405.0, # hip right
    305.0, 20.0, 0.0, #knee and ankle
    
    205.0, 405.0, 405.0,  # waist
    
    18.0, 9.0, 9.0, # shoulder left
    9.0, 9.0,  # arms

    18.0, 9.0, 9.0, # shoulder right
    9.0, 9.0  # arms
]

Kd = [
    6.1, 30.0, 6.1,
    6.1, 2.5, 0.35,
    
    6.1, 30.0, 6.1,
    6.1, 2.5, 0.35,    # legs
    
    4.1, 6.1, 6.1,             # waist
    
    0.9, 0.9, 0.9,
    0.9, 0.9,  # arms
    
    0.9, 0.9, 0.9, 
    0.9, 0.9  # arms 
]

class ADAMJointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13        
    WaistPitch = 14       
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristYaw = 19     
    RightShoulderPitch = 20
    RightShoulderRoll = 21
    RightShoulderYaw = 22
    RightElbow = 23
    RightWristYaw = 24    



class DemonController(Node):
    def __init__(self):
        super().__init__('demon_controller')
        self.time_ = 0.0
        self.control_dt_ = 0.001  # [1ms]
        self.duration_ = 3.0    # [3 s]

        self.lowcmd_pub_ = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.getstate_flag = False
        
        self.timer = self.create_timer(self.control_dt_, self.LowCmdWrite)
        self.mutex = threading.Lock()
        
        self.low_state = LowState()
        self.low_state.motor_state = [MotorState() for _ in range(ADAM_NUM_MOTOR)]
        # print("init len(low_state.motor_state)=", len(self.low_state.motor_state))

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

            if self.time_ < self.duration_ :
                # [Stage 1]: set robot to zero posture
                for i in range(ADAM_NUM_MOTOR):
                    ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                    self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                    self.low_cmd.motor_cmd[i].tau = 0. 
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q 
                    self.low_cmd.motor_cmd[i].dq = 0. 
                    self.low_cmd.motor_cmd[i].kp = Kp[i] 
                    self.low_cmd.motor_cmd[i].kd = Kd[i] 
            elif self.time_ < self.duration_ * 2 :
                max_P = np.pi * 30.0 / 180.0
                max_R = np.pi * 10.0 / 180.0
                t = self.time_ - self.duration_
                L_P_des = max_P * np.sin(2.0 * np.pi * t)
                L_R_des = max_R * np.sin(2.0 * np.pi * t)
                R_P_des = max_P * np.sin(2.0 * np.pi * t)
                R_R_des = -max_R * np.sin(2.0 * np.pi * t)

                self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnklePitch].q = L_P_des
                self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleRoll].q = L_R_des
                self.low_cmd.motor_cmd[ADAMJointIndex.RightAnklePitch].q = R_P_des
                self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleRoll].q = R_R_des                    
     
            self.lowcmd_pub_.publish(self.low_cmd)
            # print(self.low_cmd)

       
def main(args=None):
    os.environ["ROS_DOMAIN_ID"] = "2"
    rclpy.init(args=args)
    demon_controller = DemonController()
    rclpy.spin(demon_controller)
    demon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    input("Demo adam Lite robot movement by ROS2. Press enter to start")
    main()
