
import math
import rclpy
import os
import time
import threading
from rclpy.node import Node
# from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from pnd_adam.msg import LowCmd, LowState, MotorCmd, MotorState, HandCmd, HandState


class DemonController(Node):
    def __init__(self):
        super().__init__('demon_controller')
        self.handcmd_pub_ = self.create_publisher(HandCmd, 'handcmd', 10)
        
        self.dt = 0.0025 
        self.timer = self.create_timer(self.dt, self.Control)
        # self.mutex = threading.Lock()
        
        self.open_hand = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.close_hand = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.runing_time = 0.0

    def Control(self):
        step_start = time.perf_counter()
 
        self.runing_time += self.dt

        handcmd = HandCmd()
        if (self.runing_time < 3.0):
            for i in range(12):
                handcmd.position[i] = self.open_hand[i] * 0.6
        else:
            for i in range(12):
                handcmd.position[i] = self.close_hand[i] * 0.6
        self.handcmd_pub_.publish(handcmd)

        time_until_next_step = self.dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

def main(args=None):
    os.environ["ROS_DOMAIN_ID"] = "2"
    # os.environ["ROS_LOCALHOST"] = "127.0.0.1"
    rclpy.init(args=args)
    demon_controller = DemonController()
    rclpy.spin(demon_controller)
    demon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    input("Press enter to start")
    main()