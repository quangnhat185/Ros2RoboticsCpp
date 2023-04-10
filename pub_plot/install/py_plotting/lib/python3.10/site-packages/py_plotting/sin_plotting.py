import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
import os
import shutil
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sin_signal_sub')


        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sin_signal',
            self.plotting,
            10
        )

        self.declare_parameter('save_img', 'false')

        self.save_img = self.get_parameter('save_img').get_parameter_value().string_value

        # self.save_img = self.get_parameter('save_img').value

        self.x = []
        self.y = []
        self.counter = 0

        if self.save_img == 'true':
            if  os.path.isdir("./imgs"):
                shutil.rmtree("./imgs")
                os.makedirs("./imgs")
            else:
                os.makedirs("./imgs")

    def plotting(self, msg):
        self.get_logger().info(f"{self.save_img}")
        self.get_logger().info(f"{len(self.x)}")
        
        plt.ylim(-1,1)
        self.x.append(msg.data[0])
        self.y.append(msg.data[1])
        plt.plot(self.x[-100:], self.y[-100:])
        
        if (len(self.x) > 100) and (self.save_img=='true'):
            # self.get_logger().info(f"{os.getcwd()}")
            plt.savefig(f"./imgs/{self.counter}.png")
            self.counter+=1

        plt.pause(0.01)
        plt.clf()



def main(args=None):

    rclpy.init(args = args)

    min_sub = MinimalSubscriber()

    rclpy.spin(min_sub)
    min_sub.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
