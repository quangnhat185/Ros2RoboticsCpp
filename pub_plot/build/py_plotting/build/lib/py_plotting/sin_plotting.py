#!/home/quang/miniconda3/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
import ipdb; ipdb.set_trace()
import tensorflow

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sin_singla_sub')
        self.subscription = self.create_subscription(
            Float32,
            'sin_signal',
            self.plotting,
            10
        )
        self.counter = 0
        self.y = []
        plt.xlim(0, 100)

    def plotting(self, msg):
        self.y.append(msg.data)
        plt.plot(self.y[-100:])
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
