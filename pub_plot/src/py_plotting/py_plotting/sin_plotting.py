import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sin_singla_sub')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sin_signal',
            self.plotting,
            10
        )
        self.x = []
        self.y = []

    def plotting(self, msg):
        plt.ylim(-1,1)
        self.x.append(msg.data[0])
        self.y.append(msg.data[1])
        plt.plot(self.x[-100:], self.y[-100:])
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
