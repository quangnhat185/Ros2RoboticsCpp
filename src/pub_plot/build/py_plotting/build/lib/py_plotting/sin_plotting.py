import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import matplotlib.pyplot as plt

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sin_singla_sub')
        self.subscription = self.create_subscription(
            Float32,
            'sin_signal',
            self.plotting,
            10
        )

    def plotting(self, msg):
        self.get_logger().info(f"get value {msg.data}")


def main(args=None):

    rclpy.init(args = args)

    min_sub = MinimalSubscriber()

    rclpy.spin(min_sub)
    min_sub.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
