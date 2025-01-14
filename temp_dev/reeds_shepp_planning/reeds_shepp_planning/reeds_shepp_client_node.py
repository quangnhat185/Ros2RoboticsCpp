import rclpy
from rclpy.node import Node
from planning_msgs.srv import Paths

class ReedsSheppClient(Node):
    def __init__(self):
        super().__init__('reeds_shepp_client')
        self.client = self.create_client(Paths, 'calc_RS_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = Paths.Request()

    def send_request(self):
        self.request.start_x = 10.0
        self.request.start_y = 10.0
        self.request.start_yaw = 1.57
        self.request.goal_x = 50.0
        self.request.goal_y = 50.0
        self.request.goal_yaw = -1.57  # 90 degrees in radians
        self.request.max_curvature = 0.228
        self.request.step_size = 0.1

        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = ReedsSheppClient()
    response = client.send_request()
    for path in response.paths:
        import ipdb; ipdb.set_trace()
        print(f'Path: {path.x}, {path.y}, {path.yaw}, {path.directions}, {path.l}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()