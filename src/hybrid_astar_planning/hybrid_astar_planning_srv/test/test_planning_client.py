import rclpy
from rclpy.node import Node
import numpy as np
import pytest
from planning_msgs.srv import Paths  # Replace with your actual service type


class MinimalServiceClient(Node):
    def __init__(self):
        super().__init__("hybrid_astar_planning_client")
        self.cli = self.create_client(Paths, "hybrid_astar_planning_server")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = Paths.Request()

    def send_request(self):
        ox, oy = [], []

        for i in range(60):
            ox.append(float(i))
            oy.append(0.0)
        for i in range(60):
            ox.append(60.0)
            oy.append(float(i))
        for i in range(61):
            ox.append(float(i))
            oy.append(60.0)
        for i in range(61):
            ox.append(0.0)
            oy.append(float(i))
        for i in range(40):
            ox.append(20.0)
            oy.append(float(i))
        for i in range(40):
            ox.append(40.0)
            oy.append(60.0 - float(i))

        # import ipdb; ipdb.set_trace()
        self.req.start = [10.0, 10.0, np.deg2rad(90.0)]
        self.req.goal = [50.0, 50.0, np.deg2rad(-90.0)]
        self.req.ox = ox
        self.req.oy = oy
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

@pytest.fixture
def init_response():
    rclpy.init()
    minimal_service_client = MinimalServiceClient()
    response = minimal_service_client.send_request()
    return response

def test_response(init_response):
    response = init_response
    assert response.x_list, "x_list is empty"
    assert response.y_list, "y_list is empty"
    assert response.yaw_list, "yaw_list is empty"
    assert len(response.x_list) == len(response.y_list) == len(response.yaw_list), "Path lists are not of equal length"

def main(args=None):
    rclpy.init(args=args)
    minimal_service_client = MinimalServiceClient()
    response = minimal_service_client.send_request()
    print(response)
    minimal_service_client.get_logger().info("Request sent!")
    minimal_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
