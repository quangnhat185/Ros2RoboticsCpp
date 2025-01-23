import rclpy
from planning_msgs.srv import Paths
from rclpy.node import Node

from .hybrid_a_star import HybridAstar


class PlanningServer(Node):
    def __init__(self):
        super().__init__("planning_server_node")
        parameters = {
            "WB": 3.0,
            "W": 5.0,
            "LF": 3.3,
            "LB": 1.0,
            "MAX_STEER": 0.6,
            "XY_GRID_RESOLUTION": 2.0,
            "YAW_GRID_RESOLUTION": 0.2617993877991494,
            "MOTION_RESOLUTION": 0.1,  # [m] path interpolate resolution
            "N_STEER": 20,  # number of steer command
            "SB_COST": 100.0,  # switch back penalty cost
            "BACK_COST": 5.0,  # backward penalty cost
            "STEER_CHANGE_COST": 5.0,  # steer angle change penalty cost
            "STEER_COST": 1.0,  # steer angle change penalty cost
            "H_COST": 5.0,  # Heuristic cost
        }

        for param, value in parameters.items():
            self.declare_parameter(param, value)

        self.hybridAstar = HybridAstar(self)

        self.srv = self.create_service(
            Paths, "hybrid_astar_planning_server", self.execute_callback
        )

    def execute_callback(self, request, response):
        self.get_logger().info("Planning path...")
        # Call the main function to perform path planning
        start = request.start
        goal = request.goal
        ox = request.ox
        oy = request.oy

        path = self.hybridAstar.planning(start, goal, ox, oy)

        response.success = True
        response.x_list = path.x_list
        response.y_list = path.y_list
        response.yaw_list = path.yaw_list
        return response


def main(args=None):
    rclpy.init(args=args)
    planning_server = PlanningServer()
    planning_server.get_logger().info("Planning server is running...")
    rclpy.spin(planning_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
