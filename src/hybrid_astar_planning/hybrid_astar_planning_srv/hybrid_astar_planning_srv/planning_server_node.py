from .hybrid_a_star import hybrid_a_star_planning
import rclpy
from rclpy.node import Node
from planning_msgs.srv import Paths

class PlanningServer(Node):

    def __init__(self):
        super().__init__('planning_server_node')
        parameters = {
            'WB': 3.0,
            'W': 2.0,
            'LF': 3.3,
            'LB': 1.0,
            'MAX_STEER': 0.6,
            'BUBBLE_DIST': 1.15,
            'BUBBLE_R': 2.0615528128088303,
            'VRX': [3.3, 3.3, -1.0, -1.0, 3.3],
            'VRY': [1.0, -1.0, -1.0, 1.0, 1.0],
            'XY_GRID_RESOLUTION': 2.0,
            'YAW_GRID_RESOLUTION': 0.2617993877991494,
            'MOTION_RESOLUTION': 0.1,
            'N_STEER': 20,
            'SB_COST': 100.0,
            'BACK_COST': 5.0,
            'STEER_CHANGE_COST': 5.0,
            'STEER_COST': 1.0,
            'H_COST': 5.0
        }

        for param, value in parameters.items():
            self.declare_parameter(param, value)

        WB = self.get_parameter('WB').value
        W = self.get_parameter('W').value
        LF = self.get_parameter('LF').value
        LB = self.get_parameter('LB').value
        MAX_STEER = self.get_parameter('MAX_STEER').value
        BUBBLE_DIST = self.get_parameter('BUBBLE_DIST').value
        BUBBLE_R = self.get_parameter('BUBBLE_R').value
        VRX = self.get_parameter('VRX').value
        VRY = self.get_parameter('VRY').value
        self.XY_GRID_RESOLUTION = self.get_parameter('XY_GRID_RESOLUTION').value
        self.YAW_GRID_RESOLUTION = self.get_parameter('YAW_GRID_RESOLUTION').value
        MOTION_RESOLUTION = self.get_parameter('MOTION_RESOLUTION').value
        N_STEER = self.get_parameter('N_STEER').value
        SB_COST = self.get_parameter('SB_COST').value
        BACK_COST = self.get_parameter('BACK_COST').value
        STEER_CHANGE_COST = self.get_parameter('STEER_CHANGE_COST').value
        STEER_COST = self.get_parameter('STEER_COST').value
        H_COST = self.get_parameter('H_COST').value


        self.srv = self.create_service(Paths, 'hybrid_astar_planning_server', self.execute_callback)


    def execute_callback(self, request, response):

        self.get_logger().info('Planning path...')
        # Call the main function to perform path planning
        start = request.start
        goal = request.goal
        ox = request.ox
        oy = request.oy

        path = hybrid_a_star_planning(start, goal, ox, oy, self.XY_GRID_RESOLUTION, self.YAW_GRID_RESOLUTION)

        response.success = True
        response.x_list = path.x_list
        response.y_list = path.y_list
        response.yaw_list = path.yaw_list
        return response

def main(args=None):
    rclpy.init(args=args)
    planning_server = PlanningServer()
    planning_server.get_logger().info('Planning server is running...')
    rclpy.spin(planning_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()