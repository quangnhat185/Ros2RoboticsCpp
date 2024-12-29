import rclpy
from rclpy.node import Node
from planning_msgs.srv import Paths
from planning_msgs.msg import Path
import reeds_shepp_planning.reeds_shepp_path_planning as rs
from typing import List

class RSService(Node):

    def __init__(self):
        super().__init__('reeds_shepp_service')
        self.srv = self.create_service(Paths, 'calc_RS_path', self.execute_callback)
        self.get_logger().info('Reeds-Shepp service has been started!')

    def execute_callback(self, request, response):
        
        paths = rs.calc_paths(
            sx = request.start_x,
            sy = request.start_y,
            syaw = request.start_yaw,
            gx= request.goal_x,
            gy= request.goal_y,
            gyaw= request.goal_yaw,
            maxc= request.max_curvature,
            step_size = request.step_size
        )
        
        for path in paths:
            path_msg = Path()
            path_msg.x = path.x
            path_msg.y = path.y
            path_msg.yaw = path.yaw
            path_msg.directions = path.directions
            path_msg.l = path.L
            response.paths.append(path_msg)
        
        self.get_logger().info(f'Execute service')

        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = RSService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()