"""

Car model for Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""

import pathlib
import sys

root_dir = pathlib.Path(__file__).parent.parent.parent
sys.path.append(str(root_dir))

from math import cos, pi, sin, tan

import numpy as np
from rclpy.node import Node

from .utils.angle import rot_mat_2d


class Car:
    def __init__(self, node: Node):
        self.WB = node.get_parameter("WB").value
        self.W = node.get_parameter("W").value
        self.LF = node.get_parameter("LF").value
        self.LB = node.get_parameter("LB").value
        self.MAX_STEER = node.get_parameter("MAX_STEER").value
        self.N_STEER = node.get_parameter("N_STEER").value
        self.SB_COST = node.get_parameter("SB_COST").value
        self.BACK_COST = node.get_parameter("BACK_COST").value
        self.STEER_CHANGE_COST = node.get_parameter("STEER_CHANGE_COST").value
        self.STEER_COST = node.get_parameter("STEER_COST").value
        self.H_COST = node.get_parameter("H_COST").value
        self.MOTION_RESOLUTION = node.get_parameter("MOTION_RESOLUTION").value

        self.BUBBLE_DIST = (
            self.LF - self.LB
        ) / 2.0  # distance from rear to center of vehicle.
        self.BUBBLE_R = np.hypot(
            (self.LF + self.LB) / 2.0, self.W / 2.0
        )  # bubble radius

        # vehicle rectangle vertices
        self.VRX = [self.LF, self.LF, -self.LB, -self.LB, self.LF]
        self.VRY = [self.W / 2, -self.W / 2, -self.W / 2, self.W / 2, self.W / 2]

    def check_car_collision(self, x_list, y_list, yaw_list, ox, oy, kd_tree):
        for i_x, i_y, i_yaw in zip(x_list, y_list, yaw_list):
            cx = i_x + self.BUBBLE_DIST * cos(i_yaw)
            cy = i_y + self.BUBBLE_DIST * sin(i_yaw)

            ids = kd_tree.query_ball_point([cx, cy], self.BUBBLE_R)

            if not ids:
                continue

            if not self.rectangle_check(
                i_x, i_y, i_yaw, [ox[i] for i in ids], [oy[i] for i in ids]
            ):
                return False  # collision

        return True  # no collision

    def rectangle_check(self, x, y, yaw, ox, oy):
        # transform obstacles to base link frame
        rot = rot_mat_2d(yaw)
        for iox, ioy in zip(ox, oy):
            tx = iox - x
            ty = ioy - y
            converted_xy = np.stack([tx, ty]).T @ rot
            rx, ry = converted_xy[0], converted_xy[1]

            if not (
                rx > self.LF or rx < -self.LB or ry > self.W / 2.0 or ry < -self.W / 2.0
            ):
                return False  # collision

        return True  # no collision

    def pi_2_pi(self, angle):
        return (angle + pi) % (2 * pi) - pi

    def move(self, x, y, yaw, distance, steer, L=3.0):
        x += distance * cos(yaw)
        y += distance * sin(yaw)
        yaw += self.pi_2_pi(distance * tan(steer) / L)  # distance/2
        return x, y, yaw
