"""

Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""

import heapq
import math

import numpy as np
from rclpy.node import Node
from scipy.spatial import cKDTree

# from .car import move, check_car_collision, MAX_STEER, WB, BUBBLE_R
from .car import Car
from .dynamic_programming_heuristic import calc_distance_heuristic
from .ReedsSheppPath import reeds_shepp_path_planning as rs


class Node:
    def __init__(
        self,
        x_ind,
        y_ind,
        yaw_ind,
        direction,
        x_list,
        y_list,
        yaw_list,
        directions,
        steer=0.0,
        parent_index=None,
        cost=None,
    ):
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind
        self.direction = direction
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.directions = directions
        self.steer = steer
        self.parent_index = parent_index
        self.cost = cost


class Path:
    def __init__(self, x_list, y_list, yaw_list, direction_list, cost):
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.direction_list = direction_list
        self.cost = cost


class Config:
    def __init__(self, ox, oy, xy_grid_resolution, yaw_grid_resolution):
        min_x_m = min(ox)
        min_y_m = min(oy)
        max_x_m = max(ox)
        max_y_m = max(oy)

        ox.append(min_x_m)
        oy.append(min_y_m)
        ox.append(max_x_m)
        oy.append(max_y_m)

        self.min_x = round(min_x_m / xy_grid_resolution)
        self.min_y = round(min_y_m / xy_grid_resolution)
        self.max_x = round(max_x_m / xy_grid_resolution)
        self.max_y = round(max_y_m / xy_grid_resolution)

        self.x_w = round(self.max_x - self.min_x)
        self.y_w = round(self.max_y - self.min_y)

        self.min_yaw = round(-math.pi / yaw_grid_resolution) - 1
        self.max_yaw = round(math.pi / yaw_grid_resolution)
        self.yaw_w = round(self.max_yaw - self.min_yaw)

    def __str__(self):
        return (
            str(self.min_x)
            + ","
            + str(self.max_x)
            + ", "
            + str(self.min_y)
            + ","
            + str(self.max_y)
            + ", "
            + str(self.min_yaw)
            + ","
            + str(self.max_yaw)
            + ", "
            + str(self.x_w)
            + ","
            + str(self.y_w)
            + ","
            + str(self.yaw_w)
        )


class HybridAstar:
    def __init__(self, node: Node):
        self.car_obj = Car(node)
        self.MOTION_RESOLUTION = node.get_parameter(
            "MOTION_RESOLUTION"
        ).value  # [m] path interpolate resolution
        self.N_STEER = node.get_parameter("N_STEER").value  # number of steer command

        self.SB_COST = node.get_parameter("SB_COST").value  # switch back penalty cost
        self.BACK_COST = node.get_parameter("BACK_COST").value  # backward penalty cost
        self.STEER_CHANGE_COST = node.get_parameter(
            "STEER_CHANGE_COST"
        ).value  # steer angle change penalty cost
        self.STEER_COST = node.get_parameter(
            "STEER_COST"
        ).value  # steer angle change penalty cost
        self.H_COST = node.get_parameter("H_COST").value  # Heuristic cost

        self.XY_GRID_RESOLUTION = node.get_parameter("XY_GRID_RESOLUTION").value
        self.YAW_GRID_RESOLUTION = node.get_parameter("YAW_GRID_RESOLUTION").value

    def calc_motion_inputs(self):
        for steer in np.concatenate(
            (
                np.linspace(
                    -self.car_obj.MAX_STEER, self.car_obj.MAX_STEER, self.N_STEER
                ),
                [0.0],
            )
        ):
            for d in [1, -1]:
                yield [steer, d]

    def get_neighbors(self, current, config, ox, oy, kd_tree):
        for steer, d in self.calc_motion_inputs():
            node = self.calc_next_node(current, steer, d, config, ox, oy, kd_tree)
            if node and self.verify_index(node, config):
                yield node

    def calc_next_node(self, current, steer, direction, config, ox, oy, kd_tree):
        x, y, yaw = current.x_list[-1], current.y_list[-1], current.yaw_list[-1]

        arc_l = self.XY_GRID_RESOLUTION * 1.5
        x_list, y_list, yaw_list = [], [], []
        for _ in np.arange(0, arc_l, self.MOTION_RESOLUTION):
            x, y, yaw = self.car_obj.move(
                x, y, yaw, self.MOTION_RESOLUTION * direction, steer
            )
            x_list.append(x)
            y_list.append(y)
            yaw_list.append(yaw)

        if not self.car_obj.check_car_collision(
            x_list, y_list, yaw_list, ox, oy, kd_tree
        ):
            return None

        d = direction == 1
        x_ind = round(x / self.XY_GRID_RESOLUTION)
        y_ind = round(y / self.XY_GRID_RESOLUTION)
        yaw_ind = round(yaw / self.YAW_GRID_RESOLUTION)

        added_cost = 0.0

        if d != current.direction:
            added_cost += self.SB_COST

        # steer penalty
        added_cost += self.STEER_COST * abs(steer)

        # steer change penalty
        added_cost += self.STEER_CHANGE_COST * abs(current.steer - steer)

        cost = current.cost + added_cost + arc_l

        node = Node(
            x_ind,
            y_ind,
            yaw_ind,
            d,
            x_list,
            y_list,
            yaw_list,
            [d],
            parent_index=self.calc_index(current, config),
            cost=cost,
            steer=steer,
        )

        return node

    def is_same_grid(self, n1, n2):
        if (
            n1.x_index == n2.x_index
            and n1.y_index == n2.y_index
            and n1.yaw_index == n2.yaw_index
        ):
            return True
        return False

    def analytic_expansion(self, current, goal, ox, oy, kd_tree):
        start_x = current.x_list[-1]
        start_y = current.y_list[-1]
        start_yaw = current.yaw_list[-1]

        goal_x = goal.x_list[-1]
        goal_y = goal.y_list[-1]
        goal_yaw = goal.yaw_list[-1]

        max_curvature = math.tan(self.car_obj.MAX_STEER) / self.car_obj.WB
        paths = rs.calc_paths(
            start_x,
            start_y,
            start_yaw,
            goal_x,
            goal_y,
            goal_yaw,
            max_curvature,
            step_size=self.MOTION_RESOLUTION,
        )

        if not paths:
            return None
        best_path, best = None, None

        for path in paths:
            if self.car_obj.check_car_collision(
                path.x, path.y, path.yaw, ox, oy, kd_tree
            ):
                cost = self.calc_rs_path_cost(path)
                if not best or best > cost:
                    best = cost
                    best_path = path

        return best_path

    def update_node_with_analytic_expansion(self, current, goal, c, ox, oy, kd_tree):
        path = self.analytic_expansion(current, goal, ox, oy, kd_tree)

        if path:
            f_x = path.x[1:]
            f_y = path.y[1:]
            f_yaw = path.yaw[1:]

            f_cost = current.cost + self.calc_rs_path_cost(path)
            f_parent_index = self.calc_index(current, c)

            fd = []
            for d in path.directions[1:]:
                fd.append(d >= 0)

            f_steer = 0.0
            f_path = Node(
                current.x_index,
                current.y_index,
                current.yaw_index,
                current.direction,
                f_x,
                f_y,
                f_yaw,
                fd,
                cost=f_cost,
                parent_index=f_parent_index,
                steer=f_steer,
            )
            return True, f_path

        return False, None

    def calc_rs_path_cost(self, reed_shepp_path):
        cost = 0.0
        for length in reed_shepp_path.lengths:
            if length >= 0:  # forward
                cost += length
            else:  # back
                cost += abs(length) * self.BACK_COST

        # switch back penalty
        for i in range(len(reed_shepp_path.lengths) - 1):
            # switch back
            if reed_shepp_path.lengths[i] * reed_shepp_path.lengths[i + 1] < 0.0:
                cost += self.SB_COST

        # steer penalty
        for course_type in reed_shepp_path.ctypes:
            if course_type != "S":  # curve
                cost += self.STEER_COST * abs(self.car_obj.MAX_STEER)

        # steer change penalty
        # calc steer profile
        n_ctypes = len(reed_shepp_path.ctypes)
        u_list = [0.0] * n_ctypes
        for i in range(n_ctypes):
            if reed_shepp_path.ctypes[i] == "R":
                u_list[i] = -self.car_obj.MAX_STEER
            elif reed_shepp_path.ctypes[i] == "L":
                u_list[i] = self.car_obj.MAX_STEER

        for i in range(len(reed_shepp_path.ctypes) - 1):
            cost += self.STEER_CHANGE_COST * abs(u_list[i + 1] - u_list[i])

        return cost

    def planning(self, start, goal, ox, oy):
        """
        start: start node
        goal: goal node
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        """

        start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
        tox, toy = ox[:], oy[:]

        obstacle_kd_tree = cKDTree(np.vstack((tox, toy)).T)

        config = Config(tox, toy, self.XY_GRID_RESOLUTION, self.YAW_GRID_RESOLUTION)

        start_node = Node(
            round(start[0] / self.XY_GRID_RESOLUTION),
            round(start[1] / self.XY_GRID_RESOLUTION),
            round(start[2] / self.YAW_GRID_RESOLUTION),
            True,
            [start[0]],
            [start[1]],
            [start[2]],
            [True],
            cost=0,
        )
        goal_node = Node(
            round(goal[0] / self.XY_GRID_RESOLUTION),
            round(goal[1] / self.XY_GRID_RESOLUTION),
            round(goal[2] / self.YAW_GRID_RESOLUTION),
            True,
            [goal[0]],
            [goal[1]],
            [goal[2]],
            [True],
        )

        openList, closedList = {}, {}

        h_dp = calc_distance_heuristic(
            goal_node.x_list[-1],
            goal_node.y_list[-1],
            ox,
            oy,
            self.XY_GRID_RESOLUTION,
            self.car_obj.BUBBLE_R,
        )

        pq = []
        openList[self.calc_index(start_node, config)] = start_node
        heapq.heappush(
            pq,
            (
                self.calc_cost(start_node, h_dp, config),
                self.calc_index(start_node, config),
            ),
        )
        final_path = None

        while True:
            if not openList:
                print("Error: Cannot find path, No open set")
                return [], [], []

            cost, c_id = heapq.heappop(pq)
            if c_id in openList:
                current = openList.pop(c_id)
                closedList[c_id] = current
            else:
                continue

            is_updated, final_path = self.update_node_with_analytic_expansion(
                current, goal_node, config, ox, oy, obstacle_kd_tree
            )

            if is_updated:
                print("Path found...")
                break

            for neighbor in self.get_neighbors(
                current, config, ox, oy, obstacle_kd_tree
            ):
                neighbor_index = self.calc_index(neighbor, config)
                if neighbor_index in closedList:
                    continue
                if (
                    neighbor_index not in openList
                    or openList[neighbor_index].cost > neighbor.cost
                ):
                    heapq.heappush(
                        pq, (self.calc_cost(neighbor, h_dp, config), neighbor_index)
                    )
                    openList[neighbor_index] = neighbor

        path = self.get_final_path(closedList, final_path)
        return path

    def calc_cost(self, n, h_dp, c):
        ind = (n.y_index - c.min_y) * c.x_w + (n.x_index - c.min_x)
        if ind not in h_dp:
            return n.cost + 999999999  # collision cost
        return n.cost + self.H_COST * h_dp[ind].cost

    def get_final_path(self, closed, goal_node):
        reversed_x, reversed_y, reversed_yaw = (
            list(reversed(goal_node.x_list)),
            list(reversed(goal_node.y_list)),
            list(reversed(goal_node.yaw_list)),
        )
        direction = list(reversed(goal_node.directions))
        nid = goal_node.parent_index
        final_cost = goal_node.cost

        while nid:
            n = closed[nid]
            reversed_x.extend(list(reversed(n.x_list)))
            reversed_y.extend(list(reversed(n.y_list)))
            reversed_yaw.extend(list(reversed(n.yaw_list)))
            direction.extend(list(reversed(n.directions)))

            nid = n.parent_index

        reversed_x = list(reversed(reversed_x))
        reversed_y = list(reversed(reversed_y))
        reversed_yaw = list(reversed(reversed_yaw))
        direction = list(reversed(direction))

        # adjust first direction
        direction[0] = direction[1]

        path = Path(reversed_x, reversed_y, reversed_yaw, direction, final_cost)

        return path

    def verify_index(self, node, c):
        x_ind, y_ind = node.x_index, node.y_index
        if c.min_x <= x_ind <= c.max_x and c.min_y <= y_ind <= c.max_y:
            return True

        return False

    def calc_index(self, node, c):
        ind = (
            (node.yaw_index - c.min_yaw) * c.x_w * c.y_w
            + (node.y_index - c.min_y) * c.x_w
            + (node.x_index - c.min_x)
        )

        if ind <= 0:
            print("Error(calc_index):", ind)

        return ind
