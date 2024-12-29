#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "hybrid_astar_model/car.hpp"
#include "hybrid_astar_model/hybrid_astar_model.hpp"

using namespace std;
using namespace hybrid_astar_model;
using namespace Eigen;

vector<Vector2d> calc_motion_inputs(double min_steer, double max_steer, int n_steer)
{
    vector<Vector2d> motion_inputs;
    VectorXd temp_steer_values = VectorXd::LinSpaced(n_steer, min_steer, max_steer);
    VectorXd steer_values = VectorXd(temp_steer_values.size() + 1);
    steer_values << temp_steer_values, 0.0;

    for (int i = 0; i < steer_values.rows(); i++)
    {
        int d = 1;
        Vector2d sd = Vector2d(steer_values[i], d);
        motion_inputs.push_back(sd);
        sd[1] *= -1;
        motion_inputs.push_back(sd);
    }

    return motion_inputs;
}

int calc_index(PNode& node, Config& c)
{
    int ind = (node.yaw_ind - c.min_yaw) * c.x_w * c.y_w + (node.y_ind - c.min_y) * c.x_w + (node.x_ind - c.min_x);

    if (ind <= 0)
    {
        cout << "Error (calc_index): " << ind << endl;
    }

    return ind;
}

PNode calc_next_node(PNode& current, double steer, int direction, Config& config, vector<double>& ox,
                     vector<double>& oy, KdTree& kdtree)
{
    double x = current.x_list.back();
    double y = current.y_list.back();
    double yaw = current.yaw_list.back();

    constexpr double arc_l = XY_GRID_RESOLUTION * 1.5;
    vector<double> x_list, y_list, yaw_list;

    for (double i = 0; i < arc_l; i += MOTION_RESOLUTION)
    {
        CarPose currPose = CarPose(x, y, yaw, MOTION_RESOLUTION * direction);
        CarPose new_pose = car::update_pose(currPose, MOTION_RESOLUTION * direction, steer, WB);
        x_list.push_back(new_pose.x);
        y_list.push_back(new_pose.y);
        yaw_list.push_back(new_pose.yaw);
    }

    if (!car::check_car_collision(x_list, y_list, yaw_list, ox, oy, kdtree))
        return PNode();

    bool d = direction == 1;
    int x_ind = round(x / XY_GRID_RESOLUTION);
    int y_ind = round(y / XY_GRID_RESOLUTION);
    int yaw_ind = round(yaw / YAW_GRID_RESOLUTION);
    double added_cost = 0.0;

    if (d != current.direction)
        added_cost += SB_COST;

    // steer penalty
    added_cost += STEER_COST * abs(steer);

    // steer change penalty
    added_cost += STEER_CHANGE_COST * abs(current.steer - steer);
    double cost = current.cost + added_cost + arc_l;

    int parent_index = calc_index(current, config);
    PNode next_node = PNode(x_ind, y_ind, yaw_ind, d, x_list, y_list, yaw_list, { d }, parent_index, cost, steer);

    return next_node;
}
class HybridAstarModel
{
  public:
    HybridAstarModel()
    {
    }
};
