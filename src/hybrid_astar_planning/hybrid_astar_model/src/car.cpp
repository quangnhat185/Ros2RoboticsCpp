#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "hybrid_astar_model/car.hpp"

using namespace Eigen;
using namespace std;

/// @brief calculate 2D rotation matrix given angle (rad)
/// @param angle (rad)
/// @return 2x2 rotation matrix
Matrix2d rot_mat_2d(const double& angle)
{
    Matrix2d res;
    res << cos(angle), -sin(angle), sin(angle), cos(angle);

    return res;
}

bool car::rectangle_check(const double& x, const double& y, const double& yaw, const vector<double>& ox,
                     const vector<double>& oy)
{
    Matrix2d rot = rot_mat_2d(yaw);

    for (size_t i = 0; i < ox.size(); i++)
    {
        double dx = ox[i] - x;
        double dy = oy[i] - y;

        Vector2d dxy;
        dxy << dx, dy;

        auto converted_xy = dxy.transpose() * rot;
        double rx = converted_xy[0];
        double ry = converted_xy[1];

        if (!((rx > LF) || (rx < -LB) || (ry > W / 2.0) || (ry < -W / 2.0)))
        {
            return false;
        }
    }

    return true;
}

vector<int> KdTree::querry_ball_point(const Vector2d& cxy, const double& radius)
{
    vector<int> res;

    for (size_t i = 0; i < KdTree::data.size(); i++)
    {
        Vector2d deviation = data[i] - cxy;
        double loss = sqrt(pow(deviation[0], 2) + pow(deviation(1), 2));

        if (loss <= radius)
            res.push_back(i);
    }

    return res;
}

double pi_2_pi(const double& angle)
{
    return fmod(angle + PI, 2 * PI) - PI;
};

CarPose car::update_pose(CarPose& curr_pose, const double& distance, const double& steer, const double& L)
{
    CarPose newPose;
    newPose.x = curr_pose.x + distance * cos(curr_pose.yaw);
    newPose.y = curr_pose.y + distance * sin(curr_pose.yaw);
    newPose.yaw = curr_pose.yaw + pi_2_pi(distance * tan(steer) / L);

    return newPose;
}

bool car::check_car_collision(const vector<double>& x_list, const vector<double>& y_list, const vector<double>& yaw_list,
                         const vector<double>& ox, const vector<double>& oy, KdTree& kdtree)
{
    for (size_t i = 0; i < x_list.size(); i++)
    {
        double cx = x_list[i] + BUBBLE_DIST * cos(yaw_list[i]);
        double cy = y_list[i] + BUBBLE_DIST * sin(yaw_list[i]);
        Vector2d cxy;
        cxy << cx, cy;
        vector<int> idx = kdtree.querry_ball_point(cxy, BUBBLE_DIST);

        if (idx.empty())
            continue;

        vector<double> fill_ox, fill_oy;
        for (size_t i = 0; i < ox.size(); i++)
        {
            fill_ox.push_back(ox[i]);
            fill_ox.push_back(oy[i]);
        }
        bool rec_check = car::rectangle_check(x_list[i], y_list[i], yaw_list[i], fill_ox, fill_oy);

        if (!rec_check)
            return false;
    }
    return true;
}