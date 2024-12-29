#include <eigen3/Eigen/Dense>
#include "hybrid_astar_model/reeds_shepp.hpp"
#include "hybrid_astar_model/car.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

vector<VectorXd> reeds_shepp::calc_interpolate_dists_list(const VectorXd& lengths, const double& step_size)
{
    vector<VectorXd> interpolate_dists_list;

    for (int i = 0; i < lengths.size(); i++)
    {
        int vec_size = int(abs(lengths[i]) / step_size) + 1;
        VectorXd inner_vec = VectorXd::LinSpaced(vec_size, 0, abs(lengths[i]));

        if (lengths[i] < 0){
            inner_vec = -inner_vec;
            inner_vec[0] = 0;
        }
        interpolate_dists_list.push_back(inner_vec);
    }

    return interpolate_dists_list;
}


CarPose reeds_shepp::interpolate(const double& dist, const double& length, const char& mode,  const double& max_curvature, CarPose& origin)
{
    CarPose res;

    if (mode == 'S'){
        res.x = origin.x + dist / max_curvature * cos(origin.yaw);
        res.y = origin.y + dist / max_curvature * sin(origin.yaw);
        res.yaw = origin.yaw;
    }
    else
    {
        double ldx = sin(dist) / max_curvature;
        double ldy = 0.0;
        if (mode == 'L')
        {
            ldy = (1.0 - cos(dist)) / max_curvature;
            res.yaw = origin.yaw + dist;
        }
        else if (mode == 'R')
        {
            ldy = (1.0 - cos(dist)) / -max_curvature;
            res.yaw = origin.yaw - dist;
        }

        double gdx = cos(-origin.yaw) * ldx + sin(-origin.yaw) * ldy;
        double gdy = -sin(-origin.yaw) * ldx + cos(-origin.yaw) * ldy;
        res.x = origin.x + gdx;
        res.y = origin.y + gdy;
    }

    (length > 0.0) ? (res.direction = 1) : (res.direction = -1);

    return res;
};
