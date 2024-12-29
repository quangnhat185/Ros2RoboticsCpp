#ifndef REEDS_SHEPP_HPP
#define REEDS_SHEPP_HPP

#include <iostream>
#include <vector>

#include "hybrid_astar_model/car.hpp"
using namespace std;
using namespace Eigen;

namespace reeds_shepp
{
CarPose interpolate(const double& dist, const double& length, const char& mode, const double& max_curvature,
                    CarPose& origin);
vector<VectorXd> calc_interpolate_dists_list(const VectorXd& lengths, const double& step_size);
}  // namespace reeds_shepp

#endif