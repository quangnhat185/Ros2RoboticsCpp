#ifndef CAR_HPP
#define CAR_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

constexpr double PI = 3.14159265;
constexpr double WB = 3.0;         // rear to front wheel
constexpr double W = 2.0;          // width of car
constexpr double LF = 3.3;         // distance from rear to vehicle front end
constexpr double LB = 1.0;         // distance from rear to vehicle back end
constexpr double MAX_STEER = 0.6;  // [rad] maxium steering angle

constexpr double BUBBLE_DIST = (LF - LB) / 2.0;   // distance from rear to center of vehicle
constexpr double CENTER_POINT = (LF + LB) / 2.0;  // center of the vehicle
constexpr double BUBBLE_R = std::sqrt(std::pow(CENTER_POINT / 2.0, 2) + std::pow(W / 2.0, 2));  // bubble radidus

constexpr double VRX[5] = { LF, LF, -LB, -LB, LF };
constexpr double VRY[5]{ W / 2, -W / 2, W / 2, W / 2 };

struct CarPose
{
    double x;
    double y;
    double yaw;
    int direction;

    CarPose()
      : x(0)
      , y(0)
      , yaw(0)
      , direction(0) {};
    CarPose(const double& x, const double& y, const double& yaw, const int& direction)
      : x(x)
      , y(y)
      , yaw(yaw)
      , direction(direction)
    {
    }

    bool operator==(const CarPose& other) const
    {
        return x == other.x && y == other.y && yaw == other.yaw && direction == other.direction;
    }

    bool operator!=(const CarPose& other) const
    {
        return !(*this == other);
    }
};

struct KdTree
{
    vector<Vector2d> data;
    KdTree(const vector<Vector2d>& points)
      : data(points) {};
    vector<int> querry_ball_point(const Vector2d& cxy, const double& radius);
};

Matrix2d rot_mat_2d(const double& angle);
double pi_2_pi(const double& angle);

namespace car
{
bool rectangle_check(const double& x, const double& y, const double& yaw, const vector<double>& ox,
                     const vector<double>& oy);

bool check_car_collision(const vector<double>& x_list, const vector<double>& y_list, const vector<double>& yaw_list,
                         const vector<double>& ox, const vector<double>& oy, KdTree& kdtree);
CarPose update_pose(CarPose& curr_pose, const double& distance, const double& steer, const double& L);
}  // namespace car

#endif