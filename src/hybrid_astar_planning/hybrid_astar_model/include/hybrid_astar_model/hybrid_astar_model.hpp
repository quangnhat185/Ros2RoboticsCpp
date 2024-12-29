#ifndef HYBRID_ASTAR_MODEL
#define HYBRID_ASTAR_MODEL

#include <algorithm>
#include <string>
#include <vector>

#include "hybrid_astar_model/car.hpp"
#include "hybrid_astar_model/reeds_shepp.hpp"

using namespace std;
class PNode
{
  public:
    int x_ind;
    int y_ind;
    int yaw_ind;
    bool direction;
    double steer;
    vector<double> x_list;
    vector<double> y_list;
    vector<double> yaw_list;
    vector<bool> directions;
    int parent_index;
    double cost;

    PNode() {};

    PNode(int x_ind, int y_ind, int yaw_ind, bool direction, vector<double> x_list, vector<double> y_list,
          vector<double> yaw_list, vector<bool> directions, int parent_index, double cost, double steer)
      : x_ind(x_ind)
      , y_ind(y_ind)
      , yaw_ind(yaw_ind)
      , direction(direction)
      , steer(steer)
      , x_list(x_list)
      , y_list(y_list)
      , yaw_list(yaw_list)
      , directions(directions)
      , parent_index(parent_index)
      , cost(cost)

    {
    }
};

class Path
{
  public:
    vector<double> x_list;
    vector<double> y_list;
    vector<double> yaw_list;
    vector<bool> direction_list;
    double cost;

    Path(vector<double> x_list, vector<double> y_list, vector<double> yaw_list, vector<bool> direction_list,
         double cost)
      : x_list(x_list)
      , y_list(y_list)
      , yaw_list(yaw_list)
      , direction_list(direction_list)
      , cost(cost)
    {
    }
};

class Config
{
  public:
    vector<double> ox;
    vector<double> oy;
    double xy_resolution;
    double yaw_resolution;
    int min_x, min_y, max_x, max_y, x_w, y_w, min_yaw, max_yaw, yaw_w;

    Config(vector<double> ox, vector<double> oy, double xy_resolution, double yaw_resolution)
      : ox(ox)
      , oy(oy)
      , xy_resolution(xy_resolution)
      , yaw_resolution(yaw_resolution)
    {
        double min_x_m = *min_element(ox.begin(), ox.end());
        double min_y_m = *min_element(oy.begin(), oy.end());
        double max_x_m = *max_element(ox.begin(), ox.end());
        double max_y_m = *max_element(oy.begin(), oy.end());
        ox.push_back(min_x_m);
        ox.push_back(max_x_m);
        oy.push_back(min_y_m);
        oy.push_back(max_y_m);

        min_x = round(min_x_m / xy_resolution);
        min_y = round(min_y_m / xy_resolution);
        max_x = round(max_x_m / xy_resolution);
        max_y = round(max_y_m / xy_resolution);

        x_w = round(max_x - min_x);
        y_w = round(max_y - min_y);

        min_yaw = round(PI / yaw_resolution) - 1;
        max_yaw = round(PI / yaw_resolution);
        yaw_w = round(max_yaw - min_yaw);
    };
};

vector<Vector2d> calc_motion_inputs(double min_steer, double max_steer, int n_steer);
PNode calc_next_node(PNode& current, double steer, int direction, Config& config, vector<double>& ox,
                     vector<double>& oy, KdTree& kdtree);

namespace hybrid_astar_model

{
const std::string NAME = "Quang";
constexpr int AGE = 30;
constexpr double PI = 3.141592;

constexpr double XY_GRID_RESOLUTION = 2.0;                   // [m]
constexpr double YAW_GRID_RESOLUTION = 15.0 * M_PI / 180.0;  // [rad]
constexpr double MOTION_RESOLUTION = 0.1;                    // [m] path interpolate resolution
constexpr int N_STEER = 20;                                  // number of steer command

constexpr double SB_COST = 100.0;          // switch back penalty cost
constexpr double BACK_COST = 5.0;          // backward penalty cost
constexpr double STEER_CHANGE_COST = 5.0;  // steer angle change penalty cost
constexpr double STEER_COST = 1.0;         // steer angle change penalty cost
constexpr double H_COST = 5.0;             // Heuristic cost

}  // namespace hybrid_astar_model
#endif