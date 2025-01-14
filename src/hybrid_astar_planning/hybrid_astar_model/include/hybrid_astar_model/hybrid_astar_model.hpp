#ifndef HYBRID_ASTAR_MODEL
#define HYBRID_ASTAR_MODEL

#include <algorithm>
#include <string>
#include <vector>

namespace hybrid_astar_model

{
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