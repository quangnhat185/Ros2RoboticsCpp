#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;


class DWA{
private:
    // max_delta_yaw_rate is rotation velocity
    double max_speed, min_speed, max_yaw_rate, max_accel, max_delta_yaw_rate;
    double v_resolution, yaw_rate_resolution;
public:
    double pi = atan(1) * 4;
    double dt, predict_time, to_goal_cost_gain, speed_cost_gain, obstacle_cost_gain;
    double robot_stuck_flag_cons, robot_radius;
    MatrixXd obstacles;


    DWA(MatrixXd &obstacles): obstacles(obstacles)
    {
        max_speed = 1.0; //[m/s]
        min_speed = -0.5;  //[m/s]
        max_yaw_rate = 40.0 * pi / 180.0;  //[rad/s]
        max_accel = 0.2;  //[m/ss]
        max_delta_yaw_rate = 40.0 * pi / 180.0;  //[rad/ss]
        v_resolution = 0.01;  //[m/s]
        yaw_rate_resolution = 0.1 * pi / 180.0;  //[rad/s]
        dt = 0.1;  //[s] Time tick for motion prediction
        predict_time = 3.0;  //[s]
        to_goal_cost_gain = 0.15;
        speed_cost_gain = 1.0;
        obstacle_cost_gain = 1.0;
        robot_stuck_flag_cons = 0.001; // constant to prevent robot stucked        
        robot_radius = 1.0;
        
    }

    // motion model
    VectorXd motion(VectorXd x,  VectorXd u, double dt)
    {
        x(0) += u(0) * cos(x(2)) * dt; // x_pos (m)
        x(1) += u(0) * sin(x(2)) * dt; // y_pos (m)
        x(2) += u(1) * dt; // omega (rad)
        x(3) = u(0); // trans velocity (m/s)
        x(4) = u(1); // rot velocity (rad/s)

        return x;
    }

    VectorXd calc_dynamic_window(const VectorXd &x)
    // caclulate dynamic window based on current state x
    {
        // Dynamic window from robot specification
        VectorXd Vs(4);
        Vs << min_speed, max_speed, -max_yaw_rate, max_yaw_rate;
        
        // Dynamic window from motion model
        VectorXd Vd(4);
        Vd << 
            x(3) - max_accel * dt,
            x(3) + max_accel * dt,
            x(4) - max_delta_yaw_rate * dt,
            x(4) + max_delta_yaw_rate * dt;

        //  (v_min, v_max, yaw_rate_min, yaw_rate_max)
        VectorXd dw(4);
        dw << 
            max(Vs(0), Vd(0)), min(Vs(1), Vd(1)),
            max(Vs(2), Vd(2)), min(Vs(3), Vd(3));
        
        // cout << dw.transpose() << endl;
        return dw;
    }

    // predict trajectory with an input
    MatrixXd predict_trajectory(const VectorXd &x_init, double v, double y)
    {
        int steps = static_cast<int>(predict_time / dt);
        VectorXd x = x_init;
        VectorXd u(2);
        u << v, y;
        MatrixXd trajectory = MatrixXd::Zero(steps+1, x_init.size());
        trajectory.row(0) = x.transpose();

        for (int i=1; i <= steps; i++)
        {   
            x = motion(x, u, dt);            
            trajectory.row(i) = x.transpose();
        }

        // cout << trajectory << endl;
        return trajectory;
    }

    // calc to goal cost with angle difference
    double calc_to_goal_cost(const MatrixXd& trajectory, const VectorXd &goal)
    {
        double dx, dy, error_angle, cost_angle, cost;
        int rows = trajectory.rows();
        dx = goal[0] - trajectory(rows-1, 0);
        dy = goal[1] - trajectory(rows-1, 1);
        error_angle = atan2(dy, dx);
        cost_angle = error_angle - trajectory(rows-1, 2);

        // obtain smallest angle differences following periodic property
        cost = abs(atan2(sin(cost_angle), cos(cost_angle)));
        
        // cout << cost << endl;
        return cost;
    }

    double calc_obstacle_cost(const MatrixXd& trajectory)
    {

        VectorXd ox = obstacles.col(0);
        VectorXd oy = obstacles.col(1);

        int drows = ox.rows();
        int dcols = trajectory.rows();

        // MatrixXd dx(drows, dcols);
        // MatrixXd dy(drows, dcols);

        // for (int i=0; i<drows; i++)
        // {
        //     VectorXd temp_minus_x = ox(i) * VectorXd::Ones(dcols);
        //     dx.row(i) = trajectory.col(0).transpose() - temp_minus_x.transpose();

        //     VectorXd temp_minus_y = ox(i) * VectorXd::Ones(dcols);
        //     dy.row(i) = trajectory.col(1).transpose() - temp_minus_y.transpose();            
        // }

        // cout << ox.replicate(2, 2) << endl;

        MatrixXd dx = trajectory.col(0).transpose().replicate(drows, 1) - ox.replicate(1, dcols);
        MatrixXd dy = trajectory.col(1).transpose().replicate(drows, 1) - oy.replicate(1, dcols);

        // distance from every point on the trajectory
        // to each obstacles
        MatrixXd dis2obs = (dx.array().pow(2) + dy.array().pow(2)).sqrt();
        // cout << oy << endl;

        double min_dis = dis2obs.minCoeff();
        if (min_dis <= robot_radius){
            return numeric_limits<double>::infinity();
        }

        // the closer the robot to obstacles
        // the higher the cost
        return 1 / min_dis;
    }

    pair<VectorXd, MatrixXd> calc_control_and_trajectory(
        const VectorXd &x, 
        const VectorXd &dw,
        const VectorXd &goal)
    {
        MatrixXd best_trajectory;
        VectorXd x_init = x;
        double min_cost = numeric_limits<double>::infinity();
        VectorXd best_u = VectorXd::Zero(2);

        double v = dw(0), y = dw(2); // min trans and rot velocity
        
        // cout << dw.transpose() << endl;
        for (v; v <= dw(1); v+=v_resolution)
        {
            for (y; y <= dw(3); y+=yaw_rate_resolution)
            {
                MatrixXd trajectory = predict_trajectory(x_init, v, y);

                double to_goal_cost = to_goal_cost_gain * calc_to_goal_cost(trajectory, goal);
                double speed_cost = speed_cost_gain * (max_speed - trajectory(trajectory.rows()-1, 3));
                double ob_cost = obstacle_cost_gain * calc_obstacle_cost(trajectory);

                double final_cost = to_goal_cost + speed_cost + ob_cost;

                // cout << to_goal_cost << "\n" << speed_cost << "\n" << ob_cost << endl;

                if (min_cost >= final_cost)
                {
                    min_cost = final_cost;
                    best_u(0) = v;
                    best_u(1) = y;
                    best_trajectory = trajectory;

                    if(
                        abs(best_u(0)) < robot_stuck_flag_cons && 
                        abs(x(3)) < robot_stuck_flag_cons
                    )
                    {
                        best_u[1] = -max_delta_yaw_rate;
                    }
                }

            }
        }

        return make_pair(best_u, best_trajectory);
    }

    
    pair<VectorXd, MatrixXd> dwa_run(const VectorXd &x, const VectorXd &goal)
    {
        VectorXd dw = calc_dynamic_window(x);
        // cout << dw.transpose() << endl;
        return calc_control_and_trajectory(x, dw, goal);
    }
};

int main(int argc, char** argv)
{
    int num_obstacles = 15;
    vector<vector<double>> obstacles_data = {
        {-1, -1},
        {0, 2},
        {4.0, 2.0},
        {5.0, 4.0},
        {5.0, 5.0},
        {5.0, 6.0},
        {5.0, 9.0},
        {8.0, 9.0},
        {7.0, 9.0},
        {8.0, 10.0},
        {9.0, 11.0},
        {12.0, 13.0},
        {12.0, 12.0},
        {15.0, 15.0},
        {13.0, 13.0}
    };

    MatrixXd obstacles(obstacles_data.size(), 2);
    
    for (int i=0; i<obstacles_data.size(); i++)
    {
        obstacles.row(i) << obstacles_data[i][0],  obstacles_data[i][1];
    }

    // cout << obstacles << endl;

    DWA RobotDW(obstacles);

    VectorXd x(5);
    x << 0.0, 0.0, double(3.14 / 8.0), 0.0, 0.0;
    
    // cout << x.transpose() << endl;
    // double v = -0.02, y = -0.0698;
    // VectorXd u(2);
    // u << v, y;

    double gx = 10.0, gy = 10.0;
    VectorXd goal(2);
    goal << gx, gy;
    
    int counter = 0;
    while (counter < 100)
    {
        auto res_run = RobotDW.dwa_run(x, goal);
        VectorXd u = res_run.first;
        x = RobotDW.motion(x, u, RobotDW.dt);
        cout << u.transpose() << endl;
        counter++;
    }


    // MatrixXd trajectory = RobotDW.predict_trajectory(x, v, y);
    // VectorXd goal = obstacles.row(obstacles.rows() -1);
    // RobotDW.calc_to_goal_cost(trajectory, goal);
    // cout << RobotDW.calc_obstacle_cost(trajectory) << endl;

    // VectorXd dw = RobotDW.calc_dynamic_window(x);
    // auto res = RobotDW.calc_control_and_trajectory(x, dw, goal);
    // cout << res.first  << endl;
    // cout << res.second.row(5) << endl;
    return 0;
} 