#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <array>

using namespace Eigen;
using namespace std;

class DWA : public rclcpp::Node{
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;

    // parameters
    MatrixXd obstacles;
    VectorXd x, goal;
    double max_speed, min_speed, max_yaw_rate, max_accel, max_delta_yaw_rate;
    double v_resolution, yaw_rate_resolution;
    double dt, predict_time, to_goal_cost_gain, speed_cost_gain, obstacle_cost_gain;
    double robot_stuck_flag_cons, robot_radius;

public:
    static constexpr double pi = atan(1) * 4;
    vector<pair<double, double>> saved_path;

    // rgb color vector
    static constexpr array<double, 3> black = {0, 0, 0};
    static constexpr array<double, 3> green = {0, 1, 0};
    static constexpr array<double, 3> blue = {0, 0, 1};
    static constexpr array<double, 3> red = {1, 0, 0};
    static constexpr array<double, 3> yellow = {1, 1, 0};

    // constructor
    DWA(MatrixXd &obstacles, VectorXd &x, VectorXd &goal)
    : Node("dynamic_window_approach"), 
        obstacles(obstacles), 
        x(x), 
        goal(goal),
        max_speed(1.0), // [m/s]
        min_speed(-0.5), // [m/s]
        max_yaw_rate(40.0 * M_PI / 180.0), // [rad/s]
        max_accel(0.2), // [m/ss]
        max_delta_yaw_rate(40.0 * M_PI / 180.0), // [rad/ss]
        v_resolution(0.01), // [m/s]
        yaw_rate_resolution(0.1 * M_PI / 180.0), // [rad/s]
        dt(0.1), // [s] Time tick for motion prediction
        predict_time(4.0), // [s]
        to_goal_cost_gain(0.15),
        speed_cost_gain(1.0),
        obstacle_cost_gain(1.0),
        robot_stuck_flag_cons(0.001), // constant to prevent robot stuck
        robot_radius(1.0)
    {
        pose_publisher_ = create_publisher<visualization_msgs::msg::Marker>("robot_pose", 10);
        trajectory_publisher_ = create_publisher<visualization_msgs::msg::Marker>("trajectory_pose", 10);
        path_publisher_ = create_publisher<visualization_msgs::msg::Marker>("path_pose", 10);
        obstacle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_pose", 10);
        goal_publisher_ = create_publisher<visualization_msgs::msg::Marker>("goal", 10);
        
    }

    // motion model
    VectorXd motion(VectorXd &x,  VectorXd &u, double dt)
    {

        // cout << u.transpose() << endl;
        x(0) += u(0) * cos(x(2)) * dt; // x_pos (m)
        x(1) += u(0) * sin(x(2)) * dt; // y_pos (m)
        x(2) += u(1) * dt; // omega (rad)
        x(3) = u(0); // trans velocity (m/s)
        x(4) = u(1); // rot velocity (rad/s)

        return x;
    }

    // caclulate dynamic window based on current state x
    VectorXd calc_dynamic_window(const VectorXd &x) const
    {

        // cout << x.transpose() << endl;
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
        trajectory.row(0) = x;

        for (int i=1; i <= steps; i++)
        {   
            x = motion(x, u, dt); 
            trajectory.row(i) = x;
        }

        return trajectory;
    }

    // calc to goal cost with angle difference
    double calc_to_goal_cost(const MatrixXd& trajectory, const VectorXd &goal) const
    {
        double dx, dy, error_angle, cost_angle, cost;
        int rows = trajectory.rows();
        dx = goal[0] - trajectory(rows-1, 0);
        dy = goal[1] - trajectory(rows-1, 1);
        error_angle = atan2(dy, dx);
        cost_angle = error_angle - trajectory(rows-1, 2);

        // obtain smallest angle differences following periodic property
        cost = abs(atan2(sin(cost_angle), cos(cost_angle)));
        
        return cost;
    }

    // calc obstacle cost
    double calc_obstacle_cost(const MatrixXd& trajectory)
    {

        VectorXd ox = obstacles.col(0);
        VectorXd oy = obstacles.col(1);

        int drows = ox.rows();
        int dcols = trajectory.rows();

        MatrixXd dx = trajectory.col(0).transpose().replicate(drows, 1) - ox.replicate(1, dcols);
        MatrixXd dy = trajectory.col(1).transpose().replicate(drows, 1) - oy.replicate(1, dcols);

        // distance from every point on the trajectory
        // to each obstacles
        MatrixXd dis2obs = (dx.array().pow(2) + dy.array().pow(2)).sqrt();

        double min_dis = dis2obs.minCoeff();

        if (min_dis <= robot_radius){
            return numeric_limits<double>::infinity();
        }

        // the closer the robot to obstacles
        // the higher the cost
        // cout << min_dis << endl;
        return 1 / min_dis;
    }

    // calculate control input and trajectory
    pair<VectorXd, MatrixXd> calc_control_and_trajectory(
        const VectorXd &x, 
        const VectorXd &dw,
        const VectorXd &goal)
    {
        // VectorXd x_init = x;
        MatrixXd best_trajectory;
        best_trajectory = x;
        double min_cost = numeric_limits<double>::infinity();
        VectorXd best_u = VectorXd::Zero(2);

        for (double v = dw(0); v <= dw(1); v+=v_resolution)
        {
            for (double y = dw(2); y <= dw(3); y+=yaw_rate_resolution)
            {
                MatrixXd trajectory = predict_trajectory(x, v, y);
                // cout << trajectory.row(trajectory.rows()-1) << endl;
                
            
                double to_goal_cost = to_goal_cost_gain * calc_to_goal_cost(trajectory, goal);
                double speed_cost = speed_cost_gain * (max_speed - trajectory(trajectory.rows()-1, 3));
                double ob_cost = obstacle_cost_gain * calc_obstacle_cost(trajectory);

                double final_cost = to_goal_cost + speed_cost + ob_cost;

                // cout << final_cost << endl;
                // "\n" << speed_cost << "\n" << ob_cost << endl;

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
                // cout << "v: " << v << " min_cost: " << min_cost << endl;
            }
        }
        return make_pair(best_u, best_trajectory);
    }

    // calculate control input
    pair<VectorXd, MatrixXd> dwa_control(const VectorXd &x, const VectorXd &goal)
    {
        VectorXd dw = calc_dynamic_window(x);
        // cout << dw.transpose() << endl;
        return calc_control_and_trajectory(x, dw, goal);
    }


    // setup goal marker message
    void setup_goal_msg(
        const VectorXd &goal,
        shared_ptr<visualization_msgs::msg::Marker> &marker,
        const array<double, 3> &colors, 
        double scale
    )
    {
        auto timestamp = this->get_clock()->now();
        marker->header.frame_id = "map";
        marker->header.stamp = timestamp;
        marker->ns = "goal";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::SPHERE;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->lifetime = rclcpp::Duration::from_seconds(0);
        marker->scale.x = scale;
        marker->scale.y = scale;
        marker->scale.z = scale;
        marker->color.a = 1.0;
        marker->color.r = colors[0];
        marker->color.g = colors[1];
        marker->color.b = colors[2];
        marker->pose.position.x = goal(0);
        marker->pose.position.y = goal(1);
        marker->pose.orientation.z = 0.0;            
    }


    // setup obstacle marker message
    void setup_obstacle_msg(
        const MatrixXd &obstacles,
        shared_ptr<visualization_msgs::msg::MarkerArray> &ob_marker_arr,
        const array<double, 3> &colors, 
        double scale
    )
    {
        
        int rows = obstacles.rows();
        auto timestamp = this->get_clock()->now();

        for (int i=0; i<rows; i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = timestamp;
            marker.ns = "obstacle";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration::from_seconds(0);
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            marker.color.a = 1.0;
            marker.color.r = colors[0];
            marker.color.g = colors[1];
            marker.color.b = colors[2];
            marker.pose.position.x = obstacles(i, 0);
            marker.pose.position.y = obstacles(i, 1);
            marker.pose.orientation.z = 0.0;            

            ob_marker_arr->markers.emplace_back(std::move(marker));
        }
    }

    // setup robot marker message
    void setup_robot_msg(
        const VectorXd &x,
        shared_ptr<visualization_msgs::msg::Marker> &marker,
        const array<double, 3> &colors,
        double scale
    )    
    {
        auto timestamp = this->get_clock()->now();
        marker->header.frame_id = "map";
        marker->header.stamp = timestamp;
        marker->ns = "robot";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::SPHERE;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->lifetime = rclcpp::Duration::from_seconds(0);
        marker->scale.x = scale;
        marker->scale.y = scale;
        marker->scale.z = scale;
        marker->color.a = 1.0;
        marker->color.r = colors[0];
        marker->color.g = colors[1];
        marker->color.b = colors[2];
        marker->pose.position.x = x(0);
        marker->pose.position.y = x(1);
        marker->pose.orientation.z = 0.0;              
    }

    // setup trajectory marker message
    void setup_trajectory_msg(
        const MatrixXd &trajectory,
        shared_ptr<visualization_msgs::msg::Marker> &marker,
        const array<double, 3> &colors,
        double scale
    )    
    {
        auto timestamp = this->get_clock()->now();
        marker->header.frame_id = "map";
        marker->header.stamp = timestamp;
        marker->ns = "trajectory";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->lifetime = rclcpp::Duration::from_seconds(0);
        marker->scale.x = scale;
        marker->scale.y = scale;
        marker->scale.z = scale;
        marker->color.a = 1.0;
        marker->color.r = colors[0];
        marker->color.g = colors[1];
        marker->color.b = colors[2];

        int num_points = trajectory.rows();

        // cout << x(0) << " - " << x(1) << endl;
        for (int i = 0; i < num_points; i++) {
            geometry_msgs::msg::Point point;
            point.x = trajectory(i, 0);
            point.y = trajectory(i, 1);

            std_msgs::msg::ColorRGBA color;
            color.a = 1.0;
            color.r = colors[0];
            color.g = colors[1];
            color.b = colors[2];

            marker->points.emplace_back(point);
            marker->colors.emplace_back(color);
        }        
    }    

    // setup path marker message
    void setup_path_msg(
        const vector<pair<double, double>> &saved_path, 
        shared_ptr<visualization_msgs::msg::Marker> &marker,
        const array<double, 3> &colors,
        double scale        
    )
    {
        auto timestamp = this->get_clock()->now();
        marker->header.frame_id = "map";
        marker->header.stamp = timestamp;
        marker->ns = "path";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->lifetime = rclcpp::Duration::from_seconds(0);
        marker->scale.x = scale;
        marker->scale.y = scale;
        marker->scale.z = scale;
        marker->color.a = 1.0;
        marker->color.r = colors[0];
        marker->color.g = colors[1];
        marker->color.b = colors[2];

        geometry_msgs::msg::Point point;
        const auto& pair = saved_path.back();
        point.x = pair.first;
        point.y = pair.second;

        std_msgs::msg::ColorRGBA color;
        color.a = 1.0;
        color.r = colors[0];
        color.g = colors[1];
        color.b = colors[2];

        marker->points.emplace_back(point);
        marker->colors.emplace_back(color);
    }

    // main execution
    void run() 
    {
        rclcpp::Rate rate(100);
        
        double obstacle_scale = 0.7;
        double goal_scale = 1.5;
        double robot_scale = 0.8;
        double trajectory_scale = 0.25;
        double path_scale = 0.25;

        auto ob_marker_arr = std::make_shared<visualization_msgs::msg::MarkerArray>();
        setup_obstacle_msg(obstacles, ob_marker_arr, black, obstacle_scale);

        auto goal_marker = std::make_shared<visualization_msgs::msg::Marker>();
        setup_goal_msg(goal, goal_marker, yellow, goal_scale);

        auto path_marker = std::make_shared<visualization_msgs::msg::Marker>();        

        while (rclcpp::ok()){
            auto res_run = dwa_control(x, goal);
            VectorXd u = res_run.first;
            MatrixXd trajectory = res_run.second;

            saved_path.push_back({x(0), x(1)});

            auto robot_marker = std::make_shared<visualization_msgs::msg::Marker>();
            setup_robot_msg(x, robot_marker, red, robot_scale);

            auto trajectory_msg = std::make_shared<visualization_msgs::msg::Marker>();
            setup_trajectory_msg(trajectory, trajectory_msg, green, trajectory_scale);

            setup_path_msg(saved_path, path_marker, red, path_scale);

            x = motion(x, u, dt);

            double dist_to_goal = sqrt(pow((x(0) - goal(0)), 2) + pow((x(1) - goal(1)), 2));

            if (dist_to_goal <= robot_radius)
            {
                cout << "Goals!!" << endl;
                break;
            }

            obstacle_publisher_->publish(std::move(*ob_marker_arr));
            goal_publisher_->publish(std::move(*goal_marker));
            pose_publisher_->publish(std::move(*robot_marker));
            trajectory_publisher_->publish(std::move(*trajectory_msg));
            path_publisher_->publish(std::move(*path_marker));
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    // obstacles data
    vector<Eigen::Vector2d> obstacles_data = {
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
    for (size_t i = 0; i < obstacles_data.size(); i++)
    {
        obstacles.row(i) = obstacles_data[i];
    }

    // initial state
    VectorXd x(5);
    x << 0.0, 0.0, double(3.14 / 8.0), 0.0, 0.0;
    
    // goal state
    double gx = 10.0, gy = 10.0;
    VectorXd goal(2);
    goal << gx, gy;
    
    // run dwa
    auto RobotDW_node = make_shared<DWA>(obstacles, x, goal);
    RobotDW_node->run();

    rclcpp::shutdown();
    return 0;
}