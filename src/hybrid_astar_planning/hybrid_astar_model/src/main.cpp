#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hybrid_astar_model/hybrid_astar_model.hpp"
#include "planning_msgs/srv/paths.hpp"

using namespace std;
using namespace hybrid_astar_model;
using namespace std::chrono_literals;

// TODO: Visualization, documentation, linting
class HAstarRunner : public rclcpp::Node
{
  private:
    rclcpp::Client<planning_msgs::srv::Paths>::SharedPtr planning_client;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obj_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::TimerBase::SharedPtr timer_;
    vector<float> start;
    vector<float> goal;
    vector<float> ox;
    vector<float> oy;

  public:
    HAstarRunner(vector<float>& start, vector<float>& goal, vector<float>& ox, vector<float>& oy)
      : Node("hybrid_astar_node")
      , start(start)
      , goal(goal)
      , ox(ox)
      , oy(oy)
    {
        obj_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle", 10);
        traj_publisher = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);
        goal_publisher = this->create_publisher<visualization_msgs::msg::Marker>("goal", 10);
        planning_client = this->create_client<planning_msgs::srv::Paths>("hybrid_astar_planning_server");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        while (!planning_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
    }

    void publish_goal()
    {
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = "world";
        goal_marker.header.stamp = this->get_clock()->now();
        goal_marker.ns = "goal";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;

        // set color
        goal_marker.color.a = 1.0;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 1.0;
        goal_marker.color.b = 0.0;

        // set scale
        goal_marker.scale.x = 3.0;
        goal_marker.scale.y = 3.0;
        goal_marker.scale.z = 3.0;

        // set position
        goal_marker.pose.position.x = this->goal[0];
        goal_marker.pose.position.y = this->goal[1];

        goal_publisher->publish(goal_marker);
    }

    void publish_trajectory(vector<float>& traj_x, vector<float>& traj_y)
    {
        visualization_msgs::msg::Marker traj_marker;
        traj_marker.header.frame_id = "world";
        traj_marker.header.stamp = this->get_clock()->now();
        traj_marker.ns = "trajectory";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::msg::Marker::ADD;

        // set color
        traj_marker.color.a = 1.0;
        traj_marker.color.r = 0.0;
        traj_marker.color.g = 1.0;
        traj_marker.color.b = 0.0;

        // set scale
        traj_marker.scale.x = 0.3;

        // Populate points

        for (size_t i = 0; i < traj_x.size(); i++)
        {
            geometry_msgs::msg::Point point;
            point.x = traj_x[i];
            point.y = traj_y[i];
            point.z = 0.1;

            traj_marker.points.push_back(point);
        }

        traj_publisher->publish(traj_marker);
    }

    void publish_obstacles()
    {
        visualization_msgs::msg::MarkerArray obst_marker_array;
        int size_obst = ox.size();

        for (int i = 0; i < size_obst; i++)
        {
            visualization_msgs::msg::Marker obst_marker;
            obst_marker.header.frame_id = "world";
            obst_marker.header.stamp = this->get_clock()->now();
            obst_marker.ns = "obstacles";
            obst_marker.id = i;
            obst_marker.type = visualization_msgs::msg::Marker::CUBE;
            obst_marker.action = visualization_msgs::msg::Marker::ADD;

            obst_marker.pose.position.x = ox[i];
            obst_marker.pose.position.y = oy[i];
            obst_marker.pose.position.z = 0.0;

            obst_marker.pose.orientation.x = 0.0;
            obst_marker.pose.orientation.y = 0.0;
            obst_marker.pose.orientation.z = 0.0;
            obst_marker.pose.orientation.w = 1.0;

            obst_marker.scale.x = 1.0;
            obst_marker.scale.y = 1.0;
            obst_marker.scale.z = 1.0;

            obst_marker.color.a = 1.0;
            obst_marker.color.r = 1.0;
            obst_marker.color.g = 0.0;
            obst_marker.color.b = 0.0;
            obst_marker_array.markers.push_back(obst_marker);
        }

        obj_publisher->publish(obst_marker_array);
    }

    void run(std::shared_ptr<planning_msgs::srv::Paths_Response> paths)
    {
        int num_items = paths->x_list.size();
        geometry_msgs::msg::TransformStamped transform_base_1;
        transform_base_1.header.frame_id = "world";
        transform_base_1.child_frame_id = "base_link";

        geometry_msgs::msg::TransformStamped transform_base_2;
        transform_base_2.header.frame_id = "world";
        transform_base_2.child_frame_id = "base_link_2";

        transform_base_2.transform.translation.x = goal[0];
        transform_base_2.transform.translation.y = goal[1];
        transform_base_2.transform.translation.z = 0.0;

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = goal[2] - PI / 2;

        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        transform_base_2.transform.rotation.x = quaternion.x();
        transform_base_2.transform.rotation.y = quaternion.y();
        transform_base_2.transform.rotation.z = quaternion.z();
        transform_base_2.transform.rotation.w = quaternion.w();

        vector<float> traj_x, traj_y;
        // Set publish rate
        rclcpp::Rate rate(PUB_RATE);

        int i = 0;
        while (rclcpp::ok() && i < num_items)
        {
            transform_base_1.header.stamp = this->get_clock()->now();

            // Set the translation (example values, update as needed)
            transform_base_1.transform.translation.x = paths->x_list[i];
            transform_base_1.transform.translation.y = paths->y_list[i];
            transform_base_1.transform.translation.z = 0.0;

            double roll = 0.0;
            double pitch = 0.0;
            double yaw = paths->yaw_list[i] - PI / 2;

            tf2::Quaternion quaternion;
            quaternion.setRPY(roll, pitch, yaw);

            transform_base_1.transform.rotation.x = quaternion.x();
            transform_base_1.transform.rotation.y = quaternion.y();
            transform_base_1.transform.rotation.z = quaternion.z();
            transform_base_1.transform.rotation.w = quaternion.w();

            tf_broadcaster_->sendTransform(transform_base_1);
            tf_broadcaster_->sendTransform(transform_base_2);
            RCLCPP_INFO(this->get_logger(), "Publishing transform...");

            traj_x.push_back(paths->x_list[i]);
            traj_y.push_back(paths->y_list[i]);

            publish_obstacles();
            // publish_goal();
            publish_trajectory(traj_x, traj_y);
            rate.sleep();  // Sleep to maintain the publish rate
            i++;
        }

        RCLCPP_INFO(this->get_logger(), "Transform publishing complete.");
    }

    std::shared_ptr<planning_msgs::srv::Paths::Response> send_request()
    {
        auto request = std::make_shared<planning_msgs::srv::Paths::Request>();
        request->start = this->start;
        request->goal = this->goal;
        request->ox = this->ox;
        request->oy = this->oy;

        auto result = planning_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return result.get();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
            return nullptr;
        }

        return nullptr;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    vector<float> start = {
        10.0,
        10.0,
        PI / 2.0,
    };
    vector<float> goal = {
        50.0,
        50.0,
        -PI / 2.0,
    };
    vector<float> ox, oy;

    for (int i = 0; i < 60; i++)
    {
        ox.push_back(float(i));
        oy.push_back(0.0);
    }

    for (int i = 0; i < 60; i++)
    {
        ox.push_back(float(60.0));
        oy.push_back(float(i));
    }

    for (int i = 0; i < 61; i++)
    {
        ox.push_back(float(i));
        oy.push_back(float(60.0));
    }

    for (int i = 0; i < 61; i++)
    {
        ox.push_back(0.0);
        oy.push_back(float(i));
    }

    for (int i = 0; i < 40; i++)
    {
        ox.push_back(float(20.0));
        oy.push_back(float(i));
    }

    for (int i = 0; i < 40; i++)
    {
        ox.push_back(float(40.0));
        oy.push_back(60.0 - float(i));
    }
    auto node = std::make_shared<HAstarRunner>(start, goal, ox, oy);
    RCLCPP_INFO(node->get_logger(), "Hybrid Astar Model Node has been started...");

    auto response = node->send_request();
    node->run(response);
    rclcpp::shutdown();

    return 0;
}