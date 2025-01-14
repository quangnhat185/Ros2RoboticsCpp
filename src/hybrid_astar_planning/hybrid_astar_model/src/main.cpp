#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "hybrid_astar_model/hybrid_astar_model.hpp"
#include "planning_msgs/srv/paths.hpp"

using namespace std;
using namespace hybrid_astar_model;
using namespace std::chrono_literals;

class HAstarRunner : public rclcpp::Node
{
  public:
    HAstarRunner(vector<float>& start, vector<float>& goal, vector<float>& ox, vector<float>& oy)
      : Node("hybrid_astar_model_node")
      , start(start)
      , goal(goal)
      , ox(ox)
      , oy(oy)
    {
        client_ = this->create_client<planning_msgs::srv::Paths>("hybrid_astar_planning_server");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
    }

    void run(std::shared_ptr<planning_msgs::srv::Paths_Response> paths)
    {
        int num_items = paths->x_list.size();
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.frame_id = "world";
        transform_stamped.child_frame_id = "base_link";

        // Set publish rate
        rclcpp::Rate rate(5);

        int i = 0;
        while (rclcpp::ok() && i < num_items)
        {
            transform_stamped.header.stamp = this->get_clock()->now();

            // Set the translation (example values, update as needed)
            transform_stamped.transform.translation.x = paths->x_list[i];
            transform_stamped.transform.translation.y = paths->y_list[i];
            transform_stamped.transform.translation.z = 0.0;

            double roll = 0.0;
            double pitch = 0.0;
            double yaw = paths->yaw_list[i];

            tf2::Quaternion quaternion;
            quaternion.setRPY(roll, pitch, yaw);

            transform_stamped.transform.rotation.x = quaternion.x();
            transform_stamped.transform.rotation.y = quaternion.y();
            transform_stamped.transform.rotation.z = quaternion.z();
            transform_stamped.transform.rotation.w = quaternion.w();

            tf_broadcaster_->sendTransform(transform_stamped);
            RCLCPP_INFO(this->get_logger(), "Publishing transform...");
            i++;

            rate.sleep();  // Sleep to maintain the publish rate
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

        auto result = client_->async_send_request(request);
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

  private:
    rclcpp::Client<planning_msgs::srv::Paths>::SharedPtr client_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::TimerBase::SharedPtr timer_;
    vector<float> start;
    vector<float> goal;
    vector<float> ox;
    vector<float> oy;
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
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}