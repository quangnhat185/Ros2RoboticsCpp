#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>

#include "hybrid_astar_model/car.hpp"
#include "hybrid_astar_model/hybrid_astar_model.hpp"
#include "hybrid_astar_model/reeds_shepp.hpp"
#include "rclcpp/rclcpp.hpp"

class HAstarRunner : public rclcpp::Node
{
  public:
    HAstarRunner()
      : Node("hybrid_astar_model_node")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(100ms, std::bind(&HAstarRunner::update_transform, this));
    }

    void update_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "world";
        transform_stamped.child_frame_id = "base_link";

        current_x += 0.1;
        current_y += 0.1;

        // Set the translation (example values, update as needed)
        transform_stamped.transform.translation.x = current_x;
        transform_stamped.transform.translation.y = current_y;
        transform_stamped.transform.translation.z = 0.0;

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(transform_stamped);
        RCLCPP_INFO(this->get_logger(), "Publishing transform...");
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_x = 0;
    double current_y = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HAstarRunner>();
    RCLCPP_INFO(node->get_logger(), "Hybrid Astar Model Node has been started...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}