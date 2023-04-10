#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{   
    private:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    public:
        double count = 0;
        MinimalPublisher() : Node("sin_publisher")
        {
            publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sin_signal", 10);
        }

        float pi = std::atan(1) * 4;

    void run()
    {
        rclcpp::Rate rate(100);
        while(rclcpp::ok()){
            auto message = std::make_unique<std_msgs::msg::Float32MultiArray>();
            double rad_angle = count  * 2 * pi;

            message->data.push_back(rad_angle);
            message->data.push_back(std::sin(rad_angle));

            RCLCPP_INFO(this->get_logger(), "Publishing: %f and %f",message->data[0], message->data[1]);

            publisher_->publish(std::move(message));

            count += 0.005;
            rate.sleep();
        }
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    // create new shared pointer to an object 
    auto node = std::make_shared<MinimalPublisher>();
    node->run();
    rclcpp::shutdown();
    return 0;
}