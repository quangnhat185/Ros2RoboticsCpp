#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{   
    private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

    public:
        int count = 0;
        MinimalPublisher() : Node("sin_publisher")
        {
            publisher_ = this->create_publisher<std_msgs::msg::Float32>("sin_signal", 10);
        }

    void run()
    {
        rclcpp::Rate rate(10);
        while(rclcpp::ok()){
            auto message = std::make_unique<std_msgs::msg::Float32>();
            float rad_angle = count / 10 * 3.14;

            message->data = std::sin(rad_angle);
            publisher_->publish(std::move(message));

            count++;
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