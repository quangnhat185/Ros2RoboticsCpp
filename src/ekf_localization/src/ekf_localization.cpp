#include <functional>
#include <memory>
#include <string>

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/pose.hpp"

using namespace Eigen;

class EKFPublisher : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;

    public:
        double pi = atan(1) * 4;

        EKFPublisher() : Node("ekf_publisher")
        {
        // double Q[4][4];
            publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pose", 10);

            Q_init();

        }

        void Q_init(){
            double x_var = 0.1, y_var = 0.1; // x/y-axis variance
            double vel_var = 1.0; // velocity variance        
            MatrixXd Q = MatrixXd(4,4);
            double pi = std::atan(1) * 4;
            double yaw_var = pi / 180; // theta variance
            Q.diagonal() << x_var, y_var, yaw_var, vel_var;
            std::cout << Q << std::endl;
        }



};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFPublisher>();
    // Do something

    rclcpp::shutdown();
    return 0;
    ;
}