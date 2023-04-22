
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"


using namespace Eigen;

class EKFPublisher : public rclcpp::Node
{
    private:
        // rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        Matrix2d GPS_NOISE = GPS_NOISE_INIT();
        Matrix2d INPUT_NOISE = INPUT_NOISE_INIT();
            // Covariance matrix of observation noise
        Matrix2d R = R_init();
            // Covariance matrix of process noise
        MatrixXd Q = Q_init();


    public:
        double pi = atan(1) * 4;
        float DT;

        // Input
        MatrixXd u;

        EKFPublisher(float dt, MatrixXd input) : Node("ekf_publisher")
        {
        // double Q[4][4];
            // publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("robot_localizing_pose", 10);
            publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_localizing_pose", 10);
            DT = dt;
            this->u = input;
        }

        MatrixXd Q_init(){
            double pi = std::atan(1) * 4;
            double x_var = 0.1, y_var = 0.1; // x/y-axis variance
            double yaw_var = pi / 180; // theta variance
            double vel_var = 1.0; // velocity variance

            MatrixXd Q = MatrixXd(4,4);
            Q.diagonal() << x_var, y_var, yaw_var, vel_var;

            Q = Q * Q; // variance to covariance

            return Q;
        }

        Matrix2d R_init(){
            Matrix2d R(2, 2);

            R << 1.0, 0.0, 
                0.0, 1.0 ;

            R = R * R; // variacne to covariance;
            return R;
        }

        Matrix2d INPUT_NOISE_INIT(){
            Matrix2d N = Matrix2d::Zero();
            N.diagonal() << 1.0, (30.0 / 180.0 * pi);
            N = N * N;
            return N;
        }

        Matrix2d GPS_NOISE_INIT(){
            Matrix2d G = Matrix2d::Zero();
            G.diagonal() << 0.5, 0.5;

            G = G * G;
            return G;
        }

        MatrixXd compute_jacob_h(){
            // Jacobian of observation model

            MatrixXd jacob_h(2, 4);
            jacob_h << 
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0;

            return jacob_h;
        }

        MatrixXd compute_jacob_f(MatrixXd &x, MatrixXd &u){
            // Jacobian of motion model
            MatrixXd jacob_f(4, 4);
            double v = u(0, 0);
            double theta = x(2, 0);

            jacob_f <<
                1.0, 0.0, -v*std::sin(theta) * DT, std::cos(theta) * DT,
                0.0, 1.0, v*std::cos(theta) * DT, std::sin(theta) * DT,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

            return jacob_f;
        }
        
        MatrixXd motion_model(MatrixXd  &x, MatrixXd &u){
            // give prediction of state x
            // given input u

            MatrixXd F = MatrixXd::Zero(4, 4);
            F.diagonal() << 1.0, 1.0, 1.0, 0.0;

            MatrixXd B = MatrixXd::Zero(4, 2);
            B << 
                DT * std::cos(x(2, 0)), 0.0,
                DT * std::sin(x(2, 0)), 0.0,
                0.0, DT,
                1.0, 0.0;

            x = F * x + B * u;
            return x;
        }

        MatrixXd observation_model(MatrixXd &x){
            // Return observation
            // knowing current state
            MatrixXd H(2, 4);
            H << 
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0;

            MatrixXd z = H * x;
            return z;
        }

        std::vector<MatrixXd> observation(MatrixXd &xTrue, MatrixXd &xd, MatrixXd &u){
            xTrue = motion_model(xTrue, u);

            // add gps noise
            MatrixXd z = observation_model(xTrue) + GPS_NOISE * MatrixXd::Random(2,1);

            // add noise to input
            MatrixXd ud = u + INPUT_NOISE * MatrixXd::Random(2, 1);

            xd = motion_model(xd, ud);

            return std::vector<MatrixXd>{xTrue, z, xd, ud};;
        }


        std::vector<MatrixXd> ekf_estimate(MatrixXd &xEst, MatrixXd &PEst, MatrixXd &z, MatrixXd &u){

            // Predict
            MatrixXd xPred = motion_model(xEst, u);
            MatrixXd jcob_f = compute_jacob_f(xEst, u);
            MatrixXd PPred = jcob_f * PEst * jcob_f.transpose() +  Q;

            // Update
            MatrixXd jacob_h = compute_jacob_h();
            MatrixXd zPred = observation_model(xPred);
            MatrixXd y = z - zPred; 

            MatrixXd S = jacob_h * PPred * jacob_h.transpose() + R;
            MatrixXd K = PPred * jacob_h.transpose() * S.inverse();

            xEst = xPred + K * y;
            MatrixXd KJh = K * jacob_h;
            MatrixXd I = KJh.Identity(KJh.rows(), KJh.cols());
            PEst = (I - KJh) * PPred;

            return std::vector<MatrixXd>{xEst, PEst};
        }

        void run()
        {   
            double time = 0;

            // Initilize state vectors [x y theta v]
            MatrixXd xEst = MatrixXd::Zero(4, 1);
            MatrixXd xTrue = MatrixXd::Zero(4, 1);
            MatrixXd PEst = MatrixXd::Identity(4, 4);
            MatrixXd xDR = MatrixXd::Zero(4 ,1);
            MatrixXd z;
            MatrixXd ud;

            rclcpp::Rate rate(1);
            while (rclcpp::ok())
            {
                // auto pose_msg_arr = std::make_unique<geometry_msgs::msg::PoseArray>();
                auto pose_msg_arr = std::make_unique<visualization_msgs::msg::MarkerArray>();
                // pose_msg_arr->header.stamp = this->get_clock()->now();


                // auto poseTrue = std::make_unique<geometry_msgs::msg::Pose>();
                // auto poseEst = std::make_unique<geometry_msgs::msg::Pose>();
                // auto poseDr = std::make_unique<geometry_msgs::msg::Pose>();

                auto markerposeTrue = std::make_unique<visualization_msgs::msg::Marker>();
                auto markerposeEst = std::make_unique<visualization_msgs::msg::Marker>();
                auto markerposeDr = std::make_unique<visualization_msgs::msg::Marker>();

                time += DT;

                // obtain overvation
                std::vector<MatrixXd> obs_vec = observation(xTrue, xDR, u);
                xTrue = obs_vec[0];
                z = obs_vec[1];
                xDR = obs_vec[2];
                ud = obs_vec[3];
                
                auto timestamp = this->get_clock()->now();
                markerposeTrue->header.frame_id = "map";
                markerposeTrue->header.stamp = timestamp;
                markerposeTrue->ns = "poseTrue";
                markerposeTrue->id = 0;
                markerposeTrue->type = visualization_msgs::msg::Marker::POINTS;
                markerposeTrue->action = visualization_msgs::msg::Marker::ADD;
                markerposeTrue->lifetime = rclcpp::Duration(0, 0);
                markerposeTrue->scale.x = 1;
                markerposeTrue->scale.y = 1;
                markerposeTrue->scale.z = 1;
                markerposeTrue->color.a = 1;
                markerposeTrue->color.r = 1;
                markerposeTrue->color.g = 0;
                markerposeTrue->color.b = 0;
                markerposeTrue->pose.position.x = xTrue(0, 0);
                markerposeTrue->pose.position.y = xTrue(1, 0);
                markerposeTrue->pose.orientation.z = xTrue(2, 0);


                markerposeDr->header.frame_id = "map";
                markerposeDr->header.stamp = timestamp;
                markerposeDr->ns = "poseDr";
                markerposeDr->id = 1;                
                markerposeDr->type = visualization_msgs::msg::Marker::POINTS;
                markerposeDr->action = visualization_msgs::msg::Marker::ADD;
                markerposeDr->lifetime = rclcpp::Duration(0, 0);
                markerposeDr->scale.x = 1;
                markerposeDr->scale.y = 1;
                markerposeDr->scale.z = 1;
                markerposeDr->color.a = 1;
                markerposeDr->color.r = 0;
                markerposeDr->color.g = 1;
                markerposeDr->color.b = 0;
                markerposeDr->pose.position.x = xDR(0, 0);
                markerposeDr->pose.position.y = xDR(1, 0);
                markerposeDr->pose.orientation.z = xDR(2, 0);                

                std::vector<MatrixXd> run_ekf_vec = ekf_estimate(xEst, PEst, z, ud);
                xEst = run_ekf_vec[0]; 
                PEst = run_ekf_vec[1];        

                markerposeEst->header.frame_id = "map";
                markerposeEst->header.stamp = timestamp;
                markerposeEst->ns = "poseEst";
                markerposeEst->id = 2;                
                markerposeEst->type = visualization_msgs::msg::Marker::POINTS;
                markerposeEst->action = visualization_msgs::msg::Marker::ADD;
                markerposeEst->lifetime = rclcpp::Duration(0, 0);
                markerposeEst->scale.x = 1;
                markerposeEst->scale.y = 1;
                markerposeEst->scale.z = 1;
                markerposeEst->color.a = 1;
                markerposeEst->color.r = 0;
                markerposeEst->color.g = 0;
                markerposeEst->color.b = 1;
                markerposeEst->pose.position.x = xEst(0, 0);
                markerposeEst->pose.position.y = xEst(1, 0);
                markerposeEst->pose.orientation.z = xEst(2, 0);   

                // pose_msg_arr->poses.push_back(*poseTrue);
                // pose_msg_arr->poses.push_back(*poseEst);
                // pose_msg_arr->poses.push_back(*poseDr);

                pose_msg_arr->markers.push_back(*markerposeTrue);
                pose_msg_arr->markers.push_back(*markerposeEst);
                pose_msg_arr->markers.push_back(*markerposeDr);
                       
                publisher_->publish(std::move(pose_msg_arr));
                rate.sleep();
            }

        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // Initialize input vectors
    MatrixXd u(2, 1);
    double trans_vel = 1.0;
    double yaw_rate = 0.1;
    u << trans_vel, yaw_rate;

    // Set time interval
    float delta_t = 0.1;

    auto node = std::make_shared<EKFPublisher>(delta_t, u);
    node->run();

    rclcpp::shutdown();
    return 0;
}