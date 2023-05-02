#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include <random>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace Eigen;

class ParticleFilterPublisher : public rclcpp::Node
{
private:

    double pi = atan(1) * 4;
    // time interval
    float DT;

    // observation range
    float MAX_RANGE;

    // Estimation parameter of PF
    MatrixXd Q; // range error
    MatrixXd R; // input error


    // PF parameter
    int NP = 100;
    double Nth = 50.0;

    MatrixXd px; // particle states
    MatrixXd pw; // particle weights

    // Lardmark locations identifed by 
    // Radio Frequency
    MatrixXd rf_id;

    // initialize robot pose publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

    // intialize particle pose publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr particle_publisher;
     
public:
    // input
    MatrixXd u;
    
    ParticleFilterPublisher(float &dt, float &max_range, int &num_par, MatrixXd &rf_id, MatrixXd u) :
        Node("pf_publisher"),
        DT(dt), 
        MAX_RANGE(max_range),
        NP(num_par),
        rf_id(rf_id),
        u(u)
    {

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_localizing_pose", 10);
        particle_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("particle_pose", 10);

        Q_init();
        R_init();

        px = MatrixXd::Zero(4, NP);
        pw = MatrixXd::Zero(1, NP);
        pw.fill(1.0 / NP);

    }

    void Q_init()
    {   
        Q = MatrixXd(1, 1);
        Q.diagonal() << pow(0.2, 2);
    }

    void R_init()
    {
        R = MatrixXd(2, 2);
        R.diagonal() << pow(1.0, 2), pow(40.0 / 180 * pi, 2);
    }

    MatrixXd motion_model(MatrixXd &x, MatrixXd &u)
    {
        
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

    double gauss_likelihood(float input, float sigma)
    {
        double denorminator = sqrt(2.0 * pi) * sigma;
        double norminator = exp(-pow(input, 2) / (2 * pow(sigma, 2)));

        return norminator / denorminator;
    }

    std::vector<MatrixXd> observation(MatrixXd &xTrue, MatrixXd &xDr, MatrixXd &u, MatrixXd &rf_id)
    {
        xTrue = motion_model(xTrue, u);

        // add noise to gps
        std::vector<double> dnoise_vec;
        std::vector<double> dx_vec;
        std::vector<double> dy_vec;

        for (int i = 0; i < rf_id.rows(); i++){
            
            auto dx = xTrue(0, 0) - rf_id(i, 0);
            auto dy = xTrue(1, 0) - rf_id(i, 1);
            auto d = sqrt(pow(dx, 2) + pow(dy, 2));

            if (d <= MAX_RANGE)
            {
                double dnoise = d + VectorXd::Random(1)(0) * pow(Q.value(), 0.5); 
                dnoise_vec.push_back(dnoise);
                dx_vec.push_back(rf_id(i, 0));
                dy_vec.push_back(rf_id(i, 1));
            }
        }

        MatrixXd z(dnoise_vec.size(), 3);
        for (size_t i = 0; i < dnoise_vec.size(); i++){
            z(i, 0) = dnoise_vec[i];
            z(i, 1) = dx_vec[i];
            z(i, 2) = dy_vec[i];
        }

        // // add noise to input

        MatrixXd ud(2, 1);
        ud <<
            u(0, 0) + VectorXd::Random(1)(0) * pow(R(0, 0), 0.5),
            u(1, 0) + VectorXd::Random(1)(0) * pow(R(1, 1), 0.5);

        xDr = motion_model(xDr, ud);

        return std::vector<MatrixXd>{xTrue, z, xDr, ud};
    }

    // MatrixXd calc_covariance(MatrixXd &xEst, MatrixXd &px, MatrixXd &pw){
    //     MatrixXd cov = MatrixXd::Zero(3, 3);
        
    //     int n_particle = px.cols();

    //     for (int i = 0 ; i < n_particle; i++)
    //     {
    //         // MatrixXd dx = px.block(0, i, 3, i + 1) - xEst;
    //         MatrixXd dx = px(Eigen::all, i) - xEst;
    //         dx = dx.block(0, 0, 3, 1); // slice block [:3, :1]
    //         cov += pw(0, i)  * dx * dx.transpose();
    //     }

    //     MatrixXd I = MatrixXd::Identity(pw.rows(), pw.cols());

    //     cov *= 1.0 / (1.0 - (pw * pw.transpose()).value());

    //     return cov;
    // }

    // low variance re-rsampling
    std::vector<MatrixXd> re_sampling(MatrixXd &px, MatrixXd &pw)
    {

        int num_particle = pw.cols();
        MatrixXd w_cum = MatrixXd::Zero(num_particle, 1);

        // compute culumative weight
        for (int i = 0; i < num_particle; i++)
        {
            w_cum(i, 0) = pw.block(0, 0, 1, i).sum();
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0.0, 1.0/NP);
        
        VectorXd base(NP);
        VectorXd re_sample_id(NP);

        for (int i = 0; i < NP; i++)
        {
            base(i) = i * 1.0 / NP;
            re_sample_id(i) = base(i) + dis(gen);
        }

        int ind = 0;
        std::vector<int> indexes;
        for (int ip = 0; ip < NP; ip++)
        {
            while (re_sample_id(ip) > w_cum(ind, 0) && ind < NP - 1) ind++;
            indexes.push_back(ind);
        }

        px = px(Eigen::all, indexes); // resampling by new indexes
        pw.fill(1.0 / NP); // init new weights

        return std::vector<MatrixXd>{px, pw};
    }


    std::vector<MatrixXd> pf_localization(MatrixXd& px, MatrixXd &pw, MatrixXd &z, MatrixXd &u)
    {
        for (int i = 0; i < NP; i++)
        {
            MatrixXd x = px(Eigen::all, i);
            double w = pw(0, i);

            MatrixXd ud(2, 1);

            // predict with noised input
            ud <<
                u(0, 0) + VectorXd::Random(1)(0) * pow(R(0, 0), 0.5),
                u(1, 0) + VectorXd::Random(1)(0) * pow(R(1, 1), 0.5);
            x = motion_model(x, ud);

            // Calculate Importance weight
            for (int j = 0; j < z.rows(); j++)
            {
                double dx = x(0, 0) - z(j, 1);
                double dy = x(1, 0) - z(j, 2);
                double pre_z = sqrt(pow(dx, 2) + pow(dy, 2));
                double dz = pre_z - z(j, 0);
                w = w * gauss_likelihood(dz, pow(Q(0, 0), 0.5));        
            }

            // update state particles after applying motion
            px(Eigen::all, i) = x(Eigen::all, 0);

            // update weight particles after reweighted
            // with observation
            pw(0, i) = w;
        }

        pw = pw / pw.sum(); // normalize    

        MatrixXd xEst = px * pw.transpose();
        // MatrixXd PEst = calc_covariance(xEst, px, pw);

        double N_eff = 1.0 / (pw * pw.transpose()).value();
        if (N_eff < Nth) 
        {
            auto p_vec = re_sampling(px, pw);
            MatrixXd px = p_vec[0];
            MatrixXd py = p_vec[1]; 
        }   

        return {xEst, px, pw};
    }
    
    // publish poses of given landmarks
    void setup_rfid_msg(
        std::unique_ptr<visualization_msgs::msg::MarkerArray> &pose_msg_arr,
        MatrixXd &rfid, 
        float scale,
        int id)
    {
        
        int num_obtacles = rfid.rows();
        for (int i = 0; i < num_obtacles; i++)
        {   
            auto markerposerRfid = std::make_unique<visualization_msgs::msg::Marker>();
            auto timestamp = this->get_clock()->now();
            markerposerRfid->header.frame_id = "map";
            markerposerRfid->header.stamp = timestamp;
            markerposerRfid->ns = "poseRfid" + std::to_string(id);
            markerposerRfid->id = i;
            markerposerRfid->type = visualization_msgs::msg::Marker::CYLINDER;
            markerposerRfid->action = visualization_msgs::msg::Marker::ADD;
            markerposerRfid->lifetime = rclcpp::Duration::from_seconds(0);
            markerposerRfid->scale.x = scale;
            markerposerRfid->scale.y = scale;
            markerposerRfid->scale.z = scale;
            markerposerRfid->color.a = 1;
            markerposerRfid->color.r = 0.7;
            markerposerRfid->color.g = 0.46;
            markerposerRfid->color.b = 0;
            markerposerRfid->pose.position.x = rf_id(i, 0);
            markerposerRfid->pose.position.y = rf_id(i, 1);
            markerposerRfid->pose.orientation.z = 0.0;        

            pose_msg_arr->markers.push_back(*markerposerRfid);
        }
    }

    // publish poses of particles per iteration
    void setup_paricles_msg(
        std::unique_ptr<visualization_msgs::msg::MarkerArray> &pose_particle_arr,
        MatrixXd &px, 
        float scale)
    {
        
        auto markerposeParticles = std::make_unique<visualization_msgs::msg::Marker>();
        int num_particles = px.cols();

        for (int i = 0; i < num_particles; i++)
        {           
            auto timestamp = this->get_clock()->now();
            markerposeParticles->header.frame_id = "map";
            markerposeParticles->header.stamp = timestamp;
            markerposeParticles->ns = "poseParticle";
            markerposeParticles->id = i;
            markerposeParticles->type = visualization_msgs::msg::Marker::SPHERE;
            markerposeParticles->action = visualization_msgs::msg::Marker::ADD;
            markerposeParticles->scale.x = scale;
            markerposeParticles->scale.y = scale;
            markerposeParticles->scale.z = scale;
            markerposeParticles->color.a = 1;
            markerposeParticles->color.r = 0;
            markerposeParticles->color.g = 1;
            markerposeParticles->color.b = 0;
            markerposeParticles->pose.position.x = px(0, i);
            markerposeParticles->pose.position.y = px(1, i);
            markerposeParticles->pose.orientation.z = 0.0;        

            pose_particle_arr->markers.push_back(*markerposeParticles);
        }
    }    

    void run()
    {   
        MatrixXd xEst = MatrixXd::Zero(4, 1);
        MatrixXd xTrue = MatrixXd::Zero(4, 1);

        MatrixXd xDr = MatrixXd::Zero(4, 1);

        // MatrixXd z, ud, PEst;
        MatrixXd z, ud;
        
        rclcpp::Rate rate(50);

        //publish marker msg for visualization with Rviz
        auto pose_msg_arr = std::make_unique<visualization_msgs::msg::MarkerArray>();

        int counter = 0;
        float scale_rfid = 1.5;
        float scale_particles = 1;
        float scale = 0.5;

        while (rclcpp::ok())
        {   
            auto pose_particle_arr = std::make_unique<visualization_msgs::msg::MarkerArray>();

            auto markerposeTrue = std::make_unique<visualization_msgs::msg::Marker>();
            auto markerposeEst = std::make_unique<visualization_msgs::msg::Marker>();
            auto markerposeDr = std::make_unique<visualization_msgs::msg::Marker>();

            auto obs_vec = observation(xTrue, xDr, u, rf_id);
            xTrue = obs_vec[0];
            z = obs_vec[1];
            xDr = obs_vec[2];
            ud = obs_vec[3];

            setup_rfid_msg(pose_msg_arr, rf_id, scale_rfid, counter);

            // config GT pose message
            auto timestamp = this->get_clock()->now();
            markerposeTrue->header.frame_id = "map";
            markerposeTrue->header.stamp = timestamp;
            markerposeTrue->ns = "poseTrue";
            markerposeTrue->id = counter;
            markerposeTrue->type = visualization_msgs::msg::Marker::SPHERE;
            markerposeTrue->action = visualization_msgs::msg::Marker::ADD;
            markerposeTrue->lifetime = rclcpp::Duration::from_seconds(0);
            markerposeTrue->scale.x = scale;
            markerposeTrue->scale.y = scale;
            markerposeTrue->scale.z = scale;
            markerposeTrue->color.a = 0.5;
            markerposeTrue->color.r = 0;
            markerposeTrue->color.g = 0;
            markerposeTrue->color.b = 1;
            markerposeTrue->pose.position.x = xTrue(0, 0);
            markerposeTrue->pose.position.y = xTrue(1, 0);
            markerposeTrue->pose.orientation.z = xTrue(2, 0);

            // config Dead Reckoning pose message
            markerposeDr->header.frame_id = "map";
            markerposeDr->header.stamp = timestamp;
            markerposeDr->ns = "poseDr";
            markerposeDr->id = counter;                
            markerposeDr->type = visualization_msgs::msg::Marker::SPHERE;
            markerposeDr->action = visualization_msgs::msg::Marker::ADD;
            markerposeDr->lifetime = rclcpp::Duration::from_seconds(0);
            markerposeDr->scale.x = scale;
            markerposeDr->scale.y = scale;
            markerposeDr->scale.z = scale;
            markerposeDr->color.a = 1;
            markerposeDr->color.r = 0;
            markerposeDr->color.g = 0;
            markerposeDr->color.b = 0;
            markerposeDr->pose.position.x = xDr(0, 0);
            markerposeDr->pose.position.y = xDr(1, 0);
            markerposeDr->pose.orientation.z = xDr(2, 0);                 

            auto pf_localization_vec = pf_localization(px, pw, z, ud);
            xEst = pf_localization_vec[0];
            px = pf_localization_vec[1];
            pw = pf_localization_vec[2];

            markerposeEst->header.frame_id = "map";
            markerposeEst->header.stamp = timestamp;
            markerposeEst->ns = "poseEst";
            markerposeEst->id = counter;                
            markerposeEst->type = visualization_msgs::msg::Marker::SPHERE;
            markerposeEst->action = visualization_msgs::msg::Marker::ADD;
            markerposeEst->lifetime = rclcpp::Duration::from_seconds(0);
            markerposeEst->scale.x = scale;
            markerposeEst->scale.y = scale;
            markerposeEst->scale.z = scale;
            markerposeEst->color.a = 1;
            markerposeEst->color.r = 1;
            markerposeEst->color.g = 0;
            markerposeEst->color.b = 0;
            markerposeEst->pose.position.x = xEst(0, 0);
            markerposeEst->pose.position.y = xEst(1, 0);
            markerposeEst->pose.orientation.z = xEst(2, 0);               

            pose_msg_arr->markers.push_back(*markerposeTrue);
            pose_msg_arr->markers.push_back(*markerposeEst);
            pose_msg_arr->markers.push_back(*markerposeDr);

            setup_paricles_msg(pose_particle_arr, px, scale_particles);
            publisher_->publish(*pose_msg_arr);
            particle_publisher->publish(*pose_particle_arr);

            counter++;
            rate.sleep();
        }

    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    float max_range = 20.0;
    float dt = 0.1;
    int num_par = 200;

    MatrixXd rf_id(4, 2);
    rf_id << 
            10.0, 0.0,
            10.0, 10.0,
            0.0, 15.0,
            -5.0, 20.0;

    MatrixXd u(2, 1);
    double trans_vel = 1.0;
    double yaw_rate = 0.1;
    u << trans_vel, yaw_rate;

    auto node = std::make_shared<ParticleFilterPublisher>(dt, max_range, num_par, rf_id, u);
    node->run();

    rclcpp::shutdown();
    return 0;
}