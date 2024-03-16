#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random> 

using namespace Eigen;
using namespace std;


double deg2rad(double deg_angle);

const double pi = atan(1) * 4;  
const Matrix2d Q = (Matrix2d() << pow(3.0, 2), 0.0, 0.0, pow(deg2rad(10.0),2)).finished();
const Matrix2d R = (Matrix2d() << 1.0, 0.0, 0.0, pow(deg2rad(20.0), 2)).finished();
const Matrix2d Q_sim = (Matrix2d() << 0.09, 0.0, 0.0, pow(deg2rad(2.0), 2)).finished();
const Matrix2d R_sim = (Matrix2d() << 0.25, 0.0, 0.0, pow(deg2rad(10.0), 2)).finished();
const double OFFSET_YAW_RATE_NOISE = 0.01;

const double DT = 0.1;
const double MAX_RANGE = 20.0;
const double M_DIST_TH = 2.0;
const int STATE_SIZE = 3;
const int LM_SIZE = 2;
const int N_PARTICLE = 100;
const double NTH = N_PARTICLE / 1.5;

struct JacobMatrices
{
    MatrixXd zp, Hv, Hf, Sf;
};

struct CholeskyMatrices
{
    MatrixXd x, P;
};

class Particle
{
public:
    double w, x, y, yaw;
    MatrixXd lm;
    MatrixXd lmP;
    Particle(int n_landmark)
    {
        w = 1.0 / N_PARTICLE;
        x = 0.0;
        y = 0.0;
        yaw = 0.0;

        // landmark x-y position
        lm = MatrixXd::Zero(n_landmark, LM_SIZE);

        // landmark position covariance
        lmP = MatrixXd::Zero(n_landmark * LM_SIZE, LM_SIZE);

    }
};

double deg2rad(double deg_angle)
{
    double res = deg_angle * pi / 180;
    return res;
}

double pi_2_pi(double angle)
{   
    double mod_angle = fmod((abs(angle) + pi), (2 * pi)) - pi;

    if (angle >= 0) return mod_angle;
    else return -mod_angle;
}


class FastSLAM : public rclcpp::Node{

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_gt_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robotPose_gt_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robotPose_est_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robotPose_dr_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr particle_pub;

public:
    // rgb color vector
    vector<double> black = {0, 0, 0};
    vector<double> green = {0, 1, 0};
    vector<double> blue = {0, 0, 1};
    vector<double> red = {1, 0, 0};
    vector<double> yellow = {1, 1, 0};

    MatrixXd RFID;
    std::random_device rd;

    FastSLAM(MatrixXd RFID) : Node("fastSlam_node"), RFID(RFID)
    {
        // cout << RFID << endl;
        landmark_gt_pub = this->create_publisher<visualization_msgs::msg::Marker>("landmark_gt_pose", 10);
        robotPose_gt_pub = this->create_publisher<visualization_msgs::msg::Marker>("robot_gt_pose", 10);
        robotPose_est_pub = this->create_publisher<visualization_msgs::msg::Marker>("robot_est_pose", 10);
        robotPose_dr_pub = this->create_publisher<visualization_msgs::msg::Marker>("robot_dr_pose", 10);
        particle_pub = this->create_publisher<visualization_msgs::msg::Marker>("particle_pose", 10);


        
    };

    MatrixXd calc_input(double time)
    {   
        double v;
        double yaw_rate;
        if (time <= 3.0)
        {
            v = 0.0;
            yaw_rate = 0.0;
        }
        else
        {
            v = 1.0;
            yaw_rate = 0.1;
        }

        MatrixXd u(2,1);
        u << v, yaw_rate;

        // cout << u;
        return u;
    }

    MatrixXd motion_model(MatrixXd x, MatrixXd &u)
    {
        MatrixXd F = MatrixXd::Identity(3, 3);
        MatrixXd B(3, 2);
        B << 
            DT * cos(x(2,0)), 0.0,
            DT * sin(x(2,0)), 0.0,
            0.0, DT;

        x = F * x + B * u;

        x(2,0) = pi_2_pi(x(2,0));
                
        return x;
    }

    MatrixXd observation(MatrixXd &xTrue, MatrixXd &xd, MatrixXd &u)
    {
        std::mt19937 gen(rd());
        std::normal_distribution<> randn(0,1); // mean 0 and stddev 1

        // calc true state
        xTrue = motion_model(xTrue, u);

        // add noise to range observation
        MatrixXd z = MatrixXd::Zero(3, 0);

        for (int i=0; i < RFID.rows(); i++)
        {
            double dx = RFID(i, 0) - xTrue(0,0);
            double dy = RFID(i, 1) - xTrue(1,0);
            double d = sqrt(dx*dx + dy*dy);
            double angle = pi_2_pi(atan2(dy, dx) - xTrue(2,0));

            if (d <= MAX_RANGE)
            {
                // add noise range and angle observation
                double dn = d + randn(gen) * pow(Q_sim(0,0), 0.5); 
                double angle_with_noise = angle + randn(gen) * pow(Q_sim(1,1), 0.5);

                Vector3d zi(dn, pi_2_pi(angle_with_noise), i);
                z.conservativeResize(z.rows(), z.cols()+1);
                z.col(z.cols()-1) = zi;

                // cout << dn << endl;
                // cout << angle_with_noise << endl;
            }
            
        }

        // add noise to input
        MatrixXd ud = MatrixXd(2,1);
        
        u(0,0) += randn(gen) * pow(R_sim(0,0), 0.5);
        u(1,0) += randn(gen) * pow(R_sim(1,1), 0.5) + OFFSET_YAW_RATE_NOISE;

        // cout << R_sim << endl;
        xd = motion_model(xd,  u);

        // cout << "xd: " << xd.transpose() << endl;
        // cout << u << endl;
        // cout << z << endl;

        return z;
    }

    void predict_particles(vector<Particle> &particles, MatrixXd &u)
    {
        std::mt19937 gen(rd());
        std::normal_distribution<> randn(0,1); // mean 0 and stddev 1

        for (int i=0; i < N_PARTICLE; i++)
        {
            MatrixXd px = MatrixXd::Zero(STATE_SIZE, 1);
            px(0,0) = particles[i].x;
            px(1,0) = particles[i].y;
            px(2,0) = particles[i].yaw;
            MatrixXd ud(2,1);
            ud(0,0) =  u(0,0) + randn(gen) * pow(R(0,0), 0.5);
            ud(1,0) =  u(1,0) + randn(gen) * pow(R(1,1), 0.5) ;
            px = motion_model(px, ud);

            particles[i].x = px(0,0);
            particles[i].y = px(1,0);
            particles[i].yaw = px(2,0);

            // cout << particles[i].x << endl;
            // cout << "\n" << endl;
        }

    }

    void add_new_landmark(Particle &particle, Vector3d &zi, const Matrix2d &Q_cov)
    {
        double r = zi(0);
        double b = zi(1);
        int lm_id = int(zi(2));

        double s = sin(pi_2_pi(particle.yaw + b));
        double c = cos(pi_2_pi(particle.yaw + b));

        double dx = r * c;
        double dy = r * s;
        particle.lm(lm_id, 0) = particle.x + dx;
        particle.lm(lm_id, 1) = particle.y + dy;

        // covariance
        double d2 = dx * dx + dy * dy;
        double d = sqrt(d2);
        MatrixXd Gz(2,2);
        Gz <<
            dx / d, dy / d,
            -dy / d2, dx / d2;

        particle.lmP.block(2 * lm_id, 0, 2, particle.lmP.cols()) = (Gz.inverse() * Q_cov * Gz.transpose().inverse());

        // cout << particle.lmP << endl;
    }

    JacobMatrices compute_jacobians(Particle &particle, Vector2d &xf, MatrixXd &Pf, const MatrixXd &Q_cov)
    {
        // double dx = 

        JacobMatrices ans;
        double dx = xf(0) - particle.x; 
        double dy = xf(1) - particle.y;
        double d2 = dx*dx + dy*dy;
        double d = sqrt(d2);

        ans.zp = MatrixXd(2,1);
        ans.zp << d, pi_2_pi(atan2(dy, dx) - particle.yaw);

        ans.Hv = MatrixXd(2,3);
        ans.Hv << 
            -dx/d, -dy/d, 0.0,
            dy/d2, -dx/d2, -1.0;

        ans.Hf = MatrixXd(2,2);
        ans.Hf << 
            dx/d, dy/d,
            -dy/d2, dx/d2;
        
        ans.Sf = ans.Hf * Pf * ans.Hf.transpose() + Q_cov;

        return ans;
    }

    double compute_weight(Particle &particle, Vector3d &zi, const MatrixXd &Q_cov)
    {
        int lm_id = int(zi(2));
        Vector2d xf = particle.lm.row(lm_id);
        MatrixXd Pf = particle.lmP.block(2 * lm_id, 0, 2, particle.lmP.cols());

        JacobMatrices jm = compute_jacobians(particle, xf, Pf, Q_cov);

        MatrixXd dx(2,1);
        dx << 
        zi(0) - jm.zp(0), pi_2_pi(zi(1) - jm.zp(1));
        
        MatrixXd invS;
        try {
            invS = jm.Sf.inverse();
        } catch (const std::runtime_error& e) {
            std::cout << "singular" << std::endl;
            return 1.0;
        }

        double den = 2.0 * pi * sqrt(jm.Sf.determinant());
        double num = exp(-0.5*(dx.transpose() * invS * dx)(0,0));
        double w = num / den;
        return w;
    }

    CholeskyMatrices update_kf_with_cholesky(Vector2d &xf, MatrixXd &Pf, MatrixXd &v, const MatrixXd &Q_cov, MatrixXd Hf)
    {
        CholeskyMatrices ans;

        MatrixXd PHt = Pf * Hf.transpose();
        MatrixXd S = Hf * PHt + Q_cov;

        S = (S + S.transpose()) * 0.5;
        LLT<MatrixXd> lltofS = S.llt();
        MatrixXd s_chol = lltofS.matrixL();
        MatrixXd s_chol_inv = s_chol.inverse();
        MatrixXd W1 = PHt * s_chol_inv;
        MatrixXd W = W1 * s_chol_inv.transpose();

        ans.x = xf + W * v;
        ans.P = Pf - W1 * W1.transpose();

        return ans;

    }

    void update_landmark(Particle &particle, Vector3d &zi, const MatrixXd &Q_cov)
    {
        int lm_id = int(zi(2));

        Vector2d xf = particle.lm.row(lm_id);
        MatrixXd Pf = particle.lmP.block(2 * lm_id, 0, 2, particle.lmP.cols());

        JacobMatrices jm = compute_jacobians(particle, xf, Pf, Q);

        
        MatrixXd dz(2,1);
        dz << 
        zi(0) - jm.zp(0), pi_2_pi(zi(1) - jm.zp(1));

        CholeskyMatrices cm = update_kf_with_cholesky(xf, Pf, dz, Q_cov, jm.Hf);

        particle.lm.row(lm_id) = cm.x.transpose();
        particle.lmP.block(2 * lm_id, 0, 2, particle.lmP.cols()) = cm.P;

    }

    void update_with_observation(vector<Particle> &particles, MatrixXd &z)
    {
        for (int iz = 0; iz < z.cols(); iz++)
        {
            int landmark_id = int(z(2, iz));

            for (int ip = 0; ip < N_PARTICLE; ip++)
            {
                // if a landmark measurement <= 0.01 this mean it hasn't 
                // observed and added to current particle. Thus we need 
                // to add new landmark
                if (abs(particles[ip].lm(landmark_id, 0)) <= 0.01)
                {
                    Vector3d zi = z.col(iz);
                    add_new_landmark(particles[ip], zi, Q);
                }
                // update landmark if it is already known
                else{
                    Vector3d zi = z.col(iz);
                    double w = compute_weight(particles[ip], zi, Q);
                    particles[ip].w *= w;
                    update_landmark(particles[ip], zi, Q);
                }
            }
        }
    }

    void normalize_weight(vector<Particle> &particles) {
    
        double sum_w = std::accumulate(particles.begin(), particles.end(), 0.0, [](double sum, const Particle& p) {return sum + p.w;});

        if (sum_w != 0) {
            for (Particle& p : particles) {
                p.w /= sum_w;
            }
        } 
        else
        {
            double uniform_weight = 1.0 / particles.size();
            for (Particle& p : particles) {
                p.w = uniform_weight;
            }
        }

    }

    void resampling(vector<Particle>& particles)
    {
        // low variance re-sampling
        normalize_weight(particles);

        VectorXd pw(N_PARTICLE, 1);
        for (int i=0; i < N_PARTICLE; i++)
        {
            pw(i) = particles[i].w;
        }

        double n_eff = 1.0 / pw.dot(pw);  

        if (n_eff < NTH)
        {
            VectorXd w_cum(N_PARTICLE);
            w_cum(0) = pw(0);

            for(int i=1; i < N_PARTICLE; i++)
            {
                w_cum(i) = w_cum(i-1) + pw(i);
            }    

            VectorXd base = Eigen::VectorXd::LinSpaced(N_PARTICLE, 0, 1).array() - 1.0 / N_PARTICLE;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1.0 / N_PARTICLE);
            VectorXd resample_id = base.array() + dis(gen);

            VectorXi inds(N_PARTICLE);
        
            int ind = 0;

            for (int ip = 0; ip < N_PARTICLE; ip++)
            {
                while (ind < w_cum.size() - 1 && resample_id(ip) > w_cum(ind))
                {
                    ind++;
                }
                inds(ip) = ind;
            }

            vector<Particle> tmp_particles = particles;

            for (int i = 0; i< inds.size(); i++)
            {
                particles[i].x = tmp_particles[inds[i]].x;
                particles[i].y = tmp_particles[inds[i]].y;
                particles[i].yaw = tmp_particles[inds[i]].yaw;
                particles[i].lm = tmp_particles[inds[i]].lm;
                particles[i].lmP = tmp_particles[inds[i]].lmP;
                particles[i].w = 1.0 / N_PARTICLE;
            }
        }
    }

    MatrixXd calc_final_state(vector<Particle> particles)
    {
        MatrixXd xEst_final = MatrixXd::Zero(STATE_SIZE, 1); 

        normalize_weight(particles);

        for (int i = 0; i < N_PARTICLE; i++)
        {
            xEst_final(0,0) += particles[i].w * particles[i].x;
            xEst_final(1,0) += particles[i].w * particles[i].y;
            xEst_final(2,0) += particles[i].w * particles[i].yaw;
        }

        xEst_final(2,0) = pi_2_pi(xEst_final(2,0));

        return xEst_final;

    }


    void fast_slam1(vector<Particle> &particles, MatrixXd &u, MatrixXd &z)
    {
        predict_particles(particles, u);
        update_with_observation(particles, z);
        resampling(particles);
    }


    void setup_landmark_marker(
            MatrixXd& RFID,
            shared_ptr<visualization_msgs::msg::Marker> landmark_gt_marker,
            const vector<double>& color,
            double scale
    )
    {
        landmark_gt_marker->header.frame_id = "map";
        landmark_gt_marker->header.stamp = this->now();
        landmark_gt_marker->ns = "landmark_gt";
        landmark_gt_marker->id = 0;
        landmark_gt_marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
        landmark_gt_marker->action = visualization_msgs::msg::Marker::ADD;
        landmark_gt_marker->pose.orientation.w = 1.0;
        landmark_gt_marker->scale.x = scale;
        landmark_gt_marker->scale.y = scale;
        landmark_gt_marker->scale.z = scale;
        landmark_gt_marker->color.r = color[0];
        landmark_gt_marker->color.g = color[1];
        landmark_gt_marker->color.b = color[2];
        landmark_gt_marker->color.a = 1.0;

        for (int i = 0; i < RFID.rows(); i++)
        {
            geometry_msgs::msg::Point p;
            p.x = RFID(i, 0);
            p.y = RFID(i, 1);
            p.z = 0;
            landmark_gt_marker->points.push_back(p);
        }
    }

    void setup_trajectory_marker(
        vector<MatrixXd> &trajectory,
        shared_ptr<visualization_msgs::msg::Marker> trajectory_marker,
        const vector<double>& color,
        double scale,
        string node_name
    )
    {
        trajectory_marker->header.frame_id = "map";
        trajectory_marker->header.stamp = this->now();
        trajectory_marker->ns = node_name;
        trajectory_marker->id = 0;
        trajectory_marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
        trajectory_marker->action = visualization_msgs::msg::Marker::ADD;
        trajectory_marker->lifetime = rclcpp::Duration::from_seconds(0);
        trajectory_marker->pose.orientation.w = 1.0;
        trajectory_marker->scale.x = scale;
        trajectory_marker->scale.y = scale;
        trajectory_marker->scale.z = scale;
        trajectory_marker->color.r = color[0];
        trajectory_marker->color.g = color[1];
        trajectory_marker->color.b = color[2];
        trajectory_marker->color.a = 1.0;

        for (auto &v : trajectory)
        {
            geometry_msgs::msg::Point p;
            p.x = v(0, 0);
            p.y = v(1, 0);
            p.z = 0;
            trajectory_marker->points.push_back(p);
        }
    }

    void setup_particle_marker(
        vector<Particle> &particles,
        shared_ptr<visualization_msgs::msg::Marker> particle_marker,
        const vector<double>& color,
        double scale
    )
    {
        particle_marker->header.frame_id = "map";
        particle_marker->header.stamp = this->now();
        particle_marker->ns = "particles";
        particle_marker->id = 0;
        particle_marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
        particle_marker->action = visualization_msgs::msg::Marker::ADD;
        particle_marker->pose.orientation.w = 1.0;
        particle_marker->scale.x = scale;
        particle_marker->scale.y = scale;
        particle_marker->scale.z = scale;
        particle_marker->color.r = color[0];
        particle_marker->color.g = color[1];
        particle_marker->color.b = color[2];
        particle_marker->color.a = 1.0;

        std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
            return a.w > b.w;
        });
        
        for (int i = 0; i < 20; i++)
        {
            geometry_msgs::msg::Point p;
            p.x = particles[i].x;
            p.y = particles[i].y;
            p.z = 0;
            particle_marker->points.push_back(p);
        }

    }


    void run()
    {   
        rclcpp::Rate rate(100);

        int n_landmark = RFID.rows();

        //  state  vector [x y yaw v]
        MatrixXd xEst = MatrixXd::Zero(STATE_SIZE, 1); // SLAM estimation
        MatrixXd xTrue = MatrixXd::Zero(STATE_SIZE, 1); // True state
        MatrixXd xDR = MatrixXd::Zero(STATE_SIZE, 1); // Dead reckoning

        vector<Particle> particles;

        for (int i=0; i < N_PARTICLE; i++){
            Particle gen_part = Particle(n_landmark);
            particles.push_back(gen_part);
        }    
        
        double landmark_scale = 2;
        auto landmark_gt_marker = std::make_shared<visualization_msgs::msg::Marker>();
        setup_landmark_marker(RFID, landmark_gt_marker, black, landmark_scale);

        auto xTrue_trajectory_marker = std::make_shared<visualization_msgs::msg::Marker>();
        vector<MatrixXd> xTrue_trajectory;
    
        auto xEst_trajectory_marker = std::make_shared<visualization_msgs::msg::Marker>();
        vector<MatrixXd> xEst_trajectory;

        auto xDR_trajectory_marker = std::make_shared<visualization_msgs::msg::Marker>();

        auto particle_pose_marker = std::make_shared<visualization_msgs::msg::Marker>();

        vector<MatrixXd> xDR_trajectory;


        double timer = 0.0;
        while (rclcpp::ok())
        {
            timer += DT;
            MatrixXd u = this->calc_input(timer);
            MatrixXd z = this->observation(xTrue, xDR, u);
            xTrue_trajectory.push_back(xTrue);
            xDR_trajectory.push_back(xDR);

            this->fast_slam1(particles, u, z);

            xEst = this->calc_final_state(particles);
            // cout << "xEst: " << xEst.transpose() << endl;
            xEst_trajectory.push_back(xEst);

            // cout << "xDR: " << xDR.transpose() << endl;
            // particle pose
            setup_particle_marker(particles, particle_pose_marker, green, 0.5);
            // xTrue_trajectory
            setup_trajectory_marker(xTrue_trajectory, xTrue_trajectory_marker, blue, 0.5, "xTrue");
            // xEst_trajectory
            setup_trajectory_marker(xEst_trajectory, xEst_trajectory_marker, red, 0.5, "xEst"); 
            //xDR trajectory
            setup_trajectory_marker(xDR_trajectory, xDR_trajectory_marker, black, 0.5, "xDR");

            landmark_gt_pub->publish(std::move(*landmark_gt_marker));
            robotPose_gt_pub->publish(std::move(*xTrue_trajectory_marker));
            robotPose_est_pub->publish(std::move(*xEst_trajectory_marker));
            robotPose_dr_pub->publish(std::move(*xDR_trajectory_marker));
            particle_pub->publish(std::move(*particle_pose_marker));

            particle_pose_marker->points.clear();

            rate.sleep();

        }
    }

};


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    MatrixXd RFID(8, 2);
    RFID << 10.0, -2.0,
            15.0, 10.0,
            15.0, 15.0,
            10.0, 20.0,
            3.0, 15.0,
            -10.0, 20.0,
            -5.0, 5.0,
            -15.0, 15.0;

    FastSLAM solver = FastSLAM(RFID);
    solver.run();


    rclcpp::shutdown();
    return 0;   
}