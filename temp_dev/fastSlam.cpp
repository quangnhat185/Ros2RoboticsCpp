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


class FastSLAM
{
public:
    MatrixXd RFID;
    std::random_device rd;

    FastSLAM(MatrixXd RFID) : RFID(RFID)
    {
        // cout << RFID << endl;
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

    MatrixXd motion_model(MatrixXd &x, MatrixXd &u)
    {
        MatrixXd F = MatrixXd::Zero(3, 3);
        MatrixXd B(3, 2);
        B << 
            DT * cos(x(2,0)), 0.0,
            DT * sin(x(2,0)), 0.0,
            0.0, DT;

        x = F * x + B * u;

        x(2,0) = pi_2_pi(x(2,0));


        // cout << x << endl;
        // cout << u << endl;
        
        return x;
    }

    MatrixXd observation(MatrixXd &xTrue, MatrixXd &xd, MatrixXd &u, MatrixXd &rfid)
    {

        std::mt19937 gen(rd());
        std::normal_distribution<> randn(0,1); // mean 0 and stddev 1

        // calc true state
        xTrue = motion_model(xTrue, u);

        // add noise to range observation
        MatrixXd z = MatrixXd::Zero(3, 0);

        for (int i=0; i < rfid.rows(); i++)
        {
            double dx = rfid(i, 0) - xTrue(0,0);
            double dy = rfid(i, 1) - xTrue(1,0);
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
        
        u(0,0) =  u(0,0) + randn(gen) * pow(R_sim(0,0), 0.5);
        u(1,0) =  u(1,0) + randn(gen) * pow(R_sim(1,1), 0.5) + OFFSET_YAW_RATE_NOISE;

        // cout << R_sim << endl;

        xd = motion_model(xd,  u);

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
            ud(0,0) =  u(0,0) + randn(gen) * pow(R_sim(0,0), 0.5);
            ud(1,0) =  u(1,0) + randn(gen) * pow(R_sim(1,1), 0.5) ;
            px = motion_model(px, ud);

            particles[i].x = px(0,0);
            particles[i].y = px(1,0);
            particles[i].yaw = px(2,0);
            // cout << particles[i].x << endl;

            // cout << px << endl;
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

        // cout << ans.Sf << endl;

        return ans;
    }

    double compute_weight(Particle &particle, Vector3d &zi, const MatrixXd &Q_cov)
    {
        int lm_id = int(zi(2));
        Vector2d xf = particle.lm.row(lm_id);
        MatrixXd Pf = particle.lmP.block(2 * lm_id, 0, 2, particle.lmP.cols());

        JacobMatrices jm = compute_jacobians(particle, xf, Pf, Q);

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
        // cout << w << endl;
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
                particles[i] = tmp_particles[i];
                particles[i].w = 1.0 / N_PARTICLE;
            }
        }
    }

    void calc_final_state(vector<Particle> particles, MatrixXd &xEst)
    {
        normalize_weight(particles);

        for (int i = 0; i < N_PARTICLE; i++)
        {
            xEst(0,0) += particles[i].w * particles[i].x;
            xEst(1,0) += particles[i].w * particles[i].y;
            xEst(2,0) += particles[i].w * particles[i].yaw;

        }

        xEst(2,0) = pi_2_pi(xEst(2,0));

    }

    void fast_slam1(vector<Particle> &particles, MatrixXd &u, MatrixXd &z)
    {
        predict_particles(particles, u);
        update_with_observation(particles, z);
        resampling(particles);
    }

};


int main()
{   
    // cout << NTH << endl;
    // int n_landmark = 50;
    // Particle ob1 = Particle(n_landmark);

    // RFID positions [x, y]
    MatrixXd RFID(8, 2);
    RFID << 10.0, -2.0,
            15.0, 10.0,
            15.0, 15.0,
            10.0, 20.0,
            3.0, 15.0,
            -5.0, 20.0,
            -5.0, 5.0,
            -10.0, 15.0;

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

    FastSLAM solver = FastSLAM(RFID);

    double time = 0.0;

    for (int i = 0; i < 200; i++)
    {
        time += DT;
        MatrixXd u = solver.calc_input(time);
        MatrixXd z = solver.observation(xTrue, xDR, u, RFID);
        solver.fast_slam1(particles, u, z);
        solver.calc_final_state(particles, xEst);

        // cout << u << endl;
        // cout << "=============" << endl;
    }

    cout << xEst << endl;

    // cout << u << endl;

    
    // cout << u << endl;

    // MatrixXd x = MatrixXd::Zero(3,1);
    // solver.calc_input(5);
    
    // solver.motion_model(x, u);
    // solver.observation(xTrue, xDR, u, RFID);

    
    // solver.predict_particles(particles{}, u);
    // MatrixXd z_trans(3,3);
    // z_trans << 
    // 9.89, -0.22, 0,
    // 17.97, 0.58, 1,
    // 14.78, 1.32, 4;
    // MatrixXd z = z_trans.transpose();

    // cout << z.row(0) << endl;
    // Vector3d zi = z.col(0);
    // MatrixXd xf = particles[0].lm.row(0);
    // cout << xf << endl;
    // cout << z << endl;
    // cout << zi << endl;
    // cout << zi(1) << endl;

    // int lm_id = 1;
    // MatrixXd Pf = particles[0].lmP.block(2 * lm_id, 0, 2, particles[0].lmP.cols());
    // cout << Pf << endl;
    // solver.add_new_landmark(particles.front(), zi, Q);

    // Particle particle = Particle(8);
    // particle.x = 0.19;
    // particle.y = 0.00248;
    // particle.yaw = 0.084;

    // Vector2d xf;
    // xf << 9.55, -1.019;
    // MatrixXd Pf(2,2);
    // Pf << 
    //     8.92, -0.67, 
    //     -0.67, 2.807;
    // MatrixXd Hf(2,2);
    //     Hf << 
    //     0.992, -0.1248, 
    //     0.0125, 0.0999;

    // solver.compute_jacobians(particle, xf, Pf, Q);
    // solver.compute_weight(particle, zi, Q);
    // solver.update_kf_with_cholesky();
    // solver.update_landmark();
    
    // solver.resampling(particles);

    return 0;   
}