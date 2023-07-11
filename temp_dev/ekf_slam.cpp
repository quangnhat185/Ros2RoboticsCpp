#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>
#include <random>

using namespace Eigen;
using namespace std;

template<typename T>
void print_vec(vector<T> vec)
{   
    cout << "[";
    int i;
    for (i - 0; i < vec.size() - 1; i++)
    {
        cout << vec[i] << ",";
    }

    cout << vec[i] << "]" << endl;
}

class EKFSlam
{
private:
    float MAX_RANGE, M_DIST_TH;
    int STATE_SIZE, LM_SIZE;

    MatrixXd xTrue, xEst, PEst, xd, ud, rf_id, z;


public:
    float pi = atan(1) * 4;
    float DT = 0.1;
    MatrixXd u, Q_sim, R_sim, Cx;

    EKFSlam(float max_range, float m_dist_th, int state_size, int lm_size, MatrixXd &rf_id, MatrixXd& u):
        MAX_RANGE(max_range),
        M_DIST_TH(m_dist_th),
        STATE_SIZE(state_size),
        LM_SIZE(lm_size),
        rf_id(rf_id),
        u(u)

    {
        xTrue = MatrixXd::Zero(STATE_SIZE, 1);

        xEst = MatrixXd::Zero(STATE_SIZE, 1);
        PEst = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

        xd = MatrixXd::Zero(STATE_SIZE, 1);

        // EKF state covariance
        Cx = MatrixXd(3, 3);
        Cx.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(deg2rad(30), 2);
        // cout << Cx << endl;

        // Prediction noise
        Q_sim = MatrixXd(2, 2);
        Q_sim.diagonal() << pow(0.2, 2), pow(deg2rad(1.0), 2);

        // Observation noise
        R_sim = MatrixXd(2, 2);
        R_sim.diagonal() << pow(0.2, 2), pow(deg2rad(10.0), 2);

    }

    MatrixXd motion_model(MatrixXd  &x, MatrixXd &u){
        // give prediction of state x
        // given input us

        MatrixXd F = MatrixXd::Zero(3, 3);
        F.diagonal() << 1.0, 1.0, 1.0;

        MatrixXd B = MatrixXd::Zero(3, 2);
        B << 
            DT * std::cos(x(2, 0)), 0.0,
            DT * std::sin(x(2, 0)), 0.0,
            0.0, DT;

        x = F * x + B * u;
        return x;

    }

    vector<MatrixXd> observation(MatrixXd &xTrue, MatrixXd &xd, MatrixXd &u, MatrixXd &rf_id)
    {   
        xTrue = motion_model(xTrue, u);


        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);

        vector<vector<float>> z_vec;

        for (int i = 0; i < rf_id.rows(); i++)
        {
            float dx = rf_id(i, 0) - xTrue(0, 0);
            float dy = rf_id(i, 1) - xTrue(1, 0);
            float d = sqrt(pow(dx, 2) + pow(dx, 2));

            float angle = pi2pi(atan2(dy, dx) - xTrue(2,0));
            if (d <= MAX_RANGE)
            {   
                // add noise to gps
                float dn = d + dis(gen) * pow(Q_sim(0,0), 0.5);
                float angle_n = angle + dis(gen) * pow(Q_sim(1,1), 0.5);
                z_vec.push_back({dn, angle, (float)i});
            }
        }

        // add noise to input
        ud = MatrixXd(2,1);
        ud << 
            u(0, 0) + dis(gen) + pow(R_sim(0, 0), 0.5),
            u(1, 0) + dis(gen) + pow(R_sim(1, 1), 0.5);
        
        xd = motion_model(xd, ud);

        MatrixXd z = vec2mat(z_vec);

        return vector<MatrixXd>{xTrue, z, xd, ud};
    }

    // Calculate the number of landmarks currently tracked in the  state
    int calc_n_LM(MatrixXd x)
    {
        int n = (x.rows() - STATE_SIZE) / LM_SIZE;
        return n;
    }

    // Clculate the jacobian of motion model
    vector<MatrixXd> jacob_motion(MatrixXd x, MatrixXd u)
    {
        MatrixXd Fx = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

        float theta = x(2,0);

        MatrixXd jF = MatrixXd(3,3);
        jF <<
            0.0, 0.0, - DT * u(0,0) * std::sin(theta),
            0.0, 0.0, DT * u(0,0) * std::cos(theta),
            0.0, 0.0, 0.0;

        MatrixXd G = MatrixXd::Identity(STATE_SIZE, STATE_SIZE) + Fx.transpose() * jF * Fx;

        return {G, Fx};
    }

    MatrixXd calc_landmark_pos(MatrixXd x, MatrixXd zi)
    {   
        
        MatrixXd zp = MatrixXd(2, 1);
        zp << 
            x(0, 0) + zi(0) * std::cos(x(2,0) + zi(0,1)),
            x(1, 0) + zi(0) * std::sin(x(2,0) + zi(0,1));

        return zp;
    }

    MatrixXd get_landmark_position_from_state(MatrixXd x, int ind)
    {
        int start = STATE_SIZE + LM_SIZE * ind;
        int end = STATE_SIZE + LM_SIZE * (ind + 1);

        MatrixXd lm = x.block(start, 0, end-start, x.cols());

        return lm;
    }

    MatrixXd jacob_h(float q, MatrixXd delta, MatrixXd x, int i)
    {
        float sq = pow(q, 0.5);
        cout << "sq: " << sq << endl;
        MatrixXd G = MatrixXd(2, 5);
        G << 
            -sq * delta(0,0), -sq * delta(1, 0), 0, sq * delta(0,0), sq * delta(1, 0),
            delta(0, 0), -delta(0, 0), -q, -delta(0, 0), delta(0, 0);

        G = G / q;

        int nLM = calc_n_LM(x);
    
        MatrixXd F1_id = MatrixXd::Identity(3, 3);
        MatrixXd F1_zero = MatrixXd::Zero(3, 2 * nLM);
        MatrixXd F1 = MatrixXd(F1_id.rows(), F1_id.cols() + F1_zero.cols());
        F1 << F1_id, F1_zero;


        MatrixXd F2_p1 = MatrixXd::Zero(2, 3);
        MatrixXd F2_p2 = MatrixXd::Zero(2, 2 * (i - 1));
        MatrixXd F2_p3 = MatrixXd::Identity(2, 2);
        MatrixXd F2_p4 = MatrixXd::Zero(2, 2 * nLM - 2 * i);

        MatrixXd F2_p12;
        MatrixXd F2_p34;


        if (F2_p2.cols())
        {
            F2_p12 = MatrixXd(F2_p1.rows(), F2_p1.cols() + F2_p2.cols());
            F2_p12 << F2_p1, F2_p2;
        }
        else
        {
            F2_p12 = F2_p1;
        }

        if (F2_p4.cols())
        {
            F2_p34 = MatrixXd(F2_p3.rows(), F2_p3.cols() + F2_p4.cols());
            F2_p34 << F2_p3, F2_p4;
        }
        else
        {
            F2_p34 = F2_p3;
        }        
        
        MatrixXd F2 = MatrixXd(F2_p12.rows(), F2_p12.cols() + F2_p34.cols());
        F2 << F2_p12, F2_p34;

        MatrixXd F = MatrixXd(F1.rows() + F2.rows(), F1.cols());
        F << F1, F2;

        MatrixXd H = G * F;
        
        cout << G << endl;
        // cout << "#########################" << endl;
        // cout << F << endl;
        return H;
    }

    vector<MatrixXd> calc_innovation(MatrixXd &lm, MatrixXd &xEst, MatrixXd &PEst, MatrixXd &zi, int LMid)
    {   
        // MatrixXd pos = MatrixXd(lm.rows(), lm.cols());
        MatrixXd pos = xEst.block(0, 0, 2, xEst.cols());

        MatrixXd delta = lm - pos;
        
        // cout << "lm: " << lm << endl;
        float q = (delta.transpose() * delta)(0,0);

        float z_angle = atan2(delta(1, 0), delta(0,0)) - xEst(2,0);

        MatrixXd zp(1, 2);
        zp << sqrt(q), pi2pi(z_angle);
        MatrixXd y = (zi - zp).transpose();
        y(1) = pi2pi(y(1));

        MatrixXd H = jacob_h(q, delta, xEst, LMid + 1);

        MatrixXd S = H * PEst * H.transpose() + Cx.block(0, 0, 2, 2);

        return {y, S, H};
    }

    // Landmark association with Mahalanobis distance
    int search_correspond_landmark_id(MatrixXd xAug, MatrixXd PAug, MatrixXd zi)
    {
        int nLM = calc_n_LM(xAug);
        vector<float> min_dist;

        for (int i = 0; i < nLM; i++)
        {
            auto lm = get_landmark_position_from_state(xAug, i);
            auto calc_inno_res = calc_innovation(lm, xAug, PAug, zi, i);
            MatrixXd y = calc_inno_res[0];
            MatrixXd S = calc_inno_res[1];
            
            float dist =  (y.transpose() * S.inverse() * y)(0,0);
            min_dist.push_back(dist);
        }        

        min_dist.push_back(M_DIST_TH);
        auto min_iter = min_element(min_dist.begin(), min_dist.end());
        int min_idx = distance(min_dist.begin(), min_iter);

        return min_idx;
    }

    MatrixXd vec2mat(vector<vector<float>> z_vec)
    {   
        int n_rows = z_vec.size();
        int n_cols = z_vec[0].size();

        MatrixXd z = MatrixXd(n_rows, n_cols);

        for (int r = 0; r < n_rows; r++)
            for (int c = 0; c < n_cols; c++){
                z(r, c) = z_vec[r][c];
            }

        return z;
    }


    float pi2pi(float angle)
    {
        return fmod((angle + pi), (2 * pi)) - pi; 
    }

    float deg2rad(float deg_angle)
    {
        return deg_angle * pi / 180;
    }
    
    void ekf_slam(MatrixXd &xEst, MatrixXd &PEst, MatrixXd &ud, MatrixXd &z)
    {
        // Predict
        int S = STATE_SIZE;
        MatrixXd xEst_pose = xEst.block(0, 0, S, xEst.cols());
        auto res_jm = jacob_motion(xEst_pose, u);
        MatrixXd G = res_jm[0];
        MatrixXd Fx = res_jm[1];
        xEst_pose = motion_model(xEst_pose, u);

        MatrixXd PEst_pose = PEst.block(0, 0, S, S);
        PEst_pose = G.transpose() * PEst_pose * G + Fx.transpose() * Cx * Fx;

        MatrixXd initP = MatrixXd::Identity(LM_SIZE, LM_SIZE);
        
        // Update
        for (int iz = 0; iz < z.rows(); iz++)
        {   
            MatrixXd zi = z.block(iz, 0, 1, 2);
            int min_id = search_correspond_landmark_id(xEst, PEst, zi);
            int nLM = calc_n_LM(xEst);

            if (min_id == nLM)
            {
                cout << "New LM" << endl;
                MatrixXd lm_pos = calc_landmark_pos(xEst, zi);
                MatrixXd xAug = MatrixXd(xEst.rows() + 2, xEst.cols());
                
                MatrixXd PAug_new_col = MatrixXd(PEst.rows(), PEst.cols() + LM_SIZE);
                PAug_new_col << PEst, MatrixXd::Zero(xEst.rows(), LM_SIZE);

                MatrixXd PAug_new_row = MatrixXd({LM_SIZE, xEst.rows() + LM_SIZE});
                PAug_new_row << MatrixXd::Zero(LM_SIZE, xEst.rows()), initP; 

                MatrixXd PAug = MatrixXd(PAug_new_col.rows() + PAug_new_row.rows(), PAug_new_col.cols());
                PAug << PAug_new_col, PAug_new_row;

                xEst = xAug;
                PEst = PAug;
                
            MatrixXd lm = get_landmark_position_from_state(xEst, min_id);
            // cout << z.block(iz, 0, 1, 2) << endl;
            auto inno_vec = calc_innovation(lm, xEst, PEst, zi, min_id);
            MatrixXd y = inno_vec[0];
            MatrixXd S = inno_vec[1];
            MatrixXd H = inno_vec[2];

            MatrixXd K = (PEst * H.transpose()) * S.inverse();
            // cout << H << endl;
            xEst = xEst + (K * y);
            PEst = (MatrixXd::Identity(xEst.rows(), xEst.rows())) - (K * H) * PEst;


            // To-do
            // why first 3 elements = 0?

        
            }
        }

        
    }

    void run()
    {
        auto obs_res = observation(xTrue, xd, u, rf_id);
        auto xTrue = obs_res[0];
        auto z = obs_res[1];
        auto xd = obs_res[2];
        auto ud = obs_res[3];


        MatrixXd xEst = MatrixXd::Random(7,1);
        MatrixXd PEst = MatrixXd::Random(7,7);
        ekf_slam(xEst, PEst, ud, z);

        // auto jacob_res = jacob_motion(xTrue, ud);
        // cout << jacob_res[1] << endl;

        // MatrixXd zi = z.row(0);
        // calc_landmark_pos(xTrue, zi);

        // float q = 308.75216004271925;
        // MatrixXd delta = MatrixXd(2,1);
        // delta << 14.496, 9.9318;
        // MatrixXd xAug = MatrixXd::Zero(7,1);

        // jacob_h(q, delta, xAug, 2);

        // MatrixXd PAug = MatrixXd::Identity(9,9);
        
        // MatrixXd lm = MatrixXd(2, 1);
        // lm << 9.83286813, -1.76353153;

        // MatrixXd xAug = MatrixXd::Random(7, 1);
        // MatrixXd PAug = MatrixXd::Random(7, 7);
        // int LMid = 0;
        
        // MatrixXd zi(1,2);
        // zi << z(0, 0), z(0, 1);

        // calc_innovation(lm, xAug, PAug, zi, LMid);
        // search_correspond_landmark_id(xAug, PAug, zi);
    }

};

int main()
{   
    float max_range = 20.0;
    float m_dist_th = 2.0;
    int state_size = 3;
    int lm_size = 2;

    MatrixXd rf_id(4, 2);
    rf_id << 
            10.0, 0.0,
            15.0, 10.0,
            0.0, 15.0,
            -5.0, 25.0;

    MatrixXd u(2, 1);
    double trans_vel = 1.0;
    double yaw_rate = 0.1;
    u << trans_vel, yaw_rate;


    EKFSlam node(max_range, m_dist_th, state_size, lm_size, rf_id, u);
    node.run();
    return 0;
}

