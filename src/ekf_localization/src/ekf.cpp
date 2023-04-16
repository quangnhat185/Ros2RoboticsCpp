#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iomanip>

using namespace Eigen;

class EKF{
    public:
        double pi = atan(1) * 4;
        float DT = 0.1; // time tick [s]
        Matrix2d GPS_NOISE = GPS_NOISE_INIT();
        Matrix2d INPUT_NOISE = INPUT_NOISE_INIT();
            // Covariance matrix of observation noise
        Matrix2d R = R_init();
            // Covariance matrix of process noise
        MatrixXd Q = Q_init();

        // Input
        MatrixXd u;

        EKF(float dt, MatrixXd input){ 
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
            // std::cout << Q << std::endl;

            return Q;
        }

        Matrix2d R_init(){
            Matrix2d R = Matrix2d::Identity();
            R = R * R; // variacne to covariance;
            // std::cout << R << std::endl;
            return R;
        }

        Matrix2d INPUT_NOISE_INIT(){
            Matrix2d N;
            N.diagonal() << 1.0, (30.0 / 180.0 * pi);
            N = N * N;
            return N;
        }

        Matrix2d GPS_NOISE_INIT(){
            Matrix2d G;
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

        MatrixXd compute_jacob_f(MatrixXd x, MatrixXd u){
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
        
        MatrixXd motion_model(MatrixXd  x, MatrixXd u){
            // give prediction of state x
            // given input u

            MatrixXd F(4, 4);
            F.diagonal() << 1.0, 1.0, 1.0, 0;

            MatrixXd B(4, 2);
            B << 
                DT * std::cos(x(2, 0)), 0.0,
                DT * std::sin(x(2, 0)), 0.0,
                0, DT,
                1.0, 0.0;

            x = F * x + B * u;

            return x;
        }

        MatrixXd observation_model(MatrixXd x){
            // Return observation
            // knowing current state

            MatrixXd H(2, 4);
            H << 
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0;

            MatrixXd z = H * x;

            // std::cout << Z << std::endl;
            return z;
        }

        std::vector<MatrixXd> observation(MatrixXd xTrue, MatrixXd xd, MatrixXd u){
            xTrue = motion_model(xTrue, u);

            // add gps noise
            MatrixXd z = observation_model(xTrue) + GPS_NOISE * MatrixXd::Random(2,1);

            // add noise to input
            MatrixXd ud = u + INPUT_NOISE * MatrixXd::Random(2,1);

            xd = motion_model(xd, ud);

            return std::vector<MatrixXd>{xTrue, z, xd, ud};;
        }


        std::vector<MatrixXd> run_ekf_estimation(MatrixXd xEst, MatrixXd PEst, MatrixXd z, MatrixXd u){

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
};



int main()
{   

    // Initialize input vectors
    MatrixXd u(2, 1);
    double trans_vel = 1.0;
    double yaw_rate = 0.1;
    u << trans_vel, yaw_rate;

    float delta_t = 0.1;
    
    EKF ekf(delta_t, u);

    float SIM_TIME = 50.0;
    double time = 0.0;

    // Initilize state vectors [x y theta v]
    MatrixXd xEst(4, 1);

    MatrixXd xTrue(4, 1);
    MatrixXd PEst = MatrixXd::Identity(4, 4);
    MatrixXd xDR(4 ,1);
    MatrixXd z = ekf.observation_model(xEst);
    MatrixXd ud(2,1);

    // History
    auto hxEst = xEst;
    auto hxtrue = xTrue;
    auto hxDR = xTrue;
    MatrixXd hz(2,1);

    while (time <= SIM_TIME)
    {
        time += delta_t;
        
        // obtain observation
        auto obs_vec =  ekf.observation(xTrue, xDR, u);
        xTrue = obs_vec[0];
        z = obs_vec[1];
        xDR = obs_vec[2];
        ud = obs_vec[3];

        auto run_ekf_vec = ekf.run_ekf_estimation(xTrue, PEst, z, u);
        xEst = run_ekf_vec[0]; 
        PEst = run_ekf_vec[1];

        std::cout << std::setprecision(3) << xTrue << " : " << xEst << std::endl;
        
    }
};