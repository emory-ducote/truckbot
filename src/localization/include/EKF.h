#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"
#include "LocalizationHelpers.h"


using namespace Eigen;

class EKF {
    public:
        EKF(const double frequency=10,
            Vector3d x=Vector3d::Zero(), 
            MatrixXd P=MatrixXd::Identity(3, 3) * 1e-3);
        ~EKF();
        
        const double frequency;
        Vector3d x;
        MatrixXd P;
        void ekf_loop(Vector3d& u_t_1, const float& v_t, const float& w_t);
    private:
        MatrixXd generate_Gt(const Vector3d& u_t_1, const float& v_t, const float& w_t, const double& dt);
        MatrixXd generate_Vt(const Vector3d& u_t_1, const float& v_t, const float& w_t, const double& dt);
        MatrixXd generate_Mt(const float& v_t, const float& w_t);
        MatrixXd compute_Q(const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega);
        MatrixXd compute_H(Vector3d gravity_n);
        MatrixXd compute_R(const Matrix3d& Sigma_a);
        void predict(const Vector3d& u_t_1, const float& v_t, const float& w_t, const double& dt);    
        const double dt = 1.0f / frequency;
        //TODO: tune these noises
        double alpha_1 = 0.01f;
        double alpha_2 = 0.01f;
        double alpha_3 = 0.01f;
        double alpha_4 = 0.01f;
};  


#endif