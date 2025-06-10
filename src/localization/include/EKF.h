#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

using namespace Eigen;

class EKF {
    public:
        EKF(const double frequency=100,
            Vector3d position=Vector3d::Zero(), 
            Vector3d velocity=Vector3d::Zero(), 
            Quaterniond orientation=Quaterniond::Identity(),
            MatrixXd P=MatrixXd::Identity(10, 10) * 1e-3);
        ~EKF();
        void ekf_loop(Vector3d& acc_data, Vector3d& gyro_data);
        
        Vector3d position;
        Vector3d velocity;
        Quaterniond orientation;
        MatrixXd P;
    private:
        Quaterniond expq(const Vector3d& omega);
        Matrix4d leftQuatMatrix(const Quaterniond& q);
        Matrix4d rightQuatMatrix(const Quaterniond& q);
        Matrix<double, 4, 3> leftQuatJacobian();
        Matrix<double, 3, 4> dRnb_by_dq(const Quaterniond& q, const Vector3d& a_body);
        MatrixXd compute_Ft(const Quaterniond& q, const Vector3d& a_body, const Vector3d& omega_body, double dt);
        MatrixXd compute_Gt(const Quaterniond& q, const Vector3d& omega, double dt);
        MatrixXd compute_Q(const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega);
        MatrixXd compute_H(Vector3d gravity_n);
        MatrixXd compute_R(const Matrix3d& Sigma_a);
        void predict(Vector3d& acc_data, Vector3d& gyro_data, double dt, 
            const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega);
        void update(Vector3d& acc_data, double dt, const Matrix3d& Sigma_a);

    
        const double frequency;
        const double dt = 1.0f / frequency;
        const Matrix3d Sigma_omega = 0.1f * Matrix3d::Identity();
        const Matrix3d Sigma_a     = 0.5 * Matrix3d::Identity();
};  


#endif