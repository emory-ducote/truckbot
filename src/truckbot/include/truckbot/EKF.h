#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

using namespace Eigen;

class EKF {
    public:
        EKF(const double frequency=8,
            Vector3d position=Vector3d::Zero(), 
            Vector3d velocity=Vector3d::Zero(), 
            Quaterniond orientation=Quaterniond::Identity(),
            MatrixXd P=MatrixXd::Identity(10, 10) * 1e-3);
        ~EKF();
        void ekf_loop(Vector3d& acc_data, Vector3d& gyro_data, Vector3d& mag_data);
        
        Vector3d position;
        Vector3d velocity;
        Quaterniond orientation;
        MatrixXd P;
    private:
        Quaterniond expq(const Vector3d& omega);
        Matrix4d leftQuatMatrix(const Quaterniond& q);
        Matrix<double, 4, 3> leftQuatJacobian();
        Matrix<double, 3, 4> dRnb_by_dq(const Quaterniond& q, const Vector3d& a_body);
        MatrixXd compute_Ft(const Quaterniond& q, const Vector3d& a_body, const Vector3d& omega_body, double dt);
        MatrixXd compute_Gt(const Quaterniond& q, const Vector3d& omega, double dt);
        MatrixXd compute_Q(const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega);
        MatrixXd compute_H(Vector3d gravity_n, Vector3d mag_ref);
        MatrixXd compute_R(const Matrix3d& Sigma_a, const Matrix3d& Sigma_m);
        void predict(Vector3d& acc_data, Vector3d& gyro_data, double dt, 
            const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega);
        void update(Vector3d& acc_data, Vector3d& mag_data, const Vector3d& mag_ref,
            double dt, const Matrix3d& Sigma_a, const Matrix3d& Sigma_m);

    
        const double frequency;
        const double dt = 1.0f / frequency;
        const Matrix3d Sigma_omega = 7.5e-7f * Matrix3d::Identity();
        const Matrix3d Sigma_a     = 1.4e-4f * Matrix3d::Identity();
        const Matrix3d Sigma_m     = 2.25e-10f * Matrix3d::Identity();
        double declination_rad = 3.167 * M_PI / 180.0;
        Vector3d mag_ref;
};  


#endif