#include <vector>
#include <iostream>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"
#include "truckbot/EKF.h"

using namespace Eigen;
using namespace std;

template <typename T>
struct fmt::formatter<T, char, std::enable_if_t<
    std::is_base_of_v<Eigen::EigenBase<T>, T>>> {
    
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const T& mat, FormatContext& ctx) {
        std::ostringstream oss;
        oss << mat;
        return fmt::format_to(ctx.out(), "{}", oss.str());
    }
};

EKF::EKF(const double frequency, Vector3d position, Vector3d velocity, Quaterniond orientation, MatrixXd P) :
            frequency(frequency), position(position), velocity(velocity), orientation(orientation), P(P) {
        spdlog::set_level(spdlog::level::info);
            }

EKF::~EKF() {}

Quaterniond EKF::expq(const Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-6) return Quaterniond(1, 0.5 * omega(0), 0.5 * omega(1), 0.5 * omega(2));
    Vector3d axis = omega / theta;
    return Quaterniond(AngleAxisd(theta, axis));
}

Matrix4d EKF::leftQuatMatrix(const Quaterniond& q) {
    Matrix4d Q;
    Q << q.w(), -q.x(), -q.y(), -q.z(),
         q.x(),  q.w(), -q.z(),  q.y(),
         q.y(),  q.z(),  q.w(), -q.x(),
         q.z(), -q.y(),  q.x(),  q.w();
    return Q;
}

Matrix4d EKF::rightQuatMatrix(const Quaterniond& q) {
    Matrix4d Q;
    Q << q.w(), -q.x(), -q.y(), -q.z(),
         q.x(),  q.w(),  q.z(), -q.y(),
         q.y(), -q.z(),  q.w(),  q.x(),
         q.z(),  q.y(), -q.x(),  q.w();
    return Q;
}

Matrix<double, 4, 3> EKF::leftQuatJacobian() {
    Matrix<double, 4, 3> J;
    J << 0, 0, 0,
         1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return J;
}

Matrix<double, 3, 4> EKF::dRnb_by_dq(const Quaterniond& q, const Vector3d& a_body) {
    double eps = 1e-5;
    Matrix<double, 3, 4> J;
    Vector4d qv(q.w(), q.x(), q.y(), q.z());
    for (int i = 0; i < 4; ++i) {
        Vector4d dq = Vector4d::Zero();
        dq(i) = eps;
        Quaterniond q_perturbed(qv + dq);
        q_perturbed.normalize();
        Vector3d Ra_plus = q_perturbed.toRotationMatrix() * a_body;
        dq(i) = -eps;
        Quaterniond q_perturbed_neg(qv + dq);
        q_perturbed_neg.normalize();
        Vector3d Ra_minus = q_perturbed_neg.toRotationMatrix() * a_body;
        J.col(i) = (Ra_plus - Ra_minus) / (2 * eps);
    }
    return J;
}

MatrixXd EKF::compute_Ft(const Quaterniond& q, const Vector3d& a_body, const Vector3d& omega_body, double dt) {
    MatrixXd Ft = MatrixXd::Zero(10, 10);
    Matrix3d I3 = Matrix3d::Identity();
    Matrix<double, 3, 4> dR_dq = dRnb_by_dq(q, a_body);

    Ft.block<3,3>(0,0) = I3;
    Ft.block<3,3>(0,3) = dt * I3;
    Ft.block<3,4>(0,6) = 0.5 * dt * dt * dR_dq;

    Ft.block<3,3>(3,3) = I3;
    Ft.block<3,4>(3,6) = dt * dR_dq;

    Quaterniond dq = expq(0.5 * dt * omega_body);
    Ft.block<4,4>(6,6) = rightQuatMatrix(dq);

    spdlog::debug("Ft matrix:\n {}\n", Ft);
    return Ft;
}

MatrixXd EKF::compute_Gt(const Quaterniond& q, const Vector3d& omega, double dt) {
    MatrixXd Gt = MatrixXd::Zero(10, 9);
    Gt.block<6,6>(0,0) = Matrix<double,6,6>::Identity();
    Matrix<double, 4, 3> Jexp = leftQuatJacobian();
    Matrix4d Lq = leftQuatMatrix(q);
    Gt.block<4,3>(6,6) = -0.5 * dt * Lq * Jexp;

    spdlog::debug("Gt matrix:\n {}\n", Gt);
    return Gt;
}

MatrixXd EKF::compute_Q(const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega) {
    MatrixXd Q = MatrixXd::Zero(9, 9);
    Q.block<3,3>(0,0) = Sigma_a;
    Q.block<3,3>(3,3) = Sigma_a;
    Q.block<3,3>(6,6) = Sigma_omega;
    return Q;
}

MatrixXd EKF::compute_H(Vector3d gravity_n) {
    MatrixXd H = MatrixXd::Zero(3, 10);
    Matrix<double, 3, 4> H_acc = -dRnb_by_dq(this->orientation, gravity_n);
    H.block<3,4>(0,6) = H_acc;
    return H;
}

MatrixXd EKF::compute_R(const Matrix3d& Sigma_a) {
    MatrixXd R = MatrixXd::Zero(3, 3);
    R.block<3,3>(0,0) = Sigma_a;
    return R;
}

void EKF::predict(Vector3d& acc_data, Vector3d& gyro_data, double dt, 
                  const Matrix3d& Sigma_a, const Matrix3d& Sigma_omega) {
    MatrixXd Q = compute_Q(Sigma_a, Sigma_omega);
    
    MatrixXd Ft = compute_Ft(this->orientation, acc_data, gyro_data, dt);
    MatrixXd Gt = compute_Gt(this->orientation, gyro_data, dt);

    this->position += this->velocity * dt + 0.5 * dt * dt * (this->orientation * acc_data);
    this->velocity += dt * (this->orientation * acc_data);
    this->orientation = this->orientation * expq(0.5 * dt * gyro_data);
    this->orientation.normalize();

    this->P = Ft * this->P * Ft.transpose() + Gt * Q * Gt.transpose();

    spdlog::debug("Predicted position: {}\n", this->position.transpose());
    spdlog::debug("Predicted velocity: {}\n", this->velocity.transpose());
    spdlog::debug("Predicted orientation (quaternion): {}\n", this->orientation.coeffs().transpose());
    spdlog::debug("Predicted covariance P: \n {}\n", this->P);
}

void EKF::update(Vector3d& acc_data, double dt, const Matrix3d& Sigma_a) {

    Vector3d gravity_n(0, 0, 0);                    
    VectorXd y(3);
    y = -acc_data;
    
    MatrixXd H = compute_H(gravity_n);

    VectorXd y_pred(3);
    Matrix3d Rnb = this->orientation.toRotationMatrix();
    y_pred = Rnb.transpose() * gravity_n;
    
    VectorXd e = y - y_pred;
    
    spdlog::debug("Combined measurement y: {}\n", y.transpose());
    spdlog::debug("Prediction y_pred: {}\n", y_pred.transpose());
    spdlog::debug("Measurement error e: {}\n", e.transpose());
    
    MatrixXd R = compute_R(Sigma_a);
    
    MatrixXd S = H * this->P * H.transpose() + R;
    MatrixXd K = this->P * H.transpose() * S.inverse();
    
    spdlog::debug("Kalman gain K:\n {}\n", K);
    
    VectorXd dx = K * e;
    this->position += dx.segment<3>(0);
    this->velocity += dx.segment<3>(3);
    Vector4d dq = dx.segment<4>(6);
    Quaterniond dq_quat(1, 0.5 * dq(0), 0.5 * dq(1), 0.5 * dq(2));
    this->orientation = (this->orientation * dq_quat).normalized();
    this->P = (MatrixXd::Identity(10, 10) - K * H) * this->P;

    // Quaternion norm and squared norm
    Vector4d q_vec = dq_quat.coeffs();
    double norm_q = q_vec.norm();
    double norm_q_sq = norm_q * norm_q;

    // Identity matrix
    Matrix4d I = Matrix4d::Identity();

    // Outer product: q̃ * q̃ᵀ
    Matrix4d outer = q_vec * q_vec.transpose();

    // Compute Jacobian Jt
    Matrix4d P_quat_tilde = this->P.block<4, 4>(6, 6);

    Matrix4d Jt = (1.0 / norm_q) * (I - outer / norm_q_sq);

    // Renormalize quaternion covariance
    Matrix4d P_quat = Jt * P_quat_tilde * Jt.transpose();

    // Store updated quaternion covariance back into P
    this->P.block<4, 4>(6, 6) = P_quat;
    
    spdlog::debug("Updated this position: {}\n", this->position.transpose());
    spdlog::debug("Updated this velocity: {}\n", this->velocity.transpose());
    spdlog::debug("Updated this orientation (quaternion): {}\n", this->orientation.coeffs().transpose());
    spdlog::debug("Updated covariance P:\n {}\n", this->P);
}

void EKF::ekf_loop(Vector3d& acc_data, Vector3d& gyro_data) {

    predict(acc_data, gyro_data, dt, Sigma_a, Sigma_omega);
    update(acc_data, dt, Sigma_a);

}
