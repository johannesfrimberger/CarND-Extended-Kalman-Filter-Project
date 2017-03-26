#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in)
{
    Init_X(x_in);
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
}

void KalmanFilter::Init_X(Eigen::VectorXd &x_in)
{
    x_ = x_in;
}

void KalmanFilter::Predict(const float dt)
{
    //Modify the F matrix so that the time is integrated
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    
    const float dt_2 = dt * dt;
    const float dt_3 = dt_2 * dt;
    const float dt_4 = dt_3 * dt;
    
    const float noise_ax = 9.0f;
    const float noise_ay = 9.0f;
    
    // process covariance matrix
    MatrixXd Q = MatrixXd(4, 4);
    Q <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
    0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
    dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &R)
{
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &R)
{
    MatrixXd Hj = Tools::CalculateJacobian(x_);
    
    VectorXd y = z - Tools::convertStateToRadarMeas(x_);
    MatrixXd Ht = Hj.transpose();
    
    MatrixXd S = Hj * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;
}
