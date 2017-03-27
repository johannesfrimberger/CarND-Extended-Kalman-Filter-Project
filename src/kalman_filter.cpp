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
    
    // Pre calculate identity matrix
}

void KalmanFilter::Predict(const float dt)
{
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &R)
{
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
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
}
