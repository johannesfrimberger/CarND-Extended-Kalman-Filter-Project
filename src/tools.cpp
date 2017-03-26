#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth){
    
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }
    
    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
        
        VectorXd residual = estimations[i] - ground_truth[i];
        
        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse = rmse/estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    
    MatrixXd Hj(3,4);
    //recover state parameters
    const float px = x_state(0);
    const float py = x_state(1);
    const float vx = x_state(2);
    const float vy = x_state(3);
    
    //pre-compute a set of terms to avoid repeated calculation
    const float c1 = pow(px, 2) + pow(py, 2);
    const float c2 = sqrt(c1);
    const float c3 = (c1*c2);
    
    //check division by zero
    if(fabs(c1) < 0.0001){
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }
    
    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;
}

VectorXd Tools::convertStateToRadarMeas(const VectorXd& x_state)
{
    const double min_distance = 1e-4;
    const double distance = sqrt(pow(x_state[0],2) + pow(x_state[1],2));
    const double safe_distance = (distance < min_distance) ? min_distance : distance;
    
    VectorXd z_pred(3);
    z_pred << distance,
        atan2(x_state[1], x_state[0]),
        (x_state[0] * x_state[2] + x_state[1] * x_state[3]) / safe_distance;
    
    return z_pred;
}
