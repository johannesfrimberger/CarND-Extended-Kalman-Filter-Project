#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    
    //measurement matrix
    MatrixXd H = MatrixXd(2, 4);
    H << 1, 0, 0, 0,
    0, 1, 0, 0;
    
    //the initial transition matrix F_
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    VectorXd x_init = VectorXd(4);
    MatrixXd P_init = MatrixXd(4, 4);
    
    ekf_.Init(x_init, P_init, F, H);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
         TODO:
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        // first measurement
        cout << "Init EKF: " << endl;
        VectorXd x_init = VectorXd(4);
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            float ro = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            
            x_init << (ro * cos(phi)), (ro * sin(phi)), 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            x_init << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        }
        else
        {
            // Safe exit
            return;
        }
        
        ekf_.Init_X(x_init);
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
        
        // done initializing, no need to predict or update
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    // Compute the time elapsed between the current and previous measurements
    const float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    ekf_.Predict(dt);
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
    }
    else
    {
        // Laser updates
        ekf_.Update(measurement_pack.raw_measurements_, R_laser_);
    }
    
    // print the output
    //cout << "x_ = " << ekf_.getStateEstimation() << endl;
    //cout << "P_ = " << ekf_.getStateCovarianceMatrix() << endl;
}
