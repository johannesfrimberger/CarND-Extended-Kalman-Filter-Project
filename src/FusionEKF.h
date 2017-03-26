#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
    /**
     * Constructor.
     */
    FusionEKF();
    
    /**
     * Destructor.
     */
    virtual ~FusionEKF();
    
    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    
    /**
     * Return current state estimation
     * @return current state x
     */
    Eigen::VectorXd getStateEstimation() const
    {
        return ekf_.getStateEstimation();
    }
    
private:
    
    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;
    
    // tool object used to compute Jacobian and RMSE
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
    
    // previous timestamp
    long previous_timestamp_;
    
    // check whether the tracking toolbox was initiallized or not (first measurement)
    bool is_initialized_;
    
};

#endif /* FusionEKF_H_ */
