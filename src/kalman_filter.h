#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:
    
    /**
     * Constructor
     */
    KalmanFilter();
    
    /**
     * Destructor
     */
    virtual ~KalmanFilter() { }
    
    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
              Eigen::MatrixXd &H_in);
    
    /**
     * Separate init method for the state
     * @param x_in Initial state
     */
    void Init_X(Eigen::VectorXd &x_in);
    
    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param dt Time between latest and current measurement in seconds
     */
    void Predict(const float dt);
    
    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);
    
    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);
    
    /** 
     * Return current state estimation
     * @return current state x
     */
    Eigen::VectorXd getStateEstimation() const
    {
        return x_;
    }
    
    /**
     * Return current state covariane
     * @return current state covariance matrix
     */
    Eigen::MatrixXd getStateCovarianceMatrix() const
    {
        return P_;
    }

    
private:
    
    // state vector
    Eigen::VectorXd x_;
    
    // state covariance matrix
    Eigen::MatrixXd P_;
    
    // state transistion matrix
    Eigen::MatrixXd F_;

    // measurement matrix
    Eigen::MatrixXd H_;    
};

#endif /* KALMAN_FILTER_H_ */
