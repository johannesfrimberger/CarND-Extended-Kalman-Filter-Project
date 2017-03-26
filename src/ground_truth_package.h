#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage
{
public:
    enum SensorTyp
    {
        LASER,
        RADAR
    } sensor_type_;
    
    Eigen::VectorXd gt_values_;
    long timestamp_;
};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
