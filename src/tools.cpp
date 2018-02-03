#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);

    // Checking the validity of the following inputs:
    //  * The estimation vector size should not be zero
    //  * The estimation vector size should equal ground truth vector size
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        cerr << "Invalid estimation or ground_truth data" << endl;
        rmse << -1, -1, -1, -1; // -1 to indicate an undefined value
        return rmse;
    }

    rmse << 0, 0, 0, 0;

    // Accumulating squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // Calculating the mean
    rmse = rmse / estimations.size();

    // Calculating the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}

void Tools::NormalizeAngle(double& angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
}
