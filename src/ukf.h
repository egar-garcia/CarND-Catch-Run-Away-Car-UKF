#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
  public:

    ///* Initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* If this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* If this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* State covariance matrix
    MatrixXd P_;

    ///* Predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* Time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* Measurement noise covariance matrix for radar
    MatrixXd R_radar_;

    ///* Measurement noise covariance matrix for lidar
    MatrixXd R_lidar_;

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    VectorXd PredictAhead(const double& delta_t);

    VectorXd GetCatchingPoint(const double& distance_difference);

  private:

    ///* Tool object used to calculate RMSE and normalize angles
    Tools tools;

    ///* Calculates augmented sigma points
    MatrixXd AugmentedSigmaPoints();

    ///* Performs the prediction for the sigma points
    void SigmaPointPrediction(const MatrixXd& Xsig_aug, const double& delta_t);

    ///* Predicts the mean and covariance from the predicted sigma points
    void PredictMeanAndCovariance();

    ///* Sets sigma points into measurement space and predics from radar measurement
    void PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

    ///* Sets sigma points into measurement space and predics from lidar measurement
    void PredictLidarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

    ///* Predicts mean vector from a set of sigma points
    void PredictMean(VectorXd& mean_vector, const MatrixXd& pred_sig_points);

    ///* Updates the state vector, x_, and covariance, P_ from a radar measurement
    void UpdateStateRadar(
        const VectorXd& z, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S);

    ///* Updates the state vector, x_, and covariance, P_ from a lidar measurement
    void UpdateStateLidar(
        const VectorXd& z, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S);

    ///* Helper method to update mean and covariance matrix
    void UpdateStatestateMeanAndCovarianceMatrix(
        const MatrixXd& Tc, const MatrixXd& S, const VectorXd& z_diff);
};

#endif /* UKF_H */
