#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // If this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // If this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3;


    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.


    // State dimension
    n_x_ = 5;

    // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ = VectorXd(5);

    // State covariance matrix
    P_ = MatrixXd(5, 5);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // Measurement noise covariance matrix for radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;

    // Measurement noise covariance matrix for lidar
    R_lidar_ = MatrixXd(2, 2);
    R_lidar_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;

    // Augmented state dimension
    n_aug_ = 7;

    // Predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }

    // Initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // Tools object
    tools = Tools();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // Initializing the state of x_ with the first measurement.
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double rho_dot = meas_package.raw_measurements_[2];
            // Using rho_dot as firts estimate of velocity
            x_ << rho * cos(phi), rho * sin(phi), rho_dot, 0, 0;
        } else /* if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) */ {
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
        }

        // Done with initializing, no need to predict or update
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    // Checking the types of messures to ignore
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
        return;
    }
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
        return;
    }

    /*****************************************************************************
    *  Prediction
    ****************************************************************************/

    // Computing the time elapsed between the current and previous measurements
    // delta_t - expressed in seconds
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;
    // Predicting the state and covariance matrix.
    Prediction(delta_t);

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    // Using the sensor type to perform the update step.
    // Updating the state and covariance matrix.
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    } else {
        UpdateLidar(meas_package);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    // Getting sigma points
    MatrixXd Xsig_aug = AugmentedSigmaPoints();
    // Performing the prediction for the sigma points
    SigmaPointPrediction(Xsig_aug, delta_t);
    // Predict the mean and covariance from the predicted sigma points
    PredictMeanAndCovariance();
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

    // Getting lidar's data
    VectorXd z(2);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

    // Sigma points into measurement space
    MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
    // Mean predicted measurement
    VectorXd z_pred = VectorXd(2);
    // Innovation covariance matrix S
    MatrixXd S = MatrixXd(2, 2);

    // Setting sigma points into measurement space and predicting measurement
    PredictLidarMeasurement(Zsig, z_pred, S);
    // Updating the state vector, x_, and covariance, P_
    UpdateStateLidar(z, Zsig, z_pred, S);
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

    // Getting radar's data
    VectorXd z(3);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

    // Sigma points into measurement space
    MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
    // Mean predicted measurement
    VectorXd z_pred = VectorXd(3);
    // Innovation covariance matrix S
    MatrixXd S = MatrixXd(3, 3);

    // Setting sigma points into measurement space and predicting measurement
    PredictRadarMeasurement(Zsig, z_pred, S);
    // Updating the state vector, x_, and covariance, P_
    UpdateStateRadar(z, Zsig, z_pred, S);
}


/**
 *
 */
VectorXd UKF::PredictAhead(const double& delta_t) {
    // Getting sigma points
    MatrixXd Xsig_aug = AugmentedSigmaPoints();
    // Performing the prediction for the sigma points
    SigmaPointPrediction(Xsig_aug, delta_t);
    // Predict the mean
    VectorXd prediction = VectorXd(5);
    PredictMean(prediction, Xsig_pred_);
    return prediction;
}


VectorXd UKF::GetCatchingPoint(const double& distance_difference) {
    double speed = x_[2];

    // Checking division by zero
    if (speed < 0.0001) {
        throw "GetCatchingPoint( () - Error - Division by Zero";
    }

    double meeting_time = distance_difference / speed;
    return PredictAhead(meeting_time);
}



MatrixXd UKF::AugmentedSigmaPoints() {

    // Creating augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // Creating augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // Creating sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // Creating augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;
    x_aug(n_x_ + 1) = 0;

    // Creating augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    // Creating square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // Create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    return Xsig_aug;
}


void UKF::SigmaPointPrediction(const MatrixXd& Xsig_aug, const double& delta_t) {

    // Predicting sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // Extracting values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // Predicted state values
        double px_p, py_p;

        // Avoiding division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
        }
        else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // Adding noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // Writing predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
}


void UKF::PredictMeanAndCovariance() {

    // Predicted state mean
    PredictMean(x_, Xsig_pred_);

    // Predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization of state (yaw)
        tools.NormalizeAngle(x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}


void UKF::PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {

    // Transforming sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Extracting values for better readibility
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double v_x = cos(yaw) * v;
        double v_y = sin(yaw) * v;

        // Measurement model
        double p1 = sqrt(p_x * p_x + p_y * p_y);
        Zsig(0, i) = p1;                            // rho
        Zsig(1, i) = atan2(p_y, p_x);               // phi
        Zsig(2, i) = (p_x * v_x + p_y * v_y ) / p1; // rho_dot
    }

    // Mean predicted measurement
    PredictMean(z_pred, Zsig);

    // Innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // Angle normalization for residual (phi)
        tools.NormalizeAngle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // Adding measurement noise covariance matrix
    S = S + R_radar_;
}


void UKF::PredictLidarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {

    // Transforming sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Measurement model
        Zsig(0, i) = Xsig_pred_(0, i); // x
        Zsig(1, i) = Xsig_pred_(1, i); // y
    }

    // Mean predicted measurement
    PredictMean(z_pred, Zsig);

    // Innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // Adding measurement noise covariance matrix
    S = S + R_lidar_;
}


void UKF::PredictMean(VectorXd& mean_vector, const MatrixXd& pred_sig_points) {
    // Predicting mean by iterating over sigma points
    mean_vector.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        mean_vector = mean_vector + weights_(i) * pred_sig_points.col(i);
    }
}


void UKF::UpdateStateRadar(
    const VectorXd& z, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S) {

    // Creating matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, 3);

    // Calculating cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // Angle normalization of residual (phi)
        tools.NormalizeAngle(z_diff(1));

        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization of state (yaw)
        tools.NormalizeAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Residual
    VectorXd z_diff = z - z_pred;
    // Angle normalization of residual (phi)
    tools.NormalizeAngle(z_diff(1));

    UpdateStatestateMeanAndCovarianceMatrix(Tc, S, z_diff);
}


void UKF::UpdateStateLidar(
    const VectorXd& z, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S) {

    // Creating matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, 2);

    // Calculating cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Residual
    VectorXd z_diff = z - z_pred;

    UpdateStatestateMeanAndCovarianceMatrix(Tc, S, z_diff);
}


void UKF::UpdateStatestateMeanAndCovarianceMatrix(
    const MatrixXd& Tc, const MatrixXd& S, const VectorXd& z_diff) {

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // Updating state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}
