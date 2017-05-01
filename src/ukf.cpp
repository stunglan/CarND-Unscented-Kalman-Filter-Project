
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "ukf.h"
#include <string>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.setIdentity();

  
  // Experimented with values based on NIS plot
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0; // high RMSE with  30; lo

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;// program never finnished with original 3030;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15; // 0.15

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15; // 0.15

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  
  
  ///* State dimension
  n_x_ = 5;
  
  ///* Augmented state dimension
  n_aug_ = 7;
  
  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;;
  
  ///* the current NIS for radar
  NIS_radar_ = 0.0;
  
  ///* the current NIS for laser
  NIS_laser_ = 0.0;
  
  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  
  // code from quiz
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }
  
  
  
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);
  

  // logging variable
  logging = 0; // 0 no logging , 1 some logging , 2 even more logging

}


UKF::~UKF() {}

/**
 * logging all global variables 
 */
void UKF::log_all(string heading){
  // not implemented
  
}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  // First time Initializing
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
      // using the cosine and sine of position and speed
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rhodot = meas_package.raw_measurements_(2);
      
      // using the same as in EKF
      x_ << rho * cos(phi), rho * sin(phi), rhodot * cos(phi), rhodot * sin(phi),0;
      
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
       */
      // using the position
      // px, py
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1),0,0,0;
      
    }
    time_us_ = meas_package.timestamp_;  // note to self ---> not sure how to initialize
    is_initialized_ = true;
    
    if (logging > 0) {
      cout << "Initial x: " << endl;
      cout << x_ << endl;
    }
    return;
    
  }
  
  // calculate delta since last measurement
  double dt = (meas_package.timestamp_ - time_us_)  / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  // Prediction is same for radar and lidar
  Prediction(dt);
  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  // Three steps
  // Generate sigma points
  
  // Predict sigma points
  // Predict mean and covariance
  
  // Generate sigma points -- code from quiz - Augumented version since we consider noise
  
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);;
  AugmentedSigmaPoints(&Xsig_aug);
  
  // Predict sigma points -- code from quiz
  SigmaPointPrediction(Xsig_aug,delta_t);
  
  
  // Predict mean and covariance -- code from quiz
  PredictMeanAndCovariance();

}

/**
 * Generate augumented sigma points, -- code fro, quiz
 * @param MatrixXd* Xsig_out returns the sigmapoints and the noise
 */
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  
  
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  
  //write result
  *Xsig_out = Xsig_aug;
  
}

/**
 * Predicts  sigma points, -- code fro, quiz
 * @param MatrixXd* Xsig_aug returns the predicted sigmapoints and the noise
 * @param double delta_t time since last measurement
 * set object variable  MatrixXd* Xsig_  the predicted sigmapoints and the noise in the
 */

void UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t) {
  
  
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    
    //predicted state values
    double px_p, py_p;
    
    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
    
    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  

}

/**
 * Predicts the state and covariance matrix for the state.
 * @param MatrixXd &Xsig_pred the predicted sigma points
 * set object variable  x_pred_ the predicted state
 * set object variable P_pred_ the predicted state covariance matrix
 */

void UKF::PredictMeanAndCovariance() {
  
  
  if (logging > 0) {
    cout << "Xsig_pred_:" << endl;
    cout << Xsig_pred_<< endl;
  }
  //create vector for predicted state
  VectorXd x = VectorXd::Zero(n_x_);
  
  //create covariance matrix for predicted state
  MatrixXd P = MatrixXd::Zero(n_x_, n_x_);
  
  
  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }
  
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }
  
  
  //write result
  x_ = x;
  P_ = P;
}



/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 * set object variable  x_pred_ the predicted state
 * set object variable P_pred_ the predicted state covariance matrix
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // two steps
  // predict measurements
  // update state
  
  
  // predict measurements
  
  // # lidar measurements
  int n_z = 2;
  // matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);
  for(unsigned int i = 0; i < 2*n_aug_ + 1; i++) {
    double px      = Xsig_pred_(0,i);
    double py      = Xsig_pred_(1,i);
    Zsig(0,i) = px;
    Zsig(1,i) = py;
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (unsigned int j = 0; j < 2*n_aug_ + 1; j++) {
    z_pred = z_pred + (weights_(j) * Zsig.col(j));
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (unsigned int j = 0; j < 2*n_aug_ + 1; j++) {
    VectorXd z_diff = Zsig.col(j) - z_pred;
    S = S + weights_(j) * z_diff * z_diff.transpose();
  }
  
  MatrixXd R(2,2);
  R << std_laspx_ * std_laspx_, 0,
  0, std_laspy_ * std_laspy_;
  
  S = S + R;
  
  // Update state
  
  //create vector for incoming lidar measurement
  VectorXd z = VectorXd(n_z);
  z <<meas_package.raw_measurements_(0),   //px
  meas_package.raw_measurements_(1);   //py
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (unsigned int j = 0; j < 2*n_aug_ + 1; j++) {
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    VectorXd z_diff = Zsig.col(j) - z_pred;
    // angle normalization
    if (x_diff(3)> M_PI) x_diff(3) = remainder(x_diff(3), (2.*M_PI)) - M_PI;
    if (x_diff(3)<-M_PI) x_diff(3) = remainder(x_diff(3), (2.*M_PI)) + M_PI;
    Tc = Tc + weights_(j) * x_diff * z_diff.transpose();
  }
  
  MatrixXd K = Tc * S.inverse();
  
  VectorXd z_diff = z - z_pred;
  
  x_ = x_ + (K * z_diff);
  P_ = P_ - (K * S * K.transpose());
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 * set object variable  x_pred_ the predicted state
 * set object variable P_pred_ the predicted state covariance matrix
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // two steps
  // predict measurements
  // update state
  
  
  // # radar measurements
  int n_z = 3;
  // matrics for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  
  z << meas_package.raw_measurements_[0],
  meas_package.raw_measurements_[1],
  meas_package.raw_measurements_[2];
  
  // predict measurements
  
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
  0, std_radphi_*std_radphi_, 0,
  0, 0,std_radrd_*std_radrd_;
  S = S + R;
  
  
  
  // Update state
  
  //create matrix for cross correlation
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //residual
  VectorXd z_diff = z - z_pred;
  
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  
  // calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  if (logging > 1) {
    std::cout << "Updated state x: " << std::endl << x_ << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  }

  
}
