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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //state covariance matrix P
  MatrixXd P = MatrixXd(4, 4);         
  P <<  1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  ekf_.P_ = P;      

  //the initial transition matrix F_
  MatrixXd F = MatrixXd(4, 4);
  F <<  1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
  ekf_.F_ = F;


  // Laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ <<  1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0;

  ekf_.Q_ = MatrixXd(4, 4);


  //set the acceleration noise components
  noise_ax = 9.0;
  noise_ay = 9.0;


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
    cout << "EKF: " << endl;
    VectorXd x = VectorXd(4);
    x << 1, 1, 1, 1;
    ekf_.x_ = x;
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      //measurement matrix

      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi); // 0.0;
      double vy = 0.0;
    
      /*  
      if ((px == 0) || (py == 0)) {
      cout << "In RADAR measurement, px or py is zero.  Unable to create Jacobian.";
      } else {
      */
      /*

      if (px != 0 && py != 0) {
        ekf_.x_ << px, py, vx, vy;
      }
      */
      ekf_.x_ << px, py, vx, vy;
  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
  

      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];
      double vx = 0.0;
      double vy = 0.0;

      ekf_.x_ << px, py, vx, vy;

    }

    // initialize timestamp value
    previous_timestamp_ = measurement_pack.timestamp_;


    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */


  //compute the time elapsed between the current and previous measurements
  double dt =  ((double) measurement_pack.timestamp_ - (double) previous_timestamp_) / 1000000.0; //dt - expressed in seconds

  /* ERROR detection...  
  long long _dt =  ( measurement_pack.timestamp_ -  previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  if (fabs(dt - (double) _dt) > 0.0001) {
    cout << "========================= ERROR =========================" << endl;
  }
  */

  previous_timestamp_ = measurement_pack.timestamp_;

  cout << "dt is: " << dt << endl;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  /* 
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  */

  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;    

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

/*
  MatrixXd Q = MatrixXd(4, 4);
  Q <<  dt_4*noise_ax/4, 0, dt_3*noise_ax/2, 0,
        0, dt_4*noise_ay/4, 0, dt_3*noise_ay/2,
        dt_3*noise_ax/2, 0, dt_2*noise_ax, 0,
        0, dt_3*noise_ay/2, 0, dt_2*noise_ay;
  ekf_.Q_ = Q; 
*/  
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

















