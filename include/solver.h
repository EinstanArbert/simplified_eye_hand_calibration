#ifndef CALIB_SOLVER_H
#define CALIB_SOLVER_H
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>

struct sync_data
{
  // Period
  double T;
  // Left and right wheel velocities
  double velocity_left;
  double velocity_right;
  // double velocity;
  // Scan matching estimate
  double scan_match_results[3];
  // Estimated rototranslation based on odometry params.
  double o[3];
  // Estimated disagreement  sm - est_sm
  double est_sm[3];
  double err_sm[3];
  // Other way to estimate disagreement:   l (+) s  - o (+) l
  double err[3];
  int mark_as_outlier;
  double odom_result[3];
};


class cSolver{

public:

  cSolver();

  struct solver_params {
    int mode;

    double max_cond_number;

    int outliers_iterations;
    double outliers_percentage;
  };

  struct calib_result {
    double radius_l, radius_r;
    double axle;

    /** Laser pose */
    double l[3];
  };

  bool solve(const std::vector<sync_data> &calib_data, int mode, double max_cond_number, struct calib_result& res);

  void calib(std::vector<sync_data> &calib_data, int outliers_iterations, calib_result& res);

private:

  Eigen::VectorXd full_calibration_min(const Eigen::MatrixXd &M);

  Eigen::VectorXd numeric_calibration(const Eigen::MatrixXd &H);

  double calculate_error(const Eigen::VectorXd &x, const Eigen::MatrixXd &M);

  Eigen::VectorXd x_given_lambda(const Eigen::MatrixXd &M, const double &lambda, const Eigen::MatrixXd &W);

  void oplus_d(const double x1[3], const double x2[3], double res[3]);

  void compute_disagreement(sync_data &calib_data, const struct calib_result &res);

  void pose_diff_d(const double pose2[3], const double pose1[3], double res[3]);

  void ominus_d(const double x[3], double res[3]);

  void estimate_noise(std::vector<sync_data> &calib_data, const struct calib_result &res,
                      double &std_x, double &std_y, double &std_th);

  double calculate_sd(const double array[], const int s, const int e);

  Eigen::MatrixXd compute_fim(const std::vector<sync_data> &calib_data, const struct calib_result &res,
                              const Eigen::Matrix3d &inf_sm);

};

#endif
