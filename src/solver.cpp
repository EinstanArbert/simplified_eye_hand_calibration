#include <solver.h>

cSolver::cSolver()
{}

void cSolver::calib(std::vector<sync_data> &calib_data, int outliers_iterations, calib_result& res)
{
  std::vector<sync_data> calib_history[outliers_iterations + 1];

  for (int iteration = 0; iteration <= outliers_iterations; iteration++)
  {
    calib_history[iteration] = calib_data;

    // Calibration
    if (!solve(calib_data, 0, 75, res))
    {
      std::cout << "Failed CSM calibration." << std::endl;
      continue;
    }
    else
    {
      std::cout << '\n' << "-------CSM Calibration Results-------" << '\n' << "Camera to odom x: " << res.l[0] << '\n'
                << "Camera to odom y: " << res.l[1] << '\n' << "Camera to odom yaw(degree): " << res.l[2] * 180. / 3.1415926 << std::endl;
    }

    // Compute residuals
    for (int i = 0; i < calib_data.size(); i++)
    {
      compute_disagreement(calib_data[i], res);
    }

    // Sort residuals and compute thresholds
    std::vector<double> err_theta;
    for (int i = 0; i < calib_data.size(); i++)
    {
      err_theta.push_back(fabs(calib_data[i].err[2]));
    }

    std::vector<double> err_xy;
    for (int i = 0; i < calib_data.size(); i++)
    {
      double x = calib_data[i].err[0];
      double y = calib_data[i].err[1];
      err_xy.push_back(sqrt(x*x + y*y));
    }

    std::vector<double> err_theta_sorted(err_theta);
    std::sort(err_theta_sorted.begin(), err_theta_sorted.end());

    std::vector<double> err_xy_sorted(err_xy);
    std::sort(err_xy_sorted.begin(), err_xy_sorted.end());

    int threshold_index = calib_data.size() - 1;
    double threshold_theta = err_theta_sorted[threshold_index];
    double threshold_xy = err_xy_sorted[threshold_index];

    int noutliers = 0;
    int noutliers_theta = 0;
    int noutliers_xy = 0;
    int noutliers_both = 0;

    for (int i = 0; i < calib_data.size(); i++)
    {
      int xy = err_xy[i] > threshold_xy;
      int theta = err_theta[i] > threshold_theta;

      calib_data[i].mark_as_outlier = xy | theta;

      if(xy) noutliers_xy++;
      if(theta) noutliers_theta++;
      if(xy && theta) noutliers_both ++;
      if(xy || theta) noutliers ++;
    }

    std::vector<sync_data> n;
    for (int i = 0; i < calib_data.size(); i++)
    {
      if (!calib_data[i].mark_as_outlier) n.push_back(calib_data[i]);
    }

    calib_data = n;
  }

  return;
}


//https://blog.csdn.net/weixin_41959939/article/details/107434895?spm=1001.2101.3001.6650.3&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-3.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-3.nonecase
//https://blog.csdn.net/qq_17032807/article/details/92613892
//https://www.jianshu.com/p/abab87a4e393
bool cSolver::solve(const std::vector<sync_data> &calib_data,
                    int mode, double max_cond_number, calib_result &res)
{
  int n = (int)calib_data.size();

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(5, 5);
  Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd L_k = Eigen::MatrixXd::Zero(2, 5);
  Eigen::MatrixXd L_2k = Eigen::MatrixXd::Zero(2, 6);

  double c, cx, cy, cx1, cx2, cy1, cy2, t1, t2;
  double o_theta, T, v_l, v_r, w0;
  double sm[3];

  int nused = 0;
  for (int k = 0; k < n; k++) {
    const sync_data & t = calib_data[k];

    o_theta = t.odom_result[2];
    w0 = o_theta / t.T;

    if ((mode == 0) || (mode == 1)) {
      cx = t.odom_result[0];
      cy = t.odom_result[1];
      L_k << -cx, (1 - cos(o_theta)), sin(o_theta), t.scan_match_results[0], -t.scan_match_results[1],
          -cy, -sin(o_theta), (1 - cos(o_theta)), t.scan_match_results[1], t.scan_match_results[0];
      // M = M + L_k' * L_k; M is symmetric
      M = M + L_k.transpose() * L_k;
    }
    else {
      L_2k << -cx1, -cx2, (1 - cos(o_theta)), sin(o_theta), t.scan_match_results[0], -t.scan_match_results[1],
          -cy1, -cy2, -sin(o_theta), (1 - cos(o_theta)), t.scan_match_results[1], t.scan_match_results[0];
      M2 = M2 + L_2k.transpose() * L_2k;
    }
  }

  double est_b, est_d_l, est_d_r, laser_x, laser_y, laser_th;
  Eigen::VectorXd x;
  switch(mode)
  {
  case 0:
  {
    x = full_calibration_min(M);

    laser_x = x(1);
    laser_y = x(2);
    laser_th = atan2(x(4), x(3));
    break;
  }
  default:
    break;
  }

  res.l[0] = laser_x;
  res.l[1] = laser_y;
  res.l[2] = laser_th;

  return 1;
}

Eigen::VectorXd cSolver::full_calibration_min(const Eigen::MatrixXd &M)
{
  double m11 = M(0, 0);
  double m13 = M(0, 2);
  double m14 = M(0, 3);
  double m15 = M(0, 4);
  double m22 = M(1, 1);
  // double m25 = M(1, 4);
  double m34 = M(2, 3);
  double m35 = M(2, 4);
  double m44 = M(3, 3);
  // double m55 = M(4, 4);
  double a, b, c;

  a = m11 * pow(m22,2) - m22 * pow(m13,2);
  b = 2 * m13 * m22 * m35 * m15 - pow(m22,2) * pow(m15,2) - 2 * m11 * m22 * pow(m35, 2)
      + 2 * m13 * m22 * m34 * m14 - 2 * m22 * pow(m13,2) * m44 - pow(m22,2) * pow(m14,2)
      + 2 * m11 * pow(m22,2) * m44 + pow(m13,2) * pow(m35,2) - 2 * m11 * m22 * pow(m34,2)
      + pow(m13,2) * pow(m34,2);
  c = -2 * m13 * pow(m35, 3) * m15 - m22 * pow(m13,2) * pow(m44,2) + m11 * pow(m22,2) * pow(m44,2)
      + pow(m13,2) * pow(m35,2) * m44 + 2 * m13 * m22 * m34 * m14 * m44
      + pow(m13,2) * pow(m34,2) * m44 - 2 * m11 * m22 * pow(m34,2) * m44
      - 2 * m13 * pow(m34,3) * m14 - 2 * m11 * m22 * pow(m35,2) * m44
      + 2 * m11 * pow(m35,2) * pow(m34,2) + m22 * pow(m14,2) * pow(m35,2)
      - 2 * m13 * pow(m35,2) * m34 * m14 - 2 * m13 * pow(m34, 2) * m35 * m15
      + m11 * pow(m34,4) + m22 * pow(m15,2) * pow(m34,2)
      + m22 * pow(m35,2) * pow(m15,2) + m11 * pow(m35,4)
      - pow(m22,2) * pow(m14,2) * m44 + 2 * m13 * m22 * m35 * m15 * m44
      + m22 * pow(m34,2) * pow(m14,2) - pow(m22,2) * pow(m15,2) * m44;

    /* 	Calcolo radice del polinomio 	*/
  if ((pow(b,2) - 4 * a * c) >= 0)
  {
    double r0 = (-b - sqrt(pow(b,2) - 4 * a * c)) / (2 * a);
    double r1 = (-b + sqrt(pow(b,2) - 4 * a * c)) / (2 * a);

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(5, 5);
    W(3,3) = 1;
    W(4,4) = 1;
    Eigen::VectorXd x0 = x_given_lambda(M, r0, W);
    Eigen::VectorXd x1 = x_given_lambda(M, r1, W);

    double e0 = calculate_error(x0, M);
    double e1 = calculate_error(x1, M);

    return e0 < e1 ? x0 : x1;
  }
  else {
    std::cout << "Imaginary solution!" << std::endl;
    return Eigen::VectorXd(5);
  }
}

//Eigen::VectorXd cSolver::numeric_calibration(const Eigen::MatrixXd &H)
//{
//  int sz = H.cols() * H.rows();
//  Eigen::EigenSolver<Eigen::MatrixXd> es;
//  es.compute(H);
//  Eigen::MatrixXd eigenvectors = es.pseudoEigenvectors();
//  Eigen::VectorXd v0 = eigenvectors.col(sz - 5);

//  Eigen::Vector2d tmp_v;
//  tmp_v(0) = v0(sz-2);
//  tmp_v(1) = v0(sz-1);

//  double norm = tmp_v.norm();
//  double coeff = v0(0) / norm;
//  v0 = coeff * v0;

//  Eigen::VectorXd min = Eigen::VectorXd::Zero(sz);

//}

double cSolver::calculate_error(const Eigen::VectorXd &x, const Eigen::MatrixXd &M)
{
  double error;
  Eigen::VectorXd tmp = Eigen::VectorXd::Zero(x.rows());
  tmp = M * x;
  error = x.transpose() * tmp;

  return error;
}

Eigen::VectorXd cSolver::x_given_lambda(const Eigen::MatrixXd &M, const double &lambda, const Eigen::MatrixXd &W)
{
  Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(5,5);
  Eigen::MatrixXd ZZ = Eigen::MatrixXd::Zero(5,5);

  Z = M + lambda * W;

  ZZ = Z.transpose() * Z;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(ZZ);

//  Eigen::EigenSolver<Eigen::MatrixXd> es;
//  es.compute(ZZ);

//  Eigen::VectorXd eigenvalues = es.pseudoEigenvalueMatrix();
//  Eigen::MatrixXd eigenvectors = es.pseudoEigenvectors();
  Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();
  int colnum = eigenvalues.minCoeff();
  Eigen::VectorXd v0 = eigenvectors.col(eigenvalues.minCoeff());
  Eigen::Vector2d tmp_v = Eigen::Vector2d::Zero(2);
  tmp_v(0) = v0(3);
  tmp_v(1) = v0(4);

  double norm = tmp_v.norm();
  double coeff = (v0(0) >= 0 ? 1 : -1) / norm;
  v0 = coeff * v0;
  return v0;
}

/** safe if res == x1 */
void cSolver::oplus_d(const double x1[3], const double x2[3], double res[3]) {
	double c = cos(x1[2]);
	double s = sin(x1[2]);
	double x = x1[0]+c*x2[0]-s*x2[1];
	double y = x1[1]+s*x2[0]+c*x2[1];
 	double theta = x1[2]+x2[2];
	res[0]=x;
	res[1]=y;
	res[2]=theta;
}

void cSolver::compute_disagreement(sync_data &calib_data, const calib_result &res)
{
  double o_theta = calib_data.odom_result[2];

  calib_data.o[0] = calib_data.odom_result[0];
  calib_data.o[1] = calib_data.odom_result[1];
  calib_data.o[2] = o_theta;

  double l_plus_s[3];
  double o_plus_l[3];

  oplus_d(res.l, calib_data.scan_match_results, l_plus_s);
  oplus_d(calib_data.o, res.l, o_plus_l);

  for (int i = 0; i < 3; i++)
  {
    calib_data.err[i] = l_plus_s[i] - o_plus_l[i];
  }

  pose_diff_d(o_plus_l, res.l, calib_data.est_sm);

  // est_sm 就是公式 9 ， 从轮速计估计的增量 通过外参数估计的 激光坐标系两时刻之间的增量
  for (int i = 0; i < 3; i++)
  {
    calib_data.err_sm[i] = calib_data.est_sm[i] - calib_data.scan_match_results[i];
  }

}

void cSolver::ominus_d(const double x[3], double res[3]) {
	double c = cos(x[2]);
	double s = sin(x[2]);
	res[0] = -c*x[0]-s*x[1];
	res[1] =  s*x[0]-c*x[1];
	res[2] = -x[2];
}

void cSolver::pose_diff_d(const double pose2[3], const double pose1[3], double res[3]) {
	double temp[3];
	ominus_d(pose1, temp);
	oplus_d(temp, pose2, res);

	while(res[2] > +M_PI) res[2] -= 2*M_PI;
	while(res[2] < -M_PI) res[2] += 2*M_PI;
}

void cSolver::estimate_noise(std::vector<sync_data> &calib_data,
                              const calib_result &res, double &std_x, double &std_y,
                              double &std_th)
{
  int n = calib_data.size();
  double err_sm[3][n];
  for(int i = 0; i < calib_data.size(); i++)
  {
    compute_disagreement(calib_data[i], res);
    err_sm[0][i] = calib_data[i].err_sm[0];
    err_sm[1][i] = calib_data[i].err_sm[1];
    err_sm[2][i] = calib_data[i].err_sm[2];
  }

  std_x = calculate_sd(err_sm[0], 0, n);
  std_y = calculate_sd(err_sm[1], 0, n);
  std_th = calculate_sd(err_sm[2], 0, n);
}

double cSolver::calculate_sd(const double array[], const int s, const int e)
{
  double sum = 0;
  double mean = 0;
  double sd = 0;
  for (int i = s; i < e; i++)
  {
    sum += array[i];
  }
  mean = sum / (float)e;

  for (int i = s; i < e; i++)
  {
    sd += pow((array[i] - mean), 2);
  }

  sd = sqrt(sd / (float)e);

  return sd;
}

Eigen::MatrixXd cSolver::compute_fim(const std::vector<sync_data> &calib_data,
                          const calib_result &res, const Eigen::Matrix3d &inf_sm)
{
  Eigen::MatrixXd fim = Eigen::MatrixXd::Zero(6,6);

  /* Compute the derivative using the 5-point rule (x-h, x-h/2, x,
     x+h/2, x+h). Note that the central point is not used. */
  const double eps = 1e-3;
  for (size_t i = 0; i < calib_data.size(); ++i) {

    Eigen::Matrix<double, 3, 6> num_jacobian;

    // jacobian_radius_l
    {
      sync_data data0= calib_data[i],data1= calib_data[i],data2= calib_data[i],data3 = calib_data[i];
      calib_result res0 = res, res1 = res, res2 = res, res3 = res;
      res0.radius_l -= eps;
      res1.radius_l += eps;
      res2.radius_l -= eps/2.;
      res3.radius_l += eps/2.;
      compute_disagreement(data0,res0);
      compute_disagreement(data1,res1);
      compute_disagreement(data2,res2);
      compute_disagreement(data3,res3);
      for (int k = 0; k < 3; ++k) {
        double r3 = 0.5 * (data1.est_sm[k] - data0.est_sm[k]);
        double r5 = (4.0 / 3.0) * (data3.est_sm[k] - data2.est_sm[k]) - (1.0 / 3.0) * r3;
        num_jacobian(k,0) = r5 / eps ;
      }
    }

    // jacobian_radius_r
    {
      sync_data data0= calib_data[i],data1= calib_data[i],data2= calib_data[i],data3 = calib_data[i];
      calib_result res0 = res, res1 = res, res2 = res, res3 = res;
      res0.radius_r -= eps;
      res1.radius_r += eps;
      res2.radius_r -= eps/2.;
      res3.radius_r += eps/2.;
      compute_disagreement(data0,res0);
      compute_disagreement(data1,res1);
      compute_disagreement(data2,res2);
      compute_disagreement(data3,res3);
      for (int k = 0; k < 3; ++k) {
        double r3 = 0.5 * (data1.est_sm[k] - data0.est_sm[k]);
        double r5 = (4.0 / 3.0) * (data3.est_sm[k] - data2.est_sm[k]) - (1.0 / 3.0) * r3;
        num_jacobian(k,1) = r5 / eps ;
      }
    }
    // jacobian_axle
    {
      sync_data data0= calib_data[i],data1= calib_data[i],data2= calib_data[i],data3 = calib_data[i];
      calib_result res0 = res, res1 = res, res2 = res, res3 = res;
      res0.axle -= eps;
      res1.axle += eps;
      res2.axle -= eps/2.;
      res3.axle += eps/2.;
      compute_disagreement(data0,res0);
      compute_disagreement(data1,res1);
      compute_disagreement(data2,res2);
      compute_disagreement(data3,res3);
      for (int k = 0; k < 3; ++k) {
        double r3 = 0.5 * (data1.est_sm[k] - data0.est_sm[k]);
        double r5 = (4.0 / 3.0) * (data3.est_sm[k] - data2.est_sm[k]) - (1.0 / 3.0) * r3;
        num_jacobian(k,2) = r5 / eps ;
      }
    }
    // jacobian_l[0]
    {
      sync_data data0= calib_data[i],data1= calib_data[i],data2= calib_data[i],data3 = calib_data[i];
      calib_result res0 = res, res1 = res, res2 = res, res3 = res;
      res0.l[0] -= eps;
      res1.l[0] += eps;
      res2.l[0] -= eps/2.;
      res3.l[0] += eps/2.;
      compute_disagreement(data0,res0);
      compute_disagreement(data1,res1);
      compute_disagreement(data2,res2);
      compute_disagreement(data3,res3);
      for (int k = 0; k < 3; ++k) {
        double r3 = 0.5 * (data1.est_sm[k] - data0.est_sm[k]);
        double r5 = (4.0 / 3.0) * (data3.est_sm[k] - data2.est_sm[k]) - (1.0 / 3.0) * r3;
        num_jacobian(k,3) = r5 / eps ;
      }
    }
    // jacobian_l[1]
    {
      sync_data data0= calib_data[i],data1= calib_data[i],data2= calib_data[i],data3 = calib_data[i];
      calib_result res0 = res, res1 = res, res2 = res, res3 = res;
      res0.l[1] -= eps;
      res1.l[1] += eps;
      res2.l[1] -= eps/2.;
      res3.l[1] += eps/2.;
      compute_disagreement(data0,res0);
      compute_disagreement(data1,res1);
      compute_disagreement(data2,res2);
      compute_disagreement(data3,res3);
      for (int k = 0; k < 3; ++k) {
        double r3 = 0.5 * (data1.est_sm[k] - data0.est_sm[k]);
        double r5 = (4.0 / 3.0) * (data3.est_sm[k] - data2.est_sm[k]) - (1.0 / 3.0) * r3;
        num_jacobian(k,4) = r5 / eps ;
      }
    }
    // jacobian_l[2]
    {
      sync_data data0= calib_data[i],data1= calib_data[i],data2= calib_data[i],data3 = calib_data[i];
      calib_result res0 = res, res1 = res, res2 = res, res3 = res;
      res0.l[2] -= eps;
      res1.l[2] += eps;
      res2.l[2] -= eps/2.;
      res3.l[2] += eps/2.;
      compute_disagreement(data0,res0);
      compute_disagreement(data1,res1);
      compute_disagreement(data2,res2);
      compute_disagreement(data3,res3);
      for (int k = 0; k < 3; ++k) {
        double r3 = 0.5 * (data1.est_sm[k] - data0.est_sm[k]);
        double r5 = (4.0 / 3.0) * (data3.est_sm[k] - data2.est_sm[k]) - (1.0 / 3.0) * r3;
        num_jacobian(k,5) = r5 / eps ;
      }
    }

    fim += num_jacobian.transpose() * inf_sm * num_jacobian;
  }

  return fim;
}
