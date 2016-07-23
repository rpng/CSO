#include "closed_form_calibration.h"

#include "g2o/types/sclam2d/odometry_measurement.h"

#include <iostream>
#include <limits>

#include <Eigen/SVD>

#define SQR(X)		( std::pow(X,2) )
#define CUBE(X)		( std::pow(X,3) )

using namespace Eigen;

bool ClosedFormCalibration::calibrate(const MotionInformationVector& measurements, SE2& stereoOffset)
{
  // construct M
  Eigen::Matrix<double, 5, 5> M;
  M.setZero();
  for (size_t i = 0; i < measurements.size(); ++i) {
    const SE2& stereoMotion = measurements[i].stereoMotion;
    const SE2& odomMotion = measurements[i].odomMotion;
    const double& timeInterval = measurements[i].timeInterval;
    double o_theta_k = odomMotion.rotation().angle();
    Eigen::Matrix<double, 2, 5> L;
    L(0, 0) = -odomMotion.translation()[0];
    L(0, 1) = 1 - cos(o_theta_k);
    L(0, 2) = sin(o_theta_k);
    L(0, 3) = stereoMotion.translation().x();
    L(0, 4) = -stereoMotion.translation().y();
    L(1, 0) = -odomMotion.translation()[0];
    L(1, 1) = - sin(o_theta_k);
    L(1, 2) = 1 - cos(o_theta_k);
    L(1, 3) = stereoMotion.translation().y();
    L(1, 4) = stereoMotion.translation().x();
    M.noalias() += L.transpose() * L;
  }
  //std::cout << M << std::endl;

  // compute lagrange multiplier
  // and solve the constrained least squares problem
  double m11 = M(0,0);
  double m13 = M(0,2);
  double m14 = M(0,3);
  double m15 = M(0,4);
  double m22 = M(1,1);
  double m34 = M(2,3);
  double m35 = M(2,4);
  double m44 = M(3,3); 

  double a = m11 * SQR(m22) - m22 * SQR(m13);
  double b = 2*m11 * SQR(m22) * m44 - SQR(m22) * SQR(m14) - 2*m22 * SQR(m13) * m44 - 2*m11 * m22 * SQR(m34) 
    - 2*m11 * m22 * SQR(m35) - SQR(m22) * SQR(m15) + 2*m13 * m22 * m34 * m14 + SQR(m13) * SQR(m34) 
    + 2*m13 * m22 * m35 * m15 + SQR(m13) * SQR(m35);
  double c = - 2*m13 * CUBE(m35) * m15 - m22 * SQR(m13) * SQR(m44) + m11 * SQR(m22) * SQR(m44) + SQR(m13) * SQR(m35) * m44
    + 2*m13 * m22 * m34 * m14 * m44 + SQR(m13) * SQR(m34) * m44 - 2*m11 * m22 * SQR(m34) * m44 - 2 * m13 * CUBE(m34) * m14
    - 2*m11 * m22 * SQR(m35) * m44 + 2*m11 * SQR(m35) * SQR(m34) + m22 * SQR(m14) * SQR(m35) - 2*m13 * SQR(m35) * m34 * m14
    - 2*m13 * SQR(m34) * m35 * m15 + m11 * std::pow(m34, 4) + m22 * SQR(m15) * SQR(m34) + m22 * SQR(m35) * SQR(m15)
    + m11 * std::pow(m35, 4) - SQR(m22) * SQR(m14) * m44 + 2*m13 * m22 * m35 * m15 * m44 + m22 * SQR(m34) * SQR(m14)
    - SQR(m22) * SQR(m15) * m44;

  // solve the quadratic equation
  double lambda1, lambda2;
  if(a < std::numeric_limits<double>::epsilon()) {
    if(b <= std::numeric_limits<double>::epsilon())
      return false;
    lambda1 = lambda2 = -c/b;
  } else {
    double delta = b*b - 4*a*c;
    if (delta < 0)
      return false;
    lambda1 = 0.5 * (-b-sqrt(delta)) / a;
    lambda2 = 0.5 * (-b+sqrt(delta)) / a;
  }

  Eigen::VectorXd x1 = solveLagrange(M, lambda1);
  Eigen::VectorXd x2 = solveLagrange(M, lambda2);
  double err1 = x1.dot(M * x1);
  double err2 = x2.dot(M * x2);

  const Eigen::VectorXd& calibrationResult = err1 < err2 ? x1 : x2;

  stereoOffset = SE2(calibrationResult(1), calibrationResult(2), atan2(calibrationResult(4), calibrationResult(3)));

  return true;
}

Eigen::VectorXd ClosedFormCalibration::solveLagrange(const Eigen::Matrix<double,5,5>& M, double lambda)
{
  // A = M * lambda*W (see paper)
  Eigen::Matrix<double,5,5> A;
  A.setZero();
  A(3,3) = A(4,4) = lambda;
  A.noalias() += M;

  // compute the kernel of A by SVD
  Eigen::JacobiSVD< Eigen::Matrix<double,5,5> > svd(A, ComputeFullV);
  Eigen::VectorXd result = svd.matrixV().col(4);
  //for (int i = 0; i < 5; ++i)
  //std::cout << "singular value " << i << " "  << svd.singularValues()(i) << std::endl;
  //std::cout << "kernel base " << result << std::endl;

  // enforce the conditions
  // x_1 > 0
  if (result(0) < 0.)
    result *= -1;
  // x_4^2 + x_5^2 = 1
  double scale = sqrt(pow(result(3), 2) + pow(result(4), 2));
  result /= scale;

  return result;
}
