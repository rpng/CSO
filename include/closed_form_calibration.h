#ifndef G2O_CLOSED_FORM_CALIBRATION_H
#define G2O_CLOSED_FORM_CALIBRATION_H

#include <Eigen/Core>
#include "motion_information.h"

  /**
   * \brief Simultaneous calibration of the stereo offest and the parameters of a differential drive
   *
   * implementing from the article below, modify the laser's part.
   *
   * Approach described by Censi et al.
   * Andrea Censi, Luca Marchionni, and Giuseppe Oriolo.
   * Simultaneous maximum-likelihood calibration of robot and sensor parameters.
   * In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA). Pasadena, CA, May 2008.
   */
  class ClosedFormCalibration
  {
    public:
      static bool calibrate(const MotionInformationVector& measurements, SE2& stereoOffset);

    protected:
      static Eigen::VectorXd solveLagrange(const Eigen::Matrix<double,5,5>& M, double lambda);
  };

#endif
