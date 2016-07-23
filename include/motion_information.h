#ifndef G2O_MOTION_INFORMATION_H
#define G2O_MOTION_INFORMATION_H

#include <vector>
#include <Eigen/StdVector>

#include "g2o/types/slam2d/se2.h"

using namespace g2o;

struct MotionInformation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SE2 stereoMotion;
  SE2 odomMotion;
  double timeInterval;
};

typedef std::vector<MotionInformation, Eigen::aligned_allocator<MotionInformation> >     MotionInformationVector;

#endif
