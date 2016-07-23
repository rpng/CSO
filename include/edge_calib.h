#ifndef EDGE_CALIB_H
#define EDGE_CALIB_H

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/base_unary_edge.h"

/**
 * \brief calibrate odometry and stereo based on a set of measurements
 */

using namespace g2o;

// used to be the calibration measurement.
struct OdomAndStereoMotion
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SE2 odomMotion;
  SE2 StereoMotion;
};

// An edge used to calibrate the offset between odometry and stereo
class EdgeCalib : public BaseUnaryEdge<3, OdomAndStereoMotion, VertexSE2>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeCalib();

    void computeError();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
};

#endif
