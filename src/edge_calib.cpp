#include "edge_calib.h"

EdgeCalib::EdgeCalib() : BaseUnaryEdge<3, OdomAndStereoMotion, VertexSE2>(){}

void EdgeCalib::computeError()
{
    // error = inverse(wTo_k) * oTc * c_k-1_T_c_k * inverse(oTc), oTc refers to a transform from camera frame to odom fram.
    const VertexSE2* stereoOffset = static_cast<const VertexSE2*>(_vertices[0]);

    SE2 stereo_measure = measurement().StereoMotion;
    SE2 odom_measure = measurement().odomMotion;

    SE2 stereoMotionInRobotFrame = stereoOffset->estimate() * stereo_measure * stereoOffset->estimate().inverse();
    SE2 delta = odom_measure.inverse() * stereoMotionInRobotFrame;
    _error = delta.toVector();
}

bool EdgeCalib::read(std::istream &is) {return false;}

bool EdgeCalib::write(std::ostream& os) const {return false;}
