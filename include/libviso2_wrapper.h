#ifndef LIBVISO2_WRAPPER_H
#define LIBVISO2_WRAPPER_H

#include <iostream>
#include <string>

#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_odom.h"
#include "g2o/types/slam2d/se2.h"

#include <viso_stereo.h>
#include <png++/png.hpp>

using namespace std;
using namespace g2o;

/**
 * \brief libviso2 wrapper
 */

class StereoVo
{
  public:
    StereoVo();
    ~StereoVo();

    StereoVo(const string &, const int &, RobotData *, vector<double> &);

    // get 2d pose of stereo, x, y, theta( z-axis ), in a dataqueue. Return the num of vo.
    int getMotion(DataQueue &);
    // get initial pose of stereo.
    void getInitStereoPose(SE2 &);

  private:
    // input folder name.
    string rawFoldername;
    // camera pose, convert a point cp from current frame to global frame by wTc * cp.
    viso::Matrix wTc;
    // input image num, output vo num.
    int in_num, out_num;
    // A queue which stored stereo vo pose.
    DataQueue stereoVoQueue;
    // initial stereo pose in robot frame.
    SE2 initialStereoPose;
    // initial odometry pose in the world frame.
    RobotData* odomPose;
    // if it is first frame now.
    bool first;

};

#endif
