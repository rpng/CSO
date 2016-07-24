#include "libviso2_wrapper.h"

StereoVo::StereoVo():rawFoldername(""), wTc(viso::Matrix()), in_num(0), out_num(0){}

StereoVo::StereoVo(const string & _foldername, const int & _innum, RobotData * _odomp, vector<double> & timeque):
    rawFoldername(_foldername), wTc(viso::Matrix()),
    in_num(_innum), out_num(0), odomPose(_odomp), first(true)
{
    // construct a visual odometry parameters
    viso::VisualOdometryStereo::parameters param;

    param.calib.f  = 702.183; // focal length in pixels
    param.calib.cu = 677.719; // principal point (u-coordinate) in pixels
    param.calib.cv = 353.332; // principal point (v-coordinate) in pixels
    param.base     = 0.120026; // baseline in meters

    // init visual odometry
    viso::VisualOdometryStereo viso(param);

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    wTc = viso::Matrix::eye(4);

    // stereo vo, i.e. libviso2
    cerr << "\033[32mNow running Stereo Visual Odometry...\033[0m" << endl;
    for(int32_t i = 0; i < in_num; i++)
    {
        // input file names
        char base_name[256]; sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = rawFoldername + "/image_0/" + base_name;
        string right_img_file_name = rawFoldername + "/image_1/" + base_name;

        // catch image read/write errors here
        try {

          // load left and right input image
          png::image< png::gray_pixel > left_img(left_img_file_name);
          png::image< png::gray_pixel > right_img(right_img_file_name);

          // image dimensions
          int32_t width  = left_img.get_width();
          int32_t height = left_img.get_height();

          // convert input images to uint8_t buffer
          uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
          uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
          int32_t k=0;
          for (int32_t v=0; v<height; v++) {
            for (int32_t u=0; u<width; u++) {
              left_img_data[k]  = left_img.get_pixel(u,v);
              right_img_data[k] = right_img.get_pixel(u,v);
              k++;
            }
          }

          // compute visual odometry
          int32_t dims[] = {width,height,width};
          if (viso.process(left_img_data,right_img_data,dims)) {

            // on success, update current pose
            wTc = wTc * viso::Matrix::inv(viso.getMotion());

            if(first){
                SE2 x(wTc.val[0][3],wTc.val[2][3],-atan(wTc.val[0][2]/wTc.val[0][0]));
                RobotOdom* ro = dynamic_cast<RobotOdom*>(odomPose);
                initialStereoPose = ro->odomPose().inverse()*x;
                first = false;
            }

            // save stereo vo as robotOdom node.
            RobotOdom* tempVo = new RobotOdom;
            tempVo->setTimestamp(timeque[i]);
            SE2 x(wTc.val[2][3],wTc.val[0][3],atan(wTc.val[0][2]/wTc.val[0][0]));
            tempVo->setOdomPose(x);
            stereoVoQueue.add(tempVo);
            out_num++;
          } else {
            cout << " \033[1;31m... failed to add this frame, continue!\033[0m" << endl;
          }

          // release uint8_t buffers
          free(left_img_data);
          free(right_img_data);

        // catch image read errors here
        } catch (...) {
          cerr << "\033[1;31mERROR: Couldn't read input files!\033[0m" << endl;
        }
    }
}

StereoVo::~StereoVo()
{
}

int StereoVo::getMotion(DataQueue & _queue)
{
    _queue = stereoVoQueue;
    return out_num;
}

void StereoVo::getInitStereoPose(SE2 & _initp)
{
    _initp = initialStereoPose;
}
