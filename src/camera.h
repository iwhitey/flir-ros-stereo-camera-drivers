#ifndef CAMERA_HEADER
#define CAMERA_HEADER

#include "stereo_camera_driver/std_include.h"
#include "stereo_camera_driver/serialization.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <thread>
#include <mutex>

#include "StereoGigE.h"
#include "pi_controller.h"

class StereoGigE;


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;

class CameraWrapper : public ImageEventHandler {

public:

    ~CameraWrapper();

    CameraWrapper(CameraPtr, bool, StereoGigE*);

    void init();

    void deinit();

    void begin_acquisition();

    void end_acquisition();

    void setEnumValue(string, string);

    void setIntValue(string, int);

    void setFloatValue(string, float);

    void setBoolValue(string, bool);

    void setResolutionPixels(int width, int height);

    void adcBitDepth(gcstring bitDep);

    void setBufferSize(int numBuf);

    void setISPEnable();

    void setFREnable();

    void setPixelFormat(gcstring formatPic);

    void set_color(bool flag) { COLOR_ = flag; }

    double getFloatValueMax(string node_string);

    string getTLNodeStringValue(string node_string);

    string get_id();

    int get_frame_id();

    void make_master() {
        MASTER_ = true;
        ROS_DEBUG_STREAM("camera " << get_id() << " set as master");
    }

    bool is_master() { return MASTER_; }

    bool verifyBinning(int binningDesired);

    void calibrationParamsTest(int calibrationWidth, int calibrationHeight);

    void exposureTest();

    void targetGreyValueTest();

    void trigger();

    void configureImageEvent();

    void resetImageEvent();

    struct MatStamped {
        ros::Time stamp;
        cv::Mat image;
        int frame_id;
    };
    MatStamped display_;
    bool image_ready_;

    std::queue<MatStamped> qMatStampedCallback_;
    std::queue<MatStamped> qMatStampedWorker_;
    std::queue<MatStamped> qMatStampedDisplay_;
    std::mutex mutex_qc_;
    std::mutex mutex_qw_;

    StereoGigE *gige_;

    //*********************************
    // Auto gain and exposure

    float exposure_time_;
    float gain_abs_;
    bool pic_active_;
    bool use_autoexposure_;
    //*********************************
    // image timestamp in nanoseconds
    std::int64_t msg_time=0;

private:

    float convert_to_mat(ImagePtr, Mat &);

    float computeShutterAbsValue(uint32_t register_value);

    void OnImageEvent(ImagePtr);

    CameraPtr pCam_;
    int frameID_;
    int lastFrameID_;

    bool COLOR_;
    bool MASTER_;

    PIcontroller* pic_;


};

#endif
