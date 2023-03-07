//
// Created by lamor on 12. 10. 2021..
//

#include "StereoGigE.h"

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;

#include <chrono>
typedef std::chrono::high_resolution_clock::time_point TimeVar;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

std::chrono::steady_clock::time_point t_start, t_end;

bool image_set=true;

StereoGigE::StereoGigE(YAML::Node &config) {

    SystemPtr system = System::GetInstance();
    PrintBuildInfo();
    //********************************************************************************************************************

    unsigned int numCameras;
    CameraList camList = system->GetCameras();
    numCameras = camList.GetSize();
    cout << "Number of cameras detected: " << numCameras << endl;

    if (numCameras == 0) {
        return;
    }

    for (int i = 0; i < numCameras; i++) {

        cameras_.push_back(std::make_shared<CameraWrapper>(camList.GetByIndex(i), color_, this));
        cout << "  -" << cameras_[i]->get_id()
             << " " << cameras_[i]->getTLNodeStringValue("DeviceModelName")
             << " " << cameras_[i]->getTLNodeStringValue("DeviceVersion") << endl;
    }


    //***********************************************************************
    readParameters(config);

    //****************************************************************************
    dataReady_worker_ = false;
    dataReady_display_ = false;
    stopProcessing_ = false;
    std::cout << "GigE constructor!" << std::endl;

//    worker_thread_ = std::thread(&StereoGigE::workerThread, this);
    display_thread_ = std::thread(&StereoGigE::displayThread, this);

    //****************************************************************************

    nh_ = ros::NodeHandle("~");
    image_transport::ImageTransport it(nh_);
#ifndef SPLIT_IMAGES
    pub_stereo_ = it.advertise("/stereo/image_raw", 10);
#else
    for(const auto& cam : cam_aliases_ ){
        image_transport::Publisher pub = it.advertise("/"+cam+"/image_raw",2);
        pub_mono_vector_.push_back(pub);
    }
#endif

    createCamInfoPublisher(nh_);

    if (master_set_) {
        for (auto cam: cameras_) {
            if (cam->get_id().compare(master_cam_id_) == 0) {
                cam->make_master();
            }
        }
    }

    initCameras();

    for(auto cam : cameras_) {
        cam->begin_acquisition();
    }

    if (use_lidar_trigger_) {
        sub_cam_trigger_ = nh_.subscribe(trigger_topic_, 10, &StereoGigE::triggerCallback, this);
    }

    cout<<"constructor end"<<endl;
    run();
};

StereoGigE::~StereoGigE() {
    cout << endl;
    cout << "StereoGigE destructor begin..." << endl;
    deinitCameras();

    for(auto cam : cameras_) {
        cam->end_acquisition();
    }

    //signal processing thread
    {
        std::lock_guard<std::mutex> lck(mutex_notify_worker_);
        dataReady_worker_ = true;
        stopProcessing_ = true;
    }
    condVar_worker_.notify_one();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    std::cout<<"StereoGigE destructor end!"<<std::endl;
};

void StereoGigE::PrintBuildInfo() {
    SystemPtr system =System::GetInstance();
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout<<"Spinnaker library version: "
        << spinnakerLibraryVersion.major << "."
        << spinnakerLibraryVersion.minor << "."
        << spinnakerLibraryVersion.type << "."
        << spinnakerLibraryVersion.build<<endl;

    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;
}

void StereoGigE::initCameras(){
//  initialization process taken from spinnaker_sdk_camera_driver
    cout<<"*** FLUSH SEQUENCE ***"<<endl;
    for(auto cam : cameras_)
        cam->init();

    for(auto &cam : cameras_)
        cam->begin_acquisition();
    sleep(init_delay_*0.5);

    for(auto &cam : cameras_)
        cam->end_acquisition();
    sleep(init_delay_*0.5);

    for(auto &cam : cameras_)
        cam->deinit();
    sleep(init_delay_*2.0);

    // Setting camera parameters
    for(auto cam : cameras_)
        cam->init();

    for(auto cam : cameras_) {

        cam->setEnumValue("TriggerMode", "Off");

        // USE PTP TIME
        cam->setBoolValue("GevIEEE1588",true);
        cam->setEnumValue("GevIEEE1588Mode","Auto");

        cam->setEnumValue("TransferControlMode", "Automatic");
//        cam->setBoolValue("GevSCPSDoNotFragment", false);
//        cam->setBoolValue("ChunkModeActive", false);

        cam->setIntValue("GevSCPD", 210000);
        cam->setIntValue("GevSCPSPacketSize", 9000);


        if (master_set_) {
            if (cam->is_master()) {
                cout << "setting master camera " << cam->get_id() << endl;
                cam->setEnumValue("AcquisitionMode", "Continuous");
                cam->setEnumValue("LineSelector", "Line1");
                cam->setEnumValue("LineMode", "Output");
                cam->setEnumValue("LineSelector", "Line2");
                cam->setBoolValue("V3_3Enable", true);
                cam->setEnumValue("TriggerOverlap", "ReadOut");
                cam->setEnumValue("TriggerSelector", "FrameStart");
                cam->setEnumValue("TriggerSource", "Software");
                cam->setEnumValue("TriggerMode", "On");

//                cam->setBoolValue("AcquisitionFrameRateEnable", true);
//                cam->setFloatValue("AcquisitionFrameRate", 10.0);

            } else {
                cout << "setting slave camera " << cam->get_id() << endl;
                cam->setEnumValue("TriggerSource", "Line3");
                cam->setEnumValue("TriggerOverlap", "ReadOut");
                cam->setEnumValue("TriggerMode", "On");
                cam->setFloatValue("TriggerDelay", 24.0);
            }

        } else {
            cam->setEnumValue("AcquisitionMode", "Continuous");
            cam->setEnumValue("TriggerSource", "Software");

        }

        cam->set_color(color_);


        cam->setResolutionPixels(image_width_,image_height_);

        if (ISP_enable_) {
            cam->setBoolValue("IspEnable", true);
        } else {
            cam->setBoolValue("IspEnable", false);
        }
//        cam->setBoolValue("IspEnable", false);
        cam->setBoolValue("BlackLevelClampingEnable", false);
        cam->setBoolValue("GammaEnable", false);
        cam->setEnumValue("BalanceWhiteAuto", "Off");
        cam->setEnumValue("BalanceRatioSelector", "Red");
        cam->setFloatValue("BalanceRatio", 1.8);
        cam->setEnumValue("BalanceRatioSelector", "Blue");
        cam->setFloatValue("BalanceRatio", 1.8);
        if (exposure_time_ > 0 && gain_ > 0)
            if (cam->is_master())
                cam->pic_active_ = true;
            else
                cam->pic_active_ = false;

        if (exposure_time_ < 0) {
            //use camera auto continuous exposure time
            use_autoexposure_ = false;
            cam->setEnumValue("ExposureAuto", "Continuous");
        } else {
            // use PIController
            use_autoexposure_ = true;
            cam->setEnumValue("ExposureAuto", "Off");
            cam->setEnumValue("ExposureMode", "Timed");
            cam->setFloatValue("ExposureTime", exposure_time_);
        }

        if (gain_ < 0) {
            //use auto continuous exposure time
            cam->setEnumValue("GainAuto", "Continuous");
        } else {
            cam->setEnumValue("GainAuto", "Off");
            cam->setFloatValue("Gain", gain_);
        }
    }

    sleep(init_delay_);
    // Configure image events

    for(auto cam : cameras_){
        cam->configureImageEvent();
    }


    cout<<"Flush sequence done."<<endl;
}

void StereoGigE::deinitCameras(){
    for(auto &cam : cameras_)
        cam->end_acquisition();
    for(auto &cam : cameras_)
        cam->resetImageEvent();
    for(auto &cam : cameras_)
        cam->deinit();

}

void StereoGigE::run(){

    if(!use_lidar_trigger_)
    {
        startTriggering();
    }

    while( ros::ok() ) {
        if(!use_lidar_trigger_) {
            displayQueues();
        }
//        ros::spinOnce();
    }

    for(auto cam : cameras_) {
        cam->end_acquisition();
    }

    if(!use_lidar_trigger_) {
        stopTriggering();
    }
    ros::shutdown();
}
//
//void StereoGigE::stampQueues() {
////    {
//////        //wait for the signal...
////        std::unique_lock<std::mutex> lck(mutex_notify_worker_);
////        condVar_worker_.wait(lck, std::bind(&StereoGigE::dataReady_worker_, this));
////        if (stopProcessing_) {
////            return;
////        }
////        dataReady_worker_ = false;
////    }
////
////
////
////    for (auto cam: cameras_) {
////        std::lock_guard<std::mutex> lck(cam->mutex_qc_);
////        while (cam->qMatStampedCallback_.size() > 0) {
////            if (cam->qMatStampedCallback_.size() > 1) {
////                ROS_WARN_STREAM("delay on " << cam->get_id() << "queue size= " << cam->qMatStampedCallback_.size());
////            }
////            cam->qMatStampedWorker_.push(cam->qMatStampedCallback_.front());
////            cam->qMatStampedCallback_.pop();
////        }
////    }
////
////    bool all_present=true;
////    while (all_present) {
////        all_present = true;
////        for (auto cam: cameras_) {
////            if (cam->qMatStampedWorker_.size() == 0) {
////                all_present = false;
////            }
////        }
////
////        if (all_present) {
//////            t_end = std::chrono::steady_clock::now();
//////            std::cout<<"time difference: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()<<std::endl;
////
////            // front-> oldest element, back-> newest element
////            ros::Time oldest_time = cameras_[0]->qMatStampedWorker_.front().stamp;
////            ros::Time newest_time = cameras_[0]->qMatStampedWorker_.back().stamp;
////
////
////            std::shared_ptr<CameraWrapper> oldest_cam;
////            ros::Duration delta;
////
////            for (auto cam: cameras_) {
////
//////                std::cout<<"cam: "<<cam->get_id()<<" q size: "<<cam->qMatStampedWorker_.size()<<std::endl;
////
////                if (cam->qMatStampedWorker_.front().stamp < oldest_time) {
////                    oldest_time = cam->qMatStampedWorker_.front().stamp;
////                    oldest_cam = cam;
////                }
////                if (cam->qMatStampedWorker_.back().stamp > newest_time) {
////                    newest_time = cam->qMatStampedWorker_.back().stamp;
////                }
////            }
////
////            delta = newest_time - oldest_time;
////            //TODO: is synchronization needed if they all use the same time?
////            if (abs(delta.toSec()) < 100000) {
////                //consider images synced!
////                {
////                    std::lock_guard<std::mutex> lck(mutex_display_);
////                    for (auto cam : cameras_) {
////                        cam->qMatStampedDisplay_.push(cam->qMatStampedWorker_.front());
////                        cam->qMatStampedWorker_.pop();
////                    }
////                }
////
////                {
////                    std::lock_guard<std::mutex> lck(mutex_notify_display_);
////                    dataReady_display_ = true;
////                }
////                condVar_display_.notify_one();
////
////            } else {
////                ROS_WARN_STREAM("Images out of sync. Delta " << delta<<". Removing oldest data"<<" cam: "<<oldest_cam->get_id()<<" size before poping: "<<oldest_cam->qMatStampedWorker_.size());
//////                std::cout<<"cam: "<<oldest_cam->getCamID()<<" size before poping: "<<oldest_cam->qMatStampedWorker_.size()<<std::endl;
////                oldest_cam->qMatStampedWorker_.pop();
////            }
////        }
////    }
//}

void StereoGigE::displayQueues() {

    {
        //wait for the signal...
        std::unique_lock<std::mutex> lck(mutex_notify_display_);
        condVar_display_.wait(lck, std::bind(&StereoGigE::dataReady_display_, this));
        if (stopProcessing_) {
            return;
        }
        dataReady_display_ = false;
    }
    bool displaying = true;

//    image_set =true;
//
//    t_end = std::chrono::steady_clock::now();
//    auto duration = duration_cast<std::chrono::microseconds>(t_end - t_start);
//    cout << "Trig2img: " << duration.count() << " us" << endl;
//    t_start = std::chrono::steady_clock::now();


    while (displaying) {
        //safely take front images from the display queue
        {
            std::unique_lock<std::mutex> lck(mutex_display_);
            for (auto cam: cameras_) {
                std::lock_guard<std::mutex> lck(cam->mutex_qw_);

                if (cam->qMatStampedDisplay_.size() > 0) {
                    cam->display_ = cam->qMatStampedDisplay_.front();
                    cam->image_ready_=true;
                    cam->qMatStampedDisplay_.pop();
                    if (cam->qMatStampedDisplay_.size() == 0) {
                        displaying = false;
                    }
                } else {
                    //queue empty - it is possible that this thread emptied the queue in the while loop,
                    //but the notification is not cleared and this function is called in the next iteration
                    //with the empty queue...
                }
            }
        }

        static int dummy_cnt = 0;
        // for some reason SEG Fault happens in first cycle !?!
        if (dummy_cnt<3) {
            dummy_cnt++;
            continue;
        }
#ifndef SPLIT_IMAGES
        int cols = 0;
        int rows_total = 0;
        for (const auto &cam : cameras_) {
            if (cols == 0) {
                cols = cam->display_.image.cols;
                rows_total = cam->display_.image.rows;
            } else {
                rows_total += cam->display_.image.rows;
            }
        }

        if (cols > 0) {
            std_msgs::Header header = std_msgs::Header();
            cv::Mat integrated(rows_total, cols, CV_8UC3);
            uchar *data = integrated.data;
            for (const auto &cam: cameras_) {
                header.stamp = cam->display_.stamp;
                memcpy(data, cam->display_.image.data, cam->display_.image.cols * cam->display_.image.rows * 3);
                data += cam->display_.image.cols * cam->display_.image.rows * 3;
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", integrated).toImageMsg();
            pub_stereo_.publish(msg);
            for(int i=0; i<cam_aliases_.size(); i++){
                cam_info_msgs_[i]->header.stamp=msg->header.stamp;
                pub_cam_info_[i].publish(cam_info_msgs_[i]);
            }

        } else {
            std::cout << " ??? cols= " << cols << std::endl;
        }

#else
        int i = 0;
        for (auto cam: cameras_) {
            if(!cam->image_ready_){
                i++;
                continue;
            }
            cam->image_ready_=false;
            std_msgs::Header header = std_msgs::Header();
            header.stamp = cam->display_.stamp;
            sensor_msgs::ImagePtr msg;
            if(this->color_) {
//                cv::imshow("tmp", cam->display_.image);
//                cv::waitKey(0);
                msg = cv_bridge::CvImage(header, "bgr8", cam->display_.image).toImageMsg();
            } else {
                msg = cv_bridge::CvImage(header, "mono8", cam->display_.image).toImageMsg();
            }
            cam_info_msgs_[i]->header.stamp = msg->header.stamp;
            pub_mono_vector_[i].publish(msg);
            pub_cam_info_[i].publish(cam_info_msgs_[i]);
            i++;
        }
#endif
    }
}

void StereoGigE::displayThread() {
    while(!stopProcessing_){
        displayQueues();
    }
    cout<<"display thread finished"<<endl;
}

int StereoGigE::findCamera(std::string camera) {
    for(int i=0; i<cameras_.size(); i++){
        if(camera.compare(cameras_[i]->get_id())==0)
            return i;
    }
    return -1;
}

void StereoGigE::readParameters(YAML::Node &config) {
    ROS_INFO_STREAM("*** PARAMETER SETTINGS ***");

    YAML::Node cam_ids = yamlSubNodeAbort(config, "cam_ids");
    for (auto cam: cam_ids) {
        int idx = findCamera(cam.Scalar());
        if (idx < 0) {
            ROS_ERROR_STREAM("Camera with id " << cam.Scalar() << " not connected!");
            exit(-1);
        }
    }

    // sort ids in cameras_ to avoid possible errors later
    std::vector<std::shared_ptr<CameraWrapper>> tmp_cams;
    for(auto i : cam_ids){
        for(auto j : cameras_){
            if (i.Scalar().compare(j->get_id())==0)
                tmp_cams.push_back(j);
        }
    }
    cameras_=tmp_cams;


    YAML::Node cam_aliases = yamlSubNodeAbort(config, "cam_aliases");
    for (auto cam: cam_aliases) {
        cam_aliases_.push_back(cam.Scalar());
    }

    YAML::Node common = yamlSubNodeAbort(config, "common");
    yamlRead<string>(common, "master_cam", master_cam_id_, "None");
    if (master_cam_id_.compare("None") == 0) {
        master_set_ = false;
        ROS_WARN("Master not set! Using separate triggering!");
    } else{
        master_set_ = true;
        master_idx_ = findCamera(master_cam_id_);
        if(master_idx_<0) {
            ROS_ERROR_STREAM("Master camera not detected!");
            exit(-1);
        }
    }

    yamlRead<float>(common, "init_delay", init_delay_, 1.0);
    yamlRead<bool>(common, "color", color_, false);
    yamlRead<bool>(common, "ISP_enable", ISP_enable_, false);
    yamlRead<int>(common, "soft_framerate", soft_framerate_, 5);
    if(soft_framerate_==5) ROS_INFO_STREAM("using default framerate: "<<soft_framerate_);
    yamlRead<float>(common, "exposure_time", exposure_time_, -1);
    if(exposure_time_<0) ROS_WARN("Using auto exposure time");
    yamlRead<float>(common, "gain", gain_, -1.0);
    if(gain_<0) ROS_WARN("Using auto gain");
    yamlRead<bool>(common, "lidar_trigger", use_lidar_trigger_, false);
    yamlRead<string>(common, "trigger_topic", trigger_topic_, "");
    if(trigger_topic_.empty() && use_lidar_trigger_) {
        ROS_ERROR_STREAM("Trigger topic not given!");
        return;
    } else{
        if (use_lidar_trigger_)
            ROS_INFO_STREAM("Using trigger topic: "<<trigger_topic_);
    }

    yamlRead<string>(config, "distortion_model", distortion_model_, "plumb_bob");
    yamlRead<int>(config, "image_height", image_height_, -1);
    yamlRead<int>(config, "image_width", image_width_, -1);
    if(image_height_==-1 || image_width_==-1){
        ROS_ERROR_STREAM("Image size set to "<<image_width_<<"x"<<image_height_);
        exit(-1);
    }
    // assuming 8bit data
    if (!color_){
        image_stride_ = image_width_;
    } else{
        image_stride_ = 3*image_width_;
    }

    yamlRead<int>(config, "binning", binning_, 1);
    if (binning_>1) ROS_WARN_STREAM("binning not implemented yet, using binning value 1");


    YAML::Node dist = yamlSubNodeAbort(config, "distortion_coeffs");
    for(auto coef : dist) {
        std::vector<double> dist_coef;
        for (auto el: coef) {
            dist_coef.push_back(stod(el.Scalar()));
        }
        distortion_coeff_vec_.push_back(dist_coef);
    }
    YAML::Node intrinsics = yamlSubNodeAbort(config, "intrinsic_coeffs");
    for(auto coef : intrinsics) {
        std::vector<double> intrinsics_coef;
        for (auto el: coef) {
            intrinsics_coef.push_back(stod(el.Scalar()));
        }
        intrinsic_coeff_vec_.push_back(intrinsics_coef);
    }
    YAML::Node rect = yamlSubNodeAbort(config, "rectification_coeffs");
    for(auto coef : rect) {
        std::vector<double> rect_coef;
        for (auto el: coef) {
            rect_coef.push_back(stod(el.Scalar()));
        }
        rect_coeff_vec_.push_back(rect_coef);
    }
    YAML::Node proj = yamlSubNodeAbort(config, "projection_coeffs");
    for(auto coef : proj) {
        std::vector<double> proj_coef;
        for (auto el: coef) {
            proj_coef.push_back(stod(el.Scalar()));
        }
        proj_coeff_vec_.push_back(proj_coef);
    }
    YAML::Node flip_v = yamlSubNodeAbort(config, "flip_vertical");
    for(auto cam : flip_v) {
        flip_vertical_vec_.push_back(cam.Scalar().compare("true")==0);
    }
    YAML::Node flip_h = yamlSubNodeAbort(config, "flip_horizontal");
    for(auto cam : flip_h) {
        flip_horizontal_vec_.push_back(cam.Scalar().compare("true")==0);
    }
}

void StereoGigE::createCamInfoPublisher(ros::NodeHandle & nh) {
    // create messages and publisher
    for (int i = 0; i < cam_aliases_.size(); i++) {
        sensor_msgs::CameraInfoPtr ci_msg(new sensor_msgs::CameraInfo());

        ci_msg->height = image_height_;
        ci_msg->width = image_width_;
        // distortion
        ci_msg->distortion_model = distortion_model_;
        // binning
        ci_msg->binning_x = binning_;
        ci_msg->binning_y = binning_;
        ci_msg->D = distortion_coeff_vec_[i];
        // intrinsic coefficients
        for (int count = 0; count < intrinsic_coeff_vec_[i].size(); count++) {
            ci_msg->K[count] = intrinsic_coeff_vec_[i][count];
        }
        // Rectification matrix
        if (!rect_coeff_vec_.empty())
            ci_msg->R = {
                    rect_coeff_vec_[i][0], rect_coeff_vec_[i][1],
                    rect_coeff_vec_[i][2], rect_coeff_vec_[i][3],
                    rect_coeff_vec_[i][4], rect_coeff_vec_[i][5],
                    rect_coeff_vec_[i][6], rect_coeff_vec_[i][7],
                    rect_coeff_vec_[i][8]};
        // Projection/camera matrix
        if (!proj_coeff_vec_.empty()) {
            ci_msg->P = {
                    proj_coeff_vec_[i][0], proj_coeff_vec_[i][1],
                    proj_coeff_vec_[i][2], proj_coeff_vec_[i][3],
                    proj_coeff_vec_[i][4], proj_coeff_vec_[i][5],
                    proj_coeff_vec_[i][6], proj_coeff_vec_[i][7],
                    proj_coeff_vec_[i][8], proj_coeff_vec_[i][9],
                    proj_coeff_vec_[i][10], proj_coeff_vec_[i][11]};
        }
        cam_info_msgs_.push_back(ci_msg);
        ros::Publisher pub_info = nh.advertise<sensor_msgs::CameraInfo>("/"+cam_aliases_[i]+"/camera_info", 1);
        pub_cam_info_.push_back(pub_info);
    }
}

void StereoGigE::triggerCallback(const std_msgs::Time::ConstPtr &msg){
    if (master_set_)
        cameras_[master_idx_]->trigger();
    else {
        for (auto cam: cameras_)
            cam->trigger();
    }
}

void StereoGigE::triggerThread() {
    bool synchronize = true;
    int counter = 0;
    int sync_rate = 5;
    double delta_t = 0;
    double delay_acc = 0;
    auto start = std::chrono::system_clock::now();
    while (trigger_flag_) {
        for (auto cam: cameras_) {
            if (master_set_) {
                if (cam->is_master()) {
                    counter++;
                    if (counter >= sync_rate) {
                        synchronize = true;
                        counter = 0;
                    }
                    if (synchronize) {
                        auto end = std::chrono::system_clock::now();
                        std::chrono::duration<double> duration = end - start;

                        double dptp = cam->msg_time/100000.0 - floor(cam->msg_time/100000.0);
                        dptp /=10;
                        delta_t = abs(dptp - 0.09);

                        if (delta_t>0.007){
                            if (0.0905>dptp)
                                delay_acc -=0.001;
                            else
                                delay_acc +=0.001;
                        } else
                        if (delta_t>0.0003){
                            if (0.0905>dptp)
                                delay_acc -=0.0002;
                            else
                                delay_acc +=0.0002;
                        }
                        //std::cout<<"SYNC! delta_t: "<<delta_t<<" dptp: "<<dptp<<" delay acc "<<delay_acc<<std::endl;
                        synchronize = false;
                    }

                    bool trigger = false;
                    // watch sys time and trigger when sys_clock - delta_t is x.y90
                    std::chrono::system_clock::time_point curr = std::chrono::system_clock::now();
                    double ptp;

                    auto duration = curr - start;
                    auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(duration);

                    while (!trigger) {
                        std::this_thread::sleep_for(std::chrono::microseconds (100));
                        curr = std::chrono::system_clock::now();
                        duration = curr - start;
                        microsec = std::chrono::duration_cast<std::chrono::microseconds>(duration);
                        ptp = microsec.count() / 100000.0 + delay_acc*10;// + delta_t*10;
                        ptp = ptp - floor(ptp);
                        ptp /= 10;
                        auto dptp = cam->msg_time/100000.0 - floor(cam->msg_time/100000.0);
                        dptp /=10;
                        // We want image stamped at x.x9000, but there is 20ms gap between trigger and image
                        if (abs(ptp - 0.09) < 0.0001) {
                            //std::cout << "ptp " << ptp <<" real ptp " << dptp <<" delay " << delay_acc<<std::endl;
                            trigger = true;
                        }
                    }

//                    t_end = std::chrono::steady_clock::now();
//                    auto dur = duration_cast<std::chrono::microseconds>(t_end - t_start);
//                    cout << "trg2trg: " << dur.count() << " us" << endl;
                    cam->trigger();
//                    if(image_set) {
//                        t_start = std::chrono::steady_clock::now();
//                        image_set=false;
//                    }
                }
            } else {
                cam->trigger();
            }
        }
    }
}


void StereoGigE::startTriggering() {
    // to sync with lidar, trigger_flag_ is set in runSoftTrigger function!!
    trigger_flag_=true;
    trigger_thread_=thread(&StereoGigE::triggerThread,this);
}

void StereoGigE::stopTriggering() {
    trigger_flag_=false;
    if (trigger_thread_.joinable()) {
        trigger_thread_.join();
    }
}
