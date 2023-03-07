//
// Created by lamor on 12. 10. 2021..
//
#ifndef SRC_STEREOGIGE_H
#define SRC_STEREOGIGE_H

#define SPLIT_IMAGES

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <mutex>
#include <condition_variable>

#include "camera.h"
#include "stereo_camera_driver/yaml_reader.h"
#include "pi_controller.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class CameraWrapper;


class StereoGigE {
public:
    ~StereoGigE();

    StereoGigE(YAML::Node &);

    void PrintBuildInfo();

    //*********************************************************************************
    float exposure_time_;
    float gain_;
    bool use_autoexposure_;
    //*********************************************************************************

    void stampQueues();
    void displayQueues();

    std::mutex mutex_notify_worker_;
    std::condition_variable condVar_worker_;
    bool dataReady_worker_;

    ros::Duration delta_,delta_max_;

    std::mutex mutex_display_;
    std::mutex mutex_notify_display_;
    std::condition_variable condVar_display_;
    bool dataReady_display_;


    int image_width_;
    int image_height_;
    int image_stride_;

private:

    void initCameras();

    void deinitCameras();

    void run();

    void readParameters(YAML::Node &);

    void triggerCallback(const std_msgs::Time::ConstPtr &msg);

    void startTriggering();

    void stopTriggering();

    int findCamera(std::string);

    void createCamInfoPublisher(ros::NodeHandle &);

    std::vector<ros::Publisher> pub_cam_info_;
    std::vector<std::shared_ptr<CameraWrapper>> cameras_;
#ifndef SPLIT_IMAGES
    image_transport::Publisher pub_stereo_;
#else
    std::vector<image_transport::Publisher> pub_mono_vector_;
#endif
    ros::Subscriber sub_cam_trigger_;

    void workerThread();
    std::thread worker_thread_;

    void displayThread();
    std::thread display_thread_;

    void triggerThread();
    std::thread trigger_thread_;
    bool trigger_flag_;

    bool stopProcessing_;
    bool master_set_;
    ros::NodeHandle nh_;
//*********************************************
// variables set through the yaml file
    vector<string> cam_aliases_;
    string master_cam_id_;
    vector<vector<double>> intrinsic_coeff_vec_;
    vector<vector<double>> distortion_coeff_vec_;
    vector<vector<double>> rect_coeff_vec_;
    vector<vector<double>> proj_coeff_vec_;
    vector<bool> flip_horizontal_vec_;
    vector<bool> flip_vertical_vec_;
    vector<sensor_msgs::CameraInfoPtr> cam_info_msgs_;
    float init_delay_;
    bool ISP_enable_;
    int soft_framerate_;
    int binning_;
    bool color_;
    // camera is triggered when lidar pointcloud message is published
    bool use_lidar_trigger_;
    string trigger_topic_;
    string distortion_model_;

    int master_idx_;
};


#endif //SRC_STEREOGIGE_H
