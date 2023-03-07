//
// Created by lamor on 30. 09. 2021..
//

//TODO: why do I need to skip first image? (search for "for some reason SEG Fault happens in first cycle !?!" in StereoGigE.cpp)
//TODO: Add PIcontroller option to yaml file

#include "StereoGigE.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_GigE");

    YAML::Node config;
    try{
        config = YAML::LoadFile("/home/lamor/catkin_ws/src/stereo_camera_driver/params/test_params.yaml");
    }
    catch (...) {
        std::cout<<"Config file format wrong! Please check the format(e.g. indentation) " << std::endl;
        return -1;
    }

    StereoGigE stereo(config);
    std::cout << "ros::ok() exit!" << std::endl;
    return 0;
}
