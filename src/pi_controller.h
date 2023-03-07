//
// Created by igor on 02.05.19..
//
#include "stereo_camera_driver/std_include.h"

#ifndef PROJECT_PI_CONTROLLER_H
#define PROJECT_PI_CONTROLLER_H


class PIcontroller {
public:
  PIcontroller(bool color);
  void computeShutterAndGain(float &shutter_abs, float &gain_abs, const float mean);
private:

  float shutter_abs_;
  float gain_abs_;

  float integral_;
  float deadband_;
  float target_brightness_;

  bool  shutter_controller_on;
  float dt;
  float p_gain;
  float i_gain;

  bool  gain_controller_on;
  float p2_gain;
  float i2_gain;
  float integral2_;

  float shutter_abs_max_;
  float shutter_abs_min_;

  float gain_abs_max_;
  float gain_abs_min_;

  float gain_threshold_shutter_;

};

#endif //PROJECT_PI_CONTROLLER_H
