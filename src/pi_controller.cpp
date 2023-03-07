//
// Created by igor on 02.05.19..
//

#include <math.h>

#include "pi_controller.h"


PIcontroller::PIcontroller(bool color) {

  target_brightness_=100;
  integral_=10;//initial shutter time in milliseconds
  deadband_=5; //unit is pixel brightness[0..255]

  shutter_abs_=10000;
  gain_abs_=3;

  shutter_controller_on=true;
  dt=0.1;
  p_gain=1;
  i_gain=10;

  gain_controller_on=false;
  p2_gain=0.01;
  i2_gain=0.55;
  integral2_=0;

  shutter_abs_max_=20000;        //maximum exposure time in microseconds (20000 us is flicker-free on 50Hz street light)
  shutter_abs_min_=11;      //

  gain_abs_max_=48;
  gain_abs_min_=0;

  gain_threshold_shutter_=shutter_abs_max_;//allow shutter to first reach max-value, then apply gain
  //gain_threshold_shutter_=10;            //apply gain before reaching the max shutter

}

void PIcontroller::computeShutterAndGain(float &shutter_abs, float &gain_abs, const float mean) {

    double error = target_brightness_ - mean;// TARGET BRIGHTNESS = 100-overexp[%] !!!
    if (fabs(error) < deadband_) {
        error = 0;
    }

    if (shutter_controller_on) {
        integral_ += error * dt * i_gain;
        if (integral_ > shutter_abs_max_) integral_ = shutter_abs_max_;
        if (integral_ < -shutter_abs_max_) integral_ = -shutter_abs_max_;
        shutter_abs_ = integral_ + error * p_gain;
        if (shutter_abs_ > shutter_abs_max_) shutter_abs_ = shutter_abs_max_;
        if (shutter_abs_ < shutter_abs_min_) shutter_abs_ = shutter_abs_min_;
        shutter_abs = shutter_abs_;
    }

    if (shutter_abs_ >= gain_threshold_shutter_) {
        //std::cout<<"TURN ON!"<<std::endl;
        gain_controller_on = true;
        shutter_controller_on = false;
    }

    if (integral2_ < 0) {
        //std::cout<<"TURN OFF!"<<std::endl;
        gain_controller_on = false;
        shutter_controller_on = true;
    }

    /*else if (shutter_abs_<(gain_threshold_shutter_-2)) {
      gain_controller_on=false;//hysteresis turn-off
    }*/

    if (gain_controller_on) {
        //turn on gain controller
        if (gain_threshold_shutter_ == shutter_abs_max_) {
            //option 1 - allow maximum shutter time, than expand with gain...
            integral2_ += error * dt;
            gain_abs_ = integral2_ * i2_gain + error * p2_gain;
        } else {
            //option 2 - add gain together with increasing the shutter time
            float error_shutter = shutter_abs_ - gain_threshold_shutter_;
            gain_abs_ = error_shutter * (gain_abs_max_ / (shutter_abs_max_ - gain_threshold_shutter_));
        }
        if (gain_abs_ > gain_abs_max_) {
            gain_abs_ = gain_abs_max_;
        }
        if (gain_abs_ < gain_abs_min_) {
            gain_abs_ = gain_abs_min_;
        }
    } else {
        integral2_ = 0;
        gain_abs_ = 0;
    }

    gain_abs = gain_abs_;

    //std::cout<<"gain: "<<gain_abs_<<"dB, shutter: "<<shutter_abs_<<"us, mean: "<<mean<<" target: "<<target_brightness_<<std::endl;
}
