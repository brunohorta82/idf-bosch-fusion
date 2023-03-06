#pragma once

#include "MotionFusion.hpp"
esp_err_t bmi2Init(Motion::MotionFusionSensor *motionSensor);
esp_err_t bmi2Read();
esp_err_t bmi2Calibrate();
