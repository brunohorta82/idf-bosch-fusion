#include "MotionFusion.hpp"
#include "BoschInterfaceFusion.h"
#include <esp_log.h>
namespace Motion
{
    esp_err_t MotionFusionSensor::init()
    {
        return bmi2Init(this);
    }

    esp_err_t MotionFusionSensor::calibrate()
    {
        return bmi2Calibrate();
    }
    esp_err_t MotionFusionSensor::testMagnetometer()
    {
        return bmi2Init(this);
    }
    esp_err_t MotionFusionSensor::read()
    {
        esp_err_t result = bmi2Read();
        if (ESP_OK == result)
        {
            ESP_LOGD("MOTION", "Accelerometer Ms2 - X:%4.2f Y:%4.2f  Z:%4.2f", getAccelerometerMs2X(), getAccelerometerMs2Y(), getAccelerometerMs2Z());
            ESP_LOGD("MOTION", "Gyroscope DpsX    - X:%4.2f Y:%4.2f  Z:%4.2f", getGyroscopeDpsX(), getGyroscopeDpsY(), getGyroscopeDpsZ());
            ESP_LOGD("MOTION", "Gyroscope Raw    - X:%d Y:%d  Z:%d", getGyroscopeRawX(), getGyroscopeRawY(), getGyroscopeRawZ());
            ESP_LOGD("MOTION", "Gyroscope Rotation    - X:%f Y:%f  Z:%f", getGyroscopeRotationX(), getGyroscopeRotationY(), getGyroscopeRotationZ());
            ESP_LOGD("MOTION", "Magnetometer      - X:%d   Y:%d    Z:%d", getMagnetometerX(), getMagnetometerY(), getMagnetometerZ());
        }
        return result;
    }
    void MotionFusionSensor::applyCalibrationValues(float gyroscopeDpsX,
                                                    float gyroscopeDpsY,
                                                    float gyroscopeDpsZ)
    {
        if (abs(gyroscopeDpsX) > gyroXerror)
        {
            gyroXerror = abs(gyroscopeDpsX);
        }
        if (abs(gyroscopeDpsY) > gyroYerror)
        {
            gyroYerror = abs(gyroscopeDpsY);
        }
        if (abs(gyroscopeDpsZ) > gyroZerror)
        {
            gyroZerror = abs(gyroscopeDpsZ);
        }
    }
}