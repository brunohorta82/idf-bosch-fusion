#pragma once
#include "I2C.hpp"
using namespace std;
using namespace Components;
#define GRAVITY_EARTH (9.80665f)

namespace Motion
{
    class MotionFusionSensor
    {
    private:
        float gyroXerror{0.0};
        float gyroYerror{0.0};
        float gyroZerror{0.0};
        int16_t accelerometerRawX{0};
        int16_t accelerometerRawY{0};
        int16_t accelerometerRawZ{0};
        float accelerometerMpsX{0};
        float accelerometerMpsY{0};
        float accelerometerMpsZ{0};
        int16_t gyroscopeRawX{0};
        int16_t gyroscopeRawY{0};
        int16_t gyroscopeRawZ{0};
        float gyroscopeDpsX{0};
        float gyroscopeDpsY{0};
        float gyroscopeDpsZ{0};
        float gyroscopeRotationX{0};
        float gyroscopeRotationY{0};
        float gyroscopeRotationZ{0};
        int16_t magnetometerX{0};
        int16_t magnetometerY{0};
        int16_t magnetometerZ{0};

        shared_ptr<I2CMaster> i2cMasterBus;

    public:
        MotionFusionSensor(shared_ptr<I2CMaster> i2cMasterBus)
        {
            this->i2cMasterBus = i2cMasterBus;
        };
        void setAccelerometerResult(int16_t accelerometerRawX,
                                    int16_t accelerometerRawY,
                                    int16_t accelerometerRawZ,
                                    float accelerometerMpsX,
                                    float accelerometerMpsY,
                                    float accelerometerMpsZ)
        {
            this->accelerometerRawX = accelerometerRawX;
            this->accelerometerRawY = accelerometerRawY;
            this->accelerometerRawY = accelerometerRawY;
            this->accelerometerMpsX = accelerometerMpsX;
            this->accelerometerMpsY = accelerometerMpsY;
            this->accelerometerMpsZ = accelerometerMpsZ;
        };
        void setGyroscopeResult(int16_t gyroscopeRawX,
                                int16_t gyroscopeRawY,
                                int16_t gyroscopeRawZ,
                                float gyroscopeDpsX,
                                float gyroscopeDpsY,
                                float gyroscopeDpsZ)
        {
            this->gyroscopeRawX = gyroscopeRawX;
            this->gyroscopeRawY = gyroscopeRawY;
            this->gyroscopeRawZ = gyroscopeRawZ;
            this->gyroscopeDpsX = gyroscopeDpsX;
            this->gyroscopeDpsY = gyroscopeDpsY;
            this->gyroscopeDpsZ = gyroscopeDpsZ;

            if (abs(gyroscopeDpsX) > gyroXerror)
            { // current angle (º) = last angle (º) + angular velocity (º/s) * time(s)

                gyroscopeRotationX = gyroscopeRotationX + gyroscopeDpsX * 0.01;
            }
            if (abs(gyroscopeDpsY) > gyroYerror)
            {
                gyroscopeRotationY = gyroscopeRotationY + gyroscopeDpsY * 0.01;
            }
            if (abs(gyroscopeDpsZ) > gyroZerror)
            {
                gyroscopeRotationZ = gyroscopeRotationZ + gyroscopeDpsZ * 0.01;
            }
        };
        void setMagnetometerResult(int16_t magnetometerX,
                                   int16_t magnetometerY,
                                   int16_t magnetometerZ)
        {
            this->magnetometerX = magnetometerX;
            this->magnetometerY = magnetometerY;
            this->magnetometerZ = magnetometerZ;
        }
        constexpr float getAccelerometerMs2X() { return accelerometerMpsX; }
        constexpr float getAccelerometerMs2Y() { return accelerometerMpsY; }
        constexpr float getAccelerometerMs2Z() { return accelerometerMpsZ; }
        constexpr float getGyroscopeDpsX() { return gyroscopeDpsX; }
        constexpr float getGyroscopeDpsY() { return gyroscopeDpsY; }
        constexpr float getGyroscopeDpsZ() { return gyroscopeDpsZ; }

        constexpr int16_t getGyroscopeRawX() { return gyroscopeRawX; }
        constexpr int16_t getGyroscopeRawY() { return gyroscopeRawY; }
        constexpr int16_t getGyroscopeRawZ() { return gyroscopeRawZ; }

        constexpr float getGyroscopeRotationX() { return gyroscopeRotationX; }
        constexpr float getGyroscopeRotationY() { return gyroscopeRotationY; }
        constexpr float getGyroscopeRotationZ() { return gyroscopeRotationZ; }
        constexpr int16_t getMagnetometerX() { return magnetometerX; }
        constexpr int16_t getMagnetometerY() { return magnetometerY; }
        constexpr int16_t getMagnetometerZ() { return magnetometerZ; }
        shared_ptr<I2CMaster> getBus()
        {
            return this->i2cMasterBus;
        }
        esp_err_t init();
        esp_err_t calibrate();
        esp_err_t read();
        esp_err_t testMagnetometer();
        void applyCalibrationValues(float gyroscopeDpsX,
                                    float gyroscopeDpsY,
                                    float gyroscopeDpsZ);
    };
}