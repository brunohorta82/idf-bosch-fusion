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
        uint64_t sensorTimeMs{0};
        int16_t accelerometerRawX{0};
        int16_t accelerometerRawY{0};
        int16_t accelerometerRawZ{0};
        float accelerometerMpsX{0};
        float accelerometerMpsY{0};
        float accelerometerMpsZ{0};

        float gyroXerror{0.0};
        float gyroYerror{0.0};
        float gyroZerror{0.0};
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
        void gyroscopeDeadReckoning(float gyroscopeDpsX,
                                    float gyroscopeDpsY,
                                    float gyroscopeDpsZ,
                                    uint64_t latestSensorTime)
        {
            double elapsedTime = (latestSensorTime - this->sensorTimeMs) / 1000.0;
            this->gyroscopeDpsX = gyroscopeDpsX;
            this->gyroscopeDpsY = gyroscopeDpsY;
            this->gyroscopeDpsZ = gyroscopeDpsZ;
            this->sensorTimeMs = latestSensorTime;
            if (abs(gyroscopeDpsX) > gyroXerror)
            { // current angle (º) = last angle (º) + angular velocity (º/s) * time(s)
                gyroscopeRotationX += gyroscopeDpsX * elapsedTime;
            }
            if (abs(gyroscopeDpsY) > gyroYerror)
            {
                gyroscopeRotationY += gyroscopeDpsY * elapsedTime;
            }
            if (abs(gyroscopeDpsZ) > gyroZerror)
            {
                gyroscopeRotationZ += gyroscopeDpsZ * elapsedTime;
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
        constexpr uint64_t getLatestGyroscopeReckoning() { return sensorTimeMs; }
        constexpr float getAccelerometerMs2X() { return accelerometerMpsX; }
        constexpr float getAccelerometerMs2Y() { return accelerometerMpsY; }
        constexpr float getAccelerometerMs2Z() { return accelerometerMpsZ; }

        constexpr float getGyroscopeDpsX() { return gyroscopeDpsX; }
        constexpr float getGyroscopeDpsY() { return gyroscopeDpsY; }
        constexpr float getGyroscopeDpsZ() { return gyroscopeDpsZ; }
        constexpr float getGyroscopeRotationX() { return gyroscopeRotationX; }
        constexpr float getGyroscopeRotationY() { return gyroscopeRotationY; }
        constexpr float getGyroscopeRotationZ() { return gyroscopeRotationZ; }

        constexpr int16_t getMagnetometerX() { return magnetometerX; }
        constexpr int16_t getMagnetometerY() { return magnetometerY; }
        constexpr int16_t getMagnetometerZ() { return magnetometerZ; }
        shared_ptr<I2CMaster> getBus() { return this->i2cMasterBus; }
        esp_err_t init();
        esp_err_t calibrate();
        esp_err_t read();
        void applyCalibrationValues(float gyroscopeDpsX,
                                    float gyroscopeDpsY,
                                    float gyroscopeDpsZ);
    };
}