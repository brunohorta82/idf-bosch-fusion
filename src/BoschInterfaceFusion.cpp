
#include "BoschInterfaceFusion.h"
#include "freertos/Freertos.h"
#include "freertos/task.h"
#include "../driver/bmi2.h"
#include "../driver/bmi270.h"
#include "../driver/bmi2_defs.h"
#include "../driver/bmm150.h"
#include "../driver/bmm150_defs.h"
#include <esp_log.h>
#include <memory>
#include <cstring>
#define TAG_MOTION "MOTION"
using namespace Components;
using namespace std;

#define TAG_COMMON "I2C COMMON"
#define READ_WRITE_LEN UINT8_C(46)
#define ACCEL UINT8_C(0x00)
#define GYRO UINT8_C(0x01)
#define AUX UINT8_C(0x02)

float dps{2000};
float gRange{2};
struct bmi2_dev bmi270Device;
struct bmm150_dev bmm150Device;
Motion::MotionFusionSensor *_motionSensor;

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float lsbToMps(int16_t val)
{
    float half_scale = ((float)(1 << bmi270Device.resolution) / 2.0f);

    return (GRAVITY_EARTH * val * gRange) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float lsbToDps(int16_t val)
{
    float half_scale = ((float)(1 << bmi270Device.resolution) / 2.0f);

    return (dps / ((half_scale))) * (val);
}
BMI2_INTF_RETURN_TYPE aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;

    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, length, &bmi270Device);

    return rslt;
}

BMI2_INTF_RETURN_TYPE aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;

    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, length, &bmi270Device);

    return rslt;
}
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

    try
    {
        uint8_t dev_addr = *(uint8_t *)intf_ptr;
        _motionSensor->getBus()->syncWrite(I2CAddress(dev_addr), {reg_addr});
        vector<uint8_t> data = _motionSensor->getBus()->syncRead(I2CAddress(dev_addr), len);
        memcpy(reg_data, data.data(), len);
        return ESP_OK;
    }
    catch (const I2CException &e)
    {
        ESP_LOGI("BMM", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
        ESP_LOGI("BMM", "Couldn't read sensor!");
        return ESP_FAIL;
    }
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    try
    {
        uint8_t dev_addr = *(uint8_t *)intf_ptr;
        vector<uint8_t> data;
        data.push_back(reg_addr);
        for (int i = 0; i < len; i++)
        {
            data.push_back(reg_data[i]);
        }
        _motionSensor->getBus()->syncWrite(I2CAddress(dev_addr), data);
        return ESP_OK;
    }
    catch (const I2CException &e)
    {
        ESP_LOGI("TAG", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
        ESP_LOGI("TAG", "Couldn't write sensor!");
        return ESP_FAIL;
    }
}

void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}

void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
    case BMI2_OK:

        /* Do nothing */
        break;

    case BMI2_W_FIFO_EMPTY:
        ESP_LOGE(TAG_COMMON, "Warning [%d] : FIFO empty", rslt);
        break;
    case BMI2_W_PARTIAL_READ:
        ESP_LOGE(TAG_COMMON, "Warning [%d] : FIFO partial read", rslt);
        break;
    case BMI2_E_NULL_PTR:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer,"
                 " which has been initialized to NULL.",
                 rslt);
        break;

    case BMI2_E_COM_FAIL:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due "
                 "to power failure during communication",
                 rslt);
        break;

    case BMI2_E_DEV_NOT_FOUND:
        ESP_LOGE(TAG_COMMON, "Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read",
                 rslt);
        break;

    case BMI2_E_INVALID_SENSOR:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the "
                 "available one",
                 rslt);
        break;

    case BMI2_E_SELF_TEST_FAIL:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is "
                 "not satisfied",
                 rslt);
        break;

    case BMI2_E_INVALID_INT_PIN:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins "
                 "apart from INT1 and INT2",
                 rslt);
        break;

    case BMI2_E_OUT_OF_RANGE:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from "
                 "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC",
                 rslt);
        break;

    case BMI2_E_ACC_INVALID_CFG:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration"
                 " register which could be one among range, BW or filter performance in reg address 0x40",
                 rslt);
        break;

    case BMI2_E_GYRO_INVALID_CFG:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration"
                 "register which could be one among range, BW or filter performance in reg address 0x42",
                 rslt);
        break;

    case BMI2_E_ACC_GYR_INVALID_CFG:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro"
                 " configuration registers which could be one among range, BW or filter performance in reg address 0x40 "
                 "and 0x42",
                 rslt);
        break;

    case BMI2_E_CONFIG_LOAD:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration "
                 "into the sensor",
                 rslt);
        break;

    case BMI2_E_INVALID_PAGE:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration "
                 "from selected page",
                 rslt);
        break;

    case BMI2_E_SET_APS_FAIL:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration "
                 "register",
                 rslt);
        break;

    case BMI2_E_AUX_INVALID_CFG:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not "
                 "enabled properly",
                 rslt);
        break;

    case BMI2_E_AUX_BUSY:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring"
                 " the AUX",
                 rslt);
        break;

    case BMI2_E_REMAP_ERROR:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes "
                 "after change in axis position",
                 rslt);
        break;

    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status "
                 "fails",
                 rslt);
        break;

    case BMI2_E_SELF_TEST_NOT_DONE:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not "
                 "completed",
                 rslt);
        break;

    case BMI2_E_INVALID_INPUT:
        ESP_LOGE(TAG_COMMON, "Error [%d] : Invalid input error. It occurs when the sensor input validity fails", rslt);
        break;

    case BMI2_E_INVALID_STATUS:
        ESP_LOGE(TAG_COMMON, "Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails", rslt);
        break;

    case BMI2_E_CRT_ERROR:
        ESP_LOGE(TAG_COMMON, "Error [%d] : CRT error. It occurs when the CRT test has failed", rslt);
        break;

    case BMI2_E_ST_ALREADY_RUNNING:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Self-test already running error. It occurs when the self-test is already running and "
                 "another has been initiated",
                 rslt);
        break;

    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong "
                 "address location",
                 rslt);
        break;

    case BMI2_E_DL_ERROR:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length",
                 rslt);
        break;

    case BMI2_E_PRECON_ERROR:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not "
                 "completed",
                 rslt);
        break;

    case BMI2_E_ABORT_ERROR:
        ESP_LOGE(TAG_COMMON, "Error [%d] : Abort error. It occurs when the device was shaken during CRT test", rslt);
        break;

    case BMI2_E_WRITE_CYCLE_ONGOING:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another "
                 "has been initiated",
                 rslt);
        break;

    case BMI2_E_ST_NOT_RUNING:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's "
                 "running",
                 rslt);
        break;

    case BMI2_E_DATA_RDY_INT_FAILED:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit "
                 "and data ready status is not updated",
                 rslt);
        break;

    case BMI2_E_INVALID_FOC_POSITION:
        ESP_LOGE(TAG_COMMON,
                 "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong"
                 " axes",
                 rslt);
        break;

    default:
        ESP_LOGE(TAG_COMMON, "Error [%d] : Unknown error code", rslt);
        break;
    }
}

void bmm150_error_codes_print_result(int8_t rslt)
{
    if (rslt != BMM150_OK)
    {
        switch (rslt)
        {
        case BMM150_E_NULL_PTR:
            ESP_LOGE(TAG_COMMON, "Error [%d] : Null pointer error.", rslt);
            ESP_LOGE(TAG_COMMON,
                     "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.");
            break;

        case BMM150_E_COM_FAIL:
            ESP_LOGE(TAG_COMMON, "Error [%d] : Communication failure error.", rslt);
            ESP_LOGE(TAG_COMMON,
                     "It occurs due to read/write operation failure and also due to power failure during communication");
            break;

        case BMM150_E_DEV_NOT_FOUND:
            ESP_LOGE(TAG_COMMON, "Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read",
                     rslt);
            break;

        case BMM150_E_INVALID_CONFIG:
            ESP_LOGE(TAG_COMMON, "Error [%d] : Invalid sensor configuration.", rslt);
            ESP_LOGE(TAG_COMMON, " It occurs when there is a mismatch in the requested feature with the available one");
            break;

        default:
            ESP_LOGE(TAG_COMMON, "Error [%d] : Unknown error code", rslt);
            break;
        }
    }
}
esp_err_t bmi2Calibrate()
{
    int8_t rslt;
    uint16_t status = 0;
    struct bmi2_sens_data bmi270Data = {{0}};
    int calibrate{100};
    bmi270Device.delay_us(50000, bmi270Device.intf_ptr);
    rslt = bmi2_get_int_status(&status, &bmi270Device);
    bmi2_error_codes_print_result(rslt);

    if ((status & BMI2_ACC_DRDY_INT_MASK) && (status & BMI2_GYR_DRDY_INT_MASK))
    {
        while (calibrate > 0)
        {
            rslt = bmi2_get_sensor_data(&bmi270Data, &bmi270Device);
            bmi2_error_codes_print_result(rslt);
            if (rslt == BMI2_OK)
            {
                _motionSensor->applyCalibrationValues(lsbToDps(bmi270Data.gyr.x), lsbToDps(bmi270Data.gyr.y), lsbToDps(bmi270Data.gyr.z));
            }
            calibrate--;
        }
        ESP_LOGD(TAG_MOTION, "Calibration Done.");
        return ESP_OK;
    }
    ESP_LOGD(TAG_MOTION, "Can't Read Sensor");
    return ESP_ERR_INVALID_STATE;
}
esp_err_t bmi2Init(Motion::MotionFusionSensor *motionSensor)
{
    static u_int8_t dev_addr = CONFIG_BMI270_ADDRESS;
    static u_int8_t bmm150Device_addr = CONFIG_BMM150_ADDRESS;
    _motionSensor = motionSensor;
    int8_t rslt = ESP_OK;
    uint8_t regdata;
    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};
    struct bmm150_settings settings;
    struct bmi2_sens_config config[3];
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;
    config[AUX].type = BMI2_AUX;

    bmm150Device.intf_ptr = &bmm150Device_addr;
    bmm150Device.read = aux_i2c_read;
    bmm150Device.write = aux_i2c_write;
    bmm150Device.delay_us = bmi2_delay_us;
    bmm150Device.intf = BMM150_I2C_INTF;

    bmi270Device.intf = BMI2_I2C_INTF;
    bmi270Device.read = bmi2_i2c_read;
    bmi270Device.write = bmi2_i2c_write;
    bmi270Device.intf_ptr = &dev_addr;
    bmi270Device.delay_us = bmi2_delay_us;
    bmi270Device.read_write_len = READ_WRITE_LEN;
    bmi270Device.config_file_ptr = NULL;

    rslt = bmi270_init(&bmi270Device);
    bmi2_error_codes_print_result(rslt);
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_do_crt(&bmi270Device);
        bmi2_error_codes_print_result(rslt);
        if (rslt == BMI2_OK)
        {
            ESP_LOGD(TAG_MOTION, "CRT successfully completed.");
        }
    }
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &bmi270Device);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_get_sensor_config(config, 3, &bmi270Device);
    bmi2_error_codes_print_result(rslt);

    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    config[AUX].cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    config[AUX].cfg.aux.aux_en = BMI2_ENABLE;
    config[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[AUX].cfg.aux.manual_en = BMI2_ENABLE;
    config[AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    config[AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    config[AUX].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;

    rslt = bmi270_set_sensor_config(config, 3, &bmi270Device);
    bmi2_error_codes_print_result(rslt);

    rslt = bmi270_sensor_enable(sensor_list, 3, &bmi270Device);
    bmi2_error_codes_print_result(rslt);

    rslt = bmm150_init(&bmm150Device);
    bmm150_error_codes_print_result(rslt);

    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &bmm150Device);
    bmm150_error_codes_print_result(rslt);

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi270Device);
    bmi2_error_codes_print_result(rslt);
    if (bmm150Device.chip_id != CONFIG_BMM150_ADDRESS)
    {
        ESP_LOGE("I2C SCAN", "Invalid BMM150 (Aux) sensor - Chip ID : %d hex: 0x%x", bmm150Device.chip_id, bmm150Device.chip_id);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG_MOTION, "Motion initialized with success.");

    return BMI2_OK;
}
esp_err_t bmi2Read()
{
    int8_t rslt;
    uint16_t status = 0;
    struct bmi2_sens_data bmi270Data;
    struct bmm150_mag_data magnetometerData;
    uint8_t auxData[8] = {0};

    bmi270Device.delay_us(50000, bmi270Device.intf_ptr);
    rslt = bmi2_get_int_status(&status, &bmi270Device);
    bmi2_error_codes_print_result(rslt);

    if ((status & BMI2_ACC_DRDY_INT_MASK) && (status & BMI2_GYR_DRDY_INT_MASK))
    {
        rslt = bmi2_get_sensor_data(&bmi270Data, &bmi270Device);
        uint64_t sensorTime = pdTICKS_TO_MS(xTaskGetTickCount());
        bmi2_error_codes_print_result(rslt);
        if (rslt == BMI2_OK)
        {
            _motionSensor->setAccelerometerResult(bmi270Data.acc.x, bmi270Data.acc.y, bmi270Data.acc.z,
                                                  lsbToMps(bmi270Data.acc.x), lsbToMps(bmi270Data.acc.y), lsbToMps(bmi270Data.acc.z));
            _motionSensor->gyroscopeDeadReckoning(lsbToDps(bmi270Data.gyr.x), lsbToDps(bmi270Data.gyr.y), lsbToDps(bmi270Data.gyr.z), sensorTime);

            rslt = bmi2_read_aux_man_mode(BMM150_REG_DATA_X_LSB, auxData, 8, &bmi270Device);
            bmi2_error_codes_print_result(rslt);
            if (rslt == BMI2_OK)
            {
                rslt = bmm150_aux_mag_data(auxData, &magnetometerData, &bmm150Device);
                bmm150_error_codes_print_result(rslt);
                if (rslt == BMI2_OK)
                    _motionSensor->setMagnetometerResult(magnetometerData.x, magnetometerData.y, magnetometerData.z);
            }
        }

        return ESP_OK;
    }
    ESP_LOGD(TAG_MOTION, "Can't Read Sensor");

    return ESP_ERR_INVALID_STATE;
}