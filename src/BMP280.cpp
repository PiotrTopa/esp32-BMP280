/* ============================================
esp-idf library to support pressure and temperature sensor BMP280.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "BMP280.h"
#include <math.h>

static const char *TAG = "BMP280";

/**
 * Default constructor, uses default I2C device address.
 * @see BMP280_DEFAULT_ADDRESS
 */
BMP280::BMP280()
{
    devAddr = BMP280_I2C_ADDR_DEFAULT;
}

/**
 * Specific address constructor.
 * @param address Specific device address
 * @see BMP280_DEFAULT_ADDRESS
 */
BMP280::BMP280(uint8_t address)
{
    devAddr = address;
}

/**
 * Initialize device
 */
void BMP280::initialize()
{
    ESP_LOGI(TAG, "Device initialization");
    softReset();
    readDeviceId();
    readCalibrationParameters();
}

/**
 * Read deviceId.
 * @see getDeviceId()
 */
void BMP280::readDeviceId()
{
    I2Cdev::readByte(devAddr, BMP280_CHIP_ID_ADDR, &devId);
}

/**
 * Returns stored deviceId.
 * @return uint8_t deviceId
 */
uint8_t BMP280::getDeviceId()
{
    return devId;
}

/**
 * Soft reset of the sensor.
 */
bool BMP280::softReset()
{
    ESP_LOGI(TAG, "Device soft reset");
    int8_t result = I2Cdev::writeByte(devAddr, BMP280_SOFT_RESET_ADDR, BMP280_SOFT_RESET_CMD);
    /* As per the datasheet, startup time is 2 ms. */
    vTaskDelay(2 / portTICK_PERIOD_MS);
    return result;
}

/**
 * Read calibration parameters.
 */
bool BMP280::readCalibrationParameters()
{
    uint8_t temp[BMP280_CALIB_DATA_SIZE] = {0};

    ESP_LOGI(TAG, "Device calibration params reading");

    int8_t rslt = I2Cdev::readBytes(devAddr, BMP280_DIG_T1_LSB_ADDR, BMP280_CALIB_DATA_SIZE, temp);
    if (rslt == BMP280_CALIB_DATA_SIZE)
    {
        calibrationParams.digT1 =
            (uint16_t)(((uint16_t)temp[BMP280_DIG_T1_MSB_POS] << 8) | ((uint16_t)temp[BMP280_DIG_T1_LSB_POS]));
        calibrationParams.digT2 =
            (int16_t)(((int16_t)temp[BMP280_DIG_T2_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_T2_LSB_POS]));
        calibrationParams.digT3 =
            (int16_t)(((int16_t)temp[BMP280_DIG_T3_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_T3_LSB_POS]));
        calibrationParams.digP1 =
            (uint16_t)(((uint16_t)temp[BMP280_DIG_P1_MSB_POS] << 8) | ((uint16_t)temp[BMP280_DIG_P1_LSB_POS]));
        calibrationParams.digP2 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P2_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P2_LSB_POS]));
        calibrationParams.digP3 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P3_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P3_LSB_POS]));
        calibrationParams.digP4 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P4_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P4_LSB_POS]));
        calibrationParams.digP5 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P5_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P5_LSB_POS]));
        calibrationParams.digP6 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P6_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P6_LSB_POS]));
        calibrationParams.digP7 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P7_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P7_LSB_POS]));
        calibrationParams.digP8 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P8_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P8_LSB_POS]));
        calibrationParams.digP9 =
            (int16_t)(((int16_t)temp[BMP280_DIG_P9_MSB_POS] << 8) | ((int16_t)temp[BMP280_DIG_P9_LSB_POS]));

        ESP_LOGI(TAG, "Device calibration params read successfully");
    }

    setDefaultConfiguration();

    return rslt;
}

/* Set defaults conifguration parameters internally */
void BMP280::setDefaultConfiguration()
{
    configuration.filter = BMP280_FILTER_OFF;
    configuration.osPressure = BMP280_OS_NONE;
    configuration.osTemperature = BMP280_OS_NONE;
    configuration.outputDataRata = BMP280_ODR_0_5_MS;
    configuration.spi3wEnabled = BMP280_SPI3_WIRE_DISABLE;
    powerMode = BMP280_SLEEP_MODE;
}

/* Read conifguration from sensor */
bool BMP280::readConfiguration()
{
    uint8_t temp[2] = {0, 0};
    int8_t length = I2Cdev::readBytes(devAddr, BMP280_CTRL_MEAS_ADDR, 2, temp);

    if (length == 2)
    {
        configuration.osTemperature = BMP280_GET_BITS(BMP280_OS_TEMP_MASK, BMP280_OS_TEMP_POS, temp[0]);
        configuration.osPressure = BMP280_GET_BITS(BMP280_OS_PRES_MASK, BMP280_OS_PRES_POS, temp[0]);
        configuration.outputDataRata = BMP280_GET_BITS(BMP280_STANDBY_DURN_MASK, BMP280_STANDBY_DURN_POS, temp[1]);
        configuration.filter = BMP280_GET_BITS(BMP280_FILTER_MASK, BMP280_FILTER_POS, temp[1]);
        configuration.spi3wEnabled = BMP280_GET_BITS(BMP280_SPI3_ENABLE_MASK, 0x00, temp[1]);
        powerMode = BMP280_GET_BITS(BMP280_POWER_MODE_MASK, 0x00, temp[0]);
        return true;
    }
    return false;
}

/* Write conifguration to sensor */
void BMP280::writeConfiguration()
{
    uint8_t temp[2] = {0, 0};
    int8_t length = I2Cdev::readBytes(devAddr, BMP280_CTRL_MEAS_ADDR, 2, temp);
    if (length == 2)
    {
        softReset();
        temp[0] = BMP280_SET_BITS(temp[0], BMP280_OS_TEMP_MASK, BMP280_OS_TEMP_POS, configuration.osTemperature);
        temp[0] = BMP280_SET_BITS(temp[0], BMP280_OS_PRES_MASK, BMP280_OS_PRES_POS, configuration.osPressure);
        temp[1] = BMP280_SET_BITS(temp[1], BMP280_STANDBY_DURN_MASK, BMP280_STANDBY_DURN_POS, configuration.outputDataRata);
        temp[1] = BMP280_SET_BITS(temp[1], BMP280_FILTER_MASK, BMP280_FILTER_POS, configuration.filter);
        temp[1] = BMP280_SET_BITS(temp[1], BMP280_SPI3_ENABLE_MASK, 0x00, configuration.spi3wEnabled);
        I2Cdev::writeBytes(devAddr, BMP280_CTRL_MEAS_ADDR, 2, temp);

        if (powerMode != BMP280_SLEEP_MODE)
        {
            temp[0] = BMP280_SET_BITS(temp[0], BMP280_POWER_MODE_MASK, 0x00, powerMode);
            I2Cdev::writeBytes(devAddr, BMP280_CTRL_MEAS_ADDR, 1, temp);
        }
    }
}

/* Read raw data from sensor */
bool BMP280::readRawData()
{
    uint8_t temp[6] = {0};
    int8_t length = I2Cdev::readBytes(devAddr, BMP280_PRES_MSB_ADDR, 6, temp);
    if (length == 6)
    {
        rawData.rawPressure =
            (int32_t)((((uint32_t)(temp[0])) << 12) | (((uint32_t)(temp[1])) << 4) | ((uint32_t)temp[2] >> 4));
        rawData.rawTemperature =
            (int32_t)((((int32_t)(temp[3])) << 12) | (((int32_t)(temp[4])) << 4) | (((int32_t)(temp[5])) >> 4));
        return true;
    }
    return false;
}

/* Calculate compensated temperature to int32 format */
int32_t BMP280::getTemperature()
{
    int32_t rawTemp = rawData.rawTemperature;
    int32_t var1, var2;

    var1 =
        ((((rawTemp / 8) - ((int32_t)calibrationParams.digT1 << 1))) * ((int32_t)calibrationParams.digT2)) /
        2048;
    var2 =
        (((((rawTemp / 16) - ((int32_t)calibrationParams.digT1)) *
           ((rawTemp / 16) - ((int32_t)calibrationParams.digT1))) /
          4096) *
         ((int32_t)calibrationParams.digT3)) /
        16384;
    calibrationParams.tFine = var1 + var2;
    return (calibrationParams.tFine * 5 + 128) / 256;
}

/* Calculate compensated data to floating point double */
double BMP280::getTemperatureDouble()
{
    int32_t rawTemp = rawData.rawTemperature;
    double var1, var2;

    var1 = (((double)rawTemp) / 16384.0 - ((double)calibrationParams.digT1) / 1024.0) *
           ((double)calibrationParams.digT2);
    var2 =
        ((((double)rawTemp) / 131072.0 - ((double)calibrationParams.digT1) / 8192.0) *
         (((double)rawTemp) / 131072.0 - ((double)calibrationParams.digT1) / 8192.0)) *
        ((double)calibrationParams.digT3);

    calibrationParams.tFine = (int32_t)(var1 + var2);
    return ((var1 + var2) / 5120.0);
}

/* Calculate compensated pressure to int32 format */
int32_t BMP280::getPressure()
{
    int32_t rawPress = rawData.rawPressure;
    int32_t var1, var2, pressure;

    var1 = (((int32_t)calibrationParams.tFine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calibrationParams.digP6);
    var2 = var2 + ((var1 * ((int32_t)calibrationParams.digP5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calibrationParams.digP4) * 65536);
    var1 = (((calibrationParams.digP3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8) +
            ((((int32_t)calibrationParams.digP2) * var1) / 2)) /
           262144;
    var1 = ((((32768 + var1)) * ((int32_t)calibrationParams.digP1)) / 32768);
    pressure = (uint32_t)(((int32_t)(1048576 - rawPress) - (var2 / 4096)) * 3125);

    if (var1 == 0)
    {
        return 0;
    }

    if (pressure < 0x80000000)
    {
        pressure = (pressure << 1) / ((uint32_t)var1);
    }
    else
    {
        pressure = (pressure / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)calibrationParams.digP9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
    var2 = (((int32_t)(pressure / 4)) * ((int32_t)calibrationParams.digP8)) / 8192;
    pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calibrationParams.digP7) / 16));
    return pressure;
}

/* Calculate compensated pressure to floating point double format */
double BMP280::getPressureDouble()
{
    int32_t rawPress = rawData.rawPressure;
    double var1, var2, pressure;

    var1 = ((double)calibrationParams.tFine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calibrationParams.digP6) / 32768.0;
    var2 = var2 + var1 * ((double)calibrationParams.digP5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calibrationParams.digP4) * 65536.0);
    var1 = (((double)calibrationParams.digP3) * var1 * var1 / 524288.0 +
            ((double)calibrationParams.digP2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calibrationParams.digP1);

    pressure = 1048576.0 - (double)rawPress;
    if (var1 == 0.0)
    {
        return 0.0;
    }

    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)calibrationParams.digP9) * (pressure) * (pressure) / 2147483648.0;
    var2 = (pressure) * ((double)calibrationParams.digP8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double)calibrationParams.digP7)) / 16.0;

    return pressure;
}
