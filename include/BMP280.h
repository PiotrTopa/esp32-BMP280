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

#ifndef _BMP280_H_
#define _BMP280_H_

#include "I2Cdev.h"
#include <math.h>
#include "esp_log.h"

/* Sensor address */

#define BMP280_I2C_ADDR_PRIM UINT8_C(0x76)
#define BMP280_I2C_ADDR_SEC UINT8_C(0x77)
#define BMP280_I2C_ADDR_DEFAULT BMP280_I2C_ADDR_SEC

/* Calibration parameter register addresses*/

#define BMP280_DIG_T1_LSB_ADDR UINT8_C(0x88)
#define BMP280_DIG_T1_MSB_ADDR UINT8_C(0x89)
#define BMP280_DIG_T2_LSB_ADDR UINT8_C(0x8A)
#define BMP280_DIG_T2_MSB_ADDR UINT8_C(0x8B)
#define BMP280_DIG_T3_LSB_ADDR UINT8_C(0x8C)
#define BMP280_DIG_T3_MSB_ADDR UINT8_C(0x8D)
#define BMP280_DIG_P1_LSB_ADDR UINT8_C(0x8E)
#define BMP280_DIG_P1_MSB_ADDR UINT8_C(0x8F)
#define BMP280_DIG_P2_LSB_ADDR UINT8_C(0x90)
#define BMP280_DIG_P2_MSB_ADDR UINT8_C(0x91)
#define BMP280_DIG_P3_LSB_ADDR UINT8_C(0x92)
#define BMP280_DIG_P3_MSB_ADDR UINT8_C(0x93)
#define BMP280_DIG_P4_LSB_ADDR UINT8_C(0x94)
#define BMP280_DIG_P4_MSB_ADDR UINT8_C(0x95)
#define BMP280_DIG_P5_LSB_ADDR UINT8_C(0x96)
#define BMP280_DIG_P5_MSB_ADDR UINT8_C(0x97)
#define BMP280_DIG_P6_LSB_ADDR UINT8_C(0x98)
#define BMP280_DIG_P6_MSB_ADDR UINT8_C(0x99)
#define BMP280_DIG_P7_LSB_ADDR UINT8_C(0x9A)
#define BMP280_DIG_P7_MSB_ADDR UINT8_C(0x9B)
#define BMP280_DIG_P8_LSB_ADDR UINT8_C(0x9C)
#define BMP280_DIG_P8_MSB_ADDR UINT8_C(0x9D)
#define BMP280_DIG_P9_LSB_ADDR UINT8_C(0x9E)
#define BMP280_DIG_P9_MSB_ADDR UINT8_C(0x9F)

/* Calibration parameters' relative position */
#define BMP280_DIG_T1_LSB_POS UINT8_C(0)
#define BMP280_DIG_T1_MSB_POS UINT8_C(1)
#define BMP280_DIG_T2_LSB_POS UINT8_C(2)
#define BMP280_DIG_T2_MSB_POS UINT8_C(3)
#define BMP280_DIG_T3_LSB_POS UINT8_C(4)
#define BMP280_DIG_T3_MSB_POS UINT8_C(5)
#define BMP280_DIG_P1_LSB_POS UINT8_C(6)
#define BMP280_DIG_P1_MSB_POS UINT8_C(7)
#define BMP280_DIG_P2_LSB_POS UINT8_C(8)
#define BMP280_DIG_P2_MSB_POS UINT8_C(9)
#define BMP280_DIG_P3_LSB_POS UINT8_C(10)
#define BMP280_DIG_P3_MSB_POS UINT8_C(11)
#define BMP280_DIG_P4_LSB_POS UINT8_C(12)
#define BMP280_DIG_P4_MSB_POS UINT8_C(13)
#define BMP280_DIG_P5_LSB_POS UINT8_C(14)
#define BMP280_DIG_P5_MSB_POS UINT8_C(15)
#define BMP280_DIG_P6_LSB_POS UINT8_C(16)
#define BMP280_DIG_P6_MSB_POS UINT8_C(17)
#define BMP280_DIG_P7_LSB_POS UINT8_C(18)
#define BMP280_DIG_P7_MSB_POS UINT8_C(19)
#define BMP280_DIG_P8_LSB_POS UINT8_C(20)
#define BMP280_DIG_P8_MSB_POS UINT8_C(21)
#define BMP280_DIG_P9_LSB_POS UINT8_C(22)
#define BMP280_DIG_P9_MSB_POS UINT8_C(23)
#define BMP280_CALIB_DATA_SIZE UINT8_C(24)

#define BMP280_CHIP_ID_ADDR UINT8_C(0xD0)
#define BMP280_SOFT_RESET_ADDR UINT8_C(0xE0)
#define BMP280_STATUS_ADDR UINT8_C(0xF3)
#define BMP280_CTRL_MEAS_ADDR UINT8_C(0xF4)
#define BMP280_CONFIG_ADDR UINT8_C(0xF5)
#define BMP280_PRES_MSB_ADDR UINT8_C(0xF7)
#define BMP280_PRES_LSB_ADDR UINT8_C(0xF8)
#define BMP280_PRES_XLSB_ADDR UINT8_C(0xF9)
#define BMP280_TEMP_MSB_ADDR UINT8_C(0xFA)
#define BMP280_TEMP_LSB_ADDR UINT8_C(0xFB)
#define BMP280_TEMP_XLSB_ADDR UINT8_C(0xFC)

/* Configuration bits masks and positions */
#define BMP280_STATUS_IM_UPDATE_POS          UINT8_C(0)
#define BMP280_STATUS_IM_UPDATE_MASK         UINT8_C(0x01)
#define BMP280_STATUS_MEAS_POS               UINT8_C(3)
#define BMP280_STATUS_MEAS_MASK              UINT8_C(0x08)
#define BMP280_OS_TEMP_POS                   UINT8_C(5)
#define BMP280_OS_TEMP_MASK                  UINT8_C(0xE0)
#define BMP280_OS_PRES_POS                   UINT8_C(2)
#define BMP280_OS_PRES_MASK                  UINT8_C(0x1C)
#define BMP280_POWER_MODE_POS                UINT8_C(0)
#define BMP280_POWER_MODE_MASK               UINT8_C(0x03)
#define BMP280_STANDBY_DURN_POS              UINT8_C(5)
#define BMP280_STANDBY_DURN_MASK             UINT8_C(0xE0)
#define BMP280_FILTER_POS                    UINT8_C(2)
#define BMP280_FILTER_MASK                   UINT8_C(0x1C)
#define BMP280_SPI3_ENABLE_POS               UINT8_C(0)
#define BMP280_SPI3_ENABLE_MASK              UINT8_C(0x01)

/* Configuration: Output Data Rata */
#define BMP280_ODR_0_5_MS                    UINT8_C(0x00)
#define BMP280_ODR_62_5_MS                   UINT8_C(0x01)
#define BMP280_ODR_125_MS                    UINT8_C(0x02)
#define BMP280_ODR_250_MS                    UINT8_C(0x03)
#define BMP280_ODR_500_MS                    UINT8_C(0x04)
#define BMP280_ODR_1000_MS                   UINT8_C(0x05)
#define BMP280_ODR_2000_MS                   UINT8_C(0x06)
#define BMP280_ODR_4000_MS                   UINT8_C(0x07)

/* Configuration: oversampling */
#define BMP280_OS_NONE                       UINT8_C(0x00)
#define BMP280_OS_1X                         UINT8_C(0x01)
#define BMP280_OS_2X                         UINT8_C(0x02)
#define BMP280_OS_4X                         UINT8_C(0x03)
#define BMP280_OS_8X                         UINT8_C(0x04)
#define BMP280_OS_16X                        UINT8_C(0x05)

/* Configuration: Filter */
#define BMP280_FILTER_OFF                    UINT8_C(0x00)
#define BMP280_FILTER_COEFF_2                UINT8_C(0x01)
#define BMP280_FILTER_COEFF_4                UINT8_C(0x02)
#define BMP280_FILTER_COEFF_8                UINT8_C(0x03)
#define BMP280_FILTER_COEFF_16               UINT8_C(0x04)

/* Configuration: SPI 3 wire */
#define BMP280_SPI3_WIRE_ENABLE              UINT8_C(1)
#define BMP280_SPI3_WIRE_DISABLE             UINT8_C(0)

/* Configuration: Power modes */
#define BMP280_SLEEP_MODE                    UINT8_C(0x00)
#define BMP280_FORCED_MODE                   UINT8_C(0x01)
#define BMP280_NORMAL_MODE                   UINT8_C(0x03)

#define BMP280_SOFT_RESET_CMD UINT8_C(0xB6)
#define BMP280_CALIB_DATA_SIZE UINT8_C(24)

#define BMP280_GET_BITS(bitmask, bitpos, x)             ((x & bitmask) >> bitpos)
#define BMP280_SET_BITS(regvar, bitmask, bitpos, val)   ((regvar & ~bitmask) | ((val << bitpos) & bitmask))

class BMP280
{
        struct CalibrationParams
        {
                uint16_t digT1;
                int16_t digT2;
                int16_t digT3;
                uint16_t digP1;
                int16_t digP2;
                int16_t digP3;
                int16_t digP4;
                int16_t digP5;
                int16_t digP6;
                int16_t digP7;
                int16_t digP8;
                int16_t digP9;
                int32_t tFine;
        };

        struct Configuration
        {
                uint8_t osTemperature;
                uint8_t osPressure;
                uint8_t outputDataRata;
                uint8_t filter;
                uint8_t spi3wEnabled;
        };

        struct Status
        {
                uint8_t measuring;
                uint8_t imUpdate;
        };

        struct RawData
        {
                int32_t rawTemperature;
                uint32_t rawPressure;
        };

public:
        BMP280();
        BMP280(uint8_t address);

        void initialize();
        bool testConnection();

        CalibrationParams calibrationParams;
        Configuration configuration;
        Status status;
        RawData rawData;
        uint8_t powerMode;

        void readDeviceId();
        bool softReset();
        bool readConfiguration();
        void writeConfiguration();
        bool readRawData();
        int32_t getTemperature();
        double getTemperatureDouble();
        int32_t getPressure();
        double getPressureDouble();
        uint8_t getDeviceId();
private:
        uint8_t devAddr;
        uint8_t devId;
        bool readCalibrationParameters();
        void setDefaultConfiguration();
};

#endif /* _BMP280_H_ */