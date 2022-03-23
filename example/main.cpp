#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "BMP280.h"
#include "I2Cdev.h"

#define PIN_I2C_SDA GPIO_NUM_32
#define PIN_I2C_SCL GPIO_NUM_33

I2Cdev i2c = I2Cdev(I2C_NUM_0);
BMP280 bmp;
double seaLevelPressure = 103400.0;


extern "C" void update_bmp(void *pvParameters) {
	double temperature, pressure, altitude;
	while (true)
	{
		bmp.readRawData();
		temperature = bmp.getTemperatureDouble();
		pressure = bmp.getPressureDouble();
		altitude = 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
		printf("Temp: %3.2f C; Press: %3.2f Pa; Alti: %3.2f m\n", temperature, pressure, altitude);
		vTaskDelay(250/ portTICK_PERIOD_MS);
	}
}

void start(void)
{

	i2c.initialize(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

	bmp.initialize(&i2c, 0x77);
	
	bmp.configuration.osTemperature = BMP280_OS_16X;
	bmp.configuration.osPressure = BMP280_OS_16X;
	bmp.configuration.outputDataRata = BMP280_ODR_0_5_MS;
	bmp.configuration.filter = BMP280_FILTER_COEFF_16;
	bmp.powerMode = BMP280_NORMAL_MODE;
	bmp.writeConfiguration();

	xTaskCreate(update_bmp, "update_bmp", 2048, NULL, 2, NULL);
}

extern "C" void app_main(void)
{
	start();
}