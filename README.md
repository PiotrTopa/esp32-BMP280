# BMP280 sensor driver for ESP32
Sensor driver implementation based on datasheet and available sources.
It is prepared to be runned as component with dependency on I2Cdev lib (https://github.com/PiotrTopa/esp32-I2Cdev).

# Add components to your project
You need to add this repository along with I2Cdev to your project as ESP-IDF component.
```bash
cd ~/myProjects/myProject
cd components
git clone https://github.com/PiotrTopa/esp32-I2Cdev I2Cdev
git clone https://github.com/PiotrTopa/esp32-BMP280 BMP280
```

# Example